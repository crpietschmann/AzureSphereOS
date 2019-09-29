// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere IO Core communication driver
 *
 * Copyright (c) 2019 Microsoft Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <mt3620/mt3620_mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/skbuff.h>
#include <azure-sphere/security_monitor.h>
#include <azure-sphere/pluton_remoteapi.h>
#include "../../../net/azspio/azspio.h"

#define DRIVER_NAME "azure-sphere-iocore-comm"

/* list of all nodes */
static LIST_HEAD(iocore_comm_all);
/* lock for iocore_comm_all */
static DEFINE_MUTEX(iocore_comm_lock);

#define IOCORE_BUFFER_SIZE 4096
// 1024 bytes of data + 16 bytes of component ID + 4 bytes of port
// (io_core_comm doesn't care about the contents, but this is the
// agreed MTU to fit 1024 data bytes)
#define IOCORE_MTU (1024 + 16 + 4)

struct communication_buffer_header {
	uint32_t write_pointer;
	uint32_t read_pointer;
	uint32_t reserved[14];
};

#define DATA_START(buffer) ((uint8_t *)(buffer + 1))

#define RINGBUFFER_ALIGNMENT 16

#define MAILBOX_CH_MAIN 0
#define MAILBOX_SETUP_INBOUND 0xBA5E0001
#define MAILBOX_SETUP_OUTBOUND 0xBA5E0002
#define MAILBOX_START_COMMUNICATION 0xBA5E0003
#define MAILBOX_STOP_COMMUNICATION 0xBA5E0004
#define MAILBOX_USERINT_RX 1
#define MAILBOX_USERINT_TX 2

#define MAILBOX_CH_USERINT 1

#define TYPE_INBOUND 0
#define TYPE_OUTBOUND 1

enum { IOCORE_COMM_STATE_OFF, // Endpoint not registered
       IOCORE_COMM_STATE_HALTED, // Error state
       IOCORE_COMM_STATE_ACTIVE, // Communication enabled
       IOCORE_COMM_STATE_THROTTLED, // No RX because downstream queues are full
};

struct iocore_comm {
	struct device *dev;
	struct mbox_client iocore_client;
	struct mbox_chan *iocore_channel[2];
	dma_addr_t handle_outbound;
	dma_addr_t handle_inbound;
	struct communication_buffer_header *outbound;
	struct communication_buffer_header *inbound;
	loff_t buffer_size;
	struct azspio_endpoint ep;
	struct mutex rx_lock;
	int state;
	u32 core;

	// Copy of outbound->{write,read}_pointer
	// These are relative to the beginning of the buffer.
	uint32_t local_write_pointer;
	uint32_t local_read_pointer;

	struct list_head item;

	struct hrtimer incoming_retry;
};

static int iocore_mailbox_send(struct iocore_comm *comm, int channel,
			       uint32_t cmd, uint32_t data)
{
	struct mt3620_mailbox_data *mailbox_data =
		kmalloc(sizeof(struct mt3620_mailbox_data), GFP_KERNEL);
	int rc;

	if (!mailbox_data)
		return -ENOMEM;

	mailbox_data->cmd = cmd;
	mailbox_data->data = data;

	rc = mbox_send_message(comm->iocore_channel[channel], mailbox_data);

	// We intentionally set a short timeout; a timeout means that we've been
	// able to send the data, but the data was not read in time by the IO core
	// application. There's no requirement on when the IO core application reads
	// the mailbox data to initialize communication. Hence ignore -ETIME.
	if (rc < 0) {
		if (rc != -ETIME) {
			dev_warn(comm->dev, "mbox_send_message failed %d\n",
				 rc);
			kfree(mailbox_data);
		} else
			rc = 0;
	}

	return rc;
}

// The idea here is to treat the ring as full if the write pointer is "1 behind" the read pointer.
// The ring is empty if the two pointers match.  Effectively, this approach reserves one "slot".""
static bool iocore_ringbuffer_full(uint32_t write_pointer, uint32_t read_pointer, uint32_t buffer_size) {
	return (write_pointer + RINGBUFFER_ALIGNMENT) % buffer_size == read_pointer;
}

// Return the number of bytes used in the ring.
uint32_t iocore_ringbuffer_used_space(uint32_t write_pointer, uint32_t read_pointer, uint32_t buffer_size) {
	if (!iocore_ringbuffer_full(write_pointer, read_pointer, buffer_size)) {
		if (write_pointer >= read_pointer)
			return write_pointer - read_pointer;
		else
			return buffer_size + write_pointer - read_pointer;
	}

	return buffer_size - RINGBUFFER_ALIGNMENT;
}

// Return the number of bytes of free space in the ring.
static uint32_t iocore_ringbuffer_free_space(uint32_t write_pointer, uint32_t read_pointer, uint32_t buffer_size) {
	return buffer_size - RINGBUFFER_ALIGNMENT - iocore_ringbuffer_used_space(write_pointer, read_pointer, buffer_size);
}

static int iocore_ringbuffer_read(struct iocore_comm *comm, void *data,
				  size_t data_size, bool peek, bool final)
{
	struct communication_buffer_header *outbound = comm->outbound;
	struct communication_buffer_header *inbound = comm->inbound;
	uint32_t buffer_size =
		comm->buffer_size - sizeof(struct communication_buffer_header);
	uint32_t remote_write_pointer = inbound->write_pointer;
	uint32_t local_read_pointer = comm->local_read_pointer;
	uint32_t amount_of_data;
	uint32_t data_to_end;
	uint8_t *read_data;
	uint32_t to_read;

	dev_dbg(comm->dev,
		"REMOTE WRITE POINTER %08x | LOCAL READ BUFFER %08x\n",
		remote_write_pointer, local_read_pointer);

	if (remote_write_pointer >= buffer_size) {
		dev_err(comm->dev,
			"FATAL: remote write pointer >= buffer size\n");
		return -EFAULT;
	}

	if (!IS_ALIGNED(remote_write_pointer, RINGBUFFER_ALIGNMENT)) {
		dev_err(comm->dev, "FATAL: remote write pointer not aligned\n");
		return -EFAULT;
	}

	// Calculate amount of data left in buffer between write and
	// read pointer.
	amount_of_data = iocore_ringbuffer_used_space(remote_write_pointer, local_read_pointer, buffer_size);

	// Calculate amount of data after write pointer
	data_to_end = buffer_size - local_read_pointer;

	// If the ring buffer is empty, return -EAGAIN.
	if (amount_of_data < data_size) {
		dev_dbg(comm->dev, "not enough data (%u)\n", amount_of_data);
		return -EAGAIN;
	}

	// Calculate pointer to begin of data
	read_data = DATA_START(inbound) + local_read_pointer;

	// Calculate how much of the data to read
	to_read = min(data_size, data_to_end);

	dev_dbg(comm->dev, "remaining to read %d\n", to_read);

	// copy with optional wrap-around.
	if (data != NULL) {
		memcpy(data, read_data, to_read);
		memcpy((uint8_t *)data + to_read, DATA_START(inbound),
			data_size - to_read);
	}

	// Adjust read pointer with data section
	local_read_pointer += data_size;

	if (final)
		local_read_pointer =
			ALIGN(local_read_pointer, RINGBUFFER_ALIGNMENT);

	if (local_read_pointer >= buffer_size)
		local_read_pointer -= buffer_size;

	dev_dbg(comm->dev,
		"[A read] READ POINTER UPDATE %08x -> %08x, wptr %08x\n",
		outbound->read_pointer, local_read_pointer,
		remote_write_pointer);

	// update read pointer if this is not peek
	if (!peek) {
		comm->local_read_pointer = local_read_pointer;

		// update outbound read pointer and send interrupt
		// if this is final data read.
		if (final) {
			outbound->read_pointer = comm->local_read_pointer;
			wmb();
			iocore_mailbox_send(comm, MAILBOX_CH_USERINT,
					    MAILBOX_USERINT_RX, 0);
		}
	}

	// return number of bytes of data actually read.
	return data_size;
}

static int iocore_ringbuffer_write(struct iocore_comm *comm, void *header,
				   size_t header_size, const void *data,
				   size_t data_size)
{
	uint32_t remote_read_pointer;
	uint32_t local_write_pointer;
	uint32_t remaining_space;
	uint32_t data_to_end;
	uint32_t to_write;

	uint32_t total_data_size =
		ALIGN(header_size + data_size, RINGBUFFER_ALIGNMENT);

	struct communication_buffer_header *outbound = comm->outbound;
	struct communication_buffer_header *inbound = comm->inbound;
	uint32_t buffer_size =
		comm->buffer_size - sizeof(struct communication_buffer_header);

	remote_read_pointer = inbound->read_pointer;
	local_write_pointer = comm->local_write_pointer;

	dev_dbg(comm->dev,
		"remote read pointer %08x, local write pointer %08x, trying to add %u bytes total\n",
		remote_read_pointer,
		local_write_pointer,
		total_data_size);
	if (remote_read_pointer >= buffer_size) {
		dev_err(comm->dev,
			"FATAL - remote read pointer is %08x, >= %08x\n",
			remote_read_pointer, buffer_size);
		return -EFAULT;
	}

	remaining_space = iocore_ringbuffer_free_space(local_write_pointer, remote_read_pointer, buffer_size);

	dev_dbg(comm->dev, "remaining space %u, size: %u, need %u\n",
		remaining_space,
		iocore_ringbuffer_used_space(local_write_pointer, remote_read_pointer, buffer_size),
		total_data_size);
	if (remaining_space < total_data_size) {
		dev_dbg(comm->dev, "not enough space for data\n");
		return -ENOSPC;
	}

	// write header

	data_to_end = buffer_size - local_write_pointer;
	to_write = min(header_size, data_to_end);
	memcpy(DATA_START(outbound) + local_write_pointer, header, to_write);
	memcpy(DATA_START(outbound), (const uint8_t *)header + to_write, header_size - to_write);

	local_write_pointer += header_size;
	if (local_write_pointer >= buffer_size)
		local_write_pointer -= buffer_size;

	// write data

	data_to_end = buffer_size - local_write_pointer;
	to_write = min(data_size, data_to_end);

	memcpy(DATA_START(outbound) + local_write_pointer, data, to_write);
	memcpy(DATA_START(outbound), (const uint8_t *)data + to_write, data_size - to_write);
	wmb();

	local_write_pointer +=
		ALIGN(header_size + data_size, RINGBUFFER_ALIGNMENT) -
		header_size;
	if (local_write_pointer >= buffer_size)
		local_write_pointer -= buffer_size;

	// Update write pointer in both outbound buffer as well as local comm structure
	outbound->write_pointer = comm->local_write_pointer =
		local_write_pointer;
	wmb();
	iocore_mailbox_send(comm, MAILBOX_CH_USERINT, MAILBOX_USERINT_TX, 0);

	dev_dbg(comm->dev, "write pointer now %08x | rptr %08x, size: %u, full: %u\n",
		outbound->write_pointer,
		inbound->read_pointer,
		iocore_ringbuffer_used_space(local_write_pointer, remote_read_pointer, buffer_size),
		iocore_ringbuffer_full(local_write_pointer, remote_read_pointer, buffer_size));
	return 0;
}

static int iocore_comm_xmit(struct azspio_endpoint *ep, struct sk_buff *skb)
{
	struct iocore_comm *comm = container_of(ep, struct iocore_comm, ep);
	int rc;
	uint32_t size;

	BUG_ON(comm->state == IOCORE_COMM_STATE_OFF);

	if (comm->state == IOCORE_COMM_STATE_HALTED) {
		rc = -ECONNRESET;
		goto out;
	}

	rc = skb_linearize(skb);
	if (rc)
		goto out;

	size = skb->len;
	rc = iocore_ringbuffer_write(comm, &size, sizeof(size), skb->data,
				     skb->len);

	dev_dbg(comm->dev, "xmit %d bytes, ret %d\n", skb->len, rc);

out:
	if (rc == -ENOSPC) {
		// Do nothing and return ownership of SKB to caller.
	} else if (rc == 0) {
		consume_skb(skb);
	} else {
		// Drop packet
		kfree(skb);
	}

	return rc;
}

static void iocore_halt_communication(struct iocore_comm *comm)
{
	if (comm->state == IOCORE_COMM_STATE_ACTIVE ||
	    comm->state == IOCORE_COMM_STATE_THROTTLED) {
		dev_info(comm->dev, "halt communication\n");
		comm->state = IOCORE_COMM_STATE_HALTED;
	}
}

static void iocore_comm_incoming(struct iocore_comm *comm)
{
	struct sk_buff *skb;

	int ret;
	uint32_t size;

	while (comm->state == IOCORE_COMM_STATE_ACTIVE) {
		// try to dequeue a packet; first, read size
		ret = iocore_ringbuffer_read(comm, &size, sizeof(size), true,
					     false);

		// If there's nothing in the ringbuffer, return.
		if (ret == -EAGAIN)
			break;

		// All other errors are unexpected.
		if (ret != sizeof(size)) {
			dev_err(comm->dev,
				"iocore_ringbuffer_read failed (%d)\n", ret);
			iocore_halt_communication(comm);
			return;
		}

		dev_dbg(comm->dev, "got a %08x bytes packet\n", size);

		if (size > IOCORE_MTU) {
			dev_err(comm->dev,
				"packet is larger than MTU; halting communication\n");
			iocore_halt_communication(comm);
			return;
		}

		skb = netdev_alloc_skb(NULL, size);

		if (!skb) {
			dev_err(comm->dev,
				"FAILED TO ALLOCATE SKB for incoming data\n");

			// retry in one second
			hrtimer_start(&comm->incoming_retry, ktime_set(1, 0),
				      HRTIMER_MODE_REL);
			return;
		}

		skb_reset_transport_header(skb);

		// Skip size field; we're committed to read the packet now.
		// Since we successfully peeked above, we can expect this to work.
		iocore_ringbuffer_read(comm, NULL, sizeof(size), false, false);

		// Read the actual data.
		ret = iocore_ringbuffer_read(comm, skb_put(skb, size), size,
					     false, true);
		if (ret < 0) {
			dev_err(comm->dev,
				"reading the packet failed; halting communication\n");
			iocore_halt_communication(comm);
			kfree_skb(skb);
			break;
		}

		dev_dbg(comm->dev, "posting the packet..\n");

		ret = azspio_endpoint_post(&comm->ep, skb);
		if (ret < 0) {
			kfree_skb(skb);
			dev_err_ratelimited(comm->dev, "azspio_endpoint_post failed %d\n",
				ret);
		}
	}
}

static enum hrtimer_restart iocore_comm_incoming_retry(struct hrtimer *hrtimer)
{
	struct iocore_comm *comm =
		container_of(hrtimer, struct iocore_comm, incoming_retry);

	iocore_comm_incoming(comm);

	return HRTIMER_NORESTART;
}

static void iocore_comm_throttle(struct azspio_endpoint *ep, bool throttled)
{
	struct iocore_comm *comm = container_of(ep, struct iocore_comm, ep);

	dev_dbg(comm->dev, "iocore_comm_throttle %d\n", throttled);

	if (throttled) {
		if (comm->state != IOCORE_COMM_STATE_ACTIVE) {
			dev_err(comm->dev,
				"throttle request but comm is not active\n");
			return;
		}
		comm->state = IOCORE_COMM_STATE_THROTTLED;
	} else {
		if (comm->state != IOCORE_COMM_STATE_THROTTLED) {
			dev_err(comm->dev,
				"unthrottle request but comm is not throttled\n");
			return;
		}
		comm->state = IOCORE_COMM_STATE_ACTIVE;
	}

	if (!throttled) {
		mutex_lock(&comm->rx_lock);
		iocore_comm_incoming(comm);
		mutex_unlock(&comm->rx_lock);
	}
}

static int iocore_comm_start(struct iocore_comm *comm, struct device *dev,
			     int core, const u8 *component_id)
{
	int rc;
	int type;
	uint32_t cmd;

	if (comm->state != IOCORE_COMM_STATE_OFF)
		return -EBUSY;

	// Note that the mailbox is empty at this point because it is
	// flushed on IO application start.

	// clear shared memory buffers (header + data) and reset local read/write ptrs
	memset(comm->inbound, 0, comm->buffer_size);
	memset(comm->outbound, 0, comm->buffer_size);
	comm->local_write_pointer = 0;
	comm->local_read_pointer = 0;

	for (type = 0; type < 2; ++type) {
		uintptr_t ptr;

		if (type == TYPE_INBOUND) {
			ptr = comm->handle_inbound;
			cmd = MAILBOX_SETUP_INBOUND;
		} else { // type == TYPE_OUTBOUND
			ptr = comm->handle_outbound;
			cmd = MAILBOX_SETUP_OUTBOUND;
		}

		rc = azure_sphere_sm_io_core_configure_communication_buffer(
			core, type, ptr, comm->buffer_size);
		if (rc < 0)
			return rc;

		/* The low 5 bits of the buffer address are the log2 of the buffer size. */
		ptr |= ilog2(comm->buffer_size);

		rc = iocore_mailbox_send(comm, MAILBOX_CH_MAIN, cmd, ptr);
		BUG_ON(rc < 0);
		if (rc < 0)
			return rc;
	}

	comm->ep.xmit = iocore_comm_xmit;
	comm->ep.throttle = iocore_comm_throttle;

	rc = azspio_endpoint_register(&comm->ep, component_id);
	if (rc)
		return rc;

	comm->state = IOCORE_COMM_STATE_ACTIVE;

	rc = iocore_mailbox_send(comm, MAILBOX_CH_MAIN,
				 MAILBOX_START_COMMUNICATION, 0);
	if (rc < 0) {
		dev_warn(comm->dev, "mbox_send_message failed %d\n", rc);
		return rc;
	}

	return 0;
}

int iocore_communication_start(int core, const u8 *component_id)
{
	int ret;
	struct iocore_comm *comm = NULL, *entry;

	mutex_lock(&iocore_comm_lock);

	list_for_each_entry (entry, &iocore_comm_all, item) {
		if (entry->core == core) {
			comm = entry;
		}
	}

	if (comm == NULL) {
		pr_warn("iocore_communication_start(%d) - no such core\n",
			core);
		ret = -ENODEV;
	} else {
		ret = iocore_comm_start(comm, comm->dev, core, component_id);
	}

	mutex_unlock(&iocore_comm_lock);

	return ret;
}

EXPORT_SYMBOL_GPL(iocore_communication_start);

static void iocore_comm_stop(struct iocore_comm *comm)
{
	if (comm->state != IOCORE_COMM_STATE_OFF) {
		int rc;

		rc = iocore_mailbox_send(comm, MAILBOX_CH_MAIN,
					 MAILBOX_STOP_COMMUNICATION, 0);
		if (rc < 0) {
			dev_warn(comm->dev, "mbox_send_message failed %d\n",
				 rc);
			// ignore failure
		}
		mutex_lock(&comm->rx_lock);
		// unregister endpoint.
		azspio_endpoint_unregister(&comm->ep);
		comm->state = IOCORE_COMM_STATE_OFF;
		mutex_unlock(&comm->rx_lock);
	}
}

int iocore_communication_stop(int core)
{
	int ret;
	struct iocore_comm *comm = NULL, *entry;

	mutex_lock(&iocore_comm_lock);

	list_for_each_entry (entry, &iocore_comm_all, item) {
		if (entry->core == core) {
			comm = entry;
		}
	}

	if (comm == NULL) {
		pr_warn("iocore_communication_stop(%d) - no such core\n", core);
		ret = -ENODEV;
	} else {
		iocore_comm_stop(comm);
		ret = 0;
	}
	mutex_unlock(&iocore_comm_lock);

	return ret;
}

EXPORT_SYMBOL_GPL(iocore_communication_stop);

static void iocore_comm_rx_callback(struct mbox_client *cl, void *mesg)
{
	struct iocore_comm *comm =
		container_of(cl, struct iocore_comm, iocore_client);
	struct mt3620_mailbox_data *mailbox_data =
		(struct mt3620_mailbox_data *)mesg;

	if (comm->state == IOCORE_COMM_STATE_OFF ||
	    comm->state == IOCORE_COMM_STATE_HALTED) {
		dev_warn(
			comm->dev,
			"Ignoring interrupt because communication is disabled.");
		return;
	}

	if (mailbox_data->cmd & MAILBOX_USERINT_RX) {
		/* obtain rx_lock to serialize 
		 * iocore_comm_incoming in respect to throttling */
		mutex_lock(&comm->rx_lock);
		iocore_comm_incoming(comm);
		mutex_unlock(&comm->rx_lock);
	}

	if (mailbox_data->cmd & MAILBOX_USERINT_TX) {
		azspio_endpoint_notify(&comm->ep);
	}
}

static int azure_sphere_iocore_comm_probe(struct platform_device *pdev)
{
	struct iocore_comm *comm;
	struct device_node *node = pdev->dev.of_node;
	int i;

	int ret = 0;

	ret = pluton_remote_api_is_ready();
	if (ret != 0) {
		// Our dependencies haven't started yet, tell the kernel to try
		// again later
		return ret;
	}

	// Allocate memory for our driver state
	comm = devm_kzalloc(&pdev->dev, sizeof(*comm), GFP_KERNEL);
	if (!comm) {
		return -ENOMEM;
	}

	comm->dev = &pdev->dev;

	// The "reg" node will contain the IO core number
	if (of_property_read_u32(node, "reg", &comm->core))
		return -ENODEV;

	mutex_init(&comm->rx_lock);
	comm->buffer_size = IOCORE_BUFFER_SIZE;

	hrtimer_init(&comm->incoming_retry, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	comm->incoming_retry.function = iocore_comm_incoming_retry;

	// Allocate IO buffers

	comm->inbound = dma_alloc_coherent(&pdev->dev, comm->buffer_size,
					   &comm->handle_inbound, GFP_KERNEL);

	comm->outbound = dma_alloc_coherent(&pdev->dev, comm->buffer_size,
					    &comm->handle_outbound, GFP_KERNEL);

	if (!comm->inbound || !comm->outbound) {
		dev_err(&pdev->dev, "Error iocore_comm_alloc failed with %d.\n",
			ret);
		ret = -ENOMEM;
		goto exit;
	}

	// Prepare mailbox client
	comm->iocore_client.dev = comm->dev;
	comm->iocore_client.tx_block = true;
	comm->iocore_client.knows_txdone = false;
	comm->iocore_client.rx_callback = iocore_comm_rx_callback;

	// Set a low timeout. This allows to block for messages, yet
	// it doesn't block delivery into the FIFO even if the IO core
	// application doesn't read them yet.
	// This is to address a shortcoming in the mailbox subsystem which
	// waits for the IO core to read the messages.
	// (Setting tx_block=false doesn't help because those messages would
	// still be queued up.)
	// All mailbox/FIFO communication will completely fit into the FIFO.
	comm->iocore_client.tx_tout = 10; // 10 ms

	// Allocate the mailbox endpoints for our IO cores; first endpoint
	// is data, second is user interrupt.
	for (i = 0; i < ARRAY_SIZE(comm->iocore_channel); ++i) {
		comm->iocore_channel[i] =
			mbox_request_channel(&comm->iocore_client, i);
		ret = IS_ERR(comm->iocore_channel[i]);
		if (ret) {
			dev_err(&pdev->dev,
				"Error mbox_request_channel failed with %d.\n",
				ret);
			ret = PTR_ERR(comm->iocore_channel[i]);
			goto exit;
		}
	}

	mutex_lock(&iocore_comm_lock);
	list_add(&comm->item, &iocore_comm_all);
	mutex_unlock(&iocore_comm_lock);

	return 0;
exit:
	for (i = 0; i < ARRAY_SIZE(comm->iocore_channel); ++i)
		if (comm->iocore_channel[i])
			mbox_free_channel(comm->iocore_channel[i]);
	if (comm->inbound)
		dma_free_coherent(&pdev->dev, comm->buffer_size, comm->inbound,
				  comm->handle_inbound);
	if (comm->outbound)
		dma_free_coherent(&pdev->dev, comm->buffer_size, comm->outbound,
				  comm->handle_outbound);

	hrtimer_cancel(&comm->incoming_retry);

	return ret;
}

static int azure_sphere_iocore_comm_remove(struct platform_device *pdev)
{
	int i;
	struct iocore_comm *comm = platform_get_drvdata(pdev);
	if (!comm)
		return -EINVAL;

	hrtimer_cancel(&comm->incoming_retry);

	// remove from global list
	mutex_lock(&iocore_comm_lock);
	list_del(&comm->item);
	mutex_unlock(&iocore_comm_lock);

	// stop communication; this will set state to "OFF"
	iocore_comm_stop(comm);

	for (i = 0; i < ARRAY_SIZE(comm->iocore_channel); ++i)
		mbox_free_channel(comm->iocore_channel[i]);

	dma_free_coherent(&pdev->dev, comm->buffer_size, comm->inbound,
			  comm->handle_inbound);
	dma_free_coherent(&pdev->dev, comm->buffer_size, comm->outbound,
			  comm->handle_outbound);
	return 0;
}

// Driver metadata.
static const struct of_device_id azure_sphere_iocore_comm_match[] = {
	{ .compatible = "microsoft,mt3620-io-core-comm" },
	{ /* Sentinel */ }
};

static struct platform_driver azure_sphere_iocore_comm_driver = {
	.probe = azure_sphere_iocore_comm_probe,
	.remove = azure_sphere_iocore_comm_remove,
	.driver = { .name = DRIVER_NAME,
		    .of_match_table = azure_sphere_iocore_comm_match }
};

MODULE_DEVICE_TABLE(of, azure_sphere_iocore_comm_match);
module_platform_driver(azure_sphere_iocore_comm_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("Azure Sphere IO Core communication driver");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_ALIAS("platform:azure-sphere-iocore-comm");
