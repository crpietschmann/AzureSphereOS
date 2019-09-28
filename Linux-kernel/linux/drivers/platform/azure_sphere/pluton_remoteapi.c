// SPDX-License-Identifier: GPL-2.0
/*
 * Pluton remote API
 *
 * Copyright (c) 2018 Microsoft Corporation. All rights reserved.
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

#ifdef CONFIG_PLUTON_REMOTEAPI_DEBUG

// Set DEBUG to 1 to enable debug log output
#define DEBUG 1

#endif

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock_types.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <azure-sphere/pluton_remoteapi.h>

#include "pluton_remoteapi_provider.h"

#define COMPLETION_TIMEOUT_IN_MS (25000)

// Event buffer size
#define M4_EVENT_BUFFER_SIZE 2048

#define M4_MAX_EVENT_BUFFERS 128

#define M4_BUFFER_COUNT 32
#define A7_BUFFER_COUNT 16

#define TOTAL_EVENT_BUFFER_SIZE (M4_EVENT_BUFFER_SIZE * (M4_BUFFER_COUNT + A7_BUFFER_COUNT) + sizeof(struct pluton_ring_buffers))

// message payload structure
struct pluton_remoteapi_message {
    // Size of message
    u16 message_size;
	// Reserved space
	u16 padding;
    // Pointer to message data
    u8 message_data[0];
};

// A single buffer in our ring
struct pluton_ring_buffer {
	// Address relative to the M4
	dma_addr_t m4_address;
	// Address relative to the A7
	u8 *a7_buffer;
	// Length of buffer
	u16 length;
	// If 1 then this is available for use
	u8 available;
	// Index of buffer
	u8 index;
};

// Ring buffer structure for data transfer
struct pluton_ring_buffers {
	// Number of buffers for M4 originating writes
	u16 m4_buffer_count;
	// Number of buffers for A7 originating writes
	u16 a7_buffer_count;

	// List of buffers
	struct pluton_ring_buffer buffers[M4_MAX_EVENT_BUFFERS];
};

// What kind of response we're going to route for a received message
enum pluton_relay_response_type {
	NO_RESPONSE, // No action needed
	CALLBACK,    // Invoke a callback
	COMPLETION,  // Copy data and invoke a completion event
};

// Response data for a completion callback
struct pluton_relay_completion_data {
	// Completion object
	struct completion *completion;
	// Data buffer to copy response to
	void *buffer;
	// Size of buffer
	u32 buffer_size;
	// Size of the needed buffer if the caller didn't provide enough space
	u32 *needed_buffer_size;
	// Pointer to hold status from message
	u8 *status;
};

// Response data for an async callback
struct pluton_callback_data {
	// Callback when we get a response on this sequence number
	pluton_remote_api_callback callback;
	// Callback data object
	void *callback_data;
};

// Data on how to route responses
union pluton_relay_response_data {
	// Data for callback when we get a response on this sequence number
	struct pluton_callback_data callback;
	// Data for completion based responses
	struct pluton_relay_completion_data completion;
};

// Housekeeping structure for tracking relay info objects
struct pluton_relay_management_info {
	// What type of response we're going to issue
	enum pluton_relay_response_type response_type;
	// Our response data
	union pluton_relay_response_data response_data;
	// List pointer
	struct list_head list;
	// Sequence number - used to execute callbacks
	u8 sequence_number;
};

// Token representing an outstanding mailbox transfer.  Leverages the exising encoded mailbox command.
typedef u32	pluton_tranfer_token;

// Current mailbox protocol version
static const u8 PLUTON_PROTOCOL_VERSION = 2;

struct pluton_remoteapi_state {
	// linked list for relay management items
	struct list_head relay_management_list;
	// Lock construct for relay objects
	spinlock_t relay_lock;
	// Provider pointer
	struct pluton_remoteapi_provider *provider;
	// Last sequence number used
	u8 last_sequence_number;
	// Sequence numbers in use
	u32 sequence_number_use[8];
	// Allocated ring buffers
	struct pluton_ring_buffers *ring_buffers;
	// DMA address of ring buffers
	dma_addr_t ring_buffer_addr;
	// Last buffer used for transmission
	uint8_t last_tx_buffer;
};

static struct pluton_remoteapi_state g_state;

// Forward declarations
static int pluton_remoteapi_send_ring_buffers(void *buffers, dma_addr_t buffers_addr);

///
/// Alloc  event buffers - should be called on driver startup to ensure there's
/// buffers ready for data
///
/// @return - 0 for success
static int pluton_alloc_ring_buffers(void)
{
	int i;

	g_state.ring_buffers = dma_alloc_coherent(g_state.provider->dev, TOTAL_EVENT_BUFFER_SIZE,
		&g_state.ring_buffer_addr, GFP_KERNEL);
	if (g_state.ring_buffers == NULL) {
		return -ENOMEM;
	}

	// Allocate our buffers
	memset(g_state.ring_buffers, 0, sizeof(struct pluton_ring_buffers));
	g_state.ring_buffers->m4_buffer_count = M4_BUFFER_COUNT;
	g_state.ring_buffers->a7_buffer_count = A7_BUFFER_COUNT;
	
	for (i = 0; i < g_state.ring_buffers->m4_buffer_count + g_state.ring_buffers->a7_buffer_count; i++) {
		g_state.ring_buffers->buffers[i].a7_buffer = (u8 *)g_state.ring_buffers + sizeof(struct pluton_ring_buffers) + (M4_EVENT_BUFFER_SIZE * i);
		g_state.ring_buffers->buffers[i].m4_address = g_state.ring_buffer_addr + sizeof(struct pluton_ring_buffers) + (M4_EVENT_BUFFER_SIZE * i);
		
		if (g_state.ring_buffers->buffers[i].a7_buffer == NULL) {
			dev_err(g_state.provider->dev, "Out of memory allocating ring buffer");
			return -ENOMEM;
		}

		g_state.ring_buffers->buffers[i].index = i;
		g_state.ring_buffers->buffers[i].available = 1;
		g_state.ring_buffers->buffers[i].length = M4_EVENT_BUFFER_SIZE;
	}

	// Send buffer data
	pluton_remoteapi_send_ring_buffers(g_state.ring_buffers, g_state.ring_buffer_addr);

	dev_dbg(g_state.provider->dev, "Allocated %u new event buffers", i);

	return SUCCESS;
}

///
/// Initialization of Remote API
///
int pluton_remoteapi_init(void)
{
	int ret = SUCCESS;
	g_state.last_sequence_number = 0;
	g_state.ring_buffer_addr = DMA_ERROR_CODE;
	g_state.last_tx_buffer = A7_BUFFER_COUNT - 1;
	
	memset(g_state.sequence_number_use, 0, sizeof(g_state.sequence_number_use));
	g_state.sequence_number_use[0] = 0x1; // 0 is a reserved sequence number that
					// should never be handed out

	spin_lock_init(&g_state.relay_lock);
	INIT_LIST_HEAD(&g_state.relay_management_list);

	// Alloc some buffers for RX
	ret = pluton_alloc_ring_buffers();
	if (ret != SUCCESS) {
		goto exit;
	}

exit:
	return ret;
}

///
/// Shuts down Remote API
///
void pluton_remoteapi_shutdown(void)
{
	if (g_state.ring_buffer_addr != DMA_ERROR_CODE) {
		dma_free_coherent(g_state.provider->dev, TOTAL_EVENT_BUFFER_SIZE, 
			g_state.ring_buffers, g_state.ring_buffer_addr);		
			g_state.ring_buffers = NULL;
		}
}

///
/// Return a sequence number to the pool
///
/// @sequence_number - Sequence number
static void pluton_remote_api_free_sequence_number(u8 sequence_number)
{
	u32 idx;
	u32 mask;

	// Sequence number 0 is reserved for unsolicited messages
	// As a safety never mark it for available use
	if (sequence_number == 0) {
		return;
	}

	idx = sequence_number >> 5;
	mask = sequence_number & 0x1f;

	g_state.sequence_number_use[idx] &= ~(1 << mask);
}

///
/// Gets the next free sequence number
/// Sequence numbers are from 0-255
/// This means we're going to roll over but have to
/// consider any numbers actively in use.
/// This is done by keeping an active bitfield.
///
/// @returns -  Sequence number
static u8 pluton_remote_api_get_next_sequence_number(void)
{
	const int MAX_SEQUENCE_NUMBER = 255;
	int i = 0;
	u32 idx;
	u32 mask;
	u8 ret;

	spin_lock(&g_state.relay_lock);

	for (i = 0; i < MAX_SEQUENCE_NUMBER; i++) {
		g_state.last_sequence_number++;

		idx = g_state.last_sequence_number >> 5;
		mask = g_state.last_sequence_number & 0x1f;

		if ((g_state.sequence_number_use[idx] & (1 << mask)) == 0) {
			// It's free for use
			g_state.sequence_number_use[idx] |= (1 << mask);
			ret = g_state.last_sequence_number;
			break;
		}
	}

	spin_unlock(&g_state.relay_lock);

	return ret;
}

///
/// Allocate and initialize new management info item
///
/// @returns -  New item
static struct pluton_relay_management_info *
pluton_remote_api_alloc_relay_management_info(void)
{
	struct pluton_relay_management_info *ret = NULL;

	ret = devm_kzalloc(g_state.provider->dev, sizeof(*ret), GFP_KERNEL);
	if (ret) {
		// Initialize list pointer
		INIT_LIST_HEAD(&ret->list);
	}

	return ret;
}

///
/// Gets the management info item for a sequence number.
/// IMPORTANT: The item is removed from the management info list and the
///            caller is responsible for freeing item memory when finshed.
///
/// @sequence_number - Sequence number
/// @returns -  Item that matches the number or NULL
static struct pluton_relay_management_info *
pluton_remote_api_get_relay_management_info(u8 sequence_number)
{
	struct pluton_relay_management_info *mgmt_item = NULL;
	struct pluton_relay_management_info *ret = NULL;

	// Acquire lock
	spin_lock(&g_state.relay_lock);

	list_for_each_entry(mgmt_item, &g_state.relay_management_list,
			    list)
	{
		if (mgmt_item->sequence_number == sequence_number) {

			// Remove from list
			list_del(&mgmt_item->list);

			ret = mgmt_item;
			break;
		}
	}

	// Release lock
	spin_unlock(&g_state.relay_lock);

	return ret;
}

///
/// Frees a management info item.  Note that the item should already be removed from the management info list.
///
/// @item - item to free
static void pluton_remote_api_free_management_info(struct pluton_relay_management_info *item) 
{
	// Acquire lock
	spin_lock(&g_state.relay_lock);

	// Release sequence number
	if (item->sequence_number != 0) {
		pluton_remote_api_free_sequence_number(item->sequence_number);
	}

	// Free
	devm_kfree(g_state.provider->dev, item);

	// Release lock
	spin_unlock(&g_state.relay_lock);
}

///
/// Gets a free relay management info item
///
/// @returns -  Item to use
static struct pluton_relay_management_info *
pluton_remote_api_get_free_relay_management_info(void)
{
	struct pluton_relay_management_info *ret = NULL;

	// Acquire lock
	spin_lock(&g_state.relay_lock);

	// Allocate our new item
	ret = pluton_remote_api_alloc_relay_management_info();
	if (ret != NULL) {
		list_add(&ret->list, &g_state.relay_management_list);
	}

	// Release lock
	spin_unlock(&g_state.relay_lock);

	return ret;
}

///
/// Gets a TX buffer for transmission
///
/// @return - TX buffer
static struct pluton_ring_buffer *pluton_get_tx_buffer(void) 
{
	void *ret = NULL;
	int idx = 0;

	// Acquire lock
	spin_lock(&g_state.relay_lock);

	if (g_state.ring_buffers == NULL) {
		goto exit;
	}

	idx = ((g_state.last_tx_buffer + 1) % A7_BUFFER_COUNT);

	if (g_state.ring_buffers->buffers[g_state.ring_buffers->m4_buffer_count + idx].available) {
		g_state.last_tx_buffer = idx;
		g_state.ring_buffers->buffers[g_state.ring_buffers->m4_buffer_count + idx].available = 0;

		return &g_state.ring_buffers->buffers[g_state.ring_buffers->m4_buffer_count + idx];
	}

exit:
	// Release lock
	spin_unlock(&g_state.relay_lock);

	return ret;
}

///
/// Marks a buffer as available by index
///
/// @buffer_idx - index
static void pluton_release_buffer_by_idx(u8 buffer_idx) 
{
	// Acquire lock
	spin_lock(&g_state.relay_lock);

	if (g_state.ring_buffers == NULL) {
		goto exit;
	}

	g_state.ring_buffers->buffers[buffer_idx].available = 1;

exit:
	// Release lock
	spin_unlock(&g_state.relay_lock);
}

///
/// Gets a message by buffer index
///
/// @buffer_idx - index
/// @data_size - size of buffer
/// @returns - message buffer
static struct pluton_remoteapi_message *pluton_get_message_by_buffer_idx(u8 buffer_idx, u16 *data_size) 
{
	if (data_size != NULL) {
		*data_size = g_state.ring_buffers->buffers[buffer_idx].length;
	}
	return (struct pluton_remoteapi_message *)g_state.ring_buffers->buffers[buffer_idx].a7_buffer;
}

///
/// Processes a response message and fires any callbacks
///
/// @cmd - command
/// @data - data pointer
/// @data_size - data size
static void pluton_remote_api_process_response_callbacks(union pluton_remoteapi_command_encoded cmd,
										 void *data, u32 data_size) 
{
	struct pluton_relay_management_info *mgmt_item = NULL;

	// Get mgmt item for this operation
	mgmt_item =
	    pluton_remote_api_get_relay_management_info(cmd.info.sequence_number);
	if (mgmt_item == NULL) {
		dev_dbg(g_state.provider->dev,
			"Response sequence number %hhu not found.  It may have been removed by a cancellation.",
			cmd.info.sequence_number);
		return;
	}

	if (mgmt_item->response_type == CALLBACK &&
	    mgmt_item->response_data.callback.callback != NULL) {
		// Fire callback
		mgmt_item->response_data.callback.callback(data, data_size, mgmt_item->response_data.callback.callback_data);
	} else if (mgmt_item->response_type == COMPLETION) {
		*mgmt_item->response_data.completion.needed_buffer_size =
		    data_size;
		*mgmt_item->response_data.completion.status = 0;

		// We only do the copy if the caller passed a buffer large
		// enough.
		// The caller must needed_buffer_size to see if they gave us a
		// big enough buffer before reading data.
		if (data_size <=
		    mgmt_item->response_data.completion.buffer_size) {
			// Copy data to output buffer
			memcpy(mgmt_item->response_data.completion.buffer, data,
			       data_size);
		}

		// Signal we're done
		complete_all(mgmt_item->response_data.completion.completion);
	} else if (mgmt_item->response_type == NO_RESPONSE) {
		// Nothing to do
	} else {
		dev_err(g_state.provider->dev, "Unsupported response type: %#x",
			mgmt_item->response_type);
	}

	// Free memory
	pluton_remote_api_free_management_info(mgmt_item);
}

///
/// Processes a response message and fires any callbacks
///
/// @cmd - command
/// @sequence_number - message sequence number
static void pluton_remote_api_process_response(union pluton_remoteapi_command_encoded cmd, 
										u8 sequence_number)
{
	u16 buffer_length = 0;
	struct pluton_remoteapi_message *message = NULL;
	void *data = NULL;
	u32 data_size = 0;
	
	message = pluton_get_message_by_buffer_idx(cmd.info.buffer_index, &buffer_length);

	// Verify size
	if (buffer_length < sizeof(struct pluton_remoteapi_message) 
		|| buffer_length < (message->message_size + sizeof(struct pluton_remoteapi_message))) {
		dev_err(g_state.provider->dev, "Invalid message buffer size");
		goto exit;
	}

	data = message->message_data;
	data_size = message->message_size;

	pluton_remote_api_process_response_callbacks(cmd, data, data_size);

exit:
	// Free memory
	pluton_release_buffer_by_idx(cmd.info.buffer_index);
}

///
/// Dispatches an incoming message
///
/// @cmd - Command data
static void pluton_remote_api_handle_message(union pluton_remoteapi_command_encoded cmd)
{
	dev_dbg(g_state.provider->dev, "M4 message: %u seq num: %d", cmd.info.command,
		cmd.info.sequence_number);

	switch (cmd.info.command) {
	case SEND_EVENT_BUFFERS:
	case SET_POSTCODE:
	case GET_BOOT_MODE_FLAGS:
	case READ_RNG:
	case GET_SECURITY_STATE:
	case GET_ENABLED_CAPABILITIES:
	case IS_CAPABILITY_ENABLED:
	case GET_MANUFACTURING_STATE:
	case SET_MANUFACTURING_STATE:
	case GENERATE_CLIENT_AUTH_KEY:
	case COMMIT_CLIENT_AUTH_KEY:
	case GET_TENANT_PUBLIC_KEY:
	case PROCESS_ATTESTATION:
	case SIGN_WITH_TENANT_ATTESTATION_KEY:
		pluton_remote_api_process_response(cmd, cmd.info.sequence_number);
		break;
	default:
		dev_err(g_state.provider->dev,
			"Unsupported relay M4 command: %#x", cmd.info.command);
	}
}

///
/// Dispatches an incoming message
///
/// @cmd - Command ID
void pluton_remoteapi_incoming_message(union pluton_remoteapi_command_encoded cmd)
{
	// Verify version
	if (cmd.info.version != PLUTON_PROTOCOL_VERSION) {
		dev_err(g_state.provider->dev, "Unsupported version: %u",
			(u32)cmd.info.version);
		return;
	}

	dev_dbg(g_state.provider->dev, "Incoming message cmd %x buffer %x", (u32)cmd.info.command, 
			(u32)cmd.info.buffer_index);

	pluton_remote_api_handle_message(cmd);
}

///
/// Checks if the RemoteAPI has been initialized and is ready for use
///
/// @returns - 0 if ready, -EPROBE_DEFER or a simlar error if not
int pluton_remote_api_is_ready(void)
{
	if (g_state.provider == NULL) {
		// We're not initialized yet
		return -EPROBE_DEFER;
	}

	return 0;
}

///
/// Cancels the response from an earlier command sent to the M4
///
/// @transfer_token - Token identifying the transfer response to be cancelled
/// @returns -  0 for success
static int pluton_remoteapi_cancel_response_from_pluton(pluton_tranfer_token transfer_token)
{
	union pluton_remoteapi_command_encoded cmd_encoded = { .as_u32 = transfer_token };
	int ret = SUCCESS;
	struct pluton_relay_management_info *mgmt_info = NULL;

	// Validate handle
	if (g_state.provider == NULL) {
		return -EINVAL;
	}

	// Validate transfer token
	if (transfer_token == 0) {
		dev_err(g_state.provider->dev, "Invalid transfer token passed to "
					     "pluton_remoteapi_cancel_response_from_pluton");
		return -EINVAL;
	}

	// Acquire lock
	spin_lock(&g_state.relay_lock);

	mgmt_info = pluton_remote_api_get_relay_management_info(cmd_encoded.info.sequence_number);
	if (mgmt_info == NULL)
	{
		ret = -EINVAL;
        goto exit;
	}

	// Free the queued relay management entry.
	pluton_remote_api_free_management_info(mgmt_info); 

	// Note that we don't release the tx buffer here.  Releasing it runs the risk that ours has already transferred
	// and that we're releasing someone elses in the same ring buffer location.  The buffer will normally be released when 
	// the M4 finally responds. The downside is that if it never responds, we won't but then there are bigger problems.

exit:

	// Release lock
	spin_unlock(&g_state.relay_lock);

	return ret;
}

///
/// Sends a packet of data to the M4
///
/// @cmd - M4 command ID
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @response_type - How to handle response
/// @response_data - Data to use in response handling
/// @transfer_token - Token that could be used to reference the transfer to the point it's completed
/// @returns -  0 for success
static int pluton_remote_api_send_command_to_m4(
    enum pluton_remoteapi_commands cmd, void *data, u32 data_size,
	enum pluton_relay_response_type response_type,
    union pluton_relay_response_data response_data,
	pluton_tranfer_token *transfer_token)
{
	int ret = SUCCESS;
	struct pluton_ring_buffer *buffer = NULL;
	struct pluton_relay_management_info *mgmt_info = NULL;
	struct pluton_remoteapi_message header;
	union pluton_remoteapi_command_encoded cmd_encoded;
	u16 payload_size = 0;

	// Validate handle
	if (g_state.provider == NULL) {
		dev_err(g_state.provider->dev, "RemoteAPI provider not available");
		return -EINVAL;
	}

	if (data_size > 0 && data == NULL) {
		dev_err(g_state.provider->dev, "Invalid data size or pointer "
					     "passed to "
					     "pluton_remote_api_send_command_to_m4");
		return -EINVAL;
	}

	// Get a free relay info structure to set up
	mgmt_info = pluton_remote_api_get_free_relay_management_info();
	if (mgmt_info == NULL) {
		dev_err(g_state.provider->dev, "Could not allocate relay info in "
					     "pluton_remote_api_send_command_to_m4");
		return -ENOMEM;
	}

	// Build our M4 header for this request + copy data into memory we
	// control
	header.message_size = ALIGN(data_size, 4);

	payload_size = sizeof(struct pluton_remoteapi_message) + header.message_size;
	
	// Copy header + data into a final buffer
	buffer = pluton_get_tx_buffer();

	if (buffer == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	// Verify size
	if (buffer->length < payload_size) {
		dev_err(g_state.provider->dev, "Invalid M4 packet size");
		// rcho debug line
		dev_err(g_state.provider->dev, "M4 packet size %d larger than ring buf length %d", payload_size, buffer->length);
		return -ENOMEM;
	}

	memcpy(buffer->a7_buffer, &header, sizeof(struct pluton_remoteapi_message));
	memcpy(&buffer->a7_buffer[sizeof(struct pluton_remoteapi_message)], data, data_size);

	// build out relay structure
	mgmt_info->sequence_number = pluton_remote_api_get_next_sequence_number();
	mgmt_info->response_type = response_type;
	mgmt_info->response_data = response_data;

	cmd_encoded.as_u32 = 0;
	cmd_encoded.info.command = cmd;
	cmd_encoded.info.version = PLUTON_PROTOCOL_VERSION;
	cmd_encoded.info.sequence_number = mgmt_info->sequence_number;
	cmd_encoded.info.buffer_index = buffer->index;

	dev_dbg(g_state.provider->dev,
		"Sending M4 command %x with sequence number %hhx buffer at %x", cmd,
		mgmt_info->sequence_number, buffer->m4_address);

	// Send the message
	ret = g_state.provider->send_message(cmd_encoded);

exit:
	if (ret != SUCCESS) {
		dev_err(g_state.provider->dev,
			"Failed to send M4 command %x with error: %d", cmd, ret);

		// Free data
		if (buffer != NULL) {
			pluton_release_buffer_by_idx(buffer->index);
		}
		pluton_remote_api_free_management_info(mgmt_info);
	}

	// Return a tranfer token to the caller that could be used later to cancel
	// the tranfer if necessary.
	if (transfer_token != NULL)
	{
		*transfer_token = (ret == SUCCESS) ? cmd_encoded.as_u32 : 0;
	}

	return ret;
}

///
/// Sends a packet of data to the M4 asynchronously with a callback for the
/// response
///
/// @cmd - M4 command ID
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @callback - Optional callback to invoke when a response is received
/// @returns -  0 for success
int pluton_remote_api_send_command_to_m4_async(
    enum pluton_remoteapi_commands cmd, void *data, u32 data_size,
    pluton_remote_api_callback callback, void *callback_data)
{
	union pluton_relay_response_data response_data = {.callback = {.callback = callback, .callback_data = callback_data}};

	return pluton_remote_api_send_command_to_m4(
	    cmd, data, data_size, CALLBACK, response_data, NULL);
}
EXPORT_SYMBOL(pluton_remote_api_send_command_to_m4_async);

///
/// Sends a packet of data to the M4 and blocks on a response
///
/// @cmd - M4 command ID
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @buffer - Buffer to hold response data
/// @buffer_size - Size of buffer
/// @returns - 0 for success
int pluton_remote_api_send_command_to_m4_sync(enum pluton_remoteapi_commands cmd,
					      void *data, u32 data_size,
					      void *buffer, u32 buffer_size)
{
	struct completion c;
	pluton_tranfer_token token;
	u32 needed_buffer_size = 0;
	u8 status = 0;
	int ret;
	u32 time_remaining = 0;

	union pluton_relay_response_data response_data = {
	    .completion = {.completion = &c,
			   .buffer = buffer,
			   .buffer_size = buffer_size,
			   .needed_buffer_size = &needed_buffer_size,
			   .status = &status}};

	if (buffer == NULL) {
		return -EINVAL;
	}

	init_completion(&c);

	// Send command
	ret = pluton_remote_api_send_command_to_m4(
	    cmd, data, data_size, COMPLETION, response_data, &token);
	if (ret != SUCCESS) {
		return ret;
	}

	// Requests should get resposnes in a small amount of time (usec or msc)
	// We put a timeout here to catch cases where a response never comes.
	// This is always unexpected but we WARN and return so we can recover.
	time_remaining =
	    wait_for_completion_timeout(&c, msecs_to_jiffies(COMPLETION_TIMEOUT_IN_MS));

	// The wait completion routine can internally return a negative number if we receive a fatal/cancel signal.  Converted
	// to an unsigned value, it's large.  Warn on it here but don't specifically try to cancel a response request since there
	// would be a larger scope clean-up happening anyways.
	WARN_ON(time_remaining > msecs_to_jiffies(COMPLETION_TIMEOUT_IN_MS));

	// Warn if a time-out occurs.
	WARN_ON(time_remaining == 0);

	if (time_remaining == 0) {
		// Cancel the transfer response since it may not come.  If we fail to cancel the transfer, we can't exit otherwise
		// we risk a memory fault should the completion handler fire after we've returned.
		if (pluton_remoteapi_cancel_response_from_pluton(token) == SUCCESS)
		{
			dev_dbg(g_state.provider->dev,
				"Successfully cancelled M4 response request following a wait timeout.\n");

			return -ETIMEDOUT;
		}

		dev_dbg(g_state.provider->dev,
				"Failed to cancel M4 response request following a wait timeout, waiting for completion...\n");

		// Wait without time-out.
		wait_for_completion(&c);
	}

	if (needed_buffer_size != 0 && needed_buffer_size > buffer_size) {
		// Invalid buffer size
		return -ENOMEM;
	}

	return status;
}
EXPORT_SYMBOL(pluton_remote_api_send_command_to_m4_sync);

///
/// Sends a packet of data to the M4 and doesn't wait for a response
///
/// @cmd - M4 command ID
/// @set - True if this is a set command, false if it is a get command
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @returns - 0 for success
int pluton_remote_api_send_command_to_m4_no_response(enum pluton_remoteapi_commands cmd,
						     void *data, u32 data_size)
{
	union pluton_relay_response_data response_data = {};

	return pluton_remote_api_send_command_to_m4(
	    cmd, data, data_size, NO_RESPONSE, response_data, NULL);
}
EXPORT_SYMBOL(pluton_remote_api_send_command_to_m4_no_response);

///
/// Sends buffers to the M4 for later use
///
/// @buffers - ring buffers
/// @buffers_addr - DMA address of structure
/// @returns - 0 for success
static int pluton_remoteapi_send_ring_buffers(void *buffers, dma_addr_t buffers_addr)
{
	union pluton_remoteapi_command_encoded cmd_encoded;

	// Validate provider
	if (g_state.provider == NULL) {
		return -EINVAL;
	}

	if (buffers == NULL) {
		dev_err(g_state.provider->dev, "Invalid data size or pointer "
					     "passed to pluton_remoteapi_send_ring_buffers");
		return -EINVAL;
	}

	cmd_encoded.as_u32 = 0;
	cmd_encoded.info.command = SEND_EVENT_BUFFERS;
	cmd_encoded.info.version = PLUTON_PROTOCOL_VERSION;
	cmd_encoded.info.sequence_number = 0;
	cmd_encoded.info.buffer_index = 0;

	// Send the message
	return g_state.provider->send_ring_buffers(buffers_addr, cmd_encoded);
}

///
/// Register a remoteapi data provider
/// @provider - provider instance
///
int pluton_remoteapi_register_provider(struct pluton_remoteapi_provider *provider)
{
	g_state.provider = provider;

	return pluton_remoteapi_init();
}

///
/// Unregister a remoteapi data provider
/// @provider - provider instance
///
void pluton_remoteapi_unregister_provider(struct pluton_remoteapi_provider *provider)
{
	if (g_state.provider == provider) {
		g_state.provider = NULL;

		pluton_remoteapi_shutdown();
	}
}
