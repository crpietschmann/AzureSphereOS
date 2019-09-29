// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 Mailbox Controller driver
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

#ifdef CONFIG_MT3620_MAILBOX_DEBUG

// Set DEBUG to 1 to enable debug log output
#define DEBUG 1

#endif

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock_types.h>
#include <linux/workqueue.h>
#include <mt3620/mt3620_mailbox.h>
#include "mailbox.h"
#include "mt3620.h"

#define DRIVER_NAME "mt3620-mailbox"

///
/// Gets the current count of items waiting to be read
///
/// @channel - Mailbox channel
/// @returns - Item count
static int mt3620_mailbox_get_read_count(struct mt3620_mailbox_channel *channel)
{
	return readl_relaxed(channel->channel_base +
			     MBOX_FIFO_READ_COUNT_OFFSET);
}

///
/// Gets the current count of items waiting to be written to the M4
///
/// @channel - Mailbox channel
/// @returns - Item count
static int
mt3620_mailbox_get_write_count(struct mt3620_mailbox_channel *channel)
{
	return readl_relaxed(channel->channel_base +
			     MBOX_FIFO_WRITE_COUNT_OFFSET);
}

static u32 mt3620_mailbox_channel_offset(struct mt3620_mailbox *mailbox,
					 u32 channel_count)
{
	switch (mailbox->version) {
	case MBOX_VER_MT3620:
		return (0x10000 * channel_count);

	default:
		return 0;
	}
}

///
/// Converts a mbox_chan to our private data structure
///
/// @chan - Mailbox channel
/// @returns - mt3620 mailbox channel
static struct mt3620_mailbox_channel *
mbox_chan_to_mt3620_mailbox_channel(struct mbox_chan *chan)
{
	uintptr_t ptr;
	if (!chan || !chan->con_priv) {
		return NULL;
	}

	ptr = ((uintptr_t)chan->con_priv) & ~MBOX_MT3620_CHAN_TYPE_MASK;

	return (struct mt3620_mailbox_channel *)ptr;
}

int mbox_chan_to_mt3620_mailbox_channel_type(struct mbox_chan *chan)
{
	if (!chan) {
		return 0;
	}

	return ((uintptr_t)chan->con_priv) & MBOX_MT3620_CHAN_TYPE_MASK;
}

///
/// Sends data across a mailbox channel
///
/// @chan - Mailbox channel
/// @chan - Data buffer from mailbox APIs
/// @returns - mt3620 mailbox channel
/// <remarks>
/// data must point to a mt3620_mailbox_data structure
/// </remarks>
static int mt3620_mailbox_send_data(struct mbox_chan *chan, void *data)
{
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	int channel_type = mbox_chan_to_mt3620_mailbox_channel_type(chan);

	struct mt3620_mailbox_data *mailbox_data =
		(struct mt3620_mailbox_data *)data;

	if (!channel || !mailbox_data) {
		return -EINVAL;
	}

	// One reader / writer at a time
	spin_lock(&channel->lock);

	dev_dbg(channel->dev, "Sending mailbox command %#x", mailbox_data->cmd);
	dev_dbg(channel->dev, "Sending mailbox data at %#x",
		mailbox_data->data);

	if (channel_type == MBOX_MT3620_CHAN_TYPE_FIFO) {
		// Is the mailbox full?
		if (mt3620_mailbox_get_write_count(channel) ==
		    channel->max_fifo_count) {
			return -EBUSY;
		}

		if (mailbox_data->data != 0U) {
			// Write data, then cmd
			writel_relaxed(mailbox_data->data,
				       channel->channel_base +
					       MBOX_DATA_POST_OFFSET);
			// Ensure we've written data first, otherwise the mailbox is going to
			// load the wrong data
			dma_wmb();
		}

		writel_relaxed(mailbox_data->cmd,
			       channel->channel_base + MBOX_CMD_POST_OFFSET);
		dev_dbg(channel->dev, "Mailbox count is now %u",
			mt3620_mailbox_get_write_count(channel));
	} else if (channel_type == MBOX_MT3620_CHAN_TYPE_USERINT) {
		writel_relaxed(mailbox_data->cmd,
			       channel->channel_base +
				       MXBO_SW_TX_INT_PORT_OFFSET);
	}

	spin_unlock(&channel->lock);

	kfree(data);

	return 0;
}

///
/// Checks if the last TX done has completed
///
/// @chan - Mailbox channel
/// @returns - True if completed
static bool mt3620_mailbox_last_tx_done(struct mbox_chan *chan)
{
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	if (!channel) {
		return false;
	}

	if (mbox_chan_to_mt3620_mailbox_channel_type(chan) ==
	    MBOX_MT3620_CHAN_TYPE_FIFO) {
		dev_dbg(channel->dev, "Last TX done called");

		// Return false if mailbox is full
		if (mt3620_mailbox_get_write_count(channel) ==
		    channel->max_fifo_count) {
			return false;
		} else {
			return true;
		}
	} else { // MBOX_MT3620_CHAN_TYPE_USERINT
		return true;
	}
}

///
/// Checks if there is data ready to be read
///
/// @chan - Mailbox channel
/// @returns - True if data is available
static bool mt3620_mailbox_peek_data(struct mbox_chan *chan)
{
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	if (!channel) {
		return false;
	}

	if (mbox_chan_to_mt3620_mailbox_channel_type(chan) ==
	    MBOX_MT3620_CHAN_TYPE_FIFO) {
		// True if we have data to read
		return mt3620_mailbox_get_read_count(channel) > 0 ? true :
								    false;
	} else { // MBOX_MT3620_CHAN_TYPE_USERINT
		uint32_t user_int;

		user_int = readl_relaxed(channel->channel_base +
					 MBOX_SW_RX_INT_STS_OFFSET);

		return user_int != 0;
	}
}

///
/// Bottom half IRQ interrupt handler for when we're reading data
///
/// @data - Interupt data - mbox_chan pointer
///
/// Note - this is a work queue and not a tasklet so rx callbacks can alloc
/// memory
static void mt3620_mailbox_rx_interrupt_bottom_half(struct work_struct *work)
{
	struct mt3620_mailbox_data mailbox_data;
	struct mt3620_mailbox_channel *channel =
		container_of(work, struct mt3620_mailbox_channel, read_work);
	struct mbox_chan *chan = channel->mbox_chan;

	// One reader at a time
	spin_lock(&channel->lock);

	while (mt3620_mailbox_get_read_count(channel) != 0) {
		// M4 doesn't send anything through data
		mailbox_data.data = 0;

		mailbox_data.cmd = readl_relaxed(channel->channel_base +
						 MBOX_CMD_POP_OFFSET);

		dev_dbg(channel->dev, "New item read - command %x data %#x",
			mailbox_data.cmd, mailbox_data.data);

		// handle it
		mbox_chan_received_data(chan, &mailbox_data);
	}

	spin_unlock(&channel->lock);
}

///
/// IRQ interrupt handler for when we're reading data
///
/// @irq - IRQ number
/// @p - Pointer to mbox_chan object
/// @returns - IRQ result status code
static irqreturn_t mt3620_mailbox_rx_interrupt(int irq, void *p)
{
	struct mbox_chan *chan = (struct mbox_chan *)p;
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	if (!channel) {
		return IRQ_HANDLED;
	}

	// Reset interrupt flag
	writel_relaxed(1 << MBOX_INT_TYPE_RD,
		       channel->channel_base + MBOX_INT_STS_OFFSET);

	// Call handler
	schedule_work(&channel->read_work);

	return IRQ_HANDLED;
}

///
/// IRQ interrupt handler for user interrupts
///
/// @irq - IRQ number
/// @p - Pointer to mbox_chan object
/// @returns - IRQ result status code
static irqreturn_t mt3620_mailbox_user_interrupt(int irq, void *p)
{
	struct mbox_chan *chan = (struct mbox_chan *)p;
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	if (!channel) {
		return IRQ_HANDLED;
	}

	// Disable interrupts
	writel_relaxed(0, channel->channel_base + MBOX_SW_RX_INT_EN_OFFSET);

	// Call handler
	schedule_work(&channel->user_work);

	return IRQ_HANDLED;
}

///
/// Bottom half IRQ interrupt handler for user interrupts
///
/// @data - Interupt data - mbox_chan pointer
///
/// Note - this is a work queue and not a tasklet so user callbacks can alloc
/// memory
static void mt3620_mailbox_user_interrupt_bottom_half(struct work_struct *work)
{
	struct mt3620_mailbox_data mailbox_data;
	struct mt3620_mailbox_channel *channel =
		container_of(work, struct mt3620_mailbox_channel, user_work);
	struct mbox_chan *chan = channel->mbox_chan + 1;
	uint32_t user_int;

	// One reader at a time
	spin_lock(&channel->lock);

	user_int = readl_relaxed(channel->channel_base +
				 MBOX_SW_RX_INT_STS_OFFSET);

	// clear interrupts
	writel_relaxed(user_int,
		       channel->channel_base + MBOX_SW_RX_INT_STS_OFFSET);

	// re-enable interrupts
	writel_relaxed(0xFF, channel->channel_base + MBOX_SW_RX_INT_EN_OFFSET);

	mailbox_data.cmd = user_int;
	mbox_chan_received_data(chan, &mailbox_data);

	spin_unlock(&channel->lock);
}

///
/// Bottom half IRQ interrupt handler for when we're writing data
///
/// @data - Interupt data - mbox_chan pointer
static void mt3620_mailbox_tx_interrupt_bottom_half(unsigned long data)
{
	struct mbox_chan *chan = (struct mbox_chan *)data;
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	if (!channel) {
		return;
	}

	// Signal done
	mbox_chan_txdone(chan, 0);
}

///
/// IRQ interrupt handler for when we're writing data
///
/// @irq - IRQ number
/// @p - Pointer to mbox_chan object
/// @returns - IRQ result status code
static irqreturn_t mt3620_mailbox_tx_interrupt(int irq, void *p)
{
	struct mbox_chan *chan = (struct mbox_chan *)p;
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	if (!channel) {
		return IRQ_HANDLED;
	}

	// Reset interrupt flag
	writel_relaxed(1 << MBOX_INT_TYPE_WR,
		       channel->channel_base + MBOX_INT_STS_OFFSET);

	// Mark as completed via bottom half
	tasklet_schedule(&channel->write_tasklet);

	return IRQ_HANDLED;
}

///
/// Callback when a client starts a new session with a channel
///
/// @chan - Mailbox channel
/// @returns - 0 for success
static int mt3620_mailbox_startup(struct mbox_chan *chan)
{
	int ret = 0;
	int channel_type;
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	if (!channel) {
		return -EINVAL;
	}

	channel_type = mbox_chan_to_mt3620_mailbox_channel_type(chan);

	dev_dbg(channel->dev, "Mailbox startup for channel %u/%u",
		channel->channelId, channel_type);

	if (channel_type == MBOX_MT3620_CHAN_TYPE_FIFO) {
		// reset interrupt status flags
		writel_relaxed(1 << MBOX_INT_TYPE_RD,
			       channel->channel_base + MBOX_INT_STS_OFFSET);
		writel_relaxed(1 << MBOX_INT_TYPE_WR,
			       channel->channel_base + MBOX_INT_STS_OFFSET);

		// Setup our interrupts
		ret = devm_request_irq(channel->dev, channel->read_irq,
				       mt3620_mailbox_rx_interrupt, 0,
				       DRIVER_NAME, chan);
		if (unlikely(ret)) {
			dev_err(channel->dev,
				"Failed to register RX mailbox interrupt: %d",
				ret);
			return ret;
		}

		ret = devm_request_irq(channel->dev, channel->write_irq,
				       mt3620_mailbox_tx_interrupt, 0,
				       DRIVER_NAME, chan);
		if (unlikely(ret)) {
			dev_err(channel->dev,
				"Failed to register TX mailbox interrupt: %d",
				ret);
			return ret;
		}
	} else if (channel_type == MBOX_MT3620_CHAN_TYPE_USERINT) {
		ret = devm_request_irq(channel->dev, channel->user_irq,
				       mt3620_mailbox_user_interrupt, 0,
				       DRIVER_NAME, chan);
		if (unlikely(ret)) {
			dev_err(channel->dev,
				"Failed to register user mailbox interrupt: %d",
				ret);
			return ret;
		}

		writel_relaxed(0xFF, channel->channel_base +
					     MBOX_SW_RX_INT_EN_OFFSET);
	}

	return ret;
}

///
/// Callback when a client stops a session with a channel
///
/// @chan - Mailbox channel
/// @returns - 0 for success
static void mt3620_mailbox_shutdown(struct mbox_chan *chan)
{
	int channel_type;
	struct mt3620_mailbox_channel *channel =
		mbox_chan_to_mt3620_mailbox_channel(chan);

	if (!channel) {
		return;
	}

	channel_type = mbox_chan_to_mt3620_mailbox_channel_type(chan);

	dev_dbg(channel->dev, "Mailbox shutdown for channel %u/%d",
		channel->channelId, channel_type);

	if (channel_type == MBOX_MT3620_CHAN_TYPE_FIFO) {
		// Free IRQs
		devm_free_irq(channel->dev, channel->read_irq, chan);
		devm_free_irq(channel->dev, channel->write_irq, chan);

		// reset interrupt status flags
		writel_relaxed(1 << MBOX_INT_TYPE_RD,
			       channel->channel_base + MBOX_INT_STS_OFFSET);
		writel_relaxed(1 << MBOX_INT_TYPE_WR,
			       channel->channel_base + MBOX_INT_STS_OFFSET);
	} else if (channel_type == MBOX_MT3620_CHAN_TYPE_USERINT) {
		devm_free_irq(channel->dev, channel->user_irq, chan);
		writel_relaxed(0, channel->channel_base +
					  MBOX_SW_RX_INT_EN_OFFSET);
	}
}

static struct mbox_chan_ops mt3620_mailbox_ops = {
	.send_data = mt3620_mailbox_send_data,
	.startup = mt3620_mailbox_startup,
	.shutdown = mt3620_mailbox_shutdown,
	.last_tx_done = mt3620_mailbox_last_tx_done,
	.peek_data = mt3620_mailbox_peek_data,
};

///
/// Initializes our mailbox driver
///
/// @pdev - Platform device pointer for this module
/// @returns - 0 for success
static int mt3620_mailbox_probe(struct platform_device *pdev)
{
	struct mt3620_mailbox *mailbox;
	struct mt3620_mailbox_channel *channel;
	struct device_node *node = pdev->dev.of_node;
	struct resource *regs;
	struct mbox_chan *chans;
	u32 max_fifo_count;
	int ret = 0;
	unsigned int i = 0;

	// Allocate memory for our driver state
	mailbox = devm_kzalloc(&pdev->dev, sizeof(*mailbox), GFP_KERNEL);
	if (!mailbox)
		return -ENOMEM;

	mailbox->dev = &pdev->dev;

	dev_info(mailbox->dev, "Starting mt3620 mailbox driver");

	// Read out parameters
	if (of_property_read_u32(node, "mailbox-channel-count",
				 &mailbox->channel_count))
		return -ENODEV;

	dev_dbg(mailbox->dev, "Number of mailbox channels is %#x",
		mailbox->channel_count);

	if (of_property_read_u32(node, "mailbox-max-fifo-count",
				 &max_fifo_count))
		return -ENODEV;

	dev_dbg(mailbox->dev, "Max message count per FIFO is %#x",
		max_fifo_count);

	// Allocate memory for each channel
	// Allocate 2 "chan" for each physical mailbox channel
	// (FIFO and user interrupt)
	chans = devm_kzalloc(mailbox->dev,
			     sizeof(*chans) * mailbox->channel_count *
				     MBOX_MT3620_CHAN_PER_MAILBOX,
			     GFP_KERNEL);
	if (!chans)
		return -ENOMEM;

	// Get our base address
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	mailbox->mailbox_base = devm_ioremap_resource(mailbox->dev, regs);
	if (IS_ERR(mailbox->mailbox_base)) {
		return PTR_ERR(mailbox->mailbox_base);
	}

	dev_dbg(mailbox->dev, "Mailbox base vmem offset is %#x",
		(u32)mailbox->mailbox_base);

	// Read version out
	mailbox->version =
		readl_relaxed(mailbox->mailbox_base + MAILBOX_REG_VERSION);
	if (mailbox->version != MBOX_VER_MT3620) {
		mailbox->version = readl_relaxed(mailbox->mailbox_base +
						 0x8000 + MAILBOX_REG_VERSION);
	}

	dev_dbg(mailbox->dev, "Mailbox is hardware version %#x",
		mailbox->version);

	// Build out our channel objects
	for (i = 0; i < mailbox->channel_count; i++) {
		channel =
			devm_kzalloc(&pdev->dev, sizeof(*channel), GFP_KERNEL);

		if (!channel)
			return -ENOMEM;

		channel->channel_base =
			mailbox->mailbox_base +
			mt3620_mailbox_channel_offset(mailbox, i);
		channel->channelId = i;
		channel->dev = mailbox->dev;
		channel->max_fifo_count = max_fifo_count;
		channel->mbox_chan = &chans[i * MBOX_MT3620_CHAN_PER_MAILBOX];
		channel->read_irq = platform_get_irq(pdev, (3 * i));
		channel->write_irq = platform_get_irq(pdev, (3 * i) + 1);
		channel->user_irq = platform_get_irq(pdev, (3 * i) + 2);
		channel->version = mailbox->version;
		INIT_WORK(&channel->read_work,
			  mt3620_mailbox_rx_interrupt_bottom_half);
		tasklet_init(
			&channel->write_tasklet,
			mt3620_mailbox_tx_interrupt_bottom_half,
			(unsigned long)&chans[i * MBOX_MT3620_CHAN_PER_MAILBOX]);
		INIT_WORK(&channel->user_work,
			  mt3620_mailbox_user_interrupt_bottom_half);
		spin_lock_init(&channel->lock);

		// For later reference
		chans[i * MBOX_MT3620_CHAN_PER_MAILBOX +
		      MBOX_MT3620_CHAN_TYPE_FIFO]
			.con_priv = (void *)(((uintptr_t)channel) |
					     MBOX_MT3620_CHAN_TYPE_FIFO);
		chans[i * MBOX_MT3620_CHAN_PER_MAILBOX +
		      MBOX_MT3620_CHAN_TYPE_USERINT]
			.con_priv = (void *)(((uintptr_t)channel) |
					     MBOX_MT3620_CHAN_TYPE_USERINT);
	}

	mailbox->controller.dev = mailbox->dev;
	mailbox->controller.num_chans =
		mailbox->channel_count * MBOX_MT3620_CHAN_PER_MAILBOX;
	mailbox->controller.chans = chans;
	mailbox->controller.ops = &mt3620_mailbox_ops;

	// We use interrupt based completion indicators for the FIFO channels,
	// but not for the USERINT channels. Unfortunately the mailbox subsystem
	// doesn't support txdone method selection per channel, only per controller.
	// We work around by first requesting TXDONE_BY_POLL (which will initialize
	// the hrtimer for polling), but bait-and-switch back by setting the per-
	// channel method to IRQ for the FIFO channels.
	mailbox->controller.txdone_poll = true;

	ret = mbox_controller_register(&mailbox->controller);
	if (ret) {
		dev_err(&pdev->dev, "Register mailbox failed\n");
		goto err;
	}

	// Set main FIFO channels to TX done via IRQ
	for (i = 0; i < mailbox->channel_count; i++) {
		chans[i * MBOX_MT3620_CHAN_PER_MAILBOX +
		      MBOX_MT3620_CHAN_TYPE_FIFO]
			.txdone_method = TXDONE_BY_IRQ;
	}

	dev_dbg(mailbox->dev, "Mailbox setup completed");

	platform_set_drvdata(pdev, mailbox);
err:
	return ret;
}

///
/// Teardown our mailbox driver
///
/// @pdev - Platform device pointer for this module
/// @returns - 0 for success
static int mt3620_mailbox_remove(struct platform_device *pdev)
{
	struct mt3620_mailbox *mailbox = platform_get_drvdata(pdev);

	if (!mailbox) {
		return -EINVAL;
	}

	mbox_controller_unregister(&mailbox->controller);

	return 0;
}

static const struct of_device_id mt3620_mailbox_match[] = {
	{ .compatible = "mediatek,mt3620-mailbox" },
	{ /* Sentinel */ },
};

MODULE_DEVICE_TABLE(of, mt3620_mailbox_match);

static struct platform_driver mt3620_mailbox_driver = {
	.probe = mt3620_mailbox_probe,
	.remove = mt3620_mailbox_remove,
	.driver =
		{
			.name = DRIVER_NAME,
			.of_match_table = mt3620_mailbox_match,
		},
};

module_platform_driver(mt3620_mailbox_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("MT3620 mailbox controller specific functions");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_ALIAS("platform:mt3620-mailbox");
