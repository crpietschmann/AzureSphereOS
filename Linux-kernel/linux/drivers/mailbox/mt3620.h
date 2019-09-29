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

#pragma once

// Our basic device state structure
struct mt3620_mailbox {
	// Mailbox controller master structure
	struct mbox_controller controller;
	// Number of mailbox channels we're controlling
	u32 channel_count;
	// The underlying linux kernel device
	struct device *dev;
	// Base register address of the mailbox channel we're using
	void __iomem *mailbox_base;
	// Mailbox hardware version
	u32 version;
};

// Private info about a channel
struct mt3620_mailbox_channel {
	// Base memory address for the channel
	void __iomem *channel_base;
	// Numeric identifier for the channel
	u32 channelId;
	// The underlying linux kernel device
	struct device *dev;
	// Lock for reads / writes on this channel
	spinlock_t lock;
	// Maximum count of messages in a given channel
	u32 max_fifo_count;
	// Native mailbox channel object
	struct mbox_chan *mbox_chan;
	// IRQ that triggers when there's data for us to read
	int read_irq;
	// Bottom half of rd IRQ handler
	struct work_struct read_work;
	// IRQ that triggers when our data has been trasmitted
	int write_irq;
	// Bottom half of wr IRQ handler
	struct tasklet_struct write_tasklet;
	// IRQ for user interrupts
	int user_irq;
	// Bottom half of user IRQ handler
	struct work_struct user_work;
	// Hardware version number
	u32 version;
};

#define MBOX_VER_MT3620 (0x36200000)

// Channel interrupt enable and status registers
#define MXBO_SW_TX_INT_PORT_OFFSET (0x14)
#define MBOX_SW_RX_INT_EN_OFFSET (0x18)
#define MBOX_SW_RX_INT_STS_OFFSET (0x1C)
#define MBOX_INT_EN_OFFSET (0x38)
#define MBOX_INT_STS_OFFSET (0x3C)

// FIFO write registers
#define MBOX_CMD_POST_OFFSET (0x40)
#define MBOX_DATA_POST_OFFSET (0x44)

// FIFO read registers
#define MBOX_CMD_POP_OFFSET (0x50)
#define MBOX_DATA_POP_OFFSET (0x54)

// FIFO count
#define MBOX_FIFO_WRITE_COUNT_OFFSET (0x48)
#define MBOX_FIFO_READ_COUNT_OFFSET (0x58)

// General hardware version offset from last channel
#define MAILBOX_REG_VERSION 0x0

// List of interrupt types supported by the mailbox
enum mt3620_mailbox_interrupt_types {
	MBOX_INT_TYPE_WR,
	MBOX_INT_TYPE_RD,
	MBOX_INT_TYPE_NE,
	MBOX_INT_TYPE_NF,
};

// Channel type (stored in LSB of pointer)
#define MBOX_MT3620_CHAN_TYPE_MASK 1
#define MBOX_MT3620_CHAN_TYPE_FIFO 0
#define MBOX_MT3620_CHAN_TYPE_USERINT 1

// Channels per hardware mailbox
#define MBOX_MT3620_CHAN_PER_MAILBOX 2
