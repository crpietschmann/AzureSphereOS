// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Security Monitor user interface driver
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
#include <linux/mutex.h>
#include <linux/poll.h>

// Data attached to a /dev/pluton fd
struct pluton_file_data {
    // Wait queue for poll
    wait_queue_head_t waitqueue;
    // Lock for async operations
    struct mutex mutex;
    // Pending data to read
    void *data;
    // Size of pending data
    size_t data_len;
    // Offset to copy data to
    size_t data_offset;
    // Is data ready for read
    bool data_ready;
    // List pointer
    struct list_head list;
};

unsigned int pluton_poll(struct file *filp, poll_table *wait);
ssize_t pluton_read(struct file *filp, char __user *ubuf,
			        size_t count, loff_t *offp);
int pluton_open(struct inode *inode, struct file *filp);
int pluton_release(struct inode *inode, struct file *filp);
long pluton_ioctl(struct file *filp, unsigned int cmd, unsigned long arg_);

int pluton_send_mailbox_message(enum pluton_remoteapi_commands cmd,
                                void *data, u32 data_size,
                                void *buffer, u32 buffer_size,
                                size_t data_offset, size_t offset_size, 
                                struct pluton_file_data *file_data, bool async);