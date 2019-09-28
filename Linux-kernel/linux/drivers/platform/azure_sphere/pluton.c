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

#ifdef CONFIG_AZURE_SPHERE_SECURITY_MONITOR_DEBUG
// Set DEBUG to 1 to enable debug log output
#define DEBUG 1
#endif

#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <uapi/linux/azure-sphere/pluton.h>
#include <azure-sphere/pluton_remoteapi.h>

#include "caller_security.h"
#include "security_state.h"
#include "sm_user.h"
#include "attestation_runtime_operations.h"
#include "boot_mode.h"
#include "pluton.h"

static DEFINE_MUTEX(pending_async_ops_mutex);
static LIST_HEAD(pending_async_ops);

int pluton_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct pluton_file_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	mutex_init(&data->mutex);
	init_waitqueue_head(&data->waitqueue);

	filp->private_data = data;

	nonseekable_open(inode, filp);

out:
	return ret;
}

int pluton_release(struct inode *inode, struct file *filp)
{
	struct pluton_file_data *list_item;
	struct pluton_file_data *data;
	bool valid = false;

	data = filp->private_data;

	mutex_lock(&pending_async_ops_mutex);

	list_for_each_entry(list_item, &pending_async_ops, list)
	{
		if (list_item == data) {
			valid = true;
			break;
		}
	}

	if (valid) {
		list_del(&data->list);
	}

	if (data->data) {
		// Free unread message
		kfree(data->data);
	}

	if (data) {
		kfree(data);
	}

	mutex_unlock(&pending_async_ops_mutex);

	return 0;
}

unsigned int pluton_poll(struct file *filp, poll_table *wait)
{
	struct pluton_file_data *data;
	unsigned int ret;

	data = filp->private_data;
	poll_wait(filp, &data->waitqueue, wait);

	if (mutex_lock_interruptible(&data->mutex))
		return -ERESTARTSYS;

	ret = 0;
	if (data->data != NULL)
		ret |= POLLIN;

	mutex_unlock(&data->mutex);

	return ret;
}

ssize_t pluton_read(struct file *filp, char __user *ubuf,
                    size_t count, loff_t *offp)
{
	struct pluton_file_data *data;
	ssize_t ret = 0;

	data = filp->private_data;

	if (mutex_lock_interruptible(&data->mutex))
		return -ERESTARTSYS;

	if (data->data == NULL || data->data_ready == false) {
		// Nothing to read
		goto out;
	}

	count = min(count, data->data_len);

	ret = copy_to_user(ubuf, data->data, count);

	kfree(data->data);
	data->data = NULL;
	data->data_len = 0;
	data->data_ready = false;

out:
	mutex_unlock(&data->mutex);
	return ret;
}


long pluton_ioctl(struct file *filp, unsigned int cmd, unsigned long arg_)
{
	void __user *arg = (void __user *)arg_;
	struct pluton_file_data *data;
	bool async = false;

	data = filp->private_data;
	async = filp->f_flags & O_NONBLOCK;

	switch (cmd) {
	case PLUTON_GET_SECURITY_STATE:
		return pluton_get_security_state(arg, data, async);
	case PLUTON_GENERATE_CLIENT_AUTH_KEY:
		return pluton_generate_client_auth_key(arg, data, async);
	case PLUTON_COMMIT_CLIENT_AUTH_KEY:
		return pluton_commit_client_auth_key(arg, data, async);
	case PLUTON_GET_TENANT_PUBLIC_KEY:
		return pluton_get_tenant_public_key(arg, data, async);
	case PLUTON_PROCESS_ATTESTATION:
		return pluton_process_attestation(arg, data, async);
	case PLUTON_SIGN_WITH_TENANT_ATTESTATION_KEY:
		return pluton_sign_with_tenant_attestation_key(arg, data, async);
	case PLUTON_SET_POSTCODE:
		return pluton_set_postcode(arg);
	case PLUTON_GET_BOOT_MODE_FLAGS:
		return pluton_get_boot_mode_flags(arg, data, async);
	case PLUTON_IS_CAPABILITY_ENABLED:
		return pluton_is_capability_enabled(arg, data, async);
	case PLUTON_GET_ENABLED_CAPABILITIES:
		return pluton_get_enabled_capabilities(arg, data, async);
	case PLUTON_SET_MANUFACTURING_STATE:
		return pluton_set_manufacturing_state(arg, data, async);
	case PLUTON_GET_MANUFACTURING_STATE:
		return pluton_get_manufacturing_state(arg, data, async);
	default:
		return -EINVAL;
	}
}

static void pluton_async_callback(void *data, size_t data_len, void *callback_data)
{
    struct pluton_file_data *list_item = NULL;
	struct pluton_file_data *file_data = callback_data;
	int count = 0;
    bool valid = false;

	mutex_lock(&pending_async_ops_mutex);

	list_for_each_entry(list_item, &pending_async_ops, list)
	{
		if (list_item == file_data) {
			valid = true;
			break;
		}
	}

	if (!valid) {
		goto exit;
	}

	mutex_lock(&file_data->mutex);

	if (file_data->data) {
		// Copy it
		count = min(data_len, file_data->data_len - file_data->data_offset);
		memcpy(file_data->data + file_data->data_offset, data, count);

		file_data->data_ready = true;

		wake_up_interruptible(&file_data->waitqueue);
	}

	list_del(&file_data->list);

	mutex_unlock(&file_data->mutex);

exit:
    mutex_unlock(&pending_async_ops_mutex);
}

int pluton_send_mailbox_message(enum pluton_remoteapi_commands cmd,
                                void *data, u32 data_size,
                                void *buffer, u32 buffer_size,
                                size_t data_offset, size_t offset_size, 
                                struct pluton_file_data *file_data, bool async)
{
	int ret = 0;

	if (async) {
		if (mutex_lock_interruptible(&pending_async_ops_mutex))
			return -ERESTARTSYS;

		if (mutex_lock_interruptible(&file_data->mutex)) {
			ret = -ERESTARTSYS;
			goto unlock;
		}

		if (file_data->data) {
			ret = -EAGAIN;
		} else {
			// malloc space for the message
			file_data->data = kmalloc(buffer_size, GFP_KERNEL);
			if (file_data->data == NULL) {
				ret = -ENOMEM;
			} else {
				file_data->data_len = buffer_size;
				file_data->data_offset = data_offset;
				memcpy(file_data->data, buffer, buffer_size);

				// Send async op
				ret = pluton_remote_api_send_command_to_m4_async(
					cmd, data, data_size, 
					pluton_async_callback, file_data);

				if (ret != 0) {
					// This failed
					kfree(file_data->data);
					file_data->data = NULL;
					file_data->data_len = 0;
					file_data->data_offset = 0;
				} else {
					// Indicate to caller we've started the op
					list_add(&file_data->list, &pending_async_ops);
					ret = -EINPROGRESS;
				}
			}
		}

		mutex_unlock(&file_data->mutex);

unlock:
		mutex_unlock(&pending_async_ops_mutex);
	} else {
		ret = pluton_remote_api_send_command_to_m4_sync(
			cmd, data, data_size, 
			buffer + data_offset, offset_size);
	}

	return ret;
}
