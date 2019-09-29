// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Security Monitor log driver
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cred.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <uapi/linux/azure-sphere/security_monitor.h>
#include <azure-sphere/security_monitor.h>

#include "log.h"
#include "sm_user.h"

///
/// SECURITY_MONITOR_WRITE_LOG message handler
///
/// @arg - log data
/// @returns - 0 for success
int security_monitor_write_log(void __user *arg)
{
	u32 ret = 0;
	struct azure_sphere_write_log_data log_data;

	if (!capable(CAP_SYS_ADMIN)) {
		dev_err(g_sm_user->dev, "%s Access denied\n", __FUNCTION__);
		ret = -EACCES;
		goto exit;
	}

	ret = copy_from_user(&log_data, arg, sizeof(struct azure_sphere_write_log_data));
	if (unlikely(ret)) {
		goto exit;
	}

	if (!log_data.size) {
		dev_err(g_sm_user->dev, "%s Log data size should not be 0\n", __FUNCTION__);
		goto exit;
	}

	ret = azure_sphere_sm_write_log_from_user(log_data.data, log_data.size);

exit:
	return ret;
}

///
/// SECURITY_MONITOR_GET_LOG_DATA message handler
///
/// @arg - log data
/// @returns - 0 for success
int security_monitor_get_log_data(void __user *arg)
{
	u32 ret = 0;
	u32 log_buffer_size = 0;
	struct azure_sphere_get_log_data log_data;

	ret = copy_from_user(&log_data, arg, sizeof(struct azure_sphere_get_log_data));
	if (unlikely(ret)) {
		goto exit;
	}

	log_data.total_size = azure_sphere_sm_get_log_data_size(log_data.storage_type);

	if (log_data.buffer == 0) {
		// Just fill out the size
		ret = copy_to_user(arg, &log_data, sizeof(struct azure_sphere_get_log_data));
		goto exit;
	}

	if (log_data.buffer_size > 4096) {
		ret = -ENOMEM;
		goto exit;
	}

	if (log_data.offset >= log_data.total_size) {
		ret = -E2BIG;
		goto exit;
	}

	log_buffer_size = min(log_data.total_size - log_data.offset, log_data.buffer_size);

	ret = azure_sphere_sm_get_log_data(log_data.storage_type, log_data.offset, log_buffer_size, (void*)log_data.buffer);
	if (ret != 0) {
		dev_err(g_sm_user->dev, "%s azure_sphere_sm_get_log_data returned %u!!\n", __FUNCTION__, ret);
		goto exit;
	}

exit:
	return ret;
}
