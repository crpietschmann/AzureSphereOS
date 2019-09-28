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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/file.h>
#include <linux/ctype.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <azure-sphere/security_monitor.h>
#include <uapi/linux/azure-sphere/security_monitor.h>
#include <azure-sphere/security.h>
#include <linux/skbuff.h>
#include <azure-sphere/pluton_remoteapi.h>

#include "log.h"
#include "caller_security.h"
#include "sm_user.h"

///
/// SECURITY_MONITOR_SMAPI message handler
///
/// @arg - input data from IOCTL
/// @returns - 0 for success
int security_monitor_smapi_command(void __user *arg)
{
	u32 ret = 0;
	struct azure_sphere_smapi_command_request command_data;

	ret = copy_from_user(&command_data, arg, sizeof(struct azure_sphere_smapi_command_request));
	if (unlikely(ret)) {
		goto exit;
	}

	if (!azure_sphere_caller_has_capabilities_for_smapi_cmd(command_data.command)) {
		dev_err(g_sm_user->dev, "%s sender not authorized to execute requested command (cmd: %i)\n",
			__FUNCTION__,
			command_data.command);
		ret = -EACCES;
		goto exit;
	}

	ret = azure_sphere_sm_generic_command_from_user(
		command_data.command, command_data.input_params, command_data.input_params_length,
		command_data.output_params, command_data.output_params_length);
	if (ret != 0) {
		dev_err(g_sm_user->dev, "%s failed to execute command (cmd: %i)\n",
			__FUNCTION__,
			command_data.command);
	}

exit:
	return ret;
}

int security_monitor_get_peripheral_count(void __user* arg){
	u32 ret = 0;
	int result = 0;
	struct azure_sphere_peripheral_count command_data;
	command_data.count = 0;
	
	ret = copy_from_user(&command_data, arg, sizeof(command_data));
	if (unlikely(ret)) {
		command_data.status_code = EINVAL;
		goto exit;
	}

	result = azure_sphere_sm_get_peripheral_count(command_data.peripheral_type);

	//
	// In the case of an error the value will be negative, otherwise the return
	// value is the count of peripherals
	//
	if (result < 0) {
		command_data.count = 0;
		command_data.status_code = -result;
	} else {
		command_data.count = result;
		command_data.status_code = 0;
	}

exit:
	ret = copy_to_user(arg, &command_data, sizeof(command_data));
	memzero_explicit(&command_data, sizeof(command_data));
	return ret;
}

int security_monitor_list_peripherals(void __user* arg) {
	u32 ret = 0;
	struct azure_sphere_list_peripherals command_data;
	
	ret = copy_from_user(&command_data, arg, sizeof(command_data));
	if (unlikely(ret)) {
		goto exit;
	}

	ret = azure_sphere_sm_list_peripherals(command_data.peripheral_type,
		command_data.info, command_data.buffer_size);
	if (ret != 0) {
		dev_err(g_sm_user->dev, "%s failed to execute command (cmd: LIST_ALL_UARTS)\n",
			__FUNCTION__);
	}

exit:
	ret = copy_to_user(arg, &command_data, sizeof(command_data));
	memzero_explicit(&command_data, sizeof(command_data));
	return ret;
}

///
/// UUID parser code; this is simplified and doesn't correct the byte order.
///
/// @arg - uuid target
/// @arg - uuid ascii source
/// @returns - 0 for success, otherwise error
static int azure_sphere_uuid_parse(u8 *uuid, const char *buf)
{
	const char *str = buf;
	int i;

	for (i = 0; i < 16; i++) {
		if (!isxdigit(str[0]) || !isxdigit(str[1])) {
			return -EINVAL;
		}

		uuid[i] = (hex_to_bin(str[0]) << 4) | hex_to_bin(str[1]);
		str += 2;
		if (*str == '-')
			str++;
	}

	return 0;
}

///
/// SECURITY_MONITOR_DERIVE_KEY message handler
///
/// @arg - input data
/// @returns - 0 for success

#define PREFIX "/mnt/apps/"
#define PREFIX_LEN 10

int security_monitor_derive_key_command(void __user *arg)
{
	u32 ret = 0;
	struct azure_sphere_derive_key command_data;
	u8 component_uid[16] = {0};
	struct file *exe_file;
	char path_buf[96];
	char *path;

	ret = copy_from_user(&command_data, arg, sizeof(struct azure_sphere_derive_key));
	if (unlikely(ret)) {
		return ret;
	}

	command_data.result.status_code = -EIO;

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_SFS)) {
		dev_err(g_sm_user->dev, "%s sender not allowd to access SFS API\n", __FUNCTION__);
		goto exit;
	}

	// TODO(39495): obtain caller's identity, depending on command_data.request.key_type
	// For now, use executable path.

	// obtain executable file corresponding to current (calling) task
	exe_file = get_task_exe_file(current);

	if (!exe_file) {
		command_data.result.status_code = -ENOENT;
		dev_err(g_sm_user->dev, "%s: get_task_exe_file failed\n", __FUNCTION__);
		goto exit;
	}

	// obtain ascii path name for executable file.
	path = d_path(&exe_file->f_path, path_buf, sizeof(path_buf));
	fput(exe_file);

	if (IS_ERR(path)) {
		dev_err(g_sm_user->dev, "%s: d_path failed - %ld\n", __FUNCTION__, PTR_ERR(path));
		command_data.result.status_code = -ENOENT;
		goto exit;
	}

	// if this is a third-party app, it must be in a directory
	// under /mnt/apps/<component_id>/; extract component_id.
	if (memcmp(path, PREFIX, PREFIX_LEN) != 0) {
		dev_err(g_sm_user->dev, "%s: not a 3rd party app\n", __FUNCTION__);
		// otherwise, it is a non-supported application.
		command_data.result.status_code = -EINVAL;
		goto exit;
	}

	// strip prefix
	path += PREFIX_LEN;

	// parse uid
	if (azure_sphere_uuid_parse(component_uid, path) != 0) {
		dev_err(g_sm_user->dev, "%s: parsing uid failed\n", __FUNCTION__);
		command_data.result.status_code = -EINVAL;
		goto exit;
	}

	command_data.result.status_code = azure_sphere_sm_derive_key(component_uid, command_data.request.generation_delta,
		&command_data.result.key, &command_data.result.instance, &command_data.result.generation);

exit:
	ret = copy_to_user(arg, &command_data, sizeof(command_data));
	memzero_explicit(&command_data, sizeof(command_data));

	return ret;
}

long security_monitor_ioctl(struct file *filp, unsigned int cmd, unsigned long arg_)
{
	void __user *arg = (void __user *)arg_;

	switch (cmd) {
	case SECURITY_MONITOR_WRITE_LOG:
		return security_monitor_write_log(arg);
	case SECURITY_MONITOR_GET_LOG_DATA:
		return security_monitor_get_log_data(arg);
	case SECURITY_MONITOR_SMAPI_COMMAND:
		return security_monitor_smapi_command(arg);
	case SECURITY_MONITOR_DERIVE_KEY:
		return security_monitor_derive_key_command(arg);
	case SECURITY_MONITOR_GET_PERIPHERAL_COUNT:
		return security_monitor_get_peripheral_count(arg);
	case SECURITY_MONITOR_LIST_PERIPHERALS:
		return security_monitor_list_peripherals(arg);
	default:
		return -EINVAL;
	}
}