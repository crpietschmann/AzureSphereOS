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
#include <linux/fdtable.h>
#include <azure-sphere/pluton_remoteapi.h>
#include <linux/of.h>

#include "log.h"
#include "caller_security.h"
#include "sm_user.h"
#include "io_core_comm.h"

///
/// SECURITY_MONITOR_SMAPI message handler
///
/// @arg - input data from IOCTL
/// @returns - 0 for success
int security_monitor_smapi_command(void __user *arg)
{
	u32 ret = 0;
	struct azure_sphere_smapi_command_request command_data;

	ret = copy_from_user(&command_data, arg, sizeof(command_data));
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
		command_data.info, command_data.buffer_size, command_data.entry_size);
	if (ret != 0) {
		dev_err(g_sm_user->dev, "%s failed to execute command (cmd: LIST_ALL_PERIPHERALS)\n",
			__FUNCTION__);
	}

exit:
	ret = copy_to_user(arg, &command_data, sizeof(command_data));
	memzero_explicit(&command_data, sizeof(command_data));
	return ret;
}

static struct device_node *find_peripheral_device_node(u16 peripheral_type, u16 peripheral_index)
{
	int ret;
	const char *type_str;
	char alias[16];
	struct device_node *device_node;

	switch (peripheral_type) {
	case AZURE_SPHERE_UART:
		type_str = "serial";
		break;
	case AZURE_SPHERE_I2C:
		type_str = "i2c";
		break;
	case AZURE_SPHERE_SPI_MASTER:
		type_str = "spi";
		break;
	case AZURE_SPHERE_PWM:
		type_str = "pwm";
		break;
	case AZURE_SPHERE_ADC:
		type_str = "adc";
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	ret = snprintf(alias, sizeof(alias), "%s%u", type_str, (u32)peripheral_index);
	if (ret <= 0 || ret >= sizeof(alias))
		return ERR_PTR(-ENOMEM);

	device_node = of_find_node_opts_by_path(alias, NULL);
	if (!device_node) {
		dev_err(g_sm_user->dev, "Could not find node in device tree: %s\n", alias);
		return ERR_PTR(-ENOENT);
	}

	return device_node;
}

static int security_monitor_enable_peripheral_driver(void __user* arg)
{
	int ret = 0;
	struct azure_sphere_enable_peripheral_driver command_data;
	struct device_node *node = NULL;

	ret = copy_from_user(&command_data, arg, sizeof(command_data));
	if (unlikely(ret))
		return ret;

	dev_info(g_sm_user->dev, "%s driver for peripheral 0x%x:0x%x\n",
		command_data.enable ? "Probing" : "Removing",
		(u32)command_data.peripheral_type,
		(u32)command_data.peripheral_index);

	node = find_peripheral_device_node(command_data.peripheral_type,
		command_data.peripheral_index);
	if (IS_ERR(node)) {
		// Device doesn't need to be disabled if it doesn't exist
		if (command_data.enable) {
			dev_err(g_sm_user->dev, "failed to find peripheral node: 0x%x ret=%ld\n",
					command_data.peripheral_type, PTR_ERR(node));
			return PTR_ERR(node);
		} else {
			dev_info(g_sm_user->dev, "peripheral to disable not found, continuing without error: 0x%x\n",
					command_data.peripheral_type);
			return 0;
		}
	}

	dev_info(g_sm_user->dev, "Found device node: %s\n",
		node->full_name ? node->full_name : node->name);

	ret = of_device_set_available(node, command_data.enable);
	if (ret) {
		dev_err(g_sm_user->dev, "failed to update OF device enablement for peripheral 0x%x:0x%x; ret=%d\n",
			(u32)command_data.peripheral_type, (u32)command_data.peripheral_index, ret);
		return ret;
	}

	return 0;
}

///
/// SECURITY_MONITOR_DERIVE_KEY message handler
///
/// @arg - input data
/// @returns - 0 for success

int security_monitor_derive_key_command(void __user *arg)
{
	u32 ret = 0;
	struct azure_sphere_derive_key command_data;
    struct azure_sphere_guid component_id;

	ret = copy_from_user(&command_data, arg, sizeof(command_data));
	if (unlikely(ret)) {
		return ret;
	}

	command_data.result.status_code = -EIO;

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_SFS)) {
		dev_err(g_sm_user->dev, "%s sender not allowed to access SFS API\n", __FUNCTION__);
		goto exit;
	}

    //if no security entry then fail, we shouldn't get here but just incase
    if (!azure_sphere_get_component_id(&component_id, current)) {
		dev_err(g_sm_user->dev, "%s error accessing security credentials\n", __FUNCTION__);
        goto exit;
    }

	command_data.result.status_code = azure_sphere_sm_derive_key(&component_id, command_data.request.generation_delta,
		&command_data.result.key, &command_data.result.instance, &command_data.result.generation);

exit:
	ret = copy_to_user(arg, &command_data, sizeof(command_data));
	memzero_explicit(&command_data, sizeof(command_data));

	return ret;
}

static int get_physical_address_of_file_mapping(unsigned int fd, uintptr_t *value, loff_t *size)
{
	int result = 0;
	struct file *file;
	struct address_space *mapping;
	struct inode *inode;

	file = fget(fd);

	if (file) {
		mapping = file->f_mapping;
		if (!mapping->a_ops->bmap)
			result = -EINVAL;
		else {
			inode = mapping->host;

			inode_lock(inode);
			if (inode->i_state & I_DIRTY)
				result = -ETXTBSY;
			inode_unlock(inode);

			if (result == 0) {
				*value = mapping->a_ops->bmap(mapping, 0);
				*size = i_size_read(inode);
			}
		}

		fput(file);
	} else {
		result = -ENOENT;
	}

	return result;
}

int security_monitor_io_core_control(void __user *arg)
{
	int result;
	uintptr_t physical_address;
	loff_t physical_size;
	struct azure_sphere_io_core_control command_data;

	result = copy_from_user(&command_data, arg, sizeof(command_data));
	if (unlikely(result)) {
		return result;
	}

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_PERIPHERAL_PIN_MAPPING)) {
		dev_err(g_sm_user->dev, "%s sender not allowd to use IO core control API\n", __FUNCTION__);
		result = -EPERM;
		goto exit;
	}

	if (result == 0) {
		switch (command_data.request) {
		case AZURE_SPHERE_IO_CORE_CONTROL_RUN:
#ifdef CONFIG_AZURE_SPHERE_IO_CORE_COMM
			iocore_communication_stop(command_data.core);
#endif
			result = get_physical_address_of_file_mapping(command_data.fd, &physical_address, &physical_size);
			if (result == 0)
			{
				result = azure_sphere_sm_io_core_start(command_data.core, physical_address,
				                                       physical_size, command_data.flags);
			}
			break;
		case AZURE_SPHERE_IO_CORE_CONTROL_STOP:
#ifdef CONFIG_AZURE_SPHERE_IO_CORE_COMM
			iocore_communication_stop(command_data.core);
#endif
			result = azure_sphere_sm_io_core_stop(command_data.core);
			break;
		case AZURE_SPHERE_IO_CORE_CONTROL_START_COMMUNICATION:
#ifdef CONFIG_AZURE_SPHERE_IO_CORE_COMM
			result = iocore_communication_start(command_data.core, command_data.component_id);
#else
			result = -ENOSYS;
#endif
			break;
		default:
			result = -EINVAL;
			break;
		}
	}
exit:

	return result;
}

int security_monitor_record_telemetry_event_data(void __user *arg)
{
	int result;
	struct azure_sphere_telemetry_record_event_data command_data;

	result = copy_from_user(&command_data, arg, sizeof(command_data));
	if (unlikely(result)) {
		return result;
	}

	if (result == 0) {
		result = azure_sphere_sm_record_telemetry_event_data(command_data.id,
				    									     command_data.event_timestamp,
												             command_data.payload_length,
													         command_data.payload);
	}
	return result;
}

int security_monitor_get_telemetry(void __user *arg)
{
	int result;
	struct azure_sphere_get_telemetry command_data;

	result = copy_from_user(&command_data, arg, sizeof(command_data));
	if (unlikely(result)) {
		return result;
	}

	if (result == 0) {
		result = azure_sphere_sm_get_telemetry(command_data.offset, command_data.buffer, command_data.buffer_size);
	}
	return result;
}

int security_monitor_reset_retain_telemetry(void __user *arg)
{
	int result;
	struct azure_sphere_reset_retain_telemetry command_data;

	result = copy_from_user(&command_data, arg, sizeof(command_data));
	if (unlikely(result)) {
		return result;
	}

	if (result == 0) {
		result = azure_sphere_sm_reset_retain_telemetry(command_data.retain);
	}
	return result;
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
	case SECURITY_MONITOR_ENABLE_PERIPHERAL_DRIVER:
		return security_monitor_enable_peripheral_driver(arg);
	case SECURITY_MONITOR_IO_CORE_CONTROL:
		return security_monitor_io_core_control(arg);
	case SECURITY_MONITOR_RECORD_TELEMETRY_EVENT_DATA:
		return security_monitor_record_telemetry_event_data(arg);
	case SECURITY_MONITOR_GET_TELEMETRY:
		return security_monitor_get_telemetry(arg);
	case SECURITY_MONITOR_RESET_RETAIN_TELEMETRY:
		return security_monitor_reset_retain_telemetry(arg);
	default:
		return -EINVAL;
	}
}
