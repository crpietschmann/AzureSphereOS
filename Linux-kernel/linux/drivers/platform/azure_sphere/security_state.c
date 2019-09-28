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
#include <azure-sphere/security.h>

#include "caller_security.h"
#include "sm_user.h"
#include "pluton.h"

///
/// PLUTON_GET_SECURITY_STATE message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_security_state(void __user *arg, struct pluton_file_data *data, bool async)
{
	u32 ret = 0;
	struct azure_sphere_get_security_state_result output_data;

	// Call mailbox message and get response
	ret = pluton_send_mailbox_message(GET_SECURITY_STATE, 
		NULL, 0, &output_data, sizeof(output_data), 0, sizeof(output_data), data, async);
	if (ret == -EINPROGRESS) {
		// Async started
		goto exit;
	} else if (ret != 0) {	
		dev_err(g_sm_user->dev, "Failed to get security state: %u!\n", ret);
		goto exit;
	}

	ret = copy_to_user(arg, &output_data, sizeof(output_data));

exit:
	return ret;
}

///
/// PLUTON_IS_CAPABILITY_ENABLED message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_is_capability_enabled(void __user *arg, struct pluton_file_data *data, bool async)
{
	u32 ret = 0;
	struct azure_sphere_is_capability_enabled cmd_data;

	ret = copy_from_user(&cmd_data, arg, sizeof(struct azure_sphere_is_capability_enabled));
	if (unlikely(ret)) {
		goto exit;
	}

	// Call mailbox message and get response
	ret = pluton_send_mailbox_message(
		IS_CAPABILITY_ENABLED, &cmd_data.capability, sizeof(uint16_t), 
		&cmd_data, sizeof(cmd_data),
		offsetof(struct azure_sphere_is_capability_enabled, o_is_enabled), sizeof(bool),
		data, async);

exit:
	if (!ret) {
		ret = copy_to_user(arg, &cmd_data, sizeof(struct azure_sphere_is_capability_enabled));
	}
	return ret;
}

///
/// PLUTON_GET_ENABLED_CAPABILITIES message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_enabled_capabilities(void __user *arg, struct pluton_file_data *data, bool async)
{
	u32 ret = 0;
	struct azure_sphere_get_enabled_capabilities output_data;

	// Call mailbox message and get response
	ret = pluton_send_mailbox_message(GET_ENABLED_CAPABILITIES, 
		NULL, 0, &output_data, sizeof(output_data), 0, sizeof(output_data), data, async);
	if (ret == -EINPROGRESS) {
		// Async started
		goto exit;
	} else if (ret != 0) {	
		dev_err(g_sm_user->dev, "Failed to get device enabled capabilities: %u!\n", ret);
		goto exit;
	}

	ret = copy_to_user(arg, &output_data, sizeof(output_data));

exit:
	return ret;
}

///
/// PLUTON_SET_MANUFACTURING_STATE message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_set_manufacturing_state(void __user *arg,
					 struct pluton_file_data *data,
					 bool async)
{
	u32 ret = 0;
	struct azure_sphere_manufacturing_state input_data;
	struct pluton_remoteapi_set_manufacturing_state_info cmd_data;
	uint32_t status_code = 0;

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_UPDATE_SECURITY_STATE)) {
		dev_err(
		    g_sm_user->dev,
		    "%s sender not authorized to update manufacturing state\n",
		    __FUNCTION__);
		ret = -EINVAL;
		goto exit;
	}

	ret = copy_from_user(&input_data, arg,
			     sizeof(struct azure_sphere_manufacturing_state));
	if (unlikely(ret)) {
		goto exit;
	}

	// Call mailbox and get response
	cmd_data.magic_number = PLUTON_SET_MANUFACTURING_STATE_MAGICNUM;
	cmd_data.completion_state = input_data.completion_state;

	ret = pluton_send_mailbox_message(
	    SET_MANUFACTURING_STATE, &cmd_data, sizeof(cmd_data), &status_code,
	    sizeof(status_code), 0, sizeof(status_code), data, async);

	if (ret != 0) {
		goto exit;
	}

	if (status_code != 0) {
		ret = status_code;
		goto exit;
	}

exit:
	if (ret != 0) {
		dev_err(g_sm_user->dev,
			"Failed to set manufacturing state to completed: %u!\n",
			ret);
	}
	return ret;
}

///
/// PLUTON_GET_MANUFACTURING_STATE message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_manufacturing_state(void __user *arg,
					 struct pluton_file_data *data,
					 bool async)
{
	u32 ret = 0;
	struct azure_sphere_manufacturing_state output_data;
	struct pluton_remoteapi_get_manufacturing_state_info cmd_data;

	if (!azure_sphere_capable(AZURE_SPHERE_CAP_UPDATE_SECURITY_STATE)) {
		dev_err(
		    g_sm_user->dev,
		    "%s sender not authorized to read manufacturing state\n",
		    __FUNCTION__);
		ret = -EINVAL;
		goto exit;
	}

	// Call mailbox and get response
	ret = pluton_send_mailbox_message(GET_MANUFACTURING_STATE, NULL, 0,
					  &cmd_data, sizeof(cmd_data), 0,
					  sizeof(cmd_data), data, async);

	if (ret != 0) {
		goto exit;
	}

	if (cmd_data.status_code != 0) {
		ret = cmd_data.status_code;
		goto exit;
	}
exit:
	if (ret == 0) {
		output_data.completion_state = cmd_data.completion_state;
		ret =
		    copy_to_user(arg, &output_data,
				 sizeof(struct azure_sphere_manufacturing_state));
	} else {
		dev_err(g_sm_user->dev,
			"Failed to get manufacturing state: %u!\n", ret);
	}
	return ret;
}
