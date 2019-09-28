// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere POST code/boot mode operations driver
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
#include <linux/skbuff.h>
#include <uapi/linux/azure-sphere/pluton.h>
#include <azure-sphere/pluton_remoteapi.h>
#include <uapi/linux/azure-sphere/security_monitor.h>
#include <azure-sphere/security.h>
#include "sm_user.h"
#include "caller_security.h"
#include "pluton.h"

///
/// PLUTON_SET_POSTCODE ioctl handler
///
/// @arg - ioctl buffer
/// @returns - 0 for success
int pluton_set_postcode(void __user *arg)
{
    u32 ret = 0;
    uint32_t postcode;

    if (!azure_sphere_capable(AZURE_SPHERE_CAP_POSTCODE)) {
            dev_err(g_sm_user->dev, "%s not allowed to set post code\n", __FUNCTION__);
            ret = -EPERM;
            goto exit;
    }

    ret = copy_from_user(&postcode, arg, sizeof(postcode));
    if (unlikely(ret)) {
        return ret;
    }

    ret = pluton_remote_api_send_command_to_m4_no_response(
        SET_POSTCODE, &postcode, sizeof(postcode));

exit:
    return ret;
}

///
/// PLUTON_CMD_GET_BOOT_MODE_FLAGS ioctl handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_boot_mode_flags(void __user *arg, struct pluton_file_data *data, bool async)
{
    u32 ret = 0;
    struct azure_sphere_boot_mode_flags boot_mode_flags;

    // no runtime permission check necessary.
    ret = pluton_send_mailbox_message(GET_BOOT_MODE_FLAGS, 
		NULL, 0, &boot_mode_flags, sizeof(boot_mode_flags), 0, sizeof(boot_mode_flags), data, async);

    // no data is sent back on err

    if (!ret) {
        ret = copy_to_user(arg, &boot_mode_flags, sizeof(boot_mode_flags));
    }

    return ret;
}
