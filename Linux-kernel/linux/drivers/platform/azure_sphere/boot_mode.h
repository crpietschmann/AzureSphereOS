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

#include "pluton.h"

///
/// PLUTON_SET_POSTCODE ioctl handler
///
/// @arg - ioctl buffer
/// @returns - 0 for success
int pluton_set_postcode(void __user *arg);

///
/// PLUTON_CMD_GET_BOOT_MODE_FLAGS ioctl handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_boot_mode_flags(void __user *arg, struct pluton_file_data *data, bool async);
