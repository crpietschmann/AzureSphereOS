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

#include <linux/skbuff.h>
#include <net/genetlink.h>
#include "pluton.h"

///
/// PLUTON_GET_SECURITY_STATE message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_security_state(void __user *arg, struct pluton_file_data *data, bool async);

///
/// PLUTON_IS_CAPABILITY_ENABLED message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_is_capability_enabled(void __user *arg, struct pluton_file_data *data, bool async);

///
/// PLUTON_GET_ENABLED_CAPABILITIES message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_enabled_capabilities(void __user *arg, struct pluton_file_data *data, bool async);

///
/// PLUTON_SET_MANUFACTURING_STATE message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_set_manufacturing_state(void __user *arg, struct pluton_file_data *data, bool async);

///
/// PLUTON_GET_MANUFACTURING_STATE message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_manufacturing_state(void __user *arg, struct pluton_file_data *data, bool async);
