// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere DAA operations driver
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

#include "pluton.h"

///
/// PLUTON_GENERATE_CLIENT_AUTH_KEY message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_generate_client_auth_key(void __user *arg, struct pluton_file_data *data, bool async);

///
/// PLUTON_COMMIT_CLIENT_AUTH_KEY message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_commit_client_auth_key(void __user *arg, struct pluton_file_data *data, bool async);

///
/// PLUTON_GET_TENANT_PUBLIC_KEY message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_tenant_public_key(void __user *arg, struct pluton_file_data *data, bool async);

///
/// PLUTON_PROCESS_ATTESTATION message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_process_attestation(void __user *arg, struct pluton_file_data *data, bool async);

///
/// PLUTON_SIGN_WITH_TENANT_ATTESTATION_KEY message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_sign_with_tenant_attestation_key(void __user *arg, struct pluton_file_data *data, bool async);