// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Security Monitor security policy
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

#include <uapi/linux/azure-sphere/security_monitor.h>

///
/// Determines if the user has the Azure Sphere capabilities required to execute the
/// given command
///
/// @cmd - the command
/// @returns - true iff the command's required capabilities are a subset of the 
///            user's available capabilities
bool azure_sphere_caller_has_capabilities_for_smapi_cmd(azure_sphere_smapi_command cmd);