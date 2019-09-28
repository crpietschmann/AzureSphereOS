// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Pluton Provider interface
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

// Successful return code
#define SUCCESS 0

// Encoded command info
union pluton_remoteapi_command_encoded {
	// Decoded info values
	struct pluton_remoteapi_command_info info;
	// Raw command value
	u32 as_u32;
};

// Tracking struct for a remoteapi data transfer provider
struct pluton_remoteapi_provider {
	// The underlying linux kernel device
	struct device *dev;

    // Send ring bufers over data transfer channel
    int (*send_ring_buffers)(dma_addr_t buffers_addr, union pluton_remoteapi_command_encoded cmd);
    // Send a message over the data transfer channel
    int (*send_message)(union pluton_remoteapi_command_encoded cmd);
};

///
/// Register a remoteapi data provider
/// @provider - provider instance
///
int pluton_remoteapi_register_provider(struct pluton_remoteapi_provider *provider);

///
/// Unregister a remoteapi data provider
/// @provider - provider instance
///
void pluton_remoteapi_unregister_provider(struct pluton_remoteapi_provider *provider);

///
/// Process incoming message from Pluton
///
/// @cmd - Command ID
void pluton_remoteapi_incoming_message(union pluton_remoteapi_command_encoded cmd);