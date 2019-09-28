// SPDX-License-Identifier: GPL-2.0
/*
 * Pluton Remote API
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

#define PLUTON_SET_MANUFACTURING_STATE_MAGICNUM 0x5F04C5AB

// Command IDs sent over the mailbox
enum pluton_remoteapi_commands {
	// Unknown / invalid code
	INVALID = 0x0,
	// Send a set of buffers for unsolicited event messages
	SEND_EVENT_BUFFERS = 0x1,
	// Set POST code
	SET_POSTCODE = 0x02,
	// Get boot mode flags
	GET_BOOT_MODE_FLAGS = 0x03,
	// Read from the H4 TRNG
	READ_RNG = 0x40,
	// Get security state of the device
	GET_SECURITY_STATE = 0x41,
	// 0x46 - 0x48 already in use by Pluton
	/// Check if capability is enabled
	IS_CAPABILITY_ENABLED = 0x48,
	// Get enabled capabilities
	GET_ENABLED_CAPABILITIES = 0x49,
	// Generate the public/private ECC key pair
	GENERATE_CLIENT_AUTH_KEY = 0x4A,
	// Get the public key from Pluton
	GET_TENANT_PUBLIC_KEY = 0x4B,
	// process an attestation request and return results
	PROCESS_ATTESTATION = 0x4C,
	// sign digest with service attestation runtime key
	SIGN_WITH_TENANT_ATTESTATION_KEY = 0x4D,
	// commit client auth key
	COMMIT_CLIENT_AUTH_KEY = 0x4E,
	// Performs a full chip reset
	DEVICE_RESET = 0x50,
	// Get manufacturing completion state
	GET_MANUFACTURING_STATE = 0X51,
	// Set manufacturing completion state to completed
	SET_MANUFACTURING_STATE = 0X52,
};

enum pluton_device_capabilities {
	// No capabilities
	PLUTON_DEVICE_CAPABILITY_NONE = 0x0,
	// Capabilities 0x1 - 0xb are reserved
	PLUTON_DEVICE_CAPABILITY_ENABLE_RF_TEST_MODE = 0xC,
};

// Struct sent to pluton
struct pluton_remoteapi_command_info {
	// Version of the pluton protocol
	u8 version;
	// Sequence number of the request
	u8 sequence_number;
	// Command ID of the request
	u8 command;
	// The buffer index for the command data
	u8 buffer_index;
};

// Struct for setting manufacturing state
struct pluton_remoteapi_set_manufacturing_state_info {
	u32 magic_number;
	// Manufacturing completion state
	u32 completion_state;
};

// Struct for receiving manufacturing state data
struct pluton_remoteapi_get_manufacturing_state_info {
	u32 status_code;
	// Manufacturing completion state
	u32 completion_state;
};

///
/// Callback method for a remote API command.
/// This is provided by caller to be executed when the response comes in.
///
/// @data - Pointer to response data
/// @data_len - Pointer to data length
/// @callback_data - Pointer to optional callback data
typedef void (*pluton_remote_api_callback)(void *data, size_t data_len, void *callback_data);

///
/// Checks if the RemoteAPI has been initialized and is ready for use
///
/// @returns - 0 if ready, -EPROBE_DEFER or a simlar error if not
int pluton_remote_api_is_ready(void);

///
/// Sends a packet of data to the M4
///
/// @cmd - M4 command ID
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @callback - Optional callback to invoke when a response is received
/// @callback_data - Optional callback data object
/// @returns - 0 for success
int pluton_remote_api_send_command_to_m4_async(
    enum pluton_remoteapi_commands cmd, void *data, u32 data_size,
    pluton_remote_api_callback callback, void *callback_data);

///
/// Sends a packet of data to the M4 and blocks on a response
///
/// @cmd - M4 command ID
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @buffer - Buffer to hold response data
/// @buffer_size - Size of buffer
/// @returns - 0 for success
int pluton_remote_api_send_command_to_m4_sync(enum pluton_remoteapi_commands cmd,
					      void *data, u32 data_size,
					      void *buffer, u32 buffer_size);

///
/// Sends a packet of data to the M4 and doesn't wait for a response
///
/// @cmd - M4 command ID
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @returns - 0 for success
int pluton_remote_api_send_command_to_m4_no_response(enum pluton_remoteapi_commands cmd,
						     void *data, u32 data_size);
