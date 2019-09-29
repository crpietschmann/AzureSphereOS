// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 HIF API
 *
 * Copyright (c) 2018 MediaTek. All rights reserved.
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


// Commands that can be sent to the N9
enum n9_commands {
	// Untethered HQA (RF test) command
	WIFI_COMMAND_ID_UNTET_HQA_ACTION = 0x32,
	WIFI_COMMAND_ID_IOT_OWN_MAC_ADDRESS = 0xd0,
	WIFI_COMMAND_ID_IOT_SSID = 0xd1,
	WIFI_COMMAND_ID_IOT_BSSID = 0xd2,
	WIFI_COMMAND_ID_IOT_BANDWIDTH = 0xd3,
	WIFI_COMMAND_ID_IOT_SET_DISCONNECT = 0xd4,
	WIFI_COMMAND_ID_IOT_GET_LINK_STATUS = 0xd5,
	WIFI_COMMAND_ID_IOT_GET_PROFILE = 0xd6,
	WIFI_COMMAND_ID_IOT_START_SCAN = 0xd7,
	WIFI_COMMAND_ID_IOT_STOP_SCAN = 0xd8,
	WIFI_COMMAND_ID_IOT_GET_SCAN_LIST = 0xd9,
	WIFI_COMMAND_ID_IOT_RX_FILTER = 0xda,
	WIFI_COMMAND_ID_IOT_CHANNEL = 0xdb,
	WIFI_COMMAND_ID_IOT_80211_TO_HOST = 0xdc,
	// Sets STA or AP mode on the N9
	WIFI_COMMAND_ID_IOT_SWITCH_MODE = 0xdd,
	WIFI_COMMAND_ID_IOT_SET_KEY = 0xde,
	WIFI_COMMAND_ID_IOT_EXTENSION = 0xdf,
	WIFI_COMMAND_ID_EXTENSION_DEBUG_LEVEL = 0xe0,
	WIFI_COMMAND_ID_EXTENSION_SET_AUTH_MODE = 0xe1,
	WIFI_COMMAND_ID_EXTENSION_SET_ENCRYPT_TYPE = 0xe2,
	WIFI_COMMAND_ID_EXTENSION_SET_WEP_KEY = 0xe3,
	WIFI_COMMAND_ID_EXTENSION_WIRELESS_MODE = 0xe4,
	WIFI_COMMAND_ID_EXTENSION_COUNTRY_REGION = 0xe5,
	WIFI_COMMAND_ID_EXTENSION_RADIO_ON_OFF = 0xe6,
	WIFI_COMMAND_ID_EXTENSION_DTIM_INTERVAL = 0xe7,
	WIFI_COMMAND_ID_EXTENSION_LISTEN_INTERVAL = 0xe8,
	WIFI_COMMAND_ID_EXTENSION_GET_STA_LIST = 0xe9,
	WIFI_COMMAND_ID_EXTENSION_SMART_CONNECTION_FILTER = 0xea,
	WIFI_COMMAND_ID_EXTENSION_WOW_ENABLE = 0xeb,
	WIFI_COMMAND_ID_EXTENSION_WOW_MAGIC_PACKET = 0xec,
	WIFI_COMMAND_ID_EXTENSION_WOW_BITMAP_PATTERN = 0xed,
	WIFI_COMMAND_ID_EXTENSION_WOW_ARP_OFFLOAD = 0xee,
	WIFI_COMMAND_ID_EXTENSION_WOW_REKEY_OFFFLOAD = 0xef,
	WIFI_COMMAND_ID_EXTENSION_AP_STOP = 0xf0,
	WIFI_COMMAND_ID_EXTENSION_AP_START = 0xf1,
	WIFI_COMMAND_ID_EXTENSION_POWER_MANAGE_STATE = 0xf2,
	WIFI_COMMAND_ID_EXTENSION_GET_MAX_STA_NUMBER = 0xf3,
	WIFI_COMMAND_ID_EXTENSION_SET_EXTRA_IE = 0xf4,
	WIFI_COMMAND_ID_EXTENSION_UPDATE_HOMEKIT_IE = 0xf5,
	WIFI_COMMAND_ID_EXTENSION_ENABLE_AIRPLAY = 0xf6,
	WIFI_COMMAND_ID_EXTENSION_GET_CONNECTED_AP_RSSI = 0xf7,
	WIFI_COMMAND_ID_EXTENSION_SET_PSMODE = 0xf8,
	WIFI_COMMAND_ID_EXTENSION_RW_EFUSE = 0xf9,
	WIFI_COMMAND_ID_EXTENSION_TEST_N9_LP = 0xfa,
	WIFI_COMMAND_ID_EXTENSION_SET_PMKID = (0xdf + 0x24),
	WIFI_COMMAND_ID_EXTENSION_N9WDT_ON_OFF = (0xdf + 0x25),
	WIFI_COMMAND_ID_EXTENSION_N9WDT_LENGTH = (0xdf + 0x26),
	WIFI_COMMAND_ID_EXTENSION_ANT_MODE = 0x101,
	WIFI_COMMAND_ID_EXTENSION_CHANNEL_LIST = (0xdf + 0x23)
};

// Events the N9 can send to the A7
enum n9_events {
	// The FW informs the CPU of its MAC address
	EVENT_ID_IOT_MAC_ADDDRESS = 0x30,
	EVENT_ID_IOT_SCAN_LIST = 0x50,
	EVENT_ID_IOT_PROBE_REQ = 0x51,
	EVENT_ID_IOT_WLAN_EVT = 0x60,
	EVENT_ID_IOT_CONNECTED = 0x61,
	EVENT_ID_IOT_ASSOCIATED = 0x63,
	EVENT_ID_IOT_DEAUTH = 0x64,
	EVENT_ID_IOT_DISASSOC = 0x65,
	EVENT_ID_IOT_RELOAD_CONFIGURATION = 0x66,
	EVENT_ID_IOT_AP_LINK_UP = 0x67,
	EVENT_ID_IOT_REASSOCIATED = 0x68,
	EVENT_ID_IOT_MIC_ERROR = 0x69,
	EVENT_ID_IOT_ASSOC_BANDWIDTH = 0x76,
	EVENT_ID_IOT_SLP_NOTIFY                        = 0xFE,    /* Sleep notify event to request host to set FW ownership */
};

// Modes the N9 can run in
enum n9_wifi_opmode {
	// Station mode only (client)
	WIFI_MODE_STA_ONLY = 0x1,
	// AP mode only
	WIFI_MODE_AP_ONLY = 0x2,
	// Both modes
	WIFI_MODE_DUAL = WIFI_MODE_STA_ONLY | WIFI_MODE_AP_ONLY,
	//IBSS mode
	WIFI_MODE_IBSS_ONLY   = 0x6
};

#ifndef IS_ALIGN_4
#define IS_ALIGN_4(_value)      (((_value) & 0x3) ? false : true)
#define IS_NOT_ALIGN_4(_value)  (((_value) & 0x3) ? false : true)
#endif /* IS_ALIGN_4 */
#define MT3620_DMA_ENABLE 1
#define MT3620_DMA_DISABLE 0
///
/// Callback method for a remote API command.
/// This is provided by caller to be executed when the response comes in.
///
/// @cmd - Command ID that was sent
/// @status - status of the request.
///           Non zero indicates failure.  Codes are specific to each command.
///           Only N9 comands will have non zero values in status
/// @data - Pointer to response data
typedef void (*mt3620_hif_api_callback)(u32 cmd, u8 status, void *data);

///
/// Callback method for a remote API N9 event
/// Conusmers register these global handlers as part of their startup.
///
/// @event - Event ID
/// @status - Non zero value indicates a failure.  Codes are specific to each
/// command.
/// @inf - N9 interface the event is for (0 or 1)
/// @data - Response data
/// @data_size - Size of response payload
typedef void (*mt3620_hif_api_n9_event_handler)(enum n9_events event,
						   u8 status, u8 inf,
						   void *data, u32 data_size);

///
/// Callback method for N9 incoming RX data
///
/// @inf - N9 interface the event is for (0 or 1)
/// @skb - Response data
typedef void (*mt3620_hif_api_n9_data_handler)(u8 inf, struct sk_buff *skb);

///
/// Callback method for TX buffer update
///
/// @avail_buffer - remaining buffer size in bytes
typedef void (*mt3620_hif_api_n9_tx_update_handler)(u32 avail_buffer);

///
/// Opens a remote API handle based on the current device and the device tree.
///
/// @device - Device handle to the current module
/// @returns - Opaque handle for remote API calls
void *mt3620_hif_api_get_handle(struct device * device);

///
/// Gets the version number of the hardware
///
/// @handle = Remote API handle
/// @returns Version number
u32 mt3620_remote_api_get_version(void *handle);

// List of event handlers for various N9 events
// All handlers are optional
struct mt3620_hif_api_n9_event_handlers {
	// Channel bandwidth that has been negotiated after an association
	mt3620_hif_api_n9_event_handler assoc_bandwidth;
	// A data packet has been received from the N9
	mt3620_hif_api_n9_data_handler data_received;
	// Mac address was received from N9
	mt3620_hif_api_n9_event_handler mac_address;
	// A probe request was received
	mt3620_hif_api_n9_event_handler probe_req;
	// A scan list item has been received
	mt3620_hif_api_n9_event_handler scan_list;
	// General N9 wlan event was sent
	mt3620_hif_api_n9_event_handler wlan_event;
	// The available TX buffer space has been updated
	mt3620_hif_api_n9_tx_update_handler tx_update;
};

///
/// Registers global event handlers
///
/// @handlers - handler methods
void mt3620_hif_api_register_n9_event_handlers(
    struct mt3620_hif_api_n9_event_handlers *handlers);

///
/// Unregisters global event handlers
///
void mt3620_hif_api_unregister_n9_event_handlers(void);





///
/// Sends a packet of data to the N9
///
/// @handle - Remote API handle
/// @cmd - N9 command ID
/// @set - True if this is a set command, false if it is a get command
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @callback - Optional callback to invoke when a response is received
/// @returns - 0 for success
int mt3620_hif_api_send_command_to_n9_async(
    void *handle, enum n9_commands cmd, bool set, void *data, u32 data_size,
    mt3620_hif_api_callback callback);

///
/// Sends a packet of data to the N9 and blocks on a response
///
/// @handle - Remote API handle
/// @cmd - N9 command ID
/// @set - True if this is a set command, false if it is a get command
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @buffer - Buffer to hold response data
/// @buffer_size - Size of buffer
/// @returns - 0 for success
int mt3620_hif_api_send_command_to_n9_sync(void *handle,
					      enum n9_commands cmd, bool set,
					      void *data, u32 data_size,
					      void *buffer, u32 buffer_size);

///
/// Sends a packet of data to the N9 and doesn't wait for a response
///
/// @handle - Remote API handle
/// @cmd - N9 command ID
/// @set - True if this is a set command, false if it is a get command
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @returns - 0 for success
int mt3620_hif_api_send_command_to_n9_no_response(void *handle,
						     enum n9_commands cmd,
						     bool set, void *data,
						     u32 data_size);

int mt3620_hif_api_send_data_to_n9_aligned(void *handle, u8 interface,
				      u32 reserved_size, void *data,
				      u32 data_size);

///
/// Sends a TX data packet to the N9
///
/// @handle - Remote API handle
/// @interface - N9 interface ID
/// @reserved_size - How much space to allocate for the N9 to use during its
/// processing
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @returns - 0 for success
int mt3620_hif_api_send_data_to_n9(void *handle, u8 interface,
				      u32 reserved_size, void *data,
				      u32 data_size);


int32_t mt3620_hif_open(void);

void mt3620_hif_enable_interrupt(void);
int32_t mt3620_hif_tx_flow_control_check_and_update_tx(int32_t port, uint32_t pkt_len);
void mt3620_hif_fw_download(void);
