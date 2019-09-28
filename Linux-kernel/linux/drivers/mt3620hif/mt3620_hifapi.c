// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 hif API
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


#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock_types.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <mt3620/mt3620_hifapi.h>
#include <linux/delay.h>
#include <azure-sphere/security_monitor.h>

#include "mt3620_hifproc.h"

struct mt3620_hif_management_info *mt3620_hif_api_get_hif_management_info(u8 sequence_number);

static struct mt3620_hif_api_n9_event_handlers *g_n9_event_handlers = NULL;
uint8_t g_tx_buf[MAX_BUF_SIZE_MT3620 + 0x10];         
static fw_image_tailer_t *n9_fw_info;
uint8_t *g_pwifi_dma_base_address;
static u8 g_mt3620_last_sequence_number;
static u32 g_mt3620_sequence_number_use[8];
static struct dma_pool *g_mt3620_mgmt_pool;

syscfg_t g_wifi_profile;

///
/// Registers a global event handler
///
/// @handlers - handler methods
void mt3620_hif_api_register_n9_event_handlers(
    struct mt3620_hif_api_n9_event_handlers *handlers)
{
	g_n9_event_handlers = handlers;
}

///
/// Unregisters a global event handler
///
void mt3620_hif_api_unregister_n9_event_handlers(void)
{
	g_n9_event_handlers = NULL;
}


///
/// Gets a handle to use with other remote API calls
///
/// @device - device that is calling this API
/// @returns - handle or NULL if failed
///
/// Callers must have the rproc device tree node set as a pointer
/// to the remote proc node for this call to succeed
void *mt3620_hif_api_get_handle(struct device *device)
{
	struct device_node *node;

	if (g_pmt3620_hif_proc == NULL) {
		// We're not initialized yet
		return NULL;
	}

	if (device == NULL || device->of_node == NULL) {
		// Invalid input
		return NULL;
	}

	node = of_parse_phandle(device->of_node, "hifproc", 0);

	if (node == NULL) {
		dev_err(device, "Could not locate rproc device tree node");
		return NULL;
	}

	if (g_pmt3620_hif_proc->dev->of_node != node) {
		dev_err(device, "Invalid rproc reference");
		return NULL;
	}

	return g_pmt3620_hif_proc;
}


///
/// Gets the next free sequence number
/// Sequence numbers are from 0-255
/// This means we're going to roll over but have to
/// consider any numbers actively in use.
/// This is done by keeping an active bitfield.
///
/// @returns -  Sequence number
u8 mt3620_hif_api_get_next_sequence_number(void)
{
	const int MAX_SEQUENCE_NUMBER = 255;
	int i = 0;
	u32 idx;
	u32 mask;
	u8 ret;



	for (i = 0; i < MAX_SEQUENCE_NUMBER; i++) {
		g_mt3620_last_sequence_number++;

		idx = g_mt3620_last_sequence_number >> 5;
		mask = g_mt3620_last_sequence_number & 0x1f;

		if ((g_mt3620_sequence_number_use[idx] & (1 << mask)) == 0) {
			// It's free for use
			g_mt3620_sequence_number_use[idx] |= (1 << mask);
			ret = g_mt3620_last_sequence_number;
			break;
		}
	}



	return ret;
}




///
/// Initialization of Remote API
///
int mt3620_hif_api_init(void)
{
	int ret = SUCCESS;
	g_mt3620_last_sequence_number = 0;
	g_mt3620_sequence_number_use[0] = 0x1; // 0 is a reserved sequence number that
					// should never be handed out

	g_mt3620_mgmt_pool =
	    dma_pool_create("A7-HIF", g_pmt3620_hif_proc->dev,
			    sizeof(struct mt3620_hif_management_info),
			    4 /* alignment */, 0 /* no boundaries */);
	if (IS_ERR(g_mt3620_mgmt_pool)) {
		ret = PTR_ERR(g_mt3620_mgmt_pool);
		g_mt3620_mgmt_pool = NULL;
		goto exit;
	}

exit:
	return ret;
}

///
/// Shuts down Remote API
///
void mt3620_hif_api_shutdown(void)
{
	if (g_mt3620_mgmt_pool != NULL) {
		dma_pool_destroy(g_mt3620_mgmt_pool);
		g_mt3620_mgmt_pool = NULL;
	}
}


///
/// Allocate and initialize new management info item
///
/// @returns -  New item
struct mt3620_hif_management_info *
mt3620_hif_api_alloc_hif_management_info(void)
{
	struct mt3620_hif_management_info *ret = NULL;
	dma_addr_t hwaddr;

	ret = dma_pool_alloc(g_mt3620_mgmt_pool, GFP_KERNEL, &hwaddr);
	if (ret) {
		memset(ret, 0, sizeof(*ret));
		ret->hwaddr = hwaddr;

		// initialize list pointer
		INIT_LIST_HEAD(&ret->list);
	}

	return ret;
}


///
/// Return a sequence number to the pool
///
/// @sequence_number - Sequence number
void mt3620_hif_api_free_sequence_number(u8 sequence_number)
{
	u32 idx;
	u32 mask;

	// Sequence number 0 is reserved for unsolicited messages
	// As a safety never mark it for available use
	if (sequence_number == 0) {
		return;
	}

	idx = sequence_number >> 5;
	mask = sequence_number & 0x1f;

	g_mt3620_sequence_number_use[idx] &= ~(1 << mask);
}


///
/// Gets a free relay management info item
///
/// @returns -  Item to use
/// <remarks>
/// Also frees memory from past items due to the way the M4 communication
/// channel currently works
/// </remarks>
struct mt3620_hif_management_info *
mt3620_hif_api_get_free_hif_management_info(void)
{
	struct list_head *pos, *q;
	struct mt3620_hif_management_info *mgmt_item = NULL;
	struct mt3620_hif_management_info *ret = NULL;

	


	// Free any pending items.  In the future we should consider having the
	// relay protocol
	// proactively indicate which buffers can be freed rather than scanning
	// like this
	list_for_each_safe(pos, q, &g_pmt3620_hif_proc->hif_management_list)
	{
		mgmt_item =
		    list_entry(pos, struct mt3620_hif_management_info, list);

		// Cleanup this item if it's ready
		// It's ready if the M4 has marked it completed and
		// we either expect no response or the response has been
		// processed.
		if (mgmt_item->response_type == NO_RESPONSE ||
		     mgmt_item->response_processed == true) {
			dev_dbg(g_pmt3620_hif_proc->dev,
				"Freeing memory at %#x",
				(u32)mgmt_item->inner_packet);

			// Free packet memory if needed
			if (mgmt_item->inner_packet) {
				devm_kfree(g_pmt3620_hif_proc->dev,
					   (void *)mgmt_item->inner_packet);
			}

			// Remove from list
			list_del(pos);

			// Release sequence number
			mt3620_hif_api_free_sequence_number(
			    mgmt_item->sequence_number);

			// Free mgmt item
			dma_pool_free(g_mt3620_mgmt_pool, mgmt_item,
				      mgmt_item->hwaddr);
		}
	}

	// Allocate our new item
	mgmt_item = mt3620_hif_api_alloc_hif_management_info();
	if (mgmt_item != NULL) {
		list_add(&mgmt_item->list,
			 &g_pmt3620_hif_proc->hif_management_list);

		// Use the item
		ret = mgmt_item;
	}



	return ret;
}




///
/// Converts an N9 CMD to a string
///
/// @cmd - command
/// @returns - human readable command name
static const char *mt3620_hif_api_cmd_to_string(enum n9_commands cmd)
{
	switch (cmd)
	{
	case WIFI_COMMAND_ID_UNTET_HQA_ACTION: return "UNTET_HQA_ACTION";
	case WIFI_COMMAND_ID_IOT_OWN_MAC_ADDRESS: return "IOT_OWN_MAC_ADDRESS";
	case WIFI_COMMAND_ID_IOT_SSID: return "IOT_SSID";
	case WIFI_COMMAND_ID_IOT_BSSID: return "IOT_BSSID";
	case WIFI_COMMAND_ID_IOT_BANDWIDTH: return "IOT_BANDWIDTH";
	case WIFI_COMMAND_ID_IOT_SET_DISCONNECT: return "IOT_SET_DISCONNECT";
	case WIFI_COMMAND_ID_IOT_GET_LINK_STATUS: return "IOT_GET_LINK_STATUS";
	case WIFI_COMMAND_ID_IOT_GET_PROFILE: return "IOT_GET_PROFILE";
	case WIFI_COMMAND_ID_IOT_START_SCAN: return "IOT_START_SCAN";
	case WIFI_COMMAND_ID_IOT_STOP_SCAN: return "IOT_STOP_SCAN";
	case WIFI_COMMAND_ID_IOT_GET_SCAN_LIST: return "IOT_GET_SCAN_LIST";
	case WIFI_COMMAND_ID_IOT_RX_FILTER: return "IOT_RX_FILTER";
	case WIFI_COMMAND_ID_IOT_CHANNEL: return "IOT_CHANNEL";
	case WIFI_COMMAND_ID_IOT_80211_TO_HOST: return "IOT_80211_TO_HOST";
	case WIFI_COMMAND_ID_IOT_SWITCH_MODE: return "IOT_SWITCH_MODE";
	case WIFI_COMMAND_ID_IOT_SET_KEY: return "IOT_SET_KEY";
	case WIFI_COMMAND_ID_IOT_EXTENSION: return "IOT_EXTENSION";
	case WIFI_COMMAND_ID_EXTENSION_DEBUG_LEVEL: return "EXTENSION_DEBUG_LEVEL";
	case WIFI_COMMAND_ID_EXTENSION_SET_AUTH_MODE: return "EXTENSION_SET_AUTH_MODE";
	case WIFI_COMMAND_ID_EXTENSION_SET_ENCRYPT_TYPE: return "EXTENSION_SET_ENCRYPT_TYPE";
	case WIFI_COMMAND_ID_EXTENSION_SET_WEP_KEY: return "EXTENSION_SET_WEP_KEY";
	case WIFI_COMMAND_ID_EXTENSION_WIRELESS_MODE: return "EXTENSION_WIRELESS_MODE";
	case WIFI_COMMAND_ID_EXTENSION_COUNTRY_REGION: return "EXTENSION_COUNTRY_REGION";
	case WIFI_COMMAND_ID_EXTENSION_RADIO_ON_OFF: return "EXTENSION_RADIO_ON_OFF";
	case WIFI_COMMAND_ID_EXTENSION_DTIM_INTERVAL: return "EXTENSION_DTIM_INTERVAL";
	case WIFI_COMMAND_ID_EXTENSION_LISTEN_INTERVAL: return "EXTENSION_LISTEN_INTERVAL";
	case WIFI_COMMAND_ID_EXTENSION_GET_STA_LIST: return "EXTENSION_GET_STA_LIST";
	case WIFI_COMMAND_ID_EXTENSION_SMART_CONNECTION_FILTER: return "EXTENSION_SMART_CONNECTION_FILTER";
	case WIFI_COMMAND_ID_EXTENSION_WOW_ENABLE: return "EXTENSION_WOW_ENABLE";
	case WIFI_COMMAND_ID_EXTENSION_WOW_MAGIC_PACKET: return "EXTENSION_WOW_MAGIC_PACKET";
	case WIFI_COMMAND_ID_EXTENSION_WOW_BITMAP_PATTERN: return "EXTENSION_WOW_BITMAP_PATTERN";
	case WIFI_COMMAND_ID_EXTENSION_WOW_ARP_OFFLOAD: return "EXTENSION_WOW_ARP_OFFLOAD";
	case WIFI_COMMAND_ID_EXTENSION_WOW_REKEY_OFFFLOAD: return "EXTENSION_WOW_REKEY_OFFFLOAD";
	case WIFI_COMMAND_ID_EXTENSION_AP_STOP: return "EXTENSION_AP_STOP";
	case WIFI_COMMAND_ID_EXTENSION_AP_START: return "EXTENSION_AP_START";
	case WIFI_COMMAND_ID_EXTENSION_POWER_MANAGE_STATE: return "EXTENSION_POWER_MANAGE_STATE";
	case WIFI_COMMAND_ID_EXTENSION_GET_MAX_STA_NUMBER: return "EXTENSION_GET_MAX_STA_NUMBER";
	case WIFI_COMMAND_ID_EXTENSION_SET_EXTRA_IE: return "EXTENSION_SET_EXTRA_IE";
	case WIFI_COMMAND_ID_EXTENSION_UPDATE_HOMEKIT_IE: return "EXTENSION_UPDATE_HOMEKIT_IE";
	case WIFI_COMMAND_ID_EXTENSION_ENABLE_AIRPLAY: return "EXTENSION_ENABLE_AIRPLAY";
	case WIFI_COMMAND_ID_EXTENSION_GET_CONNECTED_AP_RSSI: return "EXTENSION_GET_CONNECTED_AP_RSSI";
	case WIFI_COMMAND_ID_EXTENSION_SET_PSMODE: return "EXTENSION_SET_PSMODE";
	case WIFI_COMMAND_ID_EXTENSION_TEST_N9_LP: return "EXTENSION_TEST_N9_LP";
	case WIFI_COMMAND_ID_EXTENSION_N9WDT_ON_OFF: return "EXTENSION_N9WDT_ON_OFF";
	case WIFI_COMMAND_ID_EXTENSION_N9WDT_LENGTH: return "EXTENSION_N9WDT_LENGTH";
	case WIFI_COMMAND_ID_EXTENSION_ANT_MODE: return "ANT_MODE";
	case WIFI_COMMAND_ID_EXTENSION_CHANNEL_LIST: return "EXTENSION_CHANNEL_LIST";
	default: return "<unknown>";
	}
}




///
/// Sends a packet of data to the N9
///
/// @handle - Remote API handle
/// @cmd - N9 command ID
/// @set - True if this is a set command, false if it is a get command
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @response_type - How to handle response
/// @response_data - Data to use in response handling
/// @returns -  0 for success
int mt3620_hif_api_send_command_to_n9(
    void *handle, enum n9_commands cmd, bool set, void *data, u32 data_size,
    enum mt3620_hif_response_type response_type,
    union mt3620_hif_response_data response_data)
{
	int ret = SUCCESS;
	u8 *buffer;
	u32 aligned_size = 0;
	u32 base_size = 0;
	u32 final_data_size = 0;
	u32 tx_size = 0;
	struct n9_command_header header;
	struct mt3620_hif_management_info *mgmt_info;	
	
      // get ownership back
      if (mt3620_hif_get_ownership() == false)
      {   
          return -EAGAIN;
      } 
	// Validate handle
	if (handle == NULL || handle != g_pmt3620_hif_proc) {
		dev_err(g_pmt3620_hif_proc->dev, "Invalid remote handle passed to "
			"mt3620_hif_api_send_command_"
			"to_n9");
			return -EINVAL;
	}
	
	if (data_size > 0 && data == NULL) {
		dev_err(g_pmt3620_hif_proc->dev,"Invalid data size or pointer passed to "
					"mt3620_remote_api_send_command_to_n9");
		return -EINVAL;
	}

	// Get a free management info structure to set up
	mgmt_info = mt3620_hif_api_get_free_hif_management_info();
	if (mgmt_info == NULL) {
		dev_err(g_pmt3620_hif_proc->dev, "Could not allocate relay info in "
					     "mt3620_hif_api_send_command_"
					     "to_n9");
		return -ENOMEM;
	}

	
	// Allocate space for response if it's larger than our sent data
	if (response_type == COMPLETION &&
		data_size < response_data.completion.buffer_size) {
		final_data_size = response_data.completion.buffer_size;
		} 
		else 
		{
			final_data_size = data_size;
		}
	
		// Base size
		base_size = sizeof(struct n9_command_header) + final_data_size;
		// 4 byte aligned for our data
		aligned_size = ALIGN(base_size, 4);
		// Aligned to a TX block size for transfer to the N9
		tx_size = ALIGN(aligned_size, N9_TX_BLOCK_SIZE);
	
		// Build our M4 header for this request + copy data into memory we
		// control
		memset(&header, 0, sizeof(struct n9_command_header));
	
		header.byte_count = base_size;
		header.port_queue_id = N9_CMD_PQ;
		header.wifi_command.cmd = cmd;
		header.wifi_command.sequence_number =
		mt3620_hif_api_get_next_sequence_number();
		header.wifi_command.packet_type = N9_PACKET_ID_CMD;
	
		if (cmd >= WIFI_COMMAND_ID_IOT_EXTENSION) {
			header.wifi_command.cmd = WIFI_COMMAND_ID_IOT_EXTENSION;
			header.wifi_command.extended_cmd =
			cmd - WIFI_COMMAND_ID_IOT_EXTENSION;
			}
		if (!set) {
			header.wifi_command.get = 1;
			}
	
		// Copy header + data into a final buffer
		buffer = devm_kzalloc(g_pmt3620_hif_proc->dev, tx_size, GFP_KERNEL);
	
		if (buffer == NULL) {
			ret = -ENOMEM;
			goto exit;
			}
	
		memcpy(buffer, &header, sizeof(struct n9_command_header));
		memcpy(&buffer[sizeof(struct n9_command_header)], data, data_size);

		
		//build DL list hif cmd structure
		mgmt_info->cmd =cmd;
		mgmt_info->response_type = response_type ;
		mgmt_info->response_data = response_data;
		mgmt_info->sequence_number = header.wifi_command.sequence_number;
		mgmt_info->response_processed = false;
		mgmt_info->inner_packet = buffer;
		mgmt_info->inner_packet_size = header.byte_count;
					// Now that we've filled out data we call the DMA mapping APIs to ensure
					// coherent transfer


		dev_info(g_pmt3620_hif_proc->dev,"CMD %02x (%s), seq %hhx", cmd, mt3620_hif_api_cmd_to_string(cmd),
					header.wifi_command.sequence_number);
	
					// Send the message
		ret = mt3620_hif_tx_flow_control_check_and_update_tx(WIFI_HIF_RX_PORT0_IDX, tx_size);
	
		if (ret == HIF_STATUS_SUCCESS){
			dev_dbg(g_pmt3620_hif_proc->dev,"Sending N9 Command\n");
			ret = mt3620_hif_fifo_write((unsigned char *)buffer, tx_size);
		}
	
	exit:
      
	return ret;
}


///
/// Sends a packet of data to the N9 asynchronously with a callback for the
/// response
///
/// @handle - Remote API handle
/// @cmd - N9 command ID
/// @set - True if this is a set command, false if it is a get command
/// @data - Command data packet
/// @data_size - Size of data packet in bytes
/// @callback - Optional callback to invoke when a response is received
/// @returns -  0 for success
int mt3620_hif_api_send_command_to_n9_async(
    void *handle, enum n9_commands cmd, bool set, void *data, u32 data_size,
    mt3620_hif_api_callback callback)
{
	union mt3620_hif_response_data response_data = {.callback = callback};

	return mt3620_hif_api_send_command_to_n9(
	    handle, cmd, set, data, data_size, CALLBACK, response_data);
}
EXPORT_SYMBOL(mt3620_hif_api_send_command_to_n9_async);

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
					      void *buffer, u32 buffer_size)
{
	struct completion c;
	u32 needed_buffer_size = 0;
	u8 status = 0;
	int ret;
	u32 time_remaining = 0;

	union mt3620_hif_response_data response_data = {
	    .completion = {.completion = &c,
			   .buffer = buffer,
			   .buffer_size = buffer_size,
			   .needed_buffer_size = &needed_buffer_size,
			   .status = &status}};

	if (buffer == NULL && buffer_size != 0) {
		return -EINVAL;
	}

	init_completion(&c);

	// Send command
	ret = mt3620_hif_api_send_command_to_n9(
	    handle, cmd, set, data, data_size, COMPLETION, response_data);
	if (ret != SUCCESS) {
		return ret;
	}

	// Requests should get resposnes in a small amount of time (usec or msc)
	// We put a timeout here to catch cases where a response never comes.
	// This is always unexpected but we WARN and return so we can recover.
	
	time_remaining =
	    wait_for_completion_timeout(&c, msecs_to_jiffies(5000));

	WARN_ON(time_remaining == 0);

	if (time_remaining == 0) {	
		struct mt3620_hif_management_info *mgmt_info = 
			mt3620_hif_api_get_hif_management_info(g_mt3620_last_sequence_number);
		mgmt_info->response_processed = true;
		return -ETIMEDOUT;
	}

	if (buffer_size != 0 && needed_buffer_size != 0 &&
	    needed_buffer_size > buffer_size) {
		// Invalid buffer size
		return -ENOMEM;
	}

	return status;
}


EXPORT_SYMBOL(mt3620_hif_api_send_command_to_n9_sync);



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
						     u32 data_size)
{
	union mt3620_hif_response_data response_data = {};

	return mt3620_hif_api_send_command_to_n9(
	    handle, cmd, set, data, data_size, NO_RESPONSE, response_data);
		
}


EXPORT_SYMBOL(mt3620_hif_api_send_command_to_n9_no_response);

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
int mt3620_hif_api_send_data_to_n9_aligned(void *handle, u8 interface,
				      u32 reserved_size, void *data,
				      u32 data_size)
{
	int ret = SUCCESS;
	u8 *buffer;
	u32 aligned_size = 0;
	u32 base_size = 0;
	u32 tx_size = 0;
	struct n9_wifi_data_header header;


	 // get ownership back
    if(mt3620_hif_get_ownership() == false)
    {   
    	return -EAGAIN;
    }
	
	// Validate handle
	if (handle == NULL || handle != g_pmt3620_hif_proc) {
			dev_err(g_pmt3620_hif_proc->dev, "Invalid remote handle passed to "
					"mt3620_hif_api_send_data_to_"
					"n9");
				return -EINVAL;
		}
	
	if (data_size > 0 && data == NULL) {
			dev_err(g_pmt3620_hif_proc->dev,
			"Invalid data size or pointer passed to "
			"mt3620_hif_api_send_data_to_n9");
			return -EINVAL;
		}
	
	
	// Base size
	base_size = sizeof(struct n9_wifi_data_header) + reserved_size + data_size;
	// 4 byte aligned for our data
	aligned_size = ALIGN(base_size, 4);
	// Aligned to a TX block size for transfer to the N9
	tx_size = ALIGN(aligned_size, N9_TX_BLOCK_SIZE);
	
	// Build our HIF header for this request + copy data into memory we
	// control
	memset(&header, 0, sizeof(struct n9_wifi_data_header));
	header.byte_count = base_size;
	header.port_queue_id = N9_DATA_PQ;
	header.cmd = 0;
	header.packet_type = N9_PACKET_ID_CMD;
	header.packet_offset = reserved_size;
	header.interface = interface << 4;
	
	// Copy header + data into a final buffer
	buffer = devm_kzalloc(g_pmt3620_hif_proc->dev, tx_size, GFP_KERNEL);

	if (buffer == NULL) {
		ret = -ENOMEM;
		goto exit;
		}
	
	memcpy(buffer, &header, sizeof(struct n9_wifi_data_header));
	memcpy(&buffer[sizeof(struct n9_wifi_data_header) + reserved_size],
		 	data, data_size);
	
	
	dev_dbg(g_pmt3620_hif_proc->dev, "Sending N9 data of size %x", data_size);
	
					// Send the message
	ret = mt3620_hif_tx_flow_control_check_and_update_tx(WIFI_HIF_RX_PORT0_IDX, tx_size);
	
	if (ret == HIF_STATUS_SUCCESS){
		ret = mt3620_hif_fifo_write((unsigned char *)buffer, tx_size);
	}
	
exit:
	
	if (ret != SUCCESS) {

		dev_dbg(g_pmt3620_hif_proc->dev, "Sending N9 data fail");		 
	}


	devm_kfree(g_pmt3620_hif_proc->dev,(void *)buffer);
	
	return ret;
}
	
EXPORT_SYMBOL(mt3620_hif_api_send_data_to_n9_aligned);

	

int mt3620_hif_api_send_data_to_n9(void *handle, u8 interface, 
				u32 reserved_size, void *data, u32 data_size) 
{ 
	int ret = SUCCESS; 
	u32 base_size = 0; 
	u32 tx_size = 0; 
	u8 tail_room_required = 0; 
	struct n9_wifi_data_header *header = NULL;
	struct sk_buff *skb = NULL;

    // get ownership back
    if(mt3620_hif_get_ownership() == false)
    {   
    	return -EAGAIN;
    }
	// Validate handle 
	if (handle == NULL || handle != g_pmt3620_hif_proc) { 
	dev_err(g_pmt3620_hif_proc->dev, "Invalid remote handle passed to " 
	"mt3620_hif_api_send_data_to_" 
	"n9"); 
	return -EINVAL; 
	} 

	if (data_size > 0 && data == NULL) { 
	dev_err(g_pmt3620_hif_proc->dev, 
	"Invalid data size or pointer passed to " 
	"mt3620_hif_api_send_data_to_n9"); 
	return -EINVAL; 
	} 

	skb = (struct sk_buff *)data ;

	// Base size 
	base_size = sizeof(struct n9_wifi_data_header) + reserved_size + data_size; 
	skb_push(skb, (sizeof(struct n9_wifi_data_header) + reserved_size)); 


	tx_size = ALIGN(base_size, N9_TX_BLOCK_SIZE); 

	tail_room_required = tx_size - base_size; 

	if(skb_tailroom(skb) < tail_room_required) 
	{ 
	dev_err(g_pmt3620_hif_proc->dev, "Insufficient TailRoom\n"); 
	return -ENOMEM; 

	} 
	__memzero(skb->tail,tail_room_required); 
	skb_put(skb,tail_room_required); 

	// Build our HIF header for this request + copy data into memory 

	header = (struct n9_wifi_data_header *)skb->data; 

	header->byte_count = base_size; 
	header->port_queue_id = N9_DATA_PQ; 
	header->cmd = 0; 
	header->packet_type = N9_PACKET_ID_CMD; 
	header->packet_offset = reserved_size; 
	header->interface = interface << 4; 


	dev_dbg(g_pmt3620_hif_proc->dev, "Sending N9 data of size %x", data_size); 

	// Send the message 
	ret = mt3620_hif_tx_flow_control_check_and_update_tx(WIFI_HIF_RX_PORT0_IDX, tx_size); 
	if (ret == HIF_STATUS_SUCCESS){ 
	ret = mt3620_hif_fifo_write((unsigned char *)skb->data, skb->len); 
	} 

	if (ret != SUCCESS) { 
	dev_dbg(g_pmt3620_hif_proc->dev, "Sending N9 data fail\n"); 
	} 

	return ret; 
}




EXPORT_SYMBOL(mt3620_hif_api_send_data_to_n9);



#define N9_INT_MASK 0x1f



///
/// Gets the management info item for a sequence number
///
/// @sequence_number - Sequence number
/// @returns -  Item that matches the number or NULL
struct mt3620_hif_management_info *
mt3620_hif_api_get_hif_management_info(u8 sequence_number)
{
	struct mt3620_hif_management_info *mgmt_item = NULL;
	struct mt3620_hif_management_info *ret = NULL;

	

	list_for_each_entry(mgmt_item, &g_pmt3620_hif_proc->hif_management_list,
			    list)
	{
		if (mgmt_item->sequence_number == sequence_number) {
			ret = mgmt_item;
			break;
		}
	}

	return ret;
}


///
/// Processes a response message and fires any callbacks
///
/// @cmd - command ID
/// @status - status code
/// @sequence_number - message sequence number
/// @data - pointer to data buffer
/// @data_size - length of buffer
void mt3620_hif_api_process_response(u32 cmd, u32 status, u8 sequence_number,
					void *data, u32 data_size)
{
	struct mt3620_hif_management_info *mgmt_item;

	// Get mgmt item for this operation
	mgmt_item =
	    mt3620_hif_api_get_hif_management_info(sequence_number);
	if (mgmt_item == NULL) {
		dev_err(g_pmt3620_hif_proc->dev,
			"Reponse sequence number not valid: %hhu",
			sequence_number);
		return;
	}

	if (mgmt_item->response_type == CALLBACK) {
	    if (mgmt_item->response_data.callback != NULL) {
			// Fire callback
			mgmt_item->response_data.callback(cmd, status, data);
		}
	} else if (mgmt_item->response_type == COMPLETION) {
		*mgmt_item->response_data.completion.needed_buffer_size =
		    data_size;
		*mgmt_item->response_data.completion.status = status;

		// We only do the copy if the caller passed a buffer large
		// enough.
		// The caller must needed_buffer_size to see if they gave us a
		// big enough buffer before reading data.
		if (data_size <=
		    mgmt_item->response_data.completion.buffer_size) {
			// Copy data to output buffer
			memcpy(mgmt_item->response_data.completion.buffer, data,
			       data_size);
		}

		// Signal we're done
		complete_all(mgmt_item->response_data.completion.completion);
	} else if (mgmt_item->response_type == NO_RESPONSE) {
		// Nothing to do
	} else {
		dev_err(g_pmt3620_hif_proc->dev, "Unsupported response type: %#x",
			mgmt_item->response_type);
	}

		
	mgmt_item->response_processed = true;
}





///
/// Gets the interface number from a N9 event
/// Copied from M4 firmware code
///
/// @interface - interface payload from header
/// @returns - interface number
u8 mt3620_hif_api_get_interface_number(u8 interface)
{
	// Interface is value << 4
	return (interface & N9_INT_MASK) >> 4;
}



void mt3620_hif_api_handle_n9_message(struct n9_event_header *header)
{
	struct n9_command_result *result =
	    (struct n9_command_result *)header->data_buffer;
	void *data = &header->data_buffer[sizeof(struct n9_command_result)];
	u32 data_size = header->byte_count - sizeof(struct n9_event_header) -
			sizeof(struct n9_command_result);

	dev_dbg(g_pmt3620_hif_proc->dev, "N9 message: %#x seq num: %d status: %u",
		header->event_id, header->sequence_number, (u32)result->status);

	switch (header->event_id) {
	case EVENT_ID_IOT_SCAN_LIST:
		if (g_n9_event_handlers != NULL &&
		    g_n9_event_handlers->scan_list != NULL) {
			g_n9_event_handlers->scan_list(
			    header->event_id, result->status,
			    mt3620_hif_api_get_interface_number(
				header->interface),
			    data, data_size);
		}
		break;
	case EVENT_ID_IOT_MAC_ADDDRESS:
		if (g_n9_event_handlers != NULL &&
		    g_n9_event_handlers->mac_address != NULL) {
			g_n9_event_handlers->mac_address(
			    header->event_id, result->status,
			    mt3620_hif_api_get_interface_number(
				header->interface),
			    data, data_size);
		}
		break;
	case EVENT_ID_IOT_WLAN_EVT:
		if (g_n9_event_handlers != NULL &&
		    g_n9_event_handlers->wlan_event != NULL) {
			g_n9_event_handlers->wlan_event(
			    header->event_id, result->status,
			    mt3620_hif_api_get_interface_number(
				header->interface),
			    data, data_size);
		}
		break;
	case EVENT_ID_IOT_ASSOC_BANDWIDTH:
		if (g_n9_event_handlers != NULL &&
		    g_n9_event_handlers->assoc_bandwidth != NULL) {
			g_n9_event_handlers->assoc_bandwidth(
			    header->event_id, result->status,
			    mt3620_hif_api_get_interface_number(
				header->interface),
			    data, data_size);
		}
		break;
	case EVENT_ID_IOT_PROBE_REQ:
		if (g_n9_event_handlers != NULL &&
		    g_n9_event_handlers->probe_req != NULL) {
			g_n9_event_handlers->probe_req(
			    header->event_id, result->status,
			    mt3620_hif_api_get_interface_number(
				header->interface),
			    data, data_size);
		}
		break;
    case EVENT_ID_IOT_MIC_ERROR:
		if (g_n9_event_handlers != NULL &&
		    g_n9_event_handlers->wlan_event != NULL) {
			g_n9_event_handlers->wlan_event(
			    header->event_id, result->status,
			    mt3620_hif_api_get_interface_number(
				header->interface),
			    data, data_size);
		}
		break;
	case WIFI_COMMAND_ID_IOT_EXTENSION:
	case WIFI_COMMAND_ID_IOT_OWN_MAC_ADDRESS:
	case WIFI_COMMAND_ID_IOT_RX_FILTER:
	case WIFI_COMMAND_ID_IOT_SET_DISCONNECT:
	case WIFI_COMMAND_ID_IOT_SET_KEY:
	case WIFI_COMMAND_ID_IOT_START_SCAN:
	case WIFI_COMMAND_ID_IOT_STOP_SCAN:
	case WIFI_COMMAND_ID_IOT_SWITCH_MODE:
	case WIFI_COMMAND_ID_IOT_CHANNEL:
	case WIFI_COMMAND_ID_IOT_BSSID:
	case WIFI_COMMAND_ID_IOT_SSID:
	case WIFI_COMMAND_ID_UNTET_HQA_ACTION:
		mt3620_hif_api_process_response(
		    header->event_id, result->status, header->sequence_number,
		    data, data_size);
		break;
   	case EVENT_ID_IOT_SLP_NOTIFY:
            mt3620_hif_set_ownership();
	break;
	default:
		dev_err(g_pmt3620_hif_proc->dev,
			"Unsupported relay N9 command: %#x", header->event_id);
	}
}	


void mt3620_hif_api_handle_n9_data(struct sk_buff *skb, u16 packet_size)
{
	u32 final_size;
	struct n9_data_header *header;

	header = (struct n9_data_header *)skb->data;

	// Move past N9 headers
	skb_reserve(skb, sizeof(struct n9_data_header));
	skb_reserve(skb, header->packet_offset);

	final_size = packet_size - N9_DATA_EXTRA_LENGTH - header->packet_offset;

	if (final_size > (skb->end - skb->data)) {
		dev_err(g_pmt3620_hif_proc->dev,
			"Invalid SKB size: %#x %#x %x %x", final_size, (u32)header, header->packet_offset, packet_size);
		BUG();
	}

	// Setup data length
	skb_put(skb, final_size);

	// Call helper
	if (g_n9_event_handlers != NULL &&
	    g_n9_event_handlers->data_received != NULL) {
		g_n9_event_handlers->data_received(
		    mt3620_hif_api_get_interface_number(header->interface),
		    skb);
	}

}

void mt3620_hif_api_handle_tx_update(u32 free_tx_space)
{
	// Call helper
	if (g_n9_event_handlers != NULL &&
	    g_n9_event_handlers->tx_update != NULL) {
		g_n9_event_handlers->tx_update(
		    free_tx_space);
	}
}

int32_t mt3620_hif_abnormal_interrupt_check(uint32_t value)
{
    uint32_t reason;
    int32_t err = HIF_STATUS_SUCCESS;

    if (value & ABNORMAL_INT)
    {
        err = mt3620_hif_cr_read(WASR, &reason);
        if (err)
        {
             dev_err(g_pmt3620_hif_proc->dev, "<<%s>> Read WASR failed. Error = %d.\n", __FUNCTION__, (int)err);
        }
        else
        {
            dev_dbg(g_pmt3620_hif_proc->dev, "<<%s>> Read WASR = %08x.\n", __FUNCTION__, (unsigned int)reason);
        }
    }
    return err;
}

/* ===============================================================
 * Function: mt3620_hif_create_inband_fw_scatter_txd_rom()
 * Description: to fill the scatter commnad header of rom code.
 * Input: total size of cmd, buffer, command id
 * Return: the header size
 * ===============================================================*/
uint32_t mt3620_hif_create_inband_fw_scatter_txd_rom(uint32_t cmd_len, uint8_t *buffer)
{
    P_INIT_HIF_TX_HEADER_T p_txd;
    P_INIT_WIFI_CMD_T   p_cmd;

    p_txd = (P_INIT_HIF_TX_HEADER_T)buffer;
    p_txd->u2TxByteCount = cmd_len;
    p_txd->u2PQ_ID = PQ_TO_PDA;

    p_cmd = &(p_txd->rInitWifiCmd);
    p_cmd->ucCID = MT_FW_SCATTER;
    p_cmd->ucPktTypeID = PKT_ID_CMD;

    return LEN_INBAND_CMD_HDR_ROM;
}


/* ===============================================================
 * Function: mt3620_hif_create_inband_cmd_txd_rom()
 * Description: to fill the inband commnad header of rom code.
 * Input: total size of cmd, buffer, command id
 * Return: the header size
 * ===============================================================*/
uint32_t mt3620_hif_create_inband_cmd_txd_rom(uint32_t cmd_len, uint8_t *buffer, enum MT_CMD_TYPE cmd)
{
    P_INIT_HIF_TX_HEADER_T p_txd;
    P_INIT_WIFI_CMD_T   p_cmd;

    p_txd = (P_INIT_HIF_TX_HEADER_T)buffer;
    p_txd->u2TxByteCount = cmd_len;
    p_txd->u2PQ_ID = P1_Q0;

    p_cmd = &(p_txd->rInitWifiCmd);
    p_cmd->ucCID = cmd;
    p_cmd->ucPktTypeID = PKT_ID_CMD;

    return LEN_INBAND_CMD_HDR_ROM;
}

uint8_t mt3620_hif_fw_download_CmdAddressLenReq_event_check(uint8_t *buffer)
{
    P_INIT_WIFI_EVENT_T p_event;
    P_INIT_EVENT_CMD_RESULT p_status;
    uint8_t err = HIF_STATUS_SUCCESS;

    // get stauts
    p_event = (P_INIT_WIFI_EVENT_T)buffer;
    p_status = (P_INIT_EVENT_CMD_RESULT)p_event->aucBuffer;

   
    if ((p_status->ucStatus) || (p_event->ucEID != INIT_EVENT_ID_CMD_RESULT))
    {
         dev_err(g_pmt3620_hif_proc->dev, "<<%s>> Download config event incorrect. EID = %d, Status = %d\n",
            __FUNCTION__,
            (int)p_event->ucEID,
            (int)p_status->ucStatus);
        err = HIF_STATUS_FAIL;
    }
    else
    {
         dev_dbg(g_pmt3620_hif_proc->dev,"<<%s>> Download config set Successfully.\n", __FUNCTION__);
    }

    return err;
}


uint32_t mt3620_hif_fw_download_CmdAddressLenReq(uint32_t addr, uint32_t len, uint32_t data_mode, uint8_t *buffer)
{
    uint32_t cmd_len, offset;
    P_INIT_CMD_DOWNLOAD_CONFIG p_data;

    cmd_len = sizeof(INIT_HIF_TX_HEADER_T) + sizeof(INIT_CMD_DOWNLOAD_CONFIG);

    // fill in the txd
    offset = mt3620_hif_create_inband_cmd_txd_rom(cmd_len, buffer, MT_TARGET_ADDRESS_LEN_REQ);

    // fill in the parameters
    p_data = (P_INIT_CMD_DOWNLOAD_CONFIG)(buffer + offset);
    p_data->u4Address = addr;
    p_data->u4Length = len;
    p_data->u4DataMode = data_mode;


    return cmd_len;
}

int32_t mt3620_hif_fw_download_Tx_Scatter(uint8_t *image, uint32_t len)
{
    int32_t err = 0;
    int32_t ret;
    uint32_t left_len = len;
    uint32_t read_len = 0;
    uint32_t pos = 0, offset = 0;
    uint32_t tx_len;

  //  log_hal_info( "<<%s>>\n", __FUNCTION__);
    while (left_len)
    {
        if (left_len < LEN_FW_SCATTER)
        {
            read_len = left_len;
        }
        else
        {
            read_len = LEN_FW_SCATTER;
        }

        // prepare txd
        tx_len = LEN_SDIO_TX_AGG_WRAPPER(LEN_INBAND_CMD_HDR_ROM + read_len);

       	memset(g_tx_buf, 0, tx_len);
        offset = mt3620_hif_create_inband_fw_scatter_txd_rom(LEN_INBAND_CMD_HDR_ROM + read_len, g_tx_buf);

        memcpy(&g_tx_buf[offset], &image[pos], read_len);

        pos += read_len;
        left_len -= read_len;

    	dev_dbg(g_pmt3620_hif_proc->dev,"<<%s>> Read file total_len: %d.\n", __FUNCTION__, (int)pos);
        
   

        // write to dut
        ret = mt3620_hif_fifo_write(g_tx_buf, tx_len);
        if (ret != 0)
        {
            // error cuures
       		dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Read scatter failed. Error = %d.\n", __FUNCTION__, (int)ret);
            err = 1;;
        }
    }
    return err;
}



int32_t mt3620_hif_pda_config(uint32_t dest_addr, uint32_t data_len, uint32_t data_mode)
{
    uint32_t value, rx_size, counter = 0;
    int32_t ret;
    uint8_t wait_tx_done, wait_rx_done;
    int32_t err = 0;
    uint8_t buffer[SDIO_MAX_RW_SIZE];

  	dev_info(g_pmt3620_hif_proc->dev, "<<%s>> Send fw download command...\n", __FUNCTION__);

    memset(buffer, 0, SDIO_MAX_RW_SIZE);

    value = mt3620_hif_fw_download_CmdAddressLenReq(dest_addr, data_len, data_mode, buffer);

    // write fw_download cmd to dut
    ret = mt3620_hif_fifo_write(buffer, LEN_SDIO_TX_AGG_WRAPPER(value));
    if (ret != 0)
    {
        // error cuures
        err = 1;
        return err;
    }

    wait_tx_done = 1;
    wait_rx_done = 1;
    counter = 0;
    while (1)
    {
        ret = mt3620_hif_cr_read(WHISR, &value);
        if (ret)
        {
            dev_err(g_pmt3620_hif_proc->dev, "<<%s>> Read WHISR failed. Error = %d.\n", __FUNCTION__, (int)ret);
			err = 1;
            return err;
        }

        dev_dbg(g_pmt3620_hif_proc->dev, "<<%s>> WHISR = %08x.\n", __FUNCTION__, value);

        mt3620_hif_abnormal_interrupt_check(value);

        if (wait_tx_done == 1)
        {
            if (value & TX_DONE_INT)
            {
                wait_tx_done = 0;
            }
        }

        if (wait_rx_done == 1)
        {
            if (value & RX0_DONE_INT)
            {
                wait_rx_done = 0;
            }
        }

        if ((wait_tx_done == 0) && (wait_rx_done == 0))
        {
            break;
        }

        if (counter > 2000000) // wait for at least 1 second
        {
           dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Check interrupt failed. wait_tx_done = %d, wait_tx_done = %d.\n",
                __FUNCTION__,
                (int)wait_tx_done,
                (int)wait_rx_done);
		  	 err = 1;
            return err;
        }
      

        counter++;
    }

    // rx event from dut
  	dev_dbg(g_pmt3620_hif_proc->dev,"<<%s>> Rx fw download event...\n", __FUNCTION__);
    ret = mt3620_hif_cr_read(WRPLR, &value);
    if (ret)
    {
       dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Read WRPLR failed. Error = %d.\n", __FUNCTION__, (int)ret);
	   err = 1;
        return err;
    }

  	dev_dbg(g_pmt3620_hif_proc->dev, "<<%s>> WRPLR = %08x.\n", __FUNCTION__, (unsigned int)value);


    //rx_size = LEN_SDIO_RX_PACKET_WRAPPER(LEN_FW_DOWNLOAD_EVENT); //use this one will cause data incorrect. all 0.
    rx_size = LEN_FW_DOWNLOAD_EVENT;

    if (GET_RX0_PACKET_LENGTH(value) != rx_size)
    {
    dev_err(g_pmt3620_hif_proc->dev, "<<%s>> download config event response length incoreect. WRPLR = %08x.\n",
            __FUNCTION__,
            (unsigned int)value);
		err = 1;
        return err;
    }
    mt3620_hif_fifo_read(WRDR0, buffer, rx_size);
    ret = mt3620_hif_fw_download_CmdAddressLenReq_event_check(buffer);
    return err;
}




int32_t mt3620_hif_pda_download(uint32_t dest_addr, uint32_t data_len, uint32_t data_mode, uint32_t data_offset, uint8_t *image)
{
    int32_t ret;

    ret = mt3620_hif_pda_config(dest_addr, data_len, data_mode);


    if (image != NULL)
    {
        ret = mt3620_hif_fw_download_Tx_Scatter(&(image[data_offset]), data_len);
    }
    return ret;
}

void my_init_wifi_profile(void)
{
    int32_t err;

    memset(&g_wifi_profile, 0, sizeof(g_wifi_profile));




    g_wifi_profile.OpMode = 0;
    g_wifi_profile.CountryRegion = 5;
    g_wifi_profile.CountryRegionABand = 3;
    g_wifi_profile.CountryCode[0] = 84;
    g_wifi_profile.CountryCode[1] = 87;
    g_wifi_profile.CountryCode[2] = 0;
    g_wifi_profile.CountryCode[3] = 51;
    g_wifi_profile.RadioOff = 0;
    g_wifi_profile.DbgLevel = 3;
    g_wifi_profile.RTSThreshold[0] = 0x2B;
	g_wifi_profile.RTSThreshold[1] = 0x9;
    g_wifi_profile.FragThreshold[0] = 0x2A;
	g_wifi_profile.FragThreshold[1] = 0x9;
    g_wifi_profile.STA_LocalAdminMAC = 1;
    g_wifi_profile.STA_IpAddr[0] = 10;      // g_sta_own_addr
    g_wifi_profile.STA_IpAddr[1] = 10;
    g_wifi_profile.STA_IpAddr[2] = 10;
    g_wifi_profile.STA_IpAddr[3] = 22;
    g_wifi_profile.STA_MacAddr[0] = 0;
    g_wifi_profile.STA_MacAddr[1] = 0xc;
    g_wifi_profile.STA_MacAddr[2] = 0x43;
    g_wifi_profile.STA_MacAddr[3] = 0x76;

    g_wifi_profile.STA_MacAddr[4] = 0x87;
    g_wifi_profile.STA_MacAddr[5] = 0x25;

    g_wifi_profile.STA_Ssid[0] = 'I';
    g_wifi_profile.STA_Ssid[1] = 'O';
    g_wifi_profile.STA_Ssid[2] = 'T';
    g_wifi_profile.STA_Ssid[3] = '_';
    g_wifi_profile.STA_Ssid[4] = 'S';
    g_wifi_profile.STA_Ssid[5] = 'T';
    g_wifi_profile.STA_Ssid[6] = 'A';
    g_wifi_profile.STA_SsidLen = 7;
    g_wifi_profile.STA_BssType = 1;
    g_wifi_profile.STA_Channel = 1;
    g_wifi_profile.STA_BW = 0;
    g_wifi_profile.STA_WirelessMode = 9;
    g_wifi_profile.STA_BADecline = 0;
    g_wifi_profile.STA_AutoBA = 1;
    g_wifi_profile.STA_HT_MCS = 33;
    g_wifi_profile.STA_HT_BAWinSize = 64;
    g_wifi_profile.STA_HT_GI = 1;
    g_wifi_profile.STA_HT_PROTECT = 1;
    g_wifi_profile.STA_HT_EXTCHA = 1;
    g_wifi_profile.STA_WmmCapable = 1;
    g_wifi_profile.STA_ListenInterval = 1;
    g_wifi_profile.STA_AuthMode = 0;
    g_wifi_profile.STA_EncrypType = 1;
    g_wifi_profile.STA_WpaPsk[0] = 49;
    g_wifi_profile.STA_WpaPsk[1] = 50;
    g_wifi_profile.STA_WpaPsk[2] = 51;
    g_wifi_profile.STA_WpaPsk[3] = 52;
    g_wifi_profile.STA_WpaPsk[4] = 53;
    g_wifi_profile.STA_WpaPsk[5] = 54;
    g_wifi_profile.STA_WpaPsk[6] = 55;
    g_wifi_profile.STA_WpaPsk[7] = 56;
    g_wifi_profile.STA_WpaPskLen = 8;
    g_wifi_profile.STA_PMK[0] = 48;
    g_wifi_profile.STA_PMK[1] = 0;
    g_wifi_profile.STA_PMK[2] = 48;
    g_wifi_profile.STA_PMK[3] = 59;
    g_wifi_profile.STA_PMK[4] = 48;
    g_wifi_profile.STA_PMK[5] = 59;
    g_wifi_profile.STA_PMK[6] = 49;
    g_wifi_profile.STA_PMK[7] = 59;
    g_wifi_profile.STA_PMK[8] = 97;
    g_wifi_profile.STA_PMK[9] = 97;
    g_wifi_profile.STA_PMK[10] = 97; 
    g_wifi_profile.STA_PMK[11] = 97;
    g_wifi_profile.STA_PMK[12] = 97;
    g_wifi_profile.STA_PMK[13] = 97;
    g_wifi_profile.STA_PMK[14] = 97;
    g_wifi_profile.STA_PMK[15] = 97;
    g_wifi_profile.STA_PMK[16] = 97;
    g_wifi_profile.STA_PMK[17] = 97;
    g_wifi_profile.STA_PMK[18] = 97;
    g_wifi_profile.STA_PMK[19] = 97;
    g_wifi_profile.STA_PMK[20] = 97;
    g_wifi_profile.STA_PMK[21] = 44;
    g_wifi_profile.STA_PMK[22] = 98;
    g_wifi_profile.STA_PMK[23] = 98;
    g_wifi_profile.STA_PMK[24] = 98;
    g_wifi_profile.STA_PMK[25] = 98;
    g_wifi_profile.STA_PMK[26] = 98;
    g_wifi_profile.STA_PMK[27] = 98;
    g_wifi_profile.STA_PMK[28] = 98;
    g_wifi_profile.STA_PMK[29] = 98;
    g_wifi_profile.STA_PMK[30] = 98;
    g_wifi_profile.STA_PMK[31] = 98;
    g_wifi_profile.STA_PairCipher = 0;
    g_wifi_profile.STA_GroupCipher = 0;
    g_wifi_profile.STA_DefaultKeyId = 1;
    g_wifi_profile.STA_PSMode = 0;
    g_wifi_profile.STA_KeepAlivePeriod = 10;
    
    g_wifi_profile.AP_LocalAdminMAC = 1;
    g_wifi_profile.AP_IpAddr[0] = 10;
    g_wifi_profile.AP_IpAddr[1] = 10;
    g_wifi_profile.AP_IpAddr[2] = 10;
    g_wifi_profile.AP_IpAddr[3] = 22;
    g_wifi_profile.AP_MacAddr[0] = 0x00;
    g_wifi_profile.AP_MacAddr[1] = 0x0C;
    g_wifi_profile.AP_MacAddr[2] = 0x43;
    g_wifi_profile.AP_MacAddr[3] = 0x76;
    g_wifi_profile.AP_MacAddr[4] = 0x87;
    g_wifi_profile.AP_MacAddr[5] = 0x22;
    g_wifi_profile.AP_Ssid[0] = 'I';
    g_wifi_profile.AP_Ssid[1] = 'O';
    g_wifi_profile.AP_Ssid[2] = 'T';
    g_wifi_profile.AP_Ssid[3] = '_';
    g_wifi_profile.AP_Ssid[4] = 'A';
    g_wifi_profile.AP_Ssid[5] = 'P';
    g_wifi_profile.AP_SsidLen = 6;
    g_wifi_profile.AP_Channel = 1;
    g_wifi_profile.AP_BW = 0;
    g_wifi_profile.AP_WirelessMode = 9;
    g_wifi_profile.AP_AutoBA = 1;
    g_wifi_profile.AP_HT_MCS = 33;
    g_wifi_profile.AP_HT_BAWinSize = 64;
    g_wifi_profile.AP_HT_GI = 1;
    g_wifi_profile.AP_HT_PROTECT = 1;
    g_wifi_profile.AP_HT_EXTCHA = 1;
    g_wifi_profile.AP_WmmCapable = 1;
    g_wifi_profile.AP_DtimPeriod = 1;
    g_wifi_profile.AP_HideSSID = 0;
    g_wifi_profile.AP_AutoChannelSelect = 0;
    g_wifi_profile.AP_AuthMode = 0;
    g_wifi_profile.AP_EncrypType = 1;
    g_wifi_profile.AP_WpaPsk[0] = 49;
    g_wifi_profile.AP_WpaPsk[1] = 50;
    g_wifi_profile.AP_WpaPsk[2] = 51;
    g_wifi_profile.AP_WpaPsk[3] = 52;
    g_wifi_profile.AP_WpaPsk[4] = 53;
    g_wifi_profile.AP_WpaPsk[5] = 54;
    g_wifi_profile.AP_WpaPsk[6] = 55;
    g_wifi_profile.AP_WpaPsk[7] = 56;
    g_wifi_profile.AP_WpaPskLen = 8;
    g_wifi_profile.STA_PMK[0] = 48;
    g_wifi_profile.STA_PMK[1] = 0;
    g_wifi_profile.STA_PMK[2] = 48;
    g_wifi_profile.STA_PMK[3] = 59;
    g_wifi_profile.STA_PMK[4] = 48;
    g_wifi_profile.STA_PMK[5] = 59;
    g_wifi_profile.STA_PMK[6] = 49;
    g_wifi_profile.STA_PMK[7] = 59;
    g_wifi_profile.STA_PMK[8] = 49;
    g_wifi_profile.STA_PMK[9] = 49;
    g_wifi_profile.STA_PMK[10] = 49; 
    g_wifi_profile.STA_PMK[11] = 49;
    g_wifi_profile.STA_PMK[12] = 49;
    g_wifi_profile.STA_PMK[13] = 44;
    g_wifi_profile.STA_PMK[14] = 50;
    g_wifi_profile.STA_PMK[15] = 50;
    g_wifi_profile.STA_PMK[16] = 50;
    g_wifi_profile.STA_PMK[17] = 50;
    g_wifi_profile.STA_PMK[18] = 50;
    g_wifi_profile.STA_PMK[19] = 44;
    g_wifi_profile.STA_PMK[20] = 51;
    g_wifi_profile.STA_PMK[21] = 51;
    g_wifi_profile.STA_PMK[22] = 51;
    g_wifi_profile.STA_PMK[23] = 51;
    g_wifi_profile.STA_PMK[24] = 51;
    g_wifi_profile.STA_PMK[25] = 44;
    g_wifi_profile.STA_PMK[26] = 52;
    g_wifi_profile.STA_PMK[27] = 52;
    g_wifi_profile.STA_PMK[28] = 52;
    g_wifi_profile.STA_PMK[29] = 52;
    g_wifi_profile.STA_PMK[30] = 52;
    g_wifi_profile.STA_PMK[31] = 59;    
    g_wifi_profile.AP_PairCipher = 0;
    g_wifi_profile.AP_GroupCipher = 0;
    g_wifi_profile.AP_DefaultKeyId = 1;
    g_wifi_profile.AP_Bcn_disEn = 0;

    g_wifi_profile.bg_band_entry_num = 1;
    g_wifi_profile.bg_band_triple[0].FirstChannel = 1;
    g_wifi_profile.bg_band_triple[0].NumOfCh = 14;
    g_wifi_profile.a_band_entry_num = 2;
    g_wifi_profile.a_band_triple[0].FirstChannel = 52;
    g_wifi_profile.a_band_triple[0].NumOfCh = 3;
    g_wifi_profile.a_band_triple[0].ChannelProp = 1;
    g_wifi_profile.a_band_triple[1].FirstChannel = 149;
    g_wifi_profile.a_band_triple[1].NumOfCh = 4;
    g_wifi_profile.a_band_triple[1].ChannelProp = 0;
    g_wifi_profile.forwarding_zero_copy = 0;
    
    err = mt3620_hif_pda_download(WIFI_PROFILE_ADDR, WIFI_PROFILE_LEN, WIFI_PROFILE_DATA_MODE, 0, (unsigned char *)(&g_wifi_profile));
    if (err)
    {
       // log_hal_fatal( "\n\n\n<<%s>> Send wifi profile download fail!\n\n\n", __FUNCTION__);
       dev_err(g_pmt3620_hif_proc->dev,"\n\n\n<<%s>> Send wifi profile download fail!\n\n\n", __FUNCTION__);
    }
}

int mt3620_hif_get_ilm_image_info_from_flash(fw_dl_data_t *output, uint32_t fw_length)
{

    uint32_t length = 0;

    length = *((uint32_t *)&(output->image[0]));

	if (length > fw_length) {
		return -EINVAL;
	}

    n9_fw_info = (fw_image_tailer_t *)&(output->image[length-sizeof(fw_image_tailer_t)]);

    dev_info(g_pmt3620_hif_proc->dev,"ilm_addr = %08x, ilm_len = %x, feature_set = %d.\n",
        (unsigned int)n9_fw_info->ilm_info.addr,
        (unsigned int)n9_fw_info->ilm_info.len,
        (int)n9_fw_info->ilm_info.feature_set);
    dev_info(g_pmt3620_hif_proc->dev,"dlm_addr = %08x, dlm_len = %x, feature_set = %d.\n",
        (unsigned int)n9_fw_info->dlm_info.addr,
        (unsigned int)n9_fw_info->dlm_info.len,
        (int)n9_fw_info->dlm_info.feature_set);
	dev_info(g_pmt3620_hif_proc->dev,"N9 RAM build date: %s\n", n9_fw_info->ilm_info.ram_built_date);

    output->ilm_addr = n9_fw_info->ilm_info.addr;
    output->ilm_len = n9_fw_info->ilm_info.len;

    output->ilm_encrypt = ((n9_fw_info->ilm_info.feature_set) & FEATURE_MASK_ENCRYPT_MODE) & 0xFF;
    output->ilm_encrypt_key_index= ((n9_fw_info->ilm_info.feature_set) & FEATURE_MASK_KEY_INDEX) & 0xFF;

    output->dlm_addr = n9_fw_info->dlm_info.addr;
    output->dlm_len = n9_fw_info->dlm_info.len;
    output->dlm_encrypt = ((n9_fw_info->dlm_info.feature_set) & FEATURE_MASK_ENCRYPT_MODE) & 0xFF;
    output->dlm_encrypt_key_index= ((n9_fw_info->dlm_info.feature_set) & FEATURE_MASK_KEY_INDEX) & 0xFF;

	return 0;
}


void mt3620_hif_fw_download_done(void)
{
    uint32_t counter = 0;
    uint32_t tx_len, cmd_len;
    uint32_t value, offset;
    P_INIT_CMD_WIFI_START p_data;
    uint8_t wait_tx_done, wait_rx_done;
    uint8_t buffer[SDIO_MAX_RW_SIZE];
    uint32_t real_rx_len, valid_rx_len;
    int32_t ret;

    memset(buffer, 0, SDIO_MAX_RW_SIZE);

    // FW Start command
    cmd_len = sizeof(INIT_HIF_TX_HEADER_T) + sizeof(INIT_CMD_WIFI_START);
    tx_len = LEN_SDIO_TX_AGG_WRAPPER(cmd_len);

    memset(buffer, 0, tx_len);

    // fill in the txd
    offset = mt3620_hif_create_inband_cmd_txd_rom(cmd_len, buffer, MT_FW_START_REQ);

    // fill in the parameters
    p_data = (P_INIT_CMD_WIFI_START)(buffer + offset);
    p_data->u4Address = 0;
    p_data->u4Override = 0;

	// send to dut
    dev_info(g_pmt3620_hif_proc->dev,"<<%s>> Send FW_START command ...\n", __FUNCTION__);
    // write fw_download cmd to dut
    ret = mt3620_hif_fifo_write(buffer, tx_len);
    if (ret != 0)
    {
        // error cuures
        return;
    }


    // check tx done and rx0 done interrupt status
    wait_tx_done = 1;
    wait_rx_done = 1;
    counter = 0;
    while (1)
    {
        ret = mt3620_hif_cr_read(WHISR, &value);
        if (ret)
        {
            dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Read WHISR failed. Error = %d.\n", __FUNCTION__, (int)ret);
            return;
        }

        //log_hal_info( "<<%s>> WHISR = %08x.\n", __FUNCTION__, value);

        mt3620_hif_abnormal_interrupt_check(value);


        if (wait_tx_done == 1)
        {
            if (value & TX_DONE_INT)
            {
                wait_tx_done = 0;
            }
        }

        if (wait_rx_done == 1)
        {
            if (value & RX0_DONE_INT)
            {
                wait_rx_done = 0;
            }
        }

        if ((wait_tx_done == 0) && (wait_rx_done == 0))
        {
            break;
        }

        if (counter > 20000*2) // wait for at least 2 second as hash calculation need more time
        {
            dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Check interrupt failed. wait_tx_done = %d, wait_tx_done = %d.\n",
                __FUNCTION__,
                (int)wait_tx_done,
                (int)wait_rx_done);
            return;
        }
        udelay(50); //delay 50us
        

        counter++;
    }


    // Rx FW start response
    dev_dbg(g_pmt3620_hif_proc->dev,"<<%s>> Rx fw download event...\n", __FUNCTION__);
    ret = mt3620_hif_cr_read(WRPLR, &value);
    if (ret)
    {
        dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Read WRPLR failed. Error = %d.\n", __FUNCTION__, (int)ret);
        return;
    }

    valid_rx_len = LEN_FW_DOWNLOAD_EVENT;
    real_rx_len = valid_rx_len;

    if (GET_RX0_PACKET_LENGTH(value) != real_rx_len)
    {
        dev_err(g_pmt3620_hif_proc->dev,"<<%s>> FW start response length incoreect. WRPLR = %08x.\n",
            __FUNCTION__,
            (unsigned int)value);
        return;
    }

    dev_info(g_pmt3620_hif_proc->dev,"<<%s>> Receive FW_START event ...\n", __FUNCTION__);

  	ret =  mt3620_hif_fifo_read(WRDR0, buffer, real_rx_len);
    
    if (ret != 0)
    {
        dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Read WRDR0 failed. Error = %d.\n", __FUNCTION__, (int)ret);
        return;
    }

    // check event status
    if (mt3620_hif_fw_download_CmdAddressLenReq_event_check(buffer) != HIF_STATUS_SUCCESS)
    {
        return;
    }

    // polling function ready bit.
    counter = 0;
    while (1)
    {
        ret = mt3620_hif_cr_read(WCIR, &value);
        if (ret)
        {
            dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Read WCIR failed. Error = %d.\n", __FUNCTION__, (int)ret);
            return;
        }

        dev_dbg(g_pmt3620_hif_proc->dev,"<<%s>> WCIR = %08x.\n", __FUNCTION__, (unsigned int)value);

        if (value & W_FUNC_RDY)
        {
            break;
        }

        if (counter > 20000) // wait for at least 1 second
        {
            dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Check interrupt failed. wait_tx_done = %d, wait_tx_done = %d.\n",
                __FUNCTION__,
                (int)wait_tx_done,
                (int)wait_rx_done);
            return;
        }
        udelay(50); //delay 50us

        counter++;
    }
    dev_info(g_pmt3620_hif_proc->dev,"<<%s>> FW Download Successfully.\n", __FUNCTION__);
}




void mt3620_hif_fw_download_body(fw_dl_data_t *data)
{
    uint32_t total, len, data_mode = 0;
    unsigned char *ptr = NULL;

    data_mode = (((data->dlm_encrypt) << DATA_MODE_BIT_SHFT_ENCRYPT_MODE) & DATA_MODE_MASK_ENCRYPT_MODE)
        | (((data->dlm_encrypt_key_index) << DATA_MODE_BIT_SHFT_KEY_INDEX) & DATA_MODE_MASK_KEY_INDEX)
        | ((data->dlm_encrypt == 1) ? ((1 << DATA_MODE_BIT_SHFT_RESET_IV) & DATA_MODE_MASK_RESET_IV):(0))
        | ((1 << DATA_MODE_BIT_SHFT_NEED_ACK) & DATA_MODE_MASK_NEED_ACK);


    /* ILM download */
    ptr = &(data->image[4]);    /* skip total image size */

    if(mt3620_hif_pda_config(data->ilm_addr, data->ilm_len, data_mode) != 0) {
        return;
    }

    /* transmission */
    total = 0;
    while(total < data->ilm_len) {
        len = *((unsigned int *)ptr);

        if(mt3620_hif_fw_download_Tx_Scatter(ptr + PDA_PKT_HDR_SIZE, len-PDA_PKT_HDR_SIZE) != 0) {
            return;
        }

        total += (len-PDA_PKT_HDR_SIZE);
        ptr += len; // len includes 3DW header
    }

    /* DLM download */
    if(mt3620_hif_pda_config(data->dlm_addr, data->dlm_len, data_mode) != 0) {
        return;
    }

    /* transmission */
    total = 0;
    while(total < data->dlm_len) {
        len = *((unsigned int *)ptr);

        if(mt3620_hif_fw_download_Tx_Scatter(ptr + PDA_PKT_HDR_SIZE, len-PDA_PKT_HDR_SIZE) != 0) {
            return;
        }

        total += (len-PDA_PKT_HDR_SIZE);
        ptr += len; // len includes 3DW header
    }

    mt3620_hif_fw_download_done();
    
    return;

}



void mt3620_hif_fw_download()
{
    fw_dl_data_t data;
	uint32_t wifi_address;
	uint32_t wifi_length;

	int ret = azure_sphere_sm_get_n9_firmware_location(&wifi_address, &wifi_length); 
	if (ret != 0) {
		dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Failed to get Wi-Fi firmware location. Error = %d.\n", __FUNCTION__, (int)ret);
		return;
	}

    data.image = ioremap(wifi_address, wifi_length);
    if (mt3620_hif_get_ilm_image_info_from_flash(&data, wifi_length) != 0) {
		dev_err(g_pmt3620_hif_proc->dev,"<<%s>> Failed to parse Wi-Fi firmware. Error = %d.\n", __FUNCTION__, (int)ret);
		return;
	}

    // add 4 byte CRC len
    data.ilm_len += LEN_4_BYTE_CRC;
    data.dlm_len += LEN_4_BYTE_CRC;

    dev_info(g_pmt3620_hif_proc->dev, "ilm_addr = %08x, ilm_len = %x, encryt = %d, key_index = %d.\n", data.ilm_addr, data.ilm_len, data.ilm_encrypt, data.ilm_encrypt_key_index);
    dev_info(g_pmt3620_hif_proc->dev, "dlm_addr = %08x, dlm_len = %x.\n", data.dlm_addr, data.dlm_len);

    mt3620_hif_fw_download_body((fw_dl_data_t *)&data);
}

EXPORT_SYMBOL(mt3620_hif_fw_download);


