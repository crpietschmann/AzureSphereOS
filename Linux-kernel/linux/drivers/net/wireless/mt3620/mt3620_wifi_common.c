// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 Wi-Fi driver vendor-specific commands
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

#ifdef CONFIG_MT3620_WIFI_DEBUG

// Set DEBUG to 1 to enable debug log output
#define DEBUG 1

#endif

#include <linux/inetdevice.h>
#include <linux/kernel.h>
#include <mt3620/mt3620_hifapi.h>
#include <net/cfg80211.h>
#include <net/netlink.h>

#include "mt3620_wifi_common.h"

// Disconnects from Wi-Fi if currently connected
void mt3620_wifi_disconnect_if_connected(struct mt3620_wifi_vif *vif)
{
	if (vif->port == WIFI_PORT_AP) {
		// No-op on AP mode
		return;
	}

	if (vif->state != CONNECTED) {
		// Only disconnect if currently connected
		return;
	}

	// Send disconnect event
	netif_stop_queue(vif->ndev);

	dev_info(g_wifi_hw->wifi->dev, "WiFi disconnecting (2) from '%.*s' network in channel %d",
		vif->mode_data.sta.ssid_len, vif->mode_data.sta.ssid, vif->channel);
	cfg80211_disconnected(vif->ndev, 0, NULL, 0, false, GFP_KERNEL);
	if (g_wifi_hw->ibss_start) {
		g_wifi_hw->ibss_start = false;
	}
	vif->state = DISCONNECTED;

	// Blank out BSSID / QoS
	memset(vif->mode_data.sta.bssid, 0, ETH_ALEN);
	vif->mode_data.sta.qos = false;

	netif_carrier_off(vif->ndev);
}

///
/// Helper for setting the device MAC address
///
/// @dev - net device
/// @mac_addr - ETH_ALEN-sized array containing MAC address
/// @port - The interface number to set MAC for
/// @returns - 0 on success

int mt3620_wifi_set_mac_helper(struct mt3620_wifi_hw *wifi_hw,
			       struct mt3620_wifi_set_mac_addr *mac)
{
	int ret = SUCCESS;

	if (mac == NULL) {
		return -EINVAL;
	}

	ret = mt3620_hif_api_send_command_to_n9_sync(
	    wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_OWN_MAC_ADDRESS,
	    /* is set operation */ true, mac, sizeof(*mac), NULL, 0);

	return ret;
}
