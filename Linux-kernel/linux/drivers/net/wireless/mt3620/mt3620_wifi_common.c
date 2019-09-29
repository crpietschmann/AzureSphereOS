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

// Disconnects Wi-Fi upper layer if currently connected or if forcing a disconnect
void mt3620_wifi_disconnect_cfg80211(struct mt3620_wifi_vif *vif, bool force_disconnect)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);

    if (vif->port == WIFI_PORT_AP) {
        // No-op on AP mode
        return;
    }

    if (vif->state == DISCONNECTED) {
        return;
    }

    // Force disconnect is used in cases where the N9 firmware layer could be in
    // an inconsistent state with the upper layer wi-fi software (like error condtions
    // or partial connections -- association succeeds but connection ends up failing).
    if (!force_disconnect) {
        // return if we are connecting unless it is a force disconnect 
        if (vif->state == CONNECTING) {
            dev_info(wifi_hw->wifi->dev, "WiFi - ignoring disconnect while connecting");
            return;
        }
    }

    // Send disconnect event
    netif_stop_queue(vif->ndev);

    dev_info(wifi_hw->wifi->dev, "WiFi cfg80211 disconnecting from '%.*s' network in channel %d, bssid: %pM, force: %d",
        vif->mode_data.sta.ssid_len, vif->mode_data.sta.ssid, vif->channel, vif->mode_data.sta.bssid, force_disconnect ? 1 : 0);

    cfg80211_disconnected(vif->ndev, 0, NULL, 0, false, GFP_KERNEL);
    if (wifi_hw->ibss_start) {
        wifi_hw->ibss_start = false;
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
