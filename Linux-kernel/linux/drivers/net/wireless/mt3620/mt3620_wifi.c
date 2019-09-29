// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 Wi-Fi driver
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

#include <linux/device.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/inetdevice.h>
#include <net/cfg80211.h>
#include <mt3620/mt3620_hifapi.h>
#include <linux/delay.h>
#include <net/netlink.h>
#include <azure-sphere/pluton_remoteapi.h>

#include "mt3620_wifi_common.h"
#include "vendor.h"
#include "regulatory.h"

#define DRIVER_NAME "mt3620-wifi"

// Error code that comes back from the N9 when a scan is in progress
#define WIFI_SCAN_IN_PROGRESS 3
// WIFI_MAX_CONNECTING_TIME_MSEC is a timeout value for the Wi-Fi CONNECTING state.
// Any connection attempts made within this timeout will return EBUSY. Any attempts 
// afterwards will send a disconnect to the N9 firmware and attempt the new connection.
// Note, the timeout is only referenced in the mt3620_wifi_connect call, so a connection
// attempt may continue after this timeout if no further mt3620_wifi_connect calls are
// made.
#define WIFI_MAX_CONNECTING_TIME_MSEC (10*1000) // 10 seconds

struct mt3620_tx_desc *g_mt3620_txdesc;

bool apcli_start  = false;
bool apcli_ap_start = false;
extern uint32_t mt3620_use_dma;

struct mt3620_wifi_hw *g_wifi_hw = NULL;

static const struct wiphy_wowlan_support mt3620_wowlan_support = {
    .flags = WIPHY_WOWLAN_MAGIC_PKT | WIPHY_WOWLAN_SUPPORTS_GTK_REKEY | WIPHY_WOWLAN_GTK_REKEY_FAILURE,
    .n_patterns = 0
};

/// Gets channel data from a IE payload
///
/// @ie - pointer to IE data
/// @ie_len - length of IE data
/// @returns - channel number or NO_CHANNEL if not found
static int mt3620_get_channel_data(u8 *ie, u32 ie_len)
{
    int ret = NO_CHANNEL;
    const u8 *current_ie = NULL;

    // Case 1 - if it has a DS IE then byte 1 is length and byte 2 is the
    // channel number
    current_ie = cfg80211_find_ie(WLAN_EID_DS_PARAMS, ie, ie_len);
    if (current_ie && current_ie[1] == 1) {
        ret = current_ie[2];
    } else {
        // Case 2 - HT Operation data has the channel number for HT
        // connections
        current_ie =
            cfg80211_find_ie(WLAN_EID_HT_OPERATION, ie, ie_len);
        if (current_ie &&
            current_ie[1] >= sizeof(struct ieee80211_ht_operation)) {
            struct ieee80211_ht_operation *htop =
                (void *)(current_ie + 2);

            ret = htop->primary_chan;
        }
        else
        {
            ret = ie[ie_len+1];
        }
    }

    return ret;
}

///
/// Gets the virtual interface for a given interface number
///
/// @wifi_hw - wifi HW d1_data_present
/// @interface - interface ID
/// @returns - interface pointer
static struct mt3620_wifi_vif *
get_vif_for_interface(struct mt3620_wifi_hw *wifi_hw, u8 interface)
{
    switch (interface) {
    case WIFI_PORT_STA:
        return wifi_hw->sta_vif;
    case WIFI_PORT_AP:
        return wifi_hw->ap_vif;
    case WIFI_PORT_IBSS:
         return wifi_hw->ibss_vif;
    default:
        dev_err(wifi_hw->wifi->dev, "Unsupported interface ID: %hhu",
            interface);
        BUG();
    }

    return NULL;
}

///
/// Handles incoming RX data from the N9
///
/// @inf - interface number
/// @skb - data buffer
static void mt3620_wifi_data_received_handler(u8 inf, struct sk_buff *skb)
{
    struct mt3620_wifi_vif *vif = NULL;
    if(g_wifi_hw->ibss_start == true)
        inf = WIFI_PORT_IBSS;
    vif = get_vif_for_interface(g_wifi_hw, inf);

    // bump stats
    vif->net_stats.rx_packets++;
    vif->net_stats.rx_bytes += skb->len;

    // At this point data + sizeof(ethhdr) must be 4 byte aligned
    if (!IS_ALIGNED((u32)(skb->data + sizeof(struct ethhdr)), 4)) {
        dev_err(vif->dev, "RX buffer is not properly aligned");

        // Drop the packet
        vif->net_stats.rx_dropped++;

        dev_kfree_skb(skb);
        return;
    }

    if (vif->state == DISCONNECTED) {
        // drop it
        vif->net_stats.rx_dropped++;

        dev_kfree_skb(skb);
        return;
    }

    skb->protocol = eth_type_trans(skb, vif->ndev);
    skb->dev = vif->ndev;

#ifdef CONFIG_MT3620_WIFI_PACKET_DEBUG
    print_hex_dump_bytes("mt3620 wifi rx: ", DUMP_PREFIX_OFFSET, skb->data,
                 skb->len);
#endif

    netif_rx_ni(skb);
}

///
/// Handles TX buffer update
///
/// @tx_space - free TX buffer space
static void mt3620_wifi_tx_update_handler(u32 free_tx_space)
{
    int inf;

    // There is a shared tx buffer for both interfaces, so both
    // must be throttled at the same time.
    for (inf = WIFI_PORT_STA; inf <= WIFI_PORT_AP; ++inf) {
        struct mt3620_wifi_vif *vif = get_vif_for_interface(g_wifi_hw, inf);
        if (vif->state == CONNECTED) {
                // if there's less not enough space for a full packet (MTU-sized,
                // plus N9 header length), and extra reserved space for a command,
                // throttle netif queue.
                // Otherwise, resume.
            if (free_tx_space > vif->ndev->mtu + N9_DATA_HEADER_LEN +
                QOS_RESERVED_SIZE + N9_TX_RESERVED_SPACE) {
                if (netif_queue_stopped(vif->ndev)) {
                    netif_wake_queue(vif->ndev);
                }
            } else {
                if (!netif_queue_stopped(vif->ndev)) {
                    netif_stop_queue(vif->ndev);
                }
            }
        }
    }
}

///
/// Handles incoming probe request events from the N9
///
/// @event - event ID
/// @status - event status
/// @inf - interface number
/// @data - data buffer
/// @data_size - buffer length
static void mt3620_wifi_probe_req_handler(enum n9_events event, u8 status,
                      u8 inf, void *data, u32 data_size)
{
    u32 freq = 0;
    struct mt3620_wifi_vif *vif = get_vif_for_interface(g_wifi_hw, inf);

    freq = m3620_wifi_channel_to_frequency(vif->channel);

    // Pass on MGMT packet
    cfg80211_rx_mgmt(&vif->wdev, freq, 0, data, data_size, 0);
}

///
/// Handles incoming scan list events from the N9
///
/// @event - event ID
/// @status - event status
/// @inf - interface number
/// @data - data buffer
/// @data_size - buffer length
static void mt3620_wifi_scan_list_handler(enum n9_events event, u8 status,
                      u8 inf, void *data, u32 data_size)
{
    int channel_num = NO_CHANNEL;
    u32 freq;
    s8 rssi;
    struct ieee80211_channel *channel;
    struct cfg80211_bss *bss;
    struct ieee80211_mgmt *mgmt = data;
    int ie_len;

    // We must have a valid 802.11 header and probe response + 2 bytes added
    // by the N9 on the end
    if (data_size < offsetof(struct ieee80211_mgmt, u.probe_resp.variable) +
                PROBE_RESP_N9_DATA_SIZE) {
        // Invalid data
        dev_err(g_wifi_hw->wifi->dev,
            "Invalid data size in scan list event: %#x", data_size);
        return;
    }

    // 2 bytes less than normal because RSSI is at the end
    ie_len = data_size -
         offsetof(struct ieee80211_mgmt, u.probe_resp.variable) -
         PROBE_RESP_N9_DATA_SIZE;

    // Get channel data
    channel_num =
        mt3620_get_channel_data(mgmt->u.probe_resp.variable, ie_len);

    if (channel_num == NO_CHANNEL) {
        dev_err(g_wifi_hw->wifi->dev,
            "Could not find channel for scan list packet");
        return;
    }

    freq = m3620_wifi_channel_to_frequency(channel_num);
    channel = ieee80211_get_channel(g_wifi_hw->wifi->wiphy, freq);

    if (channel == NULL) {
        dev_err(g_wifi_hw->wifi->dev,
            "Could not find channel for scan list frequency");
        return;
    }

    // Driver jams RSSI after the end of the frame
    rssi = ((s8 *)data)[data_size - PROBE_RESP_N9_DATA_SIZE];

    bss = cfg80211_inform_bss_frame(g_wifi_hw->wifi->wiphy, channel, mgmt,
                    data_size - 2, DBM_TO_MBM(rssi),
                    GFP_KERNEL);
    if (bss == NULL) {
        dev_err(g_wifi_hw->wifi->dev, "Could not parse BSS beacon");
        return;
    }

    cfg80211_put_bss(g_wifi_hw->wifi->wiphy, bss);

    dev_dbg(g_wifi_hw->wifi->dev, "Scan returned BSS: %pM", bss->bssid);
}

///
/// Process a scan completed event
///
/// @vif - interface for event
static void mt3620_wifi_scan_completed_event(struct mt3620_wifi_vif *vif)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);

    dev_info(vif->dev, "Scan completed");

    // Mark scan as completed
    if (wifi_hw->scan_request != NULL) {
        mutex_lock(&wifi_hw->scan_mutex);
        if (wifi_hw->scan_request != NULL) {
            struct cfg80211_scan_info info = {
                .aborted = false,
            };
            cfg80211_scan_done(wifi_hw->scan_request, &info);
            wifi_hw->scan_request = NULL;
        }
        mutex_unlock(&wifi_hw->scan_mutex);
    }
    if (wifi_hw->connect_scan != NULL) {
        complete_all(wifi_hw->connect_scan);
        wifi_hw->connect_scan = NULL;
    }
    if(vif->scan_in_progress)
    {
        vif->scan_in_progress = false;
        if(vif->state == CONNECTED)
        {
            dev_dbg(vif->dev, "[mt3620_wifi_scan_completed_event] scan_in_progress=0,Wake_all_Queue\n");
            netif_tx_wake_all_queues(vif->ndev);
            #ifdef CONFIG_MT3620_PIO_DATA
                schedule_work(&wifi_hw->wifi->write_work);
            #endif
        }
    }
}

///
/// Process a link up event
///
/// @vif - interface for event
/// @data - payload data
/// @data_size - size of payload
static void mt3620_wifi_linkup_event(struct mt3620_wifi_vif *vif,
                     void *data, u32 data_size)
{
    const u8 *data_buffer = data;
    u8 qos = 0;

    if (vif->port == WIFI_PORT_AP) {
        // Nothing to do in AP mode
        if(apcli_start == true)
            apcli_ap_start = true;
        return;
    }

    // sanity check size
    // MAC address + 1 byte for QoS enabled
    if (data_size < ETH_ALEN + 1) {
        dev_err(g_wifi_hw->wifi->dev,
            "Invalid size for IW_ASSOC_EVENT_FLAG: %#x", data_size);
        return;
    }

    // Set QoS bit, if specified by the driver
    qos = data_buffer[ETH_ALEN];

    if (qos != 0) {
        vif->mode_data.sta.qos = true;
    }

    dev_dbg(g_wifi_hw->wifi->dev, "Link up - QoS: %hhu", qos);
}
///
/// Process an association event
///
/// @vif - interface for event
/// @data - payload data
/// @data_size - size of payload
static void mt3620_wifi_associated_event(struct mt3620_wifi_vif *vif,
                     void *data, u32 data_size)
{
    // We've associated with an AP, we'll start getting auth packets so set
    // the network as online
    u8 *bssid = NULL;
    const u8 *ie_data = NULL;
    struct cfg80211_bss *bss = NULL;
    struct ieee80211_channel *channel = NULL;
    bool is_roaming = false;
    u8 channel_num = NO_CHANNEL;
    u32 freq = 0;
    u8 qos = 0;

    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);

    if (vif->port == WIFI_PORT_AP) {
        // This is the start of a station connect event.
        // We don't have enough info to finish it yet but if the connect
        // succeeds
        // We'll get a follow up ASSOCREQ or REASSOCREQ event where we
        // register
        // the new station.
        dev_dbg(wifi_hw->wifi->dev, "Station associated");
        return;
    }

    dev_dbg(wifi_hw->wifi->dev, "Associated");

    // sanity check size
    if (data_size < ETH_ALEN) {
        dev_err(wifi_hw->wifi->dev,
            "Invalid size for IW_ASSOC_EVENT_FLAG: %#x", data_size);
        return;
    }

    // Copy BSSID
    bssid = data;
    memcpy(vif->mode_data.sta.bssid, bssid, ETH_ALEN);

    // In IBSS mode, if data size includes QOS byte, then set vif explicity.
    // Otherwise, leave it as false.
    if (wifi_hw->ibss_start && data_size > ETH_ALEN) {
        qos = bssid[ETH_ALEN];
        if (qos != 0) {
            vif->mode_data.sta.qos = true;
        }
    }
    // Get channel data
    channel_num = vif->channel;


    freq = m3620_wifi_channel_to_frequency(channel_num);
    channel = ieee80211_get_channel(wifi_hw->wifi->wiphy, freq);
    if (!channel) {
        dev_err(wifi_hw->wifi->dev,
            "Error getting channel from freq: %u channel: %hhu",
            freq, channel_num);
        return;
    }

    // Check for QoS info in beacon
    bss = cfg80211_get_bss(wifi_hw->wifi->wiphy, channel, bssid,
                   vif->mode_data.sta.ssid,
                   vif->mode_data.sta.ssid_len,
                   IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ANY);

    if (bss != NULL) {
        // If either WMM or HT info is present we're QoS capable
        if (bss != NULL && bss->beacon_ies != NULL) {
            ie_data = cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT,
            WLAN_OUI_TYPE_MICROSOFT_WMM, 
            bss->beacon_ies->data, 
            bss->beacon_ies->len);
        }
        else if (bss->proberesp_ies != NULL){
            ie_data = cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT,
            WLAN_OUI_TYPE_MICROSOFT_WMM,
            bss->proberesp_ies->data,
            bss->proberesp_ies->len);
        }

        if (ie_data != NULL) {
            dev_dbg(wifi_hw->wifi->dev, "BSS has WMM info");
            vif->mode_data.sta.qos = true;
        }
        if(bss->beacon_ies != NULL){
            ie_data = cfg80211_find_ie(WLAN_EID_HT_CAPABILITY,
            bss->beacon_ies->data,
            bss->beacon_ies->len);
        }
        else if (bss->proberesp_ies != NULL){
            ie_data = cfg80211_find_ie(WLAN_EID_HT_CAPABILITY,
            bss->proberesp_ies->data,
            bss->proberesp_ies->len);
        }

        if (ie_data != NULL) {
            dev_dbg(wifi_hw->wifi->dev, "BSS has HT info");
            vif->mode_data.sta.qos = true;
        }
    }

    dev_dbg(wifi_hw->wifi->dev, "WMM/QoS enabled AP: %u", 
                (u32)vif->mode_data.sta.qos);

    // state flags
    is_roaming = vif->state == CONNECTED;
    vif->state = CONNECTED;

    // add BSS if not in the current scan table
    if (bss == NULL) {
        u8 ie_buf[vif->mode_data.sta.ssid_len + 2];

        ie_buf[0] = WLAN_EID_SSID;
        ie_buf[1] = vif->mode_data.sta.ssid_len;
        memcpy(&ie_buf[2], vif->mode_data.sta.ssid,
               vif->mode_data.sta.ssid_len);

        // Note - we don't have RSSI and beacon interval so we lie about
        // them.  The next scan will fix them up
        if(vif->port == WIFI_PORT_IBSS){
            dev_dbg(wifi_hw->wifi->dev,"%s Informing IBSS_Joined\n",__FUNCTION__);
            bss = cfg80211_inform_bss(wifi_hw->wifi->wiphy, channel,
                      CFG80211_BSS_FTYPE_UNKNOWN, bssid, 0,
                      WLAN_CAPABILITY_IBSS, 100, ie_buf,
                      ARRAY_SIZE(ie_buf), 0, GFP_KERNEL);
        }else{
            bss = cfg80211_inform_bss(wifi_hw->wifi->wiphy, channel,
                      CFG80211_BSS_FTYPE_UNKNOWN, bssid, 0,
                      WLAN_CAPABILITY_ESS, 100, ie_buf,
                      ARRAY_SIZE(ie_buf), 0, GFP_KERNEL);
        }
    }

    if(vif->port == WIFI_PORT_IBSS)
    {
        dev_dbg(wifi_hw->wifi->dev, "%s Sending IBSS_Joined\n",__FUNCTION__);
        cfg80211_ibss_joined(vif->ndev, bssid, channel, GFP_KERNEL);
    }
    else
    {
    if (is_roaming) {
        cfg80211_roamed(
            vif->ndev, channel, bssid, vif->mode_data.sta.req_ie,
            vif->mode_data.sta.req_ie_len, vif->mode_data.sta.resp_ie,
            vif->mode_data.sta.resp_ie_len, GFP_KERNEL);
    } else {
        cfg80211_connect_result(
            vif->ndev, bssid, vif->mode_data.sta.req_ie,
            vif->mode_data.sta.req_ie_len, vif->mode_data.sta.resp_ie,
            vif->mode_data.sta.resp_ie_len, WLAN_STATUS_SUCCESS,
            GFP_KERNEL);
    }
    }

    cfg80211_put_bss(wifi_hw->wifi->wiphy, bss);

    dev_info(wifi_hw->wifi->dev, "Connected to BSS: %pM RSSI: %d Frequency: %d", bssid, bss->signal, bss->channel->center_freq);

    // Turn on interface
    netif_carrier_on(vif->ndev);
    netif_wake_queue(vif->ndev);
}

///
/// Process a disassociate or deauth event
///
/// @vif - interface for event
/// @data - event data buffer
/// @data_size - size of data buffer
static void mt3620_wifi_disassociate_event(struct mt3620_wifi_vif *vif,
                      void *data, u32 data_size)
{
    u8 *bssid = NULL;

    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);

    // AP mode
    if (vif->port == WIFI_PORT_AP) {
        // sanity check size
        if (data_size < ETH_ALEN) {
            dev_err(wifi_hw->wifi->dev,
                "Invalid size for IW_ASSOC_EVENT_FLAG: %#x",
                data_size);
            return;
        }

        // Read BSSID
        bssid = data;

        // Indicate station is disconnected
        cfg80211_del_sta(vif->ndev, bssid, GFP_KERNEL);

        dev_dbg(wifi_hw->wifi->dev, "Station disconnected: %pM",
            bssid);
    }else if (vif->port == WIFI_PORT_STA ||  vif->port == WIFI_PORT_IBSS) {
        // Station mode
        mt3620_wifi_disconnect_cfg80211(vif, false);
    }
}

///
/// Process association request IEs event
///
/// @vif - interface for event
/// @data - event data buffer
/// @data_size - size of data buffer
/// @reported_ie_len - reported size of IEs
static void mt3620_wifi_req_ie_event(struct mt3620_wifi_vif *vif, void *data,
                     u32 data_size, u32 reported_ie_len)
{
    u8 *ie_data;
    int ie_len;

    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);

    // sanity check size
    if (data_size < reported_ie_len) {
        dev_err(wifi_hw->wifi->dev,
            "Invalid size for RT_REQ_IE_EVENT_FLAG: %#x",
            data_size);
        return;
    }

    ie_len = reported_ie_len - REQ_IE_OFFSET_SIZE;
    if (ie_len < 0) {
        dev_err(wifi_hw->wifi->dev,
            "Invalid IE len for RT_REQ_IE_EVENT_FLAG: %i", ie_len);
        return;
    }

    ie_data = data + REQ_IE_OFFSET_SIZE;

    // Free old data first if needed
    if (vif->mode_data.sta.req_ie != NULL) {
        devm_kfree(vif->dev, vif->mode_data.sta.req_ie);
    }

    // Copy data
    vif->mode_data.sta.req_ie_len = ie_len;
    vif->mode_data.sta.req_ie = devm_kzalloc(
        wifi_hw->wifi->dev, vif->mode_data.sta.req_ie_len, GFP_KERNEL);
    memcpy(vif->mode_data.sta.req_ie, ie_data,
           vif->mode_data.sta.req_ie_len);
}

///
/// Process association response IEs event
///
/// @vif - interface for event
/// @data - event data buffer
/// @data_size - size of data buffer
/// @reported_ie_len - reported size of IEs
static void mt3620_wifi_resp_ie_event(struct mt3620_wifi_vif *vif, void *data,
                      u32 data_size, u32 reported_ie_len)
{
    u8 *ie_data;
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);

    // sanity check size
    if (data_size < reported_ie_len) {
        dev_err(wifi_hw->wifi->dev,
            "Invalid size for RT_REQ_IE_EVENT_FLAG: %#x",
            data_size);
        return;
    }

    ie_data = data;

    // Free old data first if needed
    if (vif->mode_data.sta.resp_ie != NULL) {
        devm_kfree(vif->dev, vif->mode_data.sta.resp_ie);
    }
    // Copy data
    vif->mode_data.sta.resp_ie_len = reported_ie_len;
    vif->mode_data.sta.resp_ie = devm_kzalloc(
        wifi_hw->wifi->dev, vif->mode_data.sta.resp_ie_len, GFP_KERNEL);
    memcpy(vif->mode_data.sta.resp_ie, ie_data,
           vif->mode_data.sta.resp_ie_len);
}

static void mt3620_wifi_resp_mic_event(struct mt3620_wifi_vif *vif)
{
    cfg80211_michael_mic_failure(vif->ndev, vif->mode_data.sta.bssid,
        (vif->is_mac_set ? NL80211_KEYTYPE_GROUP :
            NL80211_KEYTYPE_PAIRWISE), 0, NULL,
            GFP_KERNEL);
    
}

///
/// Process IWEVCUSTOM wifi events
///
/// @vif - interface for event
/// @event_flag - flag for what custom event has been trigerred
/// @data - event data buffer
/// @data_size - size of data buffer
/// @ie_size - size of IE data in the payload
static void mt3620_wifi_custom_event_handler(struct mt3620_wifi_vif *vif,
                         u32 event_flag, void *data,
                         u32 data_size, u32 ie_size)
{
    switch (event_flag) {
    case IW_STA_LINKDOWN_EVENT_FLAG:
        // Normally we'll already be disconnected at this point
        // via disassociate events, but just in case let's handle this
        mt3620_wifi_disconnect_cfg80211(vif, false);
        break;
    case IW_STA_LINKUP_EVENT_FLAG:
        // We've finished establishing a working link, check QoS flags
        mt3620_wifi_linkup_event(vif, data, data_size);
        break;
    case IW_STA_SCAN_COMPLETED_EVENT_FLAG:
        mt3620_wifi_scan_completed_event(vif);
        break;
    case IW_ASSOC_EVENT_FLAG:
        mt3620_wifi_associated_event(vif, data, data_size);
        break;
    case IW_DISASSOC_EVENT_FLAG:
    case IW_DEAUTH_EVENT_FLAG:
        // Disassociate and Deauth and link down are all handled the same
        mt3620_wifi_disassociate_event(vif, data, data_size);
        break;
    case RT_REQ_IE_EVENT_FLAG:
        mt3620_wifi_req_ie_event(vif, data, data_size, ie_size);
        break;
    case RT_RESP_IE_EVENT_FLAG:
        mt3620_wifi_resp_ie_event(vif, data, data_size, ie_size);
        break;
    case IW_MIC_ERROR_EVENT_FLAG:
        mt3620_wifi_resp_mic_event(vif);
        break;
    case RT_ASSOC_EVENT_FLAG:
    case RT_ASSOCINFO_EVENT_FLAG:
    case RT_INTERFACE_UP:
    case RT_INTERFACE_DOWN:
    case RT_DISASSOC_EVENT_FLAG:
        // This are all informative, no action needed
        break;
    default:
        dev_err(g_wifi_hw->wifi->dev,
            "Unsupported Wi-Fi custom flag: %#x", event_flag);
    }
}

///
/// Process AP mode assocation events
///
/// @vif - interface for event
/// @event_flag - flag for what custom event has been trigerred
/// @data - event data buffer
/// @data_size - size of data buffer
/// @ie_size - reported size of IE data
static void mt3620_wifi_ap_assocaite_ie_handler(struct mt3620_wifi_vif *vif,
                        u32 event_flag, void *data,
                        u32 data_size, u32 ie_size)
{
    struct station_info sinfo;
    u8 *ie_data;
    u8 *mac_addr;
    u8 ie_offset;
    int ie_len;

    // sanity check size
    if (data_size < ie_size) {
        dev_err(g_wifi_hw->wifi->dev,
            "Invalid size for IWEVASSOCREQIE: %#x", data_size);
        return;
    }

    if (event_flag == IWEVREASSOCREQIE) {
        // MAC, AP BSS, then IEs
        ie_offset = ETH_ALEN + ETH_ALEN;
    } else {
        // MAC, then IEs
        ie_offset = ETH_ALEN;
    }

    ie_len = ie_size - ie_offset;
    if (ie_len < 0) {
        dev_err(g_wifi_hw->wifi->dev,
            "Invalid IE len for IWEVASSOCREQIE: %i", ie_len);
        return;
    }

    mac_addr = data;
    ie_data = data + ie_offset;

    memset(&sinfo, 0, sizeof(sinfo));

    sinfo.generation = vif->mode_data.ap.generation++;
    sinfo.assoc_req_ies = ie_data;
    sinfo.assoc_req_ies_len = ie_len;

    cfg80211_new_sta(vif->ndev, mac_addr, &sinfo, GFP_KERNEL);

    netif_wake_queue(vif->ndev);

    dev_dbg(vif->dev, "New station connected: %pM IE len: %i", mac_addr,
        ie_len);
}

///
/// Handles incoming Wi-Fi events from the N9
///
/// @event - event ID
/// @status - event status
/// @inf - interface number
/// @data - data buffer
/// @data_size - buffer length
static void mt3620_wifi_event_handler(enum n9_events event, u8 status, u8 inf,
                      void *data, u32 data_size)
{
    union iwreq_data *req_data = data;
    struct mt3620_wifi_vif *vif = NULL;
    u32 cmd = 0;
    void *payload;
    u32 payload_size;

    if(g_wifi_hw->ibss_start == true)
    {
        inf = WIFI_PORT_IBSS;
    }

    vif = get_vif_for_interface(g_wifi_hw, inf);

    if (data_size < sizeof(union iwreq_data) + sizeof(cmd)) {
        dev_err(g_wifi_hw->wifi->dev,
            "Invalid size for WLAN event: %#x", data_size);
        return;
    }

    // Cmd is at the end
    cmd = *(u32 *)(data + sizeof(union iwreq_data));

    // Payload is after cmd
    payload = (void *)(data + sizeof(union iwreq_data) + sizeof(cmd));
    payload_size = data_size - sizeof(union iwreq_data) - sizeof(cmd);

    switch (cmd) {
    case IWEVCUSTOM:
        // Custom events with lots of sub types, process in sub method
        mt3620_wifi_custom_event_handler(vif, req_data->data.flags,
                         payload, payload_size,
                         req_data->data.length);
        break;
    case IWEVASSOCREQIE:
    case IWEVREASSOCREQIE:
        // A station has connected or reconnected - process the
        // connection
        mt3620_wifi_ap_assocaite_ie_handler(
            vif, cmd, payload, payload_size, req_data->data.length);
        break;
    case IWEVDISASSOC:
        // Diassociate event - AP MAC address change. This is an event to indicate that AP MAC address has changes to all zeros 
        // as we have disassociated.
        // NO-OP because the firmware sends two IWEVCUSTOM events before this:
        // 1: IWEVCUSTOM with IW_STA_LINKDOWN_EVENT_FLAG
        // 2: IWEVCUSTOM with IW_DISASSOC_EVENT_FLAG
        // The indication to upper layers in the OS network stack happens in the handling of the above two custom IEs.
        dev_dbg(g_wifi_hw->wifi->dev, "Received Disassoc event - %#x",
            cmd);    
        break;
    default:
        dev_err(g_wifi_hw->wifi->dev, "Unsupported Wi-Fi event: %#x",
            cmd);
        break;
    }
}

///
/// Downloads Wifi Firmware
///
/// @wifi - driver data structure
/// @returns - 0 for success
static int mt3620_initialize_firmware(struct mt3620_wifi *wifi)
{

#ifdef CONFIG_MT3620_WIFI_PACKET_DEBUG
    u8 dbg_level = N9_DEBUG_LEVEL_LOUD;
#endif

    dev_info(wifi->dev, "Starting Wi-Fi firmware download");

      mt3620_hif_fw_download();

#ifndef    CONFIG_MT3620_PIO_DATA
    mt3620_use_dma = MT3620_DMA_ENABLE;

#endif

    dev_info(wifi->dev, "Wi-Fi firmware ready\n");

    //Enabling the Interrupt
    mt3620_hif_enable_interrupt();

    return SUCCESS;
}

// Event handlers for N9 events
static struct mt3620_hif_api_n9_event_handlers mt3620_wifi_event_handlers = {
    .data_received = mt3620_wifi_data_received_handler,
    .probe_req = mt3620_wifi_probe_req_handler,
    .scan_list = mt3620_wifi_scan_list_handler,
    .wlan_event = mt3620_wifi_event_handler,
    .tx_update = mt3620_wifi_tx_update_handler,
};

#ifdef CONFIG_MT3620_PIO_DATA

static void mt3620_tx_desc_init(struct mt3620_wifi *wifi)
{
    u8 i = 0;
    wifi->mt3620_txdesc.get = 0;
    wifi->mt3620_txdesc.put = 0;
    wifi->mt3620_txdesc.msg_count = 0;

    for(i = 0; i < MEM_POOL_MAX_SIZE; i++)
    {
        wifi->mt3620_txdesc.ar[i] =    NULL;
    }

    g_mt3620_txdesc = &wifi->mt3620_txdesc;
}

static int mt3620_add_tx_desc(void *ptr,struct mt3620_wifi *wifi)
{
    int ret = SUCCESS;
    uint32_t current_index = 0;

    if(wifi->mt3620_txdesc.msg_count == MEM_POOL_MAX_SIZE)
    {
        return -1;
    }

    wifi->mt3620_txdesc.msg_count++;
    current_index = wifi->mt3620_txdesc.put;
    wifi->mt3620_txdesc.put = ((wifi->mt3620_txdesc.put + 1) % MEM_POOL_MAX_SIZE);
    wifi->mt3620_txdesc.ar[current_index] = (u32)ptr;
    return ret;
}



static void* mt3620_remove_tx_desc(struct mt3620_wifi *wifi)
{
    void *ptr = NULL;
    uint32_t current_index = 0;

    if(wifi->mt3620_txdesc.msg_count == 0)
    {
        return NULL;
    }

   current_index = wifi->mt3620_txdesc.get;
   wifi->mt3620_txdesc.msg_count--;
   wifi->mt3620_txdesc.get = ((wifi->mt3620_txdesc.get+1) % MEM_POOL_MAX_SIZE);
   ptr = (void *)wifi->mt3620_txdesc.ar[current_index];
   wifi->mt3620_txdesc.ar[current_index] = NULL;

    return (void *)ptr;
}
#endif

///
/// Driver callback for hardware startup
///
/// @wifi_hw - hardware device
/// @vif - interface that is starting
/// @returns - 0 for success
static int mt3620_wifi_start(struct mt3620_wifi_hw *wifi_hw,
                 struct mt3620_wifi_vif *vif)
{
    int ret;
    u8 onoff = 1;
    u8 mode = WIFI_MODE_STA_ONLY;
    if (wifi_hw->num_vif != 0) {
        // Already running, don't support live reconfiguration yet
        dev_err(vif->dev, "Driver doesn't support live "
                  "reconfiguration, take interfaces down "
                  "first");
        return -EINVAL;
    }

    if (vif->port == WIFI_PORT_STA) {
        mode = WIFI_MODE_STA_ONLY;
    } else if (vif->port == WIFI_PORT_AP) {
        if(apcli_start == true)
            mode = WIFI_MODE_DUAL;
        else
            mode = WIFI_MODE_AP_ONLY;
    } else if (vif->port == WIFI_PORT_IBSS) {
        mode = WIFI_MODE_STA_ONLY;
    } else {
        WARN(true, "Unsupported port type: %u, defaulting to station",
             vif->port);
    }

    dev_info(wifi_hw->wifi->dev, "Registering N9_Event Handlers in STA mode\n");

    mt3620_hif_api_register_n9_event_handlers(
        &mt3620_wifi_event_handlers);


    if (!wifi_hw->n9_initialized) {
        // Start wifi FW download
        ret = mt3620_initialize_firmware(wifi_hw->wifi);
        if (ret != SUCCESS) {
            goto exit;
        }

        wifi_hw->n9_initialized = true;
    }

    // Set opmode
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SWITCH_MODE, true,
        &mode, sizeof(u8), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error setting STA mode: %#x", ret);
        goto exit;
    }
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_RADIO_ON_OFF,
        true, &onoff, sizeof(u8), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error setting radio on: %#x", ret);
        goto exit;
    }

exit:
    return ret;
}

///
/// Driver callback for hardware shutdown
///
/// @wifi_hw - hardware device
static void mt3620_wifi_stop(struct mt3620_wifi_hw *wifi_hw)
{
    int ret;
    u8 onoff = 0;

    // If num_vif == 0 then the last interface has just been stopped and
    // there are
    // no running interfaces.  Shut down the radio in this case to save
    // power.
    if (wifi_hw->num_vif == 0) {
        if (wifi_hw->scan_request != NULL) {
            // abort scan
            ret = mt3620_hif_api_send_command_to_n9_sync(
                wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_STOP_SCAN,
                true, NULL, 0, NULL, 0);
            if (ret != SUCCESS) {
                dev_err(wifi_hw->wifi->dev, "Error stopping scan: %#x", ret);
            }
            mt3620_wifi_scan_completed_event(wifi_hw->sta_vif);
            dev_dbg(wifi_hw->wifi->dev, "WiFi abort scan complete\n");
        }

        // Turn off radio
        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_RADIO_ON_OFF, true, &onoff,
            sizeof(u8), NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev,
                "Error setting radio off: %#x", ret);
        }

        mt3620_hif_api_unregister_n9_event_handlers();
    }
}

///
/// Called when the user attempts to change the MAC of an interface
///
/// @dev - interface device
/// @addr - mac address info
/// @returns - 0 for success
static int mt3620_wifi_set_mac_address(struct net_device *dev, void *addr)
{
    struct mt3620_wifi_vif *vif = netdev_priv(dev);
    int ret = SUCCESS;

    if (vif->is_mac_set) {
        // We can only set the mac in HW once
        return -ENOENT;
    }

    // Set the value
    ret = eth_mac_addr(dev, addr);
    if (ret != SUCCESS) {
        return ret;
    }

    dev_dbg(vif->dev, "MAC address set to: %pM", dev->dev_addr);

    return SUCCESS;
}

///
/// Notifies the driver of a new regulatory domain
///
/// @wiphy - wiphy device
/// @request - regulatory info
static void mt3620_wifi_reg_notifier(struct wiphy *wiphy,
                     struct regulatory_request *request)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    u8 ret;

    dev_dbg(wifi_hw->wifi->dev, "Reg domain update: %.2s by %s",
        request->alpha2, reg_initiator_name(request->initiator));

    // We can only apply if we are up and running.
    // We ignore requests for the core as it sets us back to world mode,
    // just to set the user value later on
    // which is disruptive since changing the setting resets our connection.
    if (wifi_hw->n9_initialized &&
        request->initiator != NL80211_REGDOM_SET_BY_CORE) {
        ret = mt3620_wifi_apply_country_code_regulatory_rules();
        if (ret != SUCCESS) {
           dev_err(wifi_hw->wifi->dev,
               "Error applying regulatory domain update: %i",
               ret);
        }
    }
}

///
/// Validates that the character is one of the allowed Alphanumeric characters for country codes
///
/// @alpha2 - The country code character
/// @returns - true for valid value, false for not a valid alpha2
static bool is_valid_country_alpha2_value(char alpha2)
{
    return ((alpha2 >= 'A' && alpha2 <= 'Z') || (alpha2 >= '0' && alpha2 <= '9'));
}

///
/// Checks that the input country code is valid
///
/// @country_code - The country code in two char alpha2 format
/// @returns - true for valid value, false for not a valid country code
static bool is_validate_country_code_data(const char *country_code)
{
    bool ret = is_valid_country_alpha2_value(country_code[0]) &&
        is_valid_country_alpha2_value(country_code[1]);
    return ret;
}

///
/// Called when our net device is opened
///
/// @dev - net device
/// @returns - 0 for success
static int mt3620_wifi_open(struct net_device *dev)
{
    int ret = SUCCESS;
    int port = WIFI_PORT_STA;
    struct mt3620_wifi_set_mac_addr mac;
    struct mt3620_wifi_vif *vif = netdev_priv(dev);
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);


    // Default country region = World (00)
    char country_code[3] = {'0', '0', '\0'};

    mutex_lock(&wifi_hw->mutex);
    //we are assigning it again to remove the dependency of sequence of add interface function.
    if (vif->port == WIFI_PORT_STA) {
        wifi_hw->sta_vif = vif;
    } else if (vif->port == WIFI_PORT_AP) {
        wifi_hw->ap_vif = vif;
    }else if (vif->port == WIFI_PORT_IBSS) {
        wifi_hw->ibss_vif = vif;
    }

    if (strstr(dev->name, "wlan3"))
    {
        apcli_start = true;
    }

    // Start if needed
    ret = mt3620_wifi_start(wifi_hw, vif);
    if (ret != SUCCESS) {
        if(apcli_start && (vif->port == WIFI_PORT_STA)) {
            dev_info(vif->dev, "staring the STA Interface of APCLI");
        }else{
            dev_err(vif->dev, "Could not start interface: %u", ret);
            goto exit;
        }
    }

    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_IOT_OWN_MAC_ADDRESS, false, &port,
        sizeof(port), mac.mac_addr, sizeof(mac.mac_addr));
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Could not get MAC %u", ret);
        goto exit;
    }

    if (!vif->is_mac_set) {
        // Set mac address with radio
        if(vif->port == WIFI_PORT_IBSS){
            mac.port = WIFI_PORT_STA;
        }else{
            mac.port = vif->port;
        }

        if (is_zero_ether_addr(mac.mac_addr)) {
            memcpy(mac.mac_addr, dev->dev_addr, ETH_ALEN);
        } else {
            memcpy(dev->dev_addr, mac.mac_addr, ETH_ALEN);
        }

        ret = mt3620_wifi_set_mac_helper(wifi_hw, &mac);
        if (ret != SUCCESS) {
            dev_err(vif->dev, "Could not set MAC to %pM: %u",
                dev->dev_addr, ret);
            goto exit;
        }

        vif->is_mac_set = true;
    }

    // Set Regulatory domain info from EFUSE

    ret = mt3620_read_country_code_efuse(wifi_hw->wifi->wiphy, &country_code[0]);
    if(ret != SUCCESS) {
        // Fall back to world if there is an error reading the EFUSE
        dev_err(wifi_hw->wifi->dev, "Could not read Regulatory region from EFUSE %u", ret);
        country_code[0] = '0';
        country_code[1] = '0';
        ret = SUCCESS;
    }

    // If it is not a valid country code, set it to 00 (World region)
    if (!is_validate_country_code_data(country_code))
    {
        dev_err(wifi_hw->wifi->dev,
                "Not a valid Regulatory region from EFUSE {0x%02hhx, 0x%02hhx}",
                country_code[0],
                country_code[1]);
        country_code[0] = '0';
        country_code[1] = '0';
    }

    // The linux stack leaves it to world mode (00) by default if this fails
    ret = regulatory_hint(wifi_hw->wifi->wiphy, country_code);
    if(ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Could not set Regulatory hint to %c%c country %u", country_code[0], country_code[1], ret);
        ret = SUCCESS;
    }

    // This should succeed
    ret = mt3620_wifi_apply_country_code_regulatory_rules();
    if(ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Could not set Regulatory rules %u", ret);
        goto exit;
    }

    vif->enabled = true;

    if (vif->state == CONNECTED) {
        netif_carrier_on(dev);
        netif_wake_queue(dev);
    } else {
        netif_carrier_off(dev);
    }

    wifi_hw->num_vif++;

exit:
    mutex_unlock(&wifi_hw->mutex);
    return ret;
}

///
/// Called when our net device is closed
///
/// @dev - net device
/// @returns - 0 for success
static int mt3620_wifi_close(struct net_device *dev)
{
    struct mt3620_wifi_vif *vif = netdev_priv(dev);
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);
    int ret = SUCCESS;
    
    mutex_lock(&wifi_hw->mutex);

    netif_stop_queue(dev);

    if (vif->port == WIFI_PORT_STA) {
        // Mark us as disconnected
        if (vif->state == CONNECTED) {
            cfg80211_disconnected(dev, 0, NULL, 0, true, GFP_KERNEL);
        } else if (vif->state == CONNECTING) {
            cfg80211_connect_result(
                dev, vif->mode_data.sta.bssid, NULL, 0, NULL, 0,
                WLAN_STATUS_UNSPECIFIED_FAILURE, GFP_KERNEL);
        }
    } else if (vif->port == WIFI_PORT_AP) {
        // Stop AP mode if we're still running
        if (vif->state == CONNECTED) {
            ret = mt3620_hif_api_send_command_to_n9_sync(
                wifi_hw->wifi->hif_api_handle,
                WIFI_COMMAND_ID_EXTENSION_AP_STOP, true, NULL, 0,
                NULL, 0);
            if (ret != SUCCESS) {
                dev_err(vif->dev, "Error stopping AP mode: %u",
                    ret);
            }
        }
        if(apcli_start)
        {
            apcli_start = false;
            apcli_ap_start = false;
        }
    }else if (vif->port == WIFI_PORT_IBSS) {
        if (vif->state == CONNECTED) {
            dev_dbg(vif->dev, "Closing IBSS_2 mode\n");
            cfg80211_disconnected(dev, 0, NULL, 0, true, GFP_KERNEL);
        }
    }

    vif->state = DISCONNECTED;

    netif_carrier_off(dev);
    vif->enabled = false;

    wifi_hw->num_vif--;

    mt3620_wifi_stop(wifi_hw);

    mutex_unlock(&wifi_hw->mutex);
    return SUCCESS;
}

///
/// Gets our network stats
///
/// @dev - net device
/// @returns - stats
static struct net_device_stats *mt3620_wifi_get_stats(struct net_device *dev)
{
    struct mt3620_wifi_vif *vif = netdev_priv(dev);

    return &vif->net_stats;
}

///
/// Sets RX mode
///
/// @dev - net device
static void mt3620_wifi_set_rx_mode(struct net_device *dev)
{
    // Noting to do here - the HW doesn't exposing changing multicast
    // settings and handles
    // things automatically
}

///
/// Sends data
///
/// @skb - data buffer
/// @dev - net device
/// @returns - 0 for success
static int mt3620_wifi_tx(struct sk_buff *skb, struct net_device *dev)
{
    struct mt3620_wifi_vif *vif = netdev_priv(dev);
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);
    u32 reserved_size;
    int ret = SUCCESS;
    u8 port;
    u32 base_size = 0;
    u8 tail_room_required = 0;


    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_set_power_mgmt] n9_initialized not done\n");
        ret = -EAGAIN;
        dev_kfree_skb(skb);
        goto exit;
    }

    // linearize data since we need to copy it
    ret = skb_linearize(skb);
    if (ret != SUCCESS) {
        dev_kfree_skb(skb);
        vif->net_stats.tx_dropped++;
        ret = NETDEV_TX_BUSY;

        goto exit;
    }

    // stats
    vif->net_stats.tx_packets++;
    vif->net_stats.tx_bytes += skb->len;

    if (vif->state == DISCONNECTED) {
        // Drop it
        dev_kfree_skb(skb);
        vif->net_stats.tx_dropped++;
        goto exit;
    }
    if(vif->scan_in_progress){
        dev_kfree_skb(skb);
        dev_dbg(wifi_hw->wifi->dev, "mt3620_wifi_tx scan_in_progress=1, stop_all_queue\n");
        netif_tx_stop_all_queues(dev);
        vif->net_stats.tx_dropped++;
        ret = NETDEV_TX_OK;
        goto exit;
    }

#ifdef CONFIG_MT3620_WIFI_PACKET_DEBUG
    print_hex_dump_bytes("mt3620 wifi tx: ", DUMP_PREFIX_OFFSET, skb->data,
                 skb->len);
#endif
#ifdef CONFIG_MT3620_PIO_DATA
       wifi_hw->wifi->act_vif = vif;
       spin_lock_irqsave(&wifi_hw->wifi->data_lock,flags);
       ret = mt3620_add_tx_desc((void *)skb, wifi_hw->wifi);
       spin_unlock_irqrestore(&wifi_hw->wifi->data_lock,flags);
       schedule_work(&wifi_hw->wifi->write_work);


#else
    // Send data
        if(vif->port == WIFI_PORT_IBSS)
            port = WIFI_PORT_STA;
        else
            port = vif->port;
        if (vif->port == WIFI_PORT_AP) {
        // AP mode is always QoS capable
            reserved_size = QOS_RESERVED_SIZE;
        } else if (vif->mode_data.sta.qos) {
            reserved_size = QOS_RESERVED_SIZE;
        } else {
            reserved_size = NON_QOS_RESERVED_SIZE;
        }

    //check that the sufficient tail room is present or not
    base_size = N9_DATA_HEADER_LEN + reserved_size + skb->len;
    tail_room_required = ALIGN(base_size, N9_TX_BLOCK_SIZE) - base_size;

    if((!(IS_ALIGN_4((uint32_t)skb->data))) && (skb_tailroom(skb) >= tail_room_required))
        ret = mt3620_hif_api_send_data_to_n9(wifi_hw->wifi->hif_api_handle,
                    port, reserved_size,
                    skb, skb->len);
    else

        ret = mt3620_hif_api_send_data_to_n9_aligned(wifi_hw->wifi->hif_api_handle,
                    port, reserved_size,
                    skb->data, skb->len);
#endif

    if (ret != SUCCESS) {
            dev_dbg(vif->dev, "Error sending TX data: %x", ret);
            dev_kfree_skb(skb);
            vif->net_stats.tx_errors++;
            goto exit;
    }
    #ifndef CONFIG_MT3620_PIO_DATA
    else
    {
        dev_kfree_skb(skb);
    }
#endif
    ret = NETDEV_TX_OK;

exit:

    return ret;
}

#ifdef CONFIG_MT3620_PIO_DATA
static void mt3620_tx_bottom_half(struct work_struct *work)
{
    struct sk_buff *skb = NULL;
    struct mt3620_wifi *wifi  = container_of(work, struct mt3620_wifi, write_work);
    struct mt3620_wifi_vif *vif = wifi->act_vif;
    u32 reserved_size, i = 0;
    uint32_t flags = 0;
    int ret = SUCCESS;
    u32 base_size = 0;
    u8 tail_room_required = 0;
    u8 port;

    while(1)
    {
        if(vif->scan_in_progress)
        {
            dev_dbg(g_wifi_hw->wifi->dev, "%s scan_in_progress=1,exit work, pkt pending=%d\n",__FUNCTION__,wifi->mt3620_txdesc.msg_count);
            break;
        }
        spin_lock_irqsave(&wifi->data_lock,flags);
        skb = mt3620_remove_tx_desc(wifi);
        spin_unlock_irqrestore(&wifi->data_lock,flags);
        if(skb == NULL) {
            break;
        }
        // Send data
        if(vif->port == WIFI_PORT_IBSS)
            port = WIFI_PORT_STA;
        else
            port = vif->port;
        if (vif->port == WIFI_PORT_AP) {
        // AP mode is always QoS capable
            reserved_size = QOS_RESERVED_SIZE;
        } else if (vif->mode_data.sta.qos) {
            reserved_size = QOS_RESERVED_SIZE;
        } else {
            reserved_size = NON_QOS_RESERVED_SIZE;
        }

    //check that the sufficient tail room is present or not
    base_size = N9_DATA_HEADER_LEN + reserved_size + skb->len;
    tail_room_required = ALIGN(base_size, N9_TX_BLOCK_SIZE) - base_size;

    if((!(IS_ALIGN_4((uint32_t)skb->data))) && (skb_tailroom(skb) >= tail_room_required))
        ret = mt3620_hif_api_send_data_to_n9(wifi->hif_api_handle,
                    port, reserved_size,
                    skb, skb->len);
    else
        ret = mt3620_hif_api_send_data_to_n9_aligned(wifi->hif_api_handle,
                    port, reserved_size,
                    skb->data, skb->len);

        // Decrement SKB ref count
        dev_kfree_skb(skb);

        if (ret != SUCCESS) {
            dev_dbg(vif->dev,"%s pkt send failed\n",__FUNCTION__);
            vif->net_stats.tx_errors++;
        }
    }
}
#endif

// Network level callbacks
static const struct net_device_ops mt3620_wifi_netdev_ops = {
    .ndo_open = mt3620_wifi_open,
    .ndo_stop = mt3620_wifi_close,
    .ndo_start_xmit = mt3620_wifi_tx,
    .ndo_get_stats = mt3620_wifi_get_stats,
    .ndo_set_mac_address = mt3620_wifi_set_mac_address,
    .ndo_set_rx_mode = mt3620_wifi_set_rx_mode,
};

///
/// Driver callback for interface startup
///
/// @wiphy - wiphy device
/// @name - new interface name
/// @name_assign_type - type of name
/// @type - interface type,
/// @returns - new wireless dev object
static struct wireless_dev *
mt3620_wifi_add_interface(struct wiphy *wiphy, const char *name,
              unsigned char name_assign_type,
              enum nl80211_iftype type)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *wifi_vif;
    struct net_device *ndev;
    struct wireless_dev *ret;
    int status;
    enum mt3620_wifi_vif_port port = WIFI_PORT_STA;

    mutex_lock(&wifi_hw->mutex);

    switch (type) {
    case NL80211_IFTYPE_STATION:
        port = WIFI_PORT_STA;
        break;
    case NL80211_IFTYPE_AP:
        port = WIFI_PORT_AP;
        break;
    case NL80211_IFTYPE_ADHOC:
         port = WIFI_PORT_IBSS;
         break;
    default:
        dev_err(wifi_hw->wifi->dev, "Unsupported VIF type: %#x", type);
        ret = ERR_PTR(-EOPNOTSUPP);
        goto exit;
    }

    // Allocate netdev
    ndev = alloc_netdev(sizeof(*wifi_vif), name, name_assign_type,
                ether_setup);
    if (!ndev) {
        ret = ERR_PTR(-ENOMEM);
        goto exit;
    }

    wifi_vif = netdev_priv(ndev);
    wifi_vif->dev = wifi_hw->wifi->dev;
    ndev->ieee80211_ptr = &wifi_vif->wdev;
    wifi_vif->wdev.wiphy = wiphy;
    wifi_vif->state = DISCONNECTED;
    SET_NETDEV_DEV(ndev, wiphy_dev(wiphy));
    wifi_vif->wdev.netdev = ndev;
    wifi_vif->wdev.iftype = type;
    wifi_vif->ndev = ndev;
    wifi_vif->port = port;

    memset(&wifi_vif->keys, 0, (sizeof(struct mt3620_wifi_key_info)*WLAN_MAX_KEY_COUNT));
    memcpy(ndev->dev_addr, wifi_hw->mac_addresses[port].addr, ETH_ALEN);
    ndev->netdev_ops = &mt3620_wifi_netdev_ops;
    ndev->destructor = free_netdev;
    ndev->watchdog_timeo = 10;
    ndev->needed_headroom = 54;//ETH_HLEN;

    if (port == WIFI_PORT_STA) {
        wifi_hw->sta_vif = wifi_vif;
    } else if (port == WIFI_PORT_AP) {
        wifi_hw->ap_vif = wifi_vif;
    }else if (port == WIFI_PORT_IBSS) {
        wifi_hw->ibss_vif = wifi_vif;
    }

    status = register_netdev(ndev);
    if (status != 0) {
        dev_err(wifi_hw->wifi->dev, "Error registering net device: %#x",
            status);
        ret = ERR_PTR(status);
        goto exit;
    }

    
    dev_dbg(wifi_hw->wifi->dev, "New interface %s with MAC: %pM",
        ndev->name, ndev->dev_addr);

    ret = &wifi_vif->wdev;

exit:
    mutex_unlock(&wifi_hw->mutex);
    return ret;
}

///
/// Starts a scan
///
/// @wifi_hw - wifi hw device
/// @ie - extra IE info for the scan
/// @ie_len - length of ie
/// @channels - array of channel information for the scan
/// @n_channels - number of channels
/// @ssid - ssid for the scan
/// @ssid_len - length of ssid
/// @returns - 0 for success
static int mt3620_wifi_start_scan(struct mt3620_wifi_hw *wifi_hw, const u8 *ie,
                  u8 ie_len,
                  struct ieee80211_channel **channels,
                  u8 n_channels, const u8 *ssid, u8 ssid_len)
{
    int i = 0;
    int current_channel_offset = 0;
    int ret = SUCCESS;
    int extra_ie_len = 0;
    int scan_size = 0;
    u8 apcli_channel = 0;
    u8 num_channels_without_no_ir = n_channels;
    struct mt3620_wifi_extra_ie *extra_ie = NULL;
    struct mt3620_wifi_scan_request *scan = NULL;
    struct mt3620_wifi_channel_desc *bg_channel_desc = NULL;
    struct mt3620_wifi_set_channel set_channel;
    u32 freq;
    const struct ieee80211_reg_rule *reg_rule;

    // Alloc memory
    extra_ie_len = sizeof(struct mt3620_wifi_extra_ie) + ie_len;
    extra_ie = devm_kzalloc(wifi_hw->wifi->dev, extra_ie_len, GFP_KERNEL);

    if (extra_ie == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    // Alloc scan + channel data
    scan_size = sizeof(struct mt3620_wifi_scan_request) +
            (num_channels_without_no_ir * sizeof(struct mt3620_wifi_channel_desc));
    scan = devm_kzalloc(wifi_hw->wifi->dev, scan_size, GFP_KERNEL);
    if (scan == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    // Set IE value
    extra_ie->port = WIFI_PORT_STA;
    extra_ie->type = WIFI_EXTRA_WPS_PROBE_REQ_IE;
    extra_ie->ie_len = ie_len;
    if (ie_len != 0) {
        memcpy(extra_ie->ie, ie, ie_len);

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_SET_EXTRA_IE, true, extra_ie,
            extra_ie_len, NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting IE: %u",
                ret);
            goto exit;
        }
    }

    // Set SSID if present
    if (ssid_len > 0) {
        // We only support one SSID so set the first value
        memcpy(scan->ssid, ssid, ssid_len);
        scan->enable_ssid = (0x80 | ssid_len);
    }

    scan->scan_mode = 0;
    if(apcli_ap_start) {
        set_channel.port = WIFI_PORT_AP;
        set_channel.channel = 0;
        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_CHANNEL, false,
            &set_channel, sizeof(struct mt3620_wifi_set_channel), &apcli_channel, sizeof(u8));
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting channel: %u", ret);
            goto exit;
        }
        dev_dbg(wifi_hw->wifi->dev, "setting apcli_channel: %d", apcli_channel);
        bg_channel_desc = scan->channels.channel;
        bg_channel_desc->first_channel = apcli_channel;
        bg_channel_desc->num_of_channel = 1;
        scan->partial = 1;
        scan->channels.num_bg_band = 1;
        scan->channels.num_a_band = 1;
    } else {
        scan->partial = 0;

        // Build channel list
        scan->channels.num_bg_band = 0;
        scan->channels.num_a_band = 0;

        // Add each channel, 2ghz first then 5ghz
        for (i = 0; i < n_channels; i++) {
            if (channels[i]->band == NL80211_BAND_2GHZ) {
                freq = MHZ_TO_KHZ(m3620_wifi_channel_to_frequency(channels[i]->hw_value));
                reg_rule = freq_reg_info(wifi_hw->wifi->wiphy, freq);

                if (IS_ERR(reg_rule)){
                    dev_err(wifi_hw->wifi->dev, "reg_rule error\n");
                    ret = -EBUSY;
                    goto exit;
                }

                if(reg_rule->flags & NL80211_RRF_NO_IR)	{
                    scan->channels.channel[current_channel_offset].channel_properties = CH_PASSIVE_SCAN; //Passive Scan
                }

                scan->channels.channel[current_channel_offset]
                    .first_channel = channels[i]->hw_value;
                scan->channels.channel[current_channel_offset]
                    .num_of_channel = 1;

                current_channel_offset++;
                scan->channels.num_bg_band++;
            }
        }

        for (i = 0; i < n_channels; i++) {
            if (channels[i]->band == NL80211_BAND_5GHZ) {
                freq = MHZ_TO_KHZ(m3620_wifi_channel_to_frequency(channels[i]->hw_value));
                reg_rule = freq_reg_info(wifi_hw->wifi->wiphy, freq);

                if (IS_ERR(reg_rule)) {
                    dev_err(wifi_hw->wifi->dev, "reg_rule error\n");
                    ret = -EBUSY;
                    goto exit;
                }

                // Do not do active scans on Radar and NO IR channels
                if((reg_rule->flags & NL80211_RRF_DFS)|| (reg_rule->flags & NL80211_RRF_NO_IR)) {
                    scan->channels.channel[current_channel_offset].channel_properties = CH_PASSIVE_SCAN; //Passive Scan
                }

                scan->channels.channel[current_channel_offset]
                    .first_channel = channels[i]->hw_value;
                scan->channels.channel[current_channel_offset]
                    .num_of_channel = 1;
                current_channel_offset++;
                scan->channels.num_a_band++;
            }
        }
    }
    // Send scan request
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_START_SCAN, true,
        scan, scan_size, NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Scan failed: %u", ret);
        // This will fail if a scan is already in progress.  No point reporting
        // that back to the user.
        if (ret != WIFI_SCAN_IN_PROGRESS) {
            dev_err(wifi_hw->wifi->dev, "Scan failed: %u", ret);
        }
        // Mark as busy so the lower layer retries later
        ret = -EBUSY;

        goto exit;
    }

    dev_dbg(wifi_hw->wifi->dev,
        "New scan started: IE len: %hhu Channels: %hhu SSID: %.*s",
        ie_len, n_channels, ssid_len, ssid);

exit:
    if (extra_ie != NULL) {
        devm_kfree(wifi_hw->wifi->dev, extra_ie);
    }
    if (scan != NULL) {
        devm_kfree(wifi_hw->wifi->dev, scan);
    }
    return ret;
}

///
/// Starts a scan
///
/// @wiphy - wiphy device
/// @request - scan parameters
/// @returns - 0 for success
static int mt3620_wifi_scan(struct wiphy *wiphy,
                struct cfg80211_scan_request *request)
{
    int ret = SUCCESS;
    u8 *ssid = NULL;
    u8 ssid_len = 0;
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Scan disabled. Device in RF Testing Mode");
        ret = -ENOSYS;
        goto exit;
    }

    if ((wifi_hw->ap_vif != NULL && wifi_hw->ap_vif->enabled) && !(apcli_ap_start)){
        // When in AP mode scans won't run since we're fixed on a
        // channel - just mark it as completed
        struct cfg80211_scan_info info = {
            .aborted = false,
        };

        cfg80211_scan_done(request, &info);
        ret = SUCCESS;
        goto exit;
    }

    // If there is an outstanding scan, block this
    if ((wifi_hw->scan_request && wifi_hw->scan_request != request) ||
        wifi_hw->connect_scan) {
        // Outstanding request
        // RFAIRFAX - temporary logging to help track down bug 52308
        dev_info(wifi_hw->wifi->dev, "Scan requested but busy");
        ret = -EBUSY;
        goto exit;
    }

    if(!wifi_hw->n9_initialized)
    {
        struct cfg80211_scan_info info = {
            .aborted = false,
        };

        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_scan] n9_initialized not done\n");

        cfg80211_scan_done(request, &info);
        ret = SUCCESS;
        goto exit;
    }

    // Set SSID if present
    if (request->n_ssids > 0 && request->ssids[0].ssid_len > 0) {
        // We only support one SSID so set the first value
        ssid = request->ssids[0].ssid;
        ssid_len = request->ssids[0].ssid_len;
    }

    wifi_hw->scan_request = request;
    //set the scan in progress flag
    wifi_hw->sta_vif->scan_in_progress = true;

    ret = mt3620_wifi_start_scan(wifi_hw, request->ie, request->ie_len,
                     request->channels, request->n_channels,
                     ssid, ssid_len);
    if (ret != SUCCESS) {

        // Clear out scan request since we're not going to process this
        // one
        wifi_hw->scan_request = NULL;
        wifi_hw->sta_vif->scan_in_progress = false;
        if(wifi_hw->sta_vif->state == CONNECTED)
        {
            dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_scan] Scan failed, restart tx queue\n");

            netif_tx_wake_all_queues(wifi_hw->sta_vif->ndev);
            #ifdef CONFIG_MT3620_PIO_DATA
                schedule_work(&wifi_hw->wifi->write_work);
            #endif
        }
        goto exit;
    }

exit:
    return ret;
}

///
/// Callback for N9 WIFI_COMMAND_ID_IOT_STOP_SCAN command.
///
/// @cmd - the stop scan command ID
/// @status - the status for the issued command
/// @data - the callback data
static void mt3620_wifi_scan_abort_callback(u32 cmd, u8 status, void *data)
{
    if (g_wifi_hw != NULL) {
        // Indicate scan complete on abort. In normal operations the N9 firmware will 
        // generate an IW_STA_SCAN_COMPLETED_EVENT_FLAG event when aborting scans, so
        // this event handling will result in a no-op. However, the WPA supplicant has
        // a bug (66130) where a scan is requested while there are no enabled wi-fi
        // profiles, it will ignore the request, but does not change its internal scan
        // state. Therefore, all subsequent scan requests will fail indicating an active
        // scan is already in progress. This will eventually be aborted (after ~30 sec),
        // but the abort will not trigger a IW_STA_SCAN_COMPLETED_EVENT_FLAG from the N9
        // because it isn't actually scanning.
        mt3620_wifi_scan_completed_event(g_wifi_hw->sta_vif);
        dev_dbg(g_wifi_hw->wifi->dev, "WiFi abort scan complete: status %u\n", status);
    }
}

///
/// Aborts a scan
///
/// @wiphy - wiphy device
/// @wdev - wireless device
/// @returns - 0 for success
static void mt3620_wifi_abort_scan(struct wiphy *wiphy, struct wireless_dev *wdev)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);

    if (wifi_hw != NULL && wifi_hw->wifi != NULL) {
        dev_info(wifi_hw->wifi->dev, "WiFi aborting scan\n");

        // abort scan
        mt3620_hif_api_send_command_to_n9_async(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_IOT_STOP_SCAN, true, NULL, 0, mt3620_wifi_scan_abort_callback);
    }
}

///
/// Disconnects from an AP
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @reason_code - disconnect reason
/// @returns - 0 for success
static int mt3620_wifi_disconnect_internal(struct wiphy *wiphy, struct net_device *ndev,
                  u16 reason_code)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    u8 port = vif->port;
    int ret = SUCCESS;

    // check current link state
    if (vif->state == DISCONNECTED) {
        goto exit;
    }

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_disconnect] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    dev_info(wifi_hw->wifi->dev, "WiFi disconnecting from '%.*s' network in channel %d, bssid: %pM",
        vif->mode_data.sta.ssid_len, vif->mode_data.sta.ssid, vif->channel, vif->mode_data.sta.bssid);

    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SET_DISCONNECT, true,
        &port, sizeof(u8), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error disconnecting: %u", ret);
        goto exit;
    }

exit:
    return ret;
}

///
/// Disconnects from an AP
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @reason_code - disconnect reason
/// @returns - 0 for success
static int mt3620_wifi_disconnect(struct wiphy *wiphy, struct net_device *ndev,
                  u16 reason_code)
{
    int ret = SUCCESS;
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);

    bool is_connecting = (vif->state == CONNECTING);

    ret = mt3620_wifi_disconnect_internal(wiphy, ndev, reason_code);

    // Force an upper layer disconnect when we are in the CONNECTING state since we normally
    // ignore disassoc/link down notifications while in this state. Otherwise, the upper layer
    // and the hardware will have different connection states. Note, this function can be called 
    // directly by the kernel prior to any new connection attempts. So not issuing an upper layer
    // disconnect will leave the device in a disconnect loop. 
    if (is_connecting) {
        mt3620_wifi_disconnect_cfg80211(vif, true);
    }

    return ret;
}

///
/// Sets auth settings prior to connect
///
/// @settings - crypto settings
/// @port - N9 port to apply settings on
/// @returns - 0 for success
static int mt3620_wifi_set_auth_mode(struct cfg80211_connect_params *params,
            struct cfg80211_ap_settings *info,enum mt3620_wifi_vif_port port)
{
    struct mt3620_wifi_set_auth_mode set_auth_mode;
    u32 wpa_versions = 0;
    u32 suite = 0;
    int ret = SUCCESS;

    if(g_wifi_hw->in_hqa_mode) {
        dev_err(g_wifi_hw->wifi->dev,
            "Cannot set auth mode. Device in RF Testing Mode");
        return -ENOSYS;
    }

    if(params!= NULL) {
        wpa_versions = params->crypto.wpa_versions;
        suite = params->crypto.akm_suites[0];
    } else {
        wpa_versions = info->crypto.wpa_versions;
        suite = info->crypto.akm_suites[0];
    }

    if ((params!= NULL) && params->auth_type == NL80211_AUTHTYPE_SHARED_KEY)
        set_auth_mode.auth_mode = WIFI_AUTH_SHARED;
    else if (wpa_versions == (NL80211_WPA_VERSION_1 | NL80211_WPA_VERSION_2) ||
             wpa_versions == NL80211_WPA_VERSION_1 ||
             wpa_versions == NL80211_WPA_VERSION_2) {
        switch (suite)
        {
        case WLAN_AKM_SUITE_8021X:
            if (wpa_versions==NL80211_WPA_VERSION_1)
                set_auth_mode.auth_mode = WIFI_AUTH_MODE_WPA;
            else if (wpa_versions==NL80211_WPA_VERSION_2)
                set_auth_mode.auth_mode = WIFI_AUTH_MODE_WPA2;
            else if (wpa_versions==(NL80211_WPA_VERSION_1 | NL80211_WPA_VERSION_2))
                set_auth_mode.auth_mode = WIFI_AUTH_MODE_WPA_PSK_WPA2;
            break;
        case WLAN_AKM_SUITE_PSK:
            if (wpa_versions==NL80211_WPA_VERSION_1)
                set_auth_mode.auth_mode = WIFI_AUTH_MODE_WPA_PSK;
            else if (wpa_versions==NL80211_WPA_VERSION_2)
                set_auth_mode.auth_mode = WIFI_AUTH_MODE_WPA2_PSK;
            else if (wpa_versions==(NL80211_WPA_VERSION_1 | NL80211_WPA_VERSION_2))
                set_auth_mode.auth_mode = WIFI_AUTH_MODE_WPA_PSK_WPA2_PSK;
            break;
        default:
            dev_err(g_wifi_hw->wifi->dev,
                "Unsupported akm_suites=%x\n",suite);
            return -EINVAL;
        }
    } else if (wpa_versions==0)
        set_auth_mode.auth_mode = WIFI_AUTH_MODE_OPEN;
    else {
        dev_err(g_wifi_hw->wifi->dev,
            "Unsupported WPA settings wpa_versions=%x\n",wpa_versions);
        return -EINVAL;
    }
    // Set auth mode
    set_auth_mode.port = port;

    ret = mt3620_hif_api_send_command_to_n9_sync(
        g_wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_SET_AUTH_MODE, true, &set_auth_mode,
        sizeof(struct mt3620_wifi_set_auth_mode), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(g_wifi_hw->wifi->dev, "Error setting auth mode: %u",
            ret);
        return ret;
    }

    dev_dbg(g_wifi_hw->wifi->dev, "Set auth mode to %#hhx",
        set_auth_mode.auth_mode);

    return SUCCESS;
}

///
/// Parse and update encryption flags
///
/// @suite - crypto suite value
/// @modes - modes flag that gets updated
/// @returns - 0 for success
static int mt3620_wifi_update_encryption_mode_flags(u32 suite, u32 *modes)
{
    switch (suite) {
    // WEP40 and WEP104 both map to the same type in the N9 firmware
    case WLAN_CIPHER_SUITE_WEP40:
    case WLAN_CIPHER_SUITE_WEP104:
        *modes |= BIT(WIFI_ENCRYPT_TYPE_WEP);
        break;
    case WLAN_CIPHER_SUITE_TKIP:
        *modes |= BIT(WIFI_ENCRYPT_TYPE_TKIP);
        break;
    case WLAN_CIPHER_SUITE_CCMP:
        *modes |= BIT(WIFI_ENCRYPT_TYPE_AES);
        break;
    case 0:
        *modes |= BIT(WIFI_ENCRYPT_TYPE_NONE);
        break;
    default:
        dev_err(g_wifi_hw->wifi->dev,
            "Unsupported cipher group value: %#x", suite);
        return -EINVAL;
    }

    return SUCCESS;
}

///
/// Sets encryption settings prior to connect
///
/// @settings - crypto settings
/// @port - N9 port to apply settings to
/// @returns - 0 for success
static int
mt3620_wifi_set_encryption_mode(struct cfg80211_crypto_settings *settings,
                enum mt3620_wifi_vif_port port)
{
    struct mt3620_wifi_set_encryption_mode set_encryption_mode;
    int ret = SUCCESS;
    int i = 0;
    u32 modes = 0;

    set_encryption_mode.encryption_mode = WIFI_ENCRYPT_TYPE_NONE;

    // Parse group ciphers
    ret = mt3620_wifi_update_encryption_mode_flags(settings->cipher_group,
                               &modes);
    if (ret != SUCCESS) {
        return ret;
    }

    // Parse pairwise ciphers
    for (i = 0; i < settings->n_ciphers_pairwise; i++) {
        ret = mt3620_wifi_update_encryption_mode_flags(
            settings->ciphers_pairwise[i], &modes);
        if (ret != SUCCESS) {
            return ret;
        }
    }

    // Choose final value
    // Note - order is used to choose the strongest type
    if((modes & BIT(WIFI_ENCRYPT_TYPE_TKIP)) && (modes & BIT(WIFI_ENCRYPT_TYPE_AES))){
        set_encryption_mode.encryption_mode = WIFI_ENCRYPT_TYPE_AES_TKIP;
    } else if (modes & BIT(WIFI_ENCRYPT_TYPE_TKIP)) {
        set_encryption_mode.encryption_mode = WIFI_ENCRYPT_TYPE_TKIP;
    } else if (modes & BIT(WIFI_ENCRYPT_TYPE_AES)) {
        set_encryption_mode.encryption_mode = WIFI_ENCRYPT_TYPE_AES;
    } else if (modes & BIT(WIFI_ENCRYPT_TYPE_WEP)) {
        set_encryption_mode.encryption_mode = WIFI_ENCRYPT_TYPE_WEP;
    } else if (modes & BIT(WIFI_ENCRYPT_TYPE_NONE)) {
        set_encryption_mode.encryption_mode = WIFI_ENCRYPT_TYPE_NONE;
    } else {
        dev_err(g_wifi_hw->wifi->dev,
            "Unsupported encryption mode flags: %#x", modes);
        return -EINVAL;
    }

    // Set encryption mode
    set_encryption_mode.port = port;

    if(g_wifi_hw->in_hqa_mode) {
        dev_err(g_wifi_hw->wifi->dev,
            "Cannot set encryption mode. Device in RF Testing Mode");
        return -ENOSYS;
    }

    ret = mt3620_hif_api_send_command_to_n9_sync(
        g_wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_SET_ENCRYPT_TYPE, true,
        &set_encryption_mode,
        sizeof(struct mt3620_wifi_set_encryption_mode), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(g_wifi_hw->wifi->dev,
            "Error setting encryption mode: %u", ret);
        return ret;
    }

    dev_dbg(g_wifi_hw->wifi->dev, "Set encryption mode to %#hhx",
        set_encryption_mode.encryption_mode);

    return SUCCESS;
}

///
/// Sends complete set of WEP keys to the N9
///
/// @wifi_hw - wifi HW device
/// @vif - virtual interface
/// @returns - 0 for success
static int mt3620_wifi_commit_wep_keys(struct mt3620_wifi_hw *wifi_hw,
                       struct mt3620_wifi_vif *vif)
{
    struct mt3620_wifi_set_wep_keys keys;
    int ret = SUCCESS;
    int i = 0;

    memset(&keys, 0, sizeof(struct mt3620_wifi_set_wep_keys));

    // Send data for all keys
    keys.port = vif->port;
    keys.default_key_index = vif->wep_tx_key_index;
    for (i = 0; i < WLAN_MAX_KEY_COUNT; i++) {
        // Is it a WEP key?
        if (vif->keys[i].cipher == WLAN_CIPHER_SUITE_WEP40 ||
            vif->keys[i].cipher == WLAN_CIPHER_SUITE_WEP104) {

            // Copy data
            memcpy(&keys.keys[i].key, vif->keys[i].key,
                   vif->keys[i].key_len);
            keys.keys[i].key_len = vif->keys[i].key_len;
        }
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Cannot set WEP keys. Device in RF Testing Mode");
        return -ENOSYS;
    }

    // Send message
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_SET_WEP_KEY,
        true, &keys, sizeof(struct mt3620_wifi_set_wep_keys), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error setting WEP keys: %u", ret);
    }

    return ret;
}

///
/// Adds a new WEP key
///
/// @wifi_hw - wifi HW device
/// @vif - virtual interface
/// @key - key data
/// @key_len - key length
/// @key_index - key index
/// @returns - 0 for success
static int mt3620_wifi_add_wep_key(struct mt3620_wifi_hw *wifi_hw,
                   struct mt3620_wifi_vif *vif, const u8 *key,
                   u8 key_len, u8 key_index)
{
    u32 cipher = 0;

    if (key_len == 0) {
        // Delete key
        cipher = 0;
    } else if (key_len == WLAN_KEY_LEN_WEP40) {
        cipher = WLAN_CIPHER_SUITE_WEP40;
    } else if (key_len == WLAN_KEY_LEN_WEP104) {
        cipher = WLAN_CIPHER_SUITE_WEP104;
    } else {
        dev_err(vif->dev, "Invalid WEP key length: %u", key_len);
        return -EINVAL;
    }

    // Update data for our key
    vif->keys[key_index].cipher = cipher;
    vif->keys[key_index].key_len = key_len;
    memcpy(&vif->keys[key_index].key, key, key_len);
    vif->keys[key_index].seq_len = 0;
    memset(&vif->keys[key_index].seq, 0,
           ARRAY_SIZE(vif->keys[key_index].seq));

    // Commit keys
    return mt3620_wifi_commit_wep_keys(wifi_hw, vif);
}

///
/// Connects to an AP
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @params - connect parameters
/// @returns - 0 for success
static int mt3620_wifi_connect(struct wiphy *wiphy, struct net_device *ndev,
                   struct cfg80211_connect_params *params)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_extra_ie *extra_ie = NULL;
    struct mt3620_wifi_set_channel set_channel;
    struct mt3620_wifi_set_bssid set_bssid;
    struct mt3620_wifi_set_ssid set_ssid;
    int ret = SUCCESS;
    int extra_ie_len = 0;
    bool need_disconnect = false;
    struct completion c;
    bool connect_attempted = false;

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_connect] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Cannot connect. Device in RF Testing Mode");
        ret = -ENOSYS;
        goto exit;
    }

    need_disconnect = vif->state != DISCONNECTED;

    if (vif->state == CONNECTING) {
        // Return EBUSY until the connecting timeout elapses, then continue the connection
        // which will disconnect the ongoing connection attempt.
        if (time_before(jiffies, vif->connecting_timeout)) {
            dev_info(wifi_hw->wifi->dev,
                "Connection attempt while connecting, returning EBUSY");
            ret = -EBUSY;
            goto exit;
        }
    } else {
        // Mark this before we disconnect so we don't reset the scan table by
        // indicating a link down event
        vif->state = CONNECTING;
        // Ignore overflow because it is handled by the 'time_before' macro above.
        vif->connecting_timeout = jiffies + msecs_to_jiffies(WIFI_MAX_CONNECTING_TIME_MSEC);
    }

    // Alloc memory
    extra_ie_len = sizeof(struct mt3620_wifi_extra_ie) + params->ie_len;
    extra_ie = devm_kzalloc(wifi_hw->wifi->dev, extra_ie_len, GFP_KERNEL);

    if (extra_ie == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    connect_attempted = true;

    // Disconnect first if needed
    if (need_disconnect) {
        dev_info(wifi_hw->wifi->dev, "Connecting to     : %pM\n", params->bssid);
        dev_info(wifi_hw->wifi->dev, "Disconnecting from: %pM\n", vif->mode_data.sta.bssid);
        mt3620_wifi_disconnect_internal(wiphy, ndev, 0);
    }

    // We need to send a fresh scan targeting the ssid / channel we want to
    // get at so that the scan table on the N9 has the BSSID we're
    // targetting
    if(!apcli_ap_start){
        init_completion(&c);
        wifi_hw->connect_scan = &c;
        vif->scan_in_progress = true;

        ret = mt3620_wifi_start_scan(
            wifi_hw, NULL, 0, params->channel == NULL ? NULL : &params->channel,
            params->channel == NULL ? 0 : 1, params->ssid, params->ssid_len);
        if (ret != SUCCESS) {
            dev_info(wifi_hw->wifi->dev, "Connect scan failed: %u", ret);
            vif->scan_in_progress = false;
            goto exit;
        }

        // We wait for a timeout here to avoid hanging connect
        // Note that this is not fatal if it times out, we may still connect
        // properly anyway so we continue
        wait_for_completion_timeout(&c, msecs_to_jiffies(5000));
        wifi_hw->connect_scan = NULL;
    }

    // Reset our associate IEs
    if (vif->mode_data.sta.req_ie != NULL) {
        devm_kfree(vif->dev, vif->mode_data.sta.req_ie);
        vif->mode_data.sta.req_ie = NULL;
    }
    if (vif->mode_data.sta.resp_ie != NULL) {
        devm_kfree(vif->dev, vif->mode_data.sta.resp_ie);
        vif->mode_data.sta.resp_ie = NULL;
    }

    // Setup encryption mode / auth mode
    ret = mt3620_wifi_set_auth_mode(params,NULL, vif->port);
    if (ret != SUCCESS) {
        goto exit;
    }

    ret = mt3620_wifi_set_encryption_mode(&params->crypto, vif->port);
    if (ret != SUCCESS) {
        goto exit;
    }

    // WEP Key
    if (params->key) {
        vif->wep_tx_key_index = params->key_idx;

        ret = mt3620_wifi_add_wep_key(wifi_hw, vif, params->key,
                          params->key_len, params->key_idx);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting WEP key: %u",
                ret);
            goto exit;
        }
    }

    // Set IE value
    extra_ie->port = vif->port;
    extra_ie->type = WIFI_EXTRA_WPS_ASSOC_REQ_IE;
    extra_ie->ie_len = params->ie_len;
    if (params->ie_len != 0) {
        memcpy(extra_ie->ie, params->ie, params->ie_len);

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_SET_EXTRA_IE, true, extra_ie,
            extra_ie_len, NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting IE: %u %u",
                ret, params->ie_len);
            goto exit;
        }
    }

    // Set channel
    if (params->channel) {
        set_channel.port = vif->port;
        set_channel.channel = params->channel->hw_value;

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_CHANNEL,
            true, &set_channel, sizeof(struct mt3620_wifi_set_channel),
            NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting channel: %u",
                ret);
            goto exit;
        }

        vif->channel = set_channel.channel;

        dev_dbg(wifi_hw->wifi->dev, "Connecting on channel %hhu",
            vif->channel);
    }

    vif->mode_data.sta.ssid_len = params->ssid_len;
    memcpy(vif->mode_data.sta.ssid, params->ssid, params->ssid_len);

    // Use BSSID mode if specified, otherwise fall back to SSID mode

    // Set BSSID
    if (!is_zero_ether_addr(params->bssid) && !apcli_ap_start) {
        set_bssid.port = vif->port;
        memcpy(set_bssid.bssid, params->bssid, ETH_ALEN);

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_BSSID, true,
            &set_bssid, sizeof(struct mt3620_wifi_set_bssid), NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting BSSID: %u",
                ret);
            goto exit;
        }

        dev_dbg(wifi_hw->wifi->dev, "Connection request sent for %pM",
            params->bssid);
    } else {
        // Set SSID
        set_ssid.port = vif->port;
        set_ssid.ssid_len = params->ssid_len;
        memcpy(set_ssid.ssid, params->ssid, params->ssid_len);

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SSID, true,
            &set_ssid, sizeof(struct mt3620_wifi_set_ssid), NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting SSID: %u",
                ret);
            goto exit;
        }

        dev_dbg(wifi_hw->wifi->dev, "Connection request sent for %.*s",
            params->ssid_len, params->ssid);
    }

exit:
    if (extra_ie != NULL) {
        devm_kfree(wifi_hw->wifi->dev, extra_ie);
    }
    if (ret != SUCCESS && connect_attempted) {
        vif->state = DISCONNECTED;
    }

    return ret;
}
static int mt3620_wifi_join_ibss(struct wiphy *wiphy, struct net_device *ndev,
                   struct cfg80211_ibss_params *params)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_extra_ie *extra_ie = NULL;
    struct mt3620_wifi_set_channel set_channel;
    struct mt3620_wifi_set_ssid set_ssid;
    struct ieee80211_channel *channel = params->chandef.chan;
    int chan = -1;
    int ret = SUCCESS;
    int extra_ie_len = 0;
    bool need_disconnect = false;
    struct mt3620_wifi_set_auth_mode set_auth_mode;
    struct mt3620_wifi_set_encryption_mode set_encryption_mode;
    u8 mode = WIFI_MODE_IBSS_ONLY;

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_join_ibss] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        ret = -EBUSY;
        goto exit;
    }

    dev_info(wifi_hw->wifi->dev, "IBSS JOIN/CREATE\n");
    extra_ie_len = sizeof(struct mt3620_wifi_extra_ie) + params->ie_len;
    extra_ie = devm_kzalloc(wifi_hw->wifi->dev, extra_ie_len, GFP_KERNEL);
    if (extra_ie == NULL) {
        ret = -ENOMEM;
        goto exit;
    }
    need_disconnect = vif->state != DISCONNECTED;

    if (channel)
        chan = ieee80211_frequency_to_channel(channel->center_freq);

    // Mark this before we disconnect so we don't reset the scan table by
    // indicating a link down event
    vif->state = CONNECTING;

    // Disconnect first if needed
    if (need_disconnect) {
        mt3620_wifi_disconnect_internal(wiphy, ndev, 0);
    }

    // Set opmode for IBSS
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SWITCH_MODE, true,
        &mode, sizeof(u8), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error setting IBSS mode: %#x", ret);
        goto exit;
    }
    wifi_hw->ibss_start = true;

    if (params->privacy) {
        set_auth_mode.auth_mode = WIFI_AUTH_MODE_OPEN;
        set_encryption_mode.encryption_mode = WIFI_ENCRYPT_TYPE_WEP;
    } else {
        set_auth_mode.auth_mode = WIFI_AUTH_MODE_OPEN;
        set_encryption_mode.encryption_mode = WIFI_ENCRYPT_TYPE_NONE;
    }

    set_auth_mode.port = vif->port ;
    set_encryption_mode.port = vif->port ;

    // Setup encryption mode / auth mode
    ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_SET_AUTH_MODE, true, &set_auth_mode,
            sizeof(struct mt3620_wifi_set_auth_mode), NULL, 0);

    if (ret != SUCCESS) {
    goto exit;
    }

    dev_dbg(wifi_hw->wifi->dev, "Set auth mode to %#hhx",
     set_auth_mode.auth_mode);

    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_SET_ENCRYPT_TYPE, true,
        &set_encryption_mode,
        sizeof(struct mt3620_wifi_set_encryption_mode), NULL, 0);
    if (ret != SUCCESS) {
        goto exit;
    }

    dev_dbg(wifi_hw->wifi->dev, "Set encryption mode to %#hhx",
        set_encryption_mode.encryption_mode);

    dev_dbg(wifi_hw->wifi->dev, "IBSS Sending Channel\n");

    // Set channel
    if (channel) {
        set_channel.port =  0 ;//vif->port;
        set_channel.channel = chan;

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_CHANNEL,
            true, &set_channel, sizeof(struct mt3620_wifi_set_channel),
            NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting channel: %u",
                ret);
            goto exit;
        }
        vif->channel = set_channel.channel;
        dev_dbg(wifi_hw->wifi->dev, "Connecting on channel %hhu",
            vif->channel);
    }

    vif->mode_data.sta.ssid_len = params->ssid_len;
    memcpy(vif->mode_data.sta.ssid, params->ssid, params->ssid_len);

    dev_dbg(wifi_hw->wifi->dev, "IBSS Sending SSID\n");
        // Set SSID
    set_ssid.port = vif->port;
    set_ssid.ssid_len = params->ssid_len;
    memcpy(set_ssid.ssid, params->ssid, params->ssid_len);

    ret = mt3620_hif_api_send_command_to_n9_sync(
          wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SSID, true,
          &set_ssid, sizeof(struct mt3620_wifi_set_ssid), NULL, 0);
    if (ret != SUCCESS){
        dev_err(wifi_hw->wifi->dev, "Error setting SSID: %u",ret);
        goto exit;
    }

    dev_dbg(wifi_hw->wifi->dev, "Connection request sent for %.*s",
        params->ssid_len, params->ssid);

    vif->state = CONNECTED;

exit:
    if (extra_ie != NULL) {
        devm_kfree(wifi_hw->wifi->dev, extra_ie);
    }
    if (ret != SUCCESS) {
        vif->state = DISCONNECTED;
    }
    return ret;
}


/*
========================================================================
Routine Description:
    Leave the IBSS.

Arguments:
    pWiphy            - Wireless hardware description
    pNdev            - Network device interface

Return Value:
    0                - success
    -x                - fail

Note:
    For iw utility: ibss leave
========================================================================
*/
static int mt3620_wifi_leave_ibss(struct wiphy *wiphy, struct net_device *ndev)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    int ret = SUCCESS;
    u8 port = WIFI_PORT_STA;

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_leave_ibss] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        ret = -EBUSY;
        goto exit;
    }

    dev_dbg(wifi_hw->wifi->dev, "Leaving the IBSS\n");

    //Sending Disconnect command
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SET_DISCONNECT, true,
        &port, sizeof(u8), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error disconnecting: %u", ret);
        goto exit;
    }

exit:
    return ret;
}

///
/// Adds an encryption key
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @key_index - key index identifier
/// @pairwise - is the key group or pairwise
/// @mac_addr - target address
/// @params - key parameters
/// @returns - 0 for success
static int mt3620_wifi_add_key(struct wiphy *wiphy, struct net_device *ndev,
                   u8 key_index, bool pairwise, const u8 *mac_addr,
                   struct key_params *params)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_set_wpa_key wpa_key;
    int ret = SUCCESS;

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        return -EBUSY;
    }

    // verify key index
    if (key_index >= WLAN_MAX_KEY_COUNT) {
        dev_err(vif->dev, "Invalid key index: %u", key_index);
        return -EINVAL;
    }

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_add_key] n9_initialized not done\n");
        return -EAGAIN;

    }

    switch (params->cipher) {
    case WLAN_CIPHER_SUITE_WEP40:
    case WLAN_CIPHER_SUITE_WEP104:
        // WEP
        ret = mt3620_wifi_add_wep_key(wifi_hw, vif, params->key,
                          params->key_len, key_index);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Set key: %u", ret);
            return ret;
        }

        break;
    case WLAN_CIPHER_SUITE_TKIP:
    case WLAN_CIPHER_SUITE_CCMP:
        // WPA
        memset(&wpa_key, 0, sizeof(struct mt3620_wifi_set_wpa_key));

        wpa_key.index = key_index;
        if (mac_addr == NULL || is_broadcast_ether_addr(mac_addr)) {
            wpa_key.is_pairwise = 0;
            memcpy(&wpa_key.mac_addr, ndev->dev_addr, ETH_ALEN);
        } else {
            wpa_key.is_pairwise = 1;
            memcpy(&wpa_key.mac_addr, mac_addr, ETH_ALEN);
        }

        if (params->cipher == WLAN_CIPHER_SUITE_TKIP) {
            wpa_key.key_data.cipher_type = WIFI_CIPHER_TKIP;

            // Verify key length
            if (params->key_len != WLAN_KEY_LEN_TKIP) {
                dev_err(vif->dev, "Invalid key length: %u",
                    params->key_len);
                return -EINVAL;
            }

            // TKIP key structure - 16 bytes of key, 8 bytes of TX
            // MIC, 8 bytes of RX MIC
            memcpy(wpa_key.key_data.key, params->key,
                   WLAN_TKIP_KEY_LEN);
            // Note: WPA supplicant gives the bytes in a different
            // order
            // than what the N9 / spec says so we swap 8 bytes
            memcpy(wpa_key.key_data.rx_mic,
                   params->key + WLAN_TKIP_RX_MIC_OFFSET,
                   WLAN_TKIP_MIC_LEN);
            memcpy(wpa_key.key_data.tx_mic,
                   params->key + WLAN_TKIP_TX_MIC_OFFSET,
                   WLAN_TKIP_MIC_LEN);

        } else if (params->cipher == WLAN_CIPHER_SUITE_CCMP) {
            wpa_key.key_data.cipher_type = WIFI_CIPHER_AES;

            // verify key length
            if (params->key_len >
                ARRAY_SIZE(wpa_key.key_data.key)) {
                dev_err(vif->dev, "Invalid key length: %u",
                    params->key_len);
                return -EINVAL;
            }

            memcpy(wpa_key.key_data.key, params->key,
                   params->key_len);
        }

        wpa_key.key_data.key_len = params->key_len;

        if(params->seq_len <= MAX_PN_NUMBER_LEN)
        {
            //dont need to reset the start_seq bcz we have already reset whole wpa_key structure
            memcpy(wpa_key.start_seq,params->seq,params->seq_len);
        }

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SET_KEY,
            true, &wpa_key, sizeof(struct mt3620_wifi_set_wpa_key),
            NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Set key: %u", ret);
            return ret;
        }

        // If we get here the key is good, copy it for later reference
        vif->keys[key_index].cipher = params->cipher;
        vif->keys[key_index].key_len = params->key_len;
        memcpy(&vif->keys[key_index].key, params->key, params->key_len);
        vif->keys[key_index].seq_len = params->seq_len;
        memcpy(&vif->keys[key_index].seq, params->seq, params->seq_len);

        break;
    default:
        dev_err(vif->dev, "Unsupported cipher: %#x", params->cipher);
        return -ENOTSUPP;
    }

    return ret;
}

static int mt3620_wifi_set_rekey_data(struct wiphy *wiphy, struct net_device *ndev,
                   struct cfg80211_gtk_rekey_data *params)
{
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);
    int ret = SUCCESS;

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_set_rekey_data] n9_initialized not done\n");
        return -EAGAIN;
    }

    printk("%s\n",__FUNCTION__);
    // Update kck in WowConf
    memcpy((u8 *)vif->mode_data.sta.wowconf.rRekey.kck,params->kck,
           sizeof(vif->mode_data.sta.wowconf.rRekey.kck));
    // Update kek data in WowConf datastructure
    memcpy((u8 *)vif->mode_data.sta.wowconf.rRekey.kek,params->kek,
           sizeof(vif->mode_data.sta.wowconf.rRekey.kek));
    vif->mode_data.sta.wowconf.rRekey.wow_rekey_enable = (u8)true;
    return ret;
}
///
/// Deletes an encryption key
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @key_index - key index identifier
/// @pairwise - is the key group or pairwise
/// @mac_addr - target address
/// @returns - 0 for success
static int mt3620_wifi_del_key(struct wiphy *wiphy, struct net_device *ndev,
                   u8 key_index, bool pairwise, const u8 *mac_addr)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_set_wpa_key wpa_key;
    int ret = SUCCESS;

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        return -EBUSY;
    }

    // verify key index
    if (key_index >= WLAN_MAX_KEY_COUNT) {
        return -EINVAL;
    }

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_del_key] n9_initialized not done\n");
        return -EAGAIN;
    }

    if (vif->keys[key_index].key_len == 0) {
        // No key to delete
        return SUCCESS;
    }

    switch (vif->keys[key_index].cipher) {
    case WLAN_CIPHER_SUITE_WEP40:
    case WLAN_CIPHER_SUITE_WEP104:
        // WEP
        ret = mt3620_wifi_add_wep_key(wifi_hw, vif, NULL, 0, key_index);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Set WEP keys: %u", ret);
            return ret;
        }

        break;
    case WLAN_CIPHER_SUITE_TKIP:
    case WLAN_CIPHER_SUITE_CCMP:
        // WPA
        // Blank out key
        memset(&wpa_key, 0, sizeof(struct mt3620_wifi_set_wpa_key));

        wpa_key.index = key_index;
        wpa_key.is_pairwise = pairwise;
        if(mac_addr != NULL)
        {
            memcpy(wpa_key.mac_addr,mac_addr,ETH_ALEN);
        }
        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SET_KEY,
            true, &wpa_key, sizeof(struct mt3620_wifi_set_wpa_key),
            NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Set key: %u", ret);
            return ret;
        }

        break;
    default:
        dev_err(vif->dev, "Unsupported cipher: %#x",
            vif->keys[key_index].cipher);
        return -ENOTSUPP;
    }

    // Blank out cached key
    vif->keys[key_index].cipher = 0;
    vif->keys[key_index].key_len = 0;
    memset(&vif->keys[key_index].key, 0,
           ARRAY_SIZE(vif->keys[key_index].key));
    vif->keys[key_index].seq_len = 0;
    memset(&vif->keys[key_index].seq, 0,
           ARRAY_SIZE(vif->keys[key_index].seq));

    return ret;
}

///
/// Sets default encryption key
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @key_index - key index identifier
/// @pairwise - is the key group or pairwise
/// @unicast - set for unicast traffic
/// @multicast - set for multicast traffic
/// @returns - 0 for success
static int mt3620_wifi_set_default_key(struct wiphy *wiphy,
                       struct net_device *ndev, u8 key_index,
                       bool unicast, bool multicast)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);

    // verify key index
    if (key_index >= WLAN_MAX_KEY_COUNT) {
        return -EINVAL;
    }

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_set_default_key] n9_initialized not done\n");
        return -EAGAIN;
    }

    if (vif->keys[key_index].cipher == WLAN_CIPHER_SUITE_WEP40 ||
        vif->keys[key_index].cipher == WLAN_CIPHER_SUITE_WEP104) {
        // Update WEP default key index
        vif->wep_tx_key_index = key_index;
        return mt3620_wifi_commit_wep_keys(wifi_hw, vif);
    }

    // No updates needed for non WEP keys

    return SUCCESS;
}

///
/// Toggles power management
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @pmgmt - is PM enabled?
/// @timeout - timeout for change
/// @returns - 0 for success
static int mt3620_wifi_set_power_mgmt(struct wiphy *wiphy,
                      struct net_device *ndev, bool pmgmt,
                      int timeout)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    enum mt3620_wifi_power_mode power_mode;
    int ret = SUCCESS;

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_set_power_mgmt] n9_initialized not done\n");
        return -EAGAIN;
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        return -EBUSY;
    }

    if (pmgmt) {
        power_mode = POWER_MODE_FAST_PSP;
    } else {
        power_mode = POWER_MODE_CAM;
    }

    // Update power mode
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_SET_PSMODE,
        true, &power_mode, sizeof(enum mt3620_wifi_power_mode), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Set power management error: %u",
            ret);
    }

    return ret;
}

///
/// Gets info about a station
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @mac - station mac address
/// @sinfo - output station info
/// @returns - 0 for success
static int mt3620_wifi_get_station(struct wiphy *wiphy, struct net_device *ndev,
                   const u8 *mac, struct station_info *sinfo)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_station_list station_list;
    int ret = SUCCESS;
    s8 rssi = 0;
    int i = 0;

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_get_station] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        ret = -EBUSY;
        goto exit;
    }

    // If MAC is null, we're in station mode, or the passed in MAC matches
    // our interface
    // we're being asked for stats on our outbound connection to an AP,
    // otherwise
    // we're being asked for stats on a specific connected station
    if (mac == NULL || vif->port == WIFI_PORT_STA ||
        ether_addr_equal(mac, ndev->dev_addr)) {
        // AP connection info
        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_GET_CONNECTED_AP_RSSI, false,
            NULL, 0, &rssi, sizeof(s8));
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error getting RSSI: %#x",
                ret);
            goto exit;
        }

        sinfo->filled = BIT(NL80211_STA_INFO_SIGNAL) |
                BIT(NL80211_STA_INFO_RX_PACKETS) |
                BIT(NL80211_STA_INFO_TX_PACKETS) |
                BIT(NL80211_STA_INFO_TX_FAILED);

        sinfo->signal = rssi;
        sinfo->rx_packets = vif->net_stats.rx_packets;
        sinfo->tx_packets = vif->net_stats.tx_packets;
        sinfo->tx_failed = vif->net_stats.tx_errors;
    } else {
        // Connected station info
        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_GET_STA_LIST, false, NULL, 0,
            &station_list, sizeof(station_list));
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev,
                "Error getting station list: %u", ret);
            goto exit;
        }

        for (i = 0; i < station_list.count && i < MAX_STATION_COUNT;
             i++) {
            if (ether_addr_equal(
                mac, station_list.stations[i].mac_addr)) {
                sinfo->filled =
                    BIT(NL80211_STA_INFO_SIGNAL) |
                    BIT(NL80211_STA_INFO_TX_BITRATE) |
                    BIT(NL80211_STA_INFO_RX_BITRATE) |
                    BIT(NL80211_STA_INFO_SIGNAL_AVG);

                sinfo->signal =
                    station_list.stations[i].last_rssi;
                sinfo->signal_avg =
                    station_list.stations[i].average_rssi;
                sinfo->txrate.legacy =
                    station_list.stations[i].last_tx_rate;
                sinfo->rxrate.legacy =
                    station_list.stations[i].last_rx_rate;

                ret = SUCCESS;
                goto exit;
            }
        }

        // We couldn't find the specified station
        ret = -EINVAL;
    }

exit:
    return ret;
}

///
/// Sets a key association
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @pmksa - association parameters
/// @returns - 0 for success
static int mt3620_wifi_set_pmksa(struct wiphy *wiphy, struct net_device *ndev,
                 struct cfg80211_pmksa *pmksa)
{
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    int idx=0;
    int ret = SUCCESS;
    vif->mt3620_sta_pmkid.Length=PMKID_NO*(ETH_ALEN+PMKID_LEN);
    vif->mt3620_sta_pmkid.BSSIDInfoCount=PMKID_NO;

    for (idx = 0; idx < PMKID_NO; idx++) {
        if (!memcmp((vif->mt3620_sta_pmkid.BSSIDInfo[idx].BSSID), pmksa->bssid, 6))
            {
                memcpy((vif->mt3620_sta_pmkid.BSSIDInfo[idx].BSSID),pmksa->bssid,ETH_ALEN);
                memcpy((vif->mt3620_sta_pmkid.BSSIDInfo[idx].PMKID),pmksa->pmkid,PMKID_LEN);
                break;
            }
        else if (is_zero_ether_addr(vif->mt3620_sta_pmkid.BSSIDInfo[idx].BSSID))
            {
                memcpy((vif->mt3620_sta_pmkid.BSSIDInfo[idx].BSSID),pmksa->bssid,ETH_ALEN);
                memcpy((vif->mt3620_sta_pmkid.BSSIDInfo[idx].PMKID),pmksa->pmkid,PMKID_LEN);
                break;
            }

    }

    ret = mt3620_hif_api_send_command_to_n9_sync(
        g_wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_SET_PMKID, true,
        &vif->mt3620_sta_pmkid,
        sizeof(NDIS_802_11_pmkid), NULL, 0);

    return ret;
}

///
/// Removes a key association
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @pmksa - association parameters
/// @returns - 0 for success
static int mt3620_wifi_del_pmksa(struct wiphy *wiphy, struct net_device *ndev,
                 struct cfg80211_pmksa *pmksa)
{
    int idx=0;
    int ret = SUCCESS;
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    vif->mt3620_sta_pmkid.Length=PMKID_NO*(ETH_ALEN+PMKID_LEN);
    vif->mt3620_sta_pmkid.BSSIDInfoCount=PMKID_NO;
    /* Search added PMKID, deleted it if existed */
    for (idx = 0; idx < PMKID_NO; idx++)
    {
        if (!memcmp((vif->mt3620_sta_pmkid.BSSIDInfo[idx].BSSID), pmksa->bssid, 6))
        {
            memset((vif->mt3620_sta_pmkid.BSSIDInfo[idx].BSSID), 0, ETH_ALEN);
            memset((vif->mt3620_sta_pmkid.BSSIDInfo[idx].PMKID), 0, PMKID_LEN);
            break;
        }
    }
    ret = mt3620_hif_api_send_command_to_n9_sync(
        g_wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_SET_PMKID, true,
        &vif->mt3620_sta_pmkid,
        sizeof(NDIS_802_11_pmkid), NULL, 0);
   
    return ret;
}

///
/// Flushes key associations
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @returns - 0 for success
static int mt3620_wifi_flush_pmksa(struct wiphy *wiphy, struct net_device *ndev)
{
    // Note: we must succeed here since this is called early on in startup,
    // even if the connected BSS doesn't use PMKSA

    int ret = SUCCESS;
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    memset(&vif->mt3620_sta_pmkid, 0, sizeof(NDIS_802_11_pmkid));
    vif->mt3620_sta_pmkid.Length=PMKID_NO*(ETH_ALEN+PMKID_LEN);
    vif->mt3620_sta_pmkid.BSSIDInfoCount=PMKID_NO;

    ret = mt3620_hif_api_send_command_to_n9_sync(
         g_wifi_hw->wifi->hif_api_handle,
         WIFI_COMMAND_ID_EXTENSION_SET_PMKID, true,
         &vif->mt3620_sta_pmkid,
         sizeof(NDIS_802_11_pmkid), NULL, 0);

    return ret;
}

///
/// Changes AP mode becaon data
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @beacon - beacon data
/// @returns - 0 for success
static int mt3620_wifi_change_beacon(struct wiphy *wiphy,
                     struct net_device *ndev,
                     struct cfg80211_beacon_data *beacon)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_extra_ie *beacon_ies = NULL;
    struct mt3620_wifi_extra_ie *assoc_resp_ies = NULL;
    struct mt3620_wifi_extra_ie *probe_resp_ies = NULL;
    u32 beacon_ies_len;
    u32 assoc_resp_ies_len;
    u32 probe_resp_ies_len;
    int ret = SUCCESS;

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_change_beacon] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    dev_dbg(vif->dev, "Updating AP mode beacon settings");

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        ret = -EBUSY;
        goto exit;
    }

    // Alloc memory
    beacon_ies_len =
        sizeof(struct mt3620_wifi_extra_ie) + beacon->beacon_ies_len;
    beacon_ies = devm_kzalloc(vif->dev, beacon_ies_len, GFP_KERNEL);

    if (beacon_ies == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    probe_resp_ies_len =
        sizeof(struct mt3620_wifi_extra_ie) + beacon->proberesp_ies_len;
    probe_resp_ies = devm_kzalloc(vif->dev, probe_resp_ies_len, GFP_KERNEL);

    if (probe_resp_ies == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    assoc_resp_ies_len =
        sizeof(struct mt3620_wifi_extra_ie) + beacon->assocresp_ies_len;
    assoc_resp_ies = devm_kzalloc(vif->dev, assoc_resp_ies_len, GFP_KERNEL);

    if (assoc_resp_ies == NULL) {
        ret = -ENOMEM;
        goto exit;
    }

    // Set IE values
    beacon_ies->port = vif->port;
    beacon_ies->type = WIFI_EXTRA_WPS_BEACON_IE;
    beacon_ies->ie_len = beacon->beacon_ies_len;
    if (beacon->beacon_ies_len != 0) {
        memcpy(beacon_ies->ie, beacon->beacon_ies,
               beacon->beacon_ies_len);

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_SET_EXTRA_IE, true, beacon_ies,
            beacon_ies_len, NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting IE: %u",
                ret);
            goto exit;
        }
    }

    probe_resp_ies->port = vif->port;
    probe_resp_ies->type = WIFI_EXTRA_WPS_PROBE_RSP_IE;
    probe_resp_ies->ie_len = beacon->proberesp_ies_len;
    if (beacon->proberesp_ies_len != 0) {
        memcpy(probe_resp_ies->ie, beacon->proberesp_ies,
               beacon->proberesp_ies_len);

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_SET_EXTRA_IE, true,
            probe_resp_ies, probe_resp_ies_len, NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting IE: %u",
                ret);
            goto exit;
        }
    }

    assoc_resp_ies->port = vif->port;
    assoc_resp_ies->type = WIFI_EXTRA_WPS_ASSOC_RSP_IE;
    assoc_resp_ies->ie_len = beacon->assocresp_ies_len;
    if (beacon->assocresp_ies_len != 0) {
        memcpy(assoc_resp_ies->ie, beacon->assocresp_ies,
               beacon->assocresp_ies_len);

        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_SET_EXTRA_IE, true,
            assoc_resp_ies, assoc_resp_ies_len, NULL, 0);
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev, "Error setting IE: %u",
                ret);
            goto exit;
        }
    }

exit:
    if (beacon_ies != NULL) {
        devm_kfree(vif->dev, beacon_ies);
    }
    if (assoc_resp_ies != NULL) {
        devm_kfree(vif->dev, assoc_resp_ies);
    }
    if (probe_resp_ies != NULL) {
        devm_kfree(vif->dev, probe_resp_ies);
    }

    return ret;
}

///
/// Starts AP mode
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @info - AP settings
/// @returns - 0 for success
static int mt3620_wifi_start_ap(struct wiphy *wiphy, struct net_device *ndev,
                struct cfg80211_ap_settings *info)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_set_channel set_channel;
    struct mt3620_wifi_set_ssid set_ssid;
    u8 beacon_interval;
    u8 dtim_period;
    int ret = SUCCESS;
    bool connect_attempted = false;

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_start_ap] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        ret = -EBUSY;
        goto exit;
    }

    if (vif->state == CONNECTING) {
        // Return EBUSY until the connecting timeout elapses, then continue the connection
        // which will disconnect the ongoing connection attempt.
        if (time_before(jiffies, vif->connecting_timeout)) {
            dev_info(wifi_hw->wifi->dev,
                "Connection attempt while connecting, returning EBUSY");
            ret = -EBUSY;
            goto exit;
        }
    } else {
        // Mark this before we disconnect so we don't reset the scan table by
        // indicating a link down event
        vif->state = CONNECTING;
        // Ignore overflow because it is handled by the 'time_before' macro above.
        vif->connecting_timeout = jiffies + msecs_to_jiffies(WIFI_MAX_CONNECTING_TIME_MSEC);
    }

    connect_attempted = true;

    // Setup encryption mode / auth mode
    ret = mt3620_wifi_set_auth_mode(NULL,info, vif->port);
    if (ret != SUCCESS) {
        goto exit;
    }

    ret = mt3620_wifi_set_encryption_mode(&info->crypto, vif->port);
    if (ret != SUCCESS) {
        goto exit;
    }

    // Update beacon settings
    ret = mt3620_wifi_change_beacon(wiphy, ndev, &info->beacon);
    if (ret != SUCCESS) {
        goto exit;
    }

    // Set channel
    set_channel.port = vif->port;
    set_channel.channel = info->chandef.chan->hw_value;

    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_CHANNEL, true,
        &set_channel, sizeof(struct mt3620_wifi_set_channel), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error setting channel: %u", ret);
        goto exit;
    }

    vif->channel = set_channel.channel;

    dev_dbg(wifi_hw->wifi->dev, "AP starting on channel %hhu",
        vif->channel);

    // Set info->beacon_interval
    beacon_interval = info->beacon_interval;
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_LISTEN_INTERVAL, true, &beacon_interval,
        sizeof(u8), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error setting beacon interval: %u",
            ret);
        goto exit;
    }

    // Set DTIM (info->dtim_period)
    dtim_period = info->dtim_period;
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_DTIM_INTERVAL,
        true, &dtim_period, sizeof(u8), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error setting DTIM interval: %u",
            ret);
        goto exit;
    }

    // Set SSID
    set_ssid.port = vif->port;
    set_ssid.ssid_len = info->ssid_len;
    memcpy(set_ssid.ssid, info->ssid, info->ssid_len);

    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SSID, true,
        &set_ssid, sizeof(struct mt3620_wifi_set_ssid), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error setting SSID: %u", ret);
        goto exit;
    }

    // Send start AP mode command
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_AP_START, true,
        NULL, 0, NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error starting AP mode: %u", ret);
        goto exit;
    }

    dev_dbg(wifi_hw->wifi->dev, "AP mode started for %.*s", info->ssid_len,
        info->ssid);

    // start network interface, set connected
    vif->state = CONNECTED;

    // Turn on interface
    netif_carrier_on(vif->ndev);
    netif_wake_queue(vif->ndev);

exit:
    if (ret != SUCCESS && connect_attempted) {
        vif->state = DISCONNECTED;
    }
    return ret;
}

///
/// Stops AP mode
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @returns - 0 for success
static int mt3620_wifi_stop_ap(struct wiphy *wiphy, struct net_device *ndev)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    int ret = SUCCESS;

    dev_dbg(vif->dev, "Stopping AP mode");

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[ASK][mt3620_wifi_stop_ap] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        ret = -EBUSY;
        goto exit;
    }

    // check connected
    if (vif->state != CONNECTED) {
        // Not connected, nothing to stop
        ret = SUCCESS;
        goto exit;
    }

    // Send stop AP mode command
    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_AP_STOP, true,
        NULL, 0, NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error stopping AP mode: %u", ret);
        goto exit;
    }
    if(apcli_start)
    {
        apcli_ap_start = false;
    }

exit:
    // disable connection
    netif_carrier_off(vif->ndev);

    vif->state = DISCONNECTED;

    return ret;
}

///
/// Disconnects a connected station
///
/// @wifi_hw - wifi HW device
/// @mac - station mac address
/// @returns - 0 for success
static int mt3620_wifi_disconnect_station(struct mt3620_wifi_hw *wifi_hw,
                      const u8 *mac)
{
    struct mt3620_wifi_disconnect_station disconnect;
    int ret = SUCCESS;

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        return -EBUSY;
    }

    disconnect.port = WIFI_PORT_AP;

    memcpy(disconnect.mac_addr, mac, ETH_ALEN);

    ret = mt3620_hif_api_send_command_to_n9_sync(
        wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SET_DISCONNECT, true,
        &disconnect, sizeof(disconnect), NULL, 0);
    if (ret != SUCCESS) {
        dev_err(wifi_hw->wifi->dev, "Error disconnecting client: %u",
            ret);
    }

    return ret;
}

///
/// Disconnects a station
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @params - station info
/// @returns - 0 for success
static int mt3620_wifi_del_station(struct wiphy *wiphy, struct net_device *ndev,
                   struct station_del_parameters *params)
{
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_station_list station_list;
    int ret = SUCCESS;
    int status = SUCCESS;
    int i = 0;

    dev_dbg(vif->dev, "AP Mode - disconnect station with MAC: %pM",
        params->mac);
    
    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[ASK][mt3620_wifi_del_station] n9_initialized not done\n");
        ret = -EAGAIN;
        goto exit;
    }

    if(wifi_hw->in_hqa_mode) {
        dev_err(wifi_hw->wifi->dev,
            "Device busy in RF Testing Mode");
        ret = -EBUSY;
        goto exit;
    }

    // Bump generation as our station list is changing
    vif->mode_data.ap.generation++;

    memset(&station_list, 0, sizeof(station_list));

    if (params->mac == NULL || is_broadcast_ether_addr(params->mac) ||
        is_zero_ether_addr(params->mac)) {
        // Disconnect all clients
        ret = mt3620_hif_api_send_command_to_n9_sync(
            wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_GET_STA_LIST, false, NULL, 0,
            &station_list, sizeof(station_list));
        if (ret != SUCCESS) {
            dev_err(wifi_hw->wifi->dev,
                "Error getting station list: %u", ret);
            goto exit;
        }

        for (i = 0; i < station_list.count && i < MAX_STATION_COUNT;
             i++) {
            status = mt3620_wifi_disconnect_station(
                wifi_hw, station_list.stations[i].mac_addr);
            if (status != SUCCESS) {
                // Mark it as a failure, but try the rest of the
                // disconnects
                ret = status;
            }
        }

    } else {
        // Disconnect just one client
        ret = mt3620_wifi_disconnect_station(wifi_hw, params->mac);
        if (ret != SUCCESS) {
            goto exit;
        }
    }

exit:
    return ret;
}

///
/// Changes info about a station
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @mac - station MAC address
/// @params - station info
/// @returns - 0 for success
static int mt3620_wifi_change_station(struct wiphy *wiphy,
                      struct net_device *ndev, const u8 *mac,
                      struct station_parameters *params)
{
    // Since our driver fully manages our state machine we have nothing to
    // do here
    return SUCCESS;
}

///
/// Sends a 802.11 management packet
///
/// @wiphy - wiphy device
/// @wdev - wireless device
/// @params - packet info
/// @cookie - cookie for TX callback
/// @returns - 0 for success
static int mt3620_wifi_mgmt_tx(struct wiphy *wiphy, struct wireless_dev *wdev,
                   struct cfg80211_mgmt_tx_params *params,
                   u64 *cookie)
{
    // We don't support raw management transmissions as our hardware does
    // offloading of the SME state machine.
    // WPA supplicant likes to call this in response to probe requests even
    // though we've indicated we offload the responses
    // so we need to fail here.  Supplicant handles this failure correctly
    // inside it's code so this error code is safe.

    return -EOPNOTSUPP;
}

///
/// Called when registration for 802.11 management frame reports is updated
///
/// @wiphy - wiphy device
/// @wdev - wireless device
/// @frame_type - type of frames to report on
/// @reg - should we enable or disable reporting
static void mt3620_wifi_mgmt_frame_register(struct wiphy *wiphy,
                        struct wireless_dev *wdev,
                        u16 frame_type, bool reg)
{
    // The N9 doesn't keep a table of registered events so there's no extra
    // work to do here
}

///
/// Change interface configuration
///
/// @wiphy - wiphy device
/// @ndev - net device
/// @type - interface type
/// @flags - flags for changes
/// @params - parameters for changes
static int mt3620_wifi_change_interface(struct wiphy *wiphy,
                    struct net_device *ndev,
                    enum nl80211_iftype type, u32 *flags,
                    struct vif_params *params)
{
    struct mt3620_wifi_vif *vif = netdev_priv(ndev);
    struct mt3620_wifi_hw *wifi_hw = wiphy_priv(vif->wdev.wiphy);

    if(!wifi_hw->n9_initialized)
    {
        dev_err(wifi_hw->wifi->dev, "[ASK][mt3620_wifi_change_interface] n9_initialized not done\n");
        return -EAGAIN;
    }

    // WPA supplicant tries to change interface to STA on shutdown
    // even though we've reported as not supporting that.
    // This leaves the interface up when it should be down.
    // To work around this issue we need to return ENODEV
    // which supplicant treats as a special error case and doesn't attempt
    // to modify interface state
    if (vif->port == WIFI_PORT_AP) {
        if (type == NL80211_IFTYPE_STATION) {
            return -ENODEV;
        } else if (type != NL80211_IFTYPE_AP) {
            return -EINVAL;
        }
    } else if (vif->port == WIFI_PORT_STA) {
        if (type != NL80211_IFTYPE_STATION) {
            return -EINVAL;
        }
    }

    return SUCCESS;
}

#define CHAN_2GHZ(num, freq)                                                   \
    {                                                                      \
        .band = NL80211_BAND_2GHZ, .hw_value = num,                    \
        .center_freq = freq                                            \
    }

#define CHAN_5GHZ(num)                                                         \
    {                                                                      \
        .band = NL80211_BAND_5GHZ, .hw_value = num,                    \
        .center_freq = 5000 + (5 * (num))                              \
    }

// Can't be const, mac80211 needs to write to this
// This is the standard list of 80211 2.4Ghz channels
static struct ieee80211_channel mt3620_2ghz_channels[] = {
    CHAN_2GHZ(1, 2412),  CHAN_2GHZ(2, 2417),  CHAN_2GHZ(3, 2422),
    CHAN_2GHZ(4, 2427),  CHAN_2GHZ(5, 2432),  CHAN_2GHZ(6, 2437),
    CHAN_2GHZ(7, 2442),  CHAN_2GHZ(8, 2447),  CHAN_2GHZ(9, 2452),
    CHAN_2GHZ(10, 2457), CHAN_2GHZ(11, 2462), CHAN_2GHZ(12, 2467),
    CHAN_2GHZ(13, 2472),
};

static struct ieee80211_channel mt3620_5ghz_channels[] = {
    CHAN_5GHZ(36),  CHAN_5GHZ(40),  CHAN_5GHZ(44),  CHAN_5GHZ(48),
    CHAN_5GHZ(52),  CHAN_5GHZ(56),  CHAN_5GHZ(60),  CHAN_5GHZ(64),
    CHAN_5GHZ(100), CHAN_5GHZ(104), CHAN_5GHZ(108), CHAN_5GHZ(112),
    CHAN_5GHZ(116), CHAN_5GHZ(120), CHAN_5GHZ(124), CHAN_5GHZ(128),
    CHAN_5GHZ(132), CHAN_5GHZ(136), CHAN_5GHZ(140), CHAN_5GHZ(149),
    CHAN_5GHZ(153), CHAN_5GHZ(157), CHAN_5GHZ(161), CHAN_5GHZ(165),
};

// Can't be const, mac80211 needs to write to this
// This is the standard list of 80211 2.4Ghz bit rates, up to 54MB
static struct ieee80211_rate mt3620_2ghz_bitrates[] = {
    {.bitrate = 10,
     .hw_value = 0x1,
     .hw_value_short = 0x1,
     .flags = IEEE80211_RATE_SHORT_PREAMBLE},
    {.bitrate = 20,
     .hw_value = 0x2,
     .hw_value_short = 0x2,
     .flags = IEEE80211_RATE_SHORT_PREAMBLE},
    {.bitrate = 55,
     .hw_value = 0x4,
     .hw_value_short = 0x4,
     .flags = IEEE80211_RATE_SHORT_PREAMBLE},
    {.bitrate = 110,
     .hw_value = 0x20,
     .hw_value_short = 0x20,
     .flags = IEEE80211_RATE_SHORT_PREAMBLE},
    {.bitrate = 60, .hw_value = 0x8, .hw_value_short = 0x8},
    {.bitrate = 90, .hw_value = 0x10, .hw_value_short = 0x10},
    {.bitrate = 120, .hw_value = 0x40, .hw_value_short = 0x40},
    {.bitrate = 180, .hw_value = 0x80, .hw_value_short = 0x80},
    {.bitrate = 240, .hw_value = 0x200, .hw_value_short = 0x200},
    {.bitrate = 360, .hw_value = 0x400, .hw_value_short = 0x400},
    {.bitrate = 480, .hw_value = 0x800, .hw_value_short = 0x800},
    {.bitrate = 540, .hw_value = 0x1000, .hw_value_short = 0x1000},
};

static struct ieee80211_rate mt3620_5ghz_bitrates[] = {
    {.bitrate = 60, .hw_value = 0x8, .hw_value_short = 0x8},
    {.bitrate = 90, .hw_value = 0x10, .hw_value_short = 0x10},
    {.bitrate = 120, .hw_value = 0x40, .hw_value_short = 0x40},
    {.bitrate = 180, .hw_value = 0x80, .hw_value_short = 0x80},
    {.bitrate = 240, .hw_value = 0x200, .hw_value_short = 0x200},
    {.bitrate = 360, .hw_value = 0x400, .hw_value_short = 0x400},
    {.bitrate = 480, .hw_value = 0x800, .hw_value_short = 0x800},
    {.bitrate = 540, .hw_value = 0x1000, .hw_value_short = 0x1000},
};

// Info on supported 2GHZ bands
static struct ieee80211_supported_band mt3620_band_2ghz = {
    .channels = mt3620_2ghz_channels,
    .n_channels = ARRAY_SIZE(mt3620_2ghz_channels),
    .bitrates = mt3620_2ghz_bitrates,
    .n_bitrates = ARRAY_SIZE(mt3620_2ghz_bitrates),
};

static struct ieee80211_supported_band mt3620_band_5ghz = {
    .channels = mt3620_5ghz_channels,
    .n_channels = ARRAY_SIZE(mt3620_5ghz_channels),
    .bitrates = mt3620_5ghz_bitrates,
    .n_bitrates = ARRAY_SIZE(mt3620_5ghz_bitrates),
};

static const u32 mt3620_wifi_cipher_suites[] = {
    WLAN_CIPHER_SUITE_WEP40, WLAN_CIPHER_SUITE_WEP104, WLAN_CIPHER_SUITE_TKIP,
    WLAN_CIPHER_SUITE_CCMP,
};

// Pointer to our callbacks
static const struct cfg80211_ops mt3620_wifi_ops = {
    .scan = mt3620_wifi_scan,
    .abort_scan = mt3620_wifi_abort_scan,
    .connect = mt3620_wifi_connect,
    .disconnect = mt3620_wifi_disconnect,
    .join_ibss = mt3620_wifi_join_ibss,
    .leave_ibss =  mt3620_wifi_leave_ibss,
    .add_key = mt3620_wifi_add_key,
    .del_key = mt3620_wifi_del_key,
    .set_default_key = mt3620_wifi_set_default_key,
    .set_power_mgmt = mt3620_wifi_set_power_mgmt,
    .get_station = mt3620_wifi_get_station,
    .set_pmksa = mt3620_wifi_set_pmksa,
    .del_pmksa = mt3620_wifi_del_pmksa,
    .set_rekey_data = mt3620_wifi_set_rekey_data,
    .flush_pmksa = mt3620_wifi_flush_pmksa,
    .start_ap = mt3620_wifi_start_ap,
    .change_beacon = mt3620_wifi_change_beacon,
    .stop_ap = mt3620_wifi_stop_ap,
    .del_station = mt3620_wifi_del_station,
    .change_station = mt3620_wifi_change_station,
    .mgmt_tx = mt3620_wifi_mgmt_tx,
    .mgmt_frame_register = mt3620_wifi_mgmt_frame_register,
    .change_virtual_intf = mt3620_wifi_change_interface,
};

///
/// Initialize our CFG80211 integration
///
/// @wifi - device data
/// @returns - 0 for success
static int mt3620_initialize_cfg80211(struct mt3620_wifi *wifi)
{
    struct wiphy *wiphy;
    struct mt3620_wifi_hw *wifi_hw;
    struct wireless_dev *wdev;
    int ret;
    int i;

    wiphy = wiphy_new(&mt3620_wifi_ops, sizeof(*wifi_hw));
    if (IS_ERR(wiphy)) {
        return PTR_ERR(wiphy);
    }

    wifi_hw = wiphy_priv(wiphy);
    memset(wifi_hw, 0, sizeof(*wifi_hw));

    wifi->wiphy = wiphy;
    wifi_hw->wifi = wifi;

    g_wifi_hw = wifi_hw;

    set_wiphy_dev(wiphy, wifi->dev);

    // Generate MAC address
    // Future TODO 21461: Once we have persistent config storage we should
    // make this value survive reboot
    for (i = 0; i < ARRAY_SIZE(wifi_hw->mac_addresses); i++) {
        eth_random_addr(wifi_hw->mac_addresses[i].addr);
    }

    wifi->wiphy->addresses = wifi_hw->mac_addresses;
    wifi->wiphy->n_addresses = ARRAY_SIZE(wifi_hw->mac_addresses);

    mutex_init(&wifi_hw->mutex);
    mutex_init(&wifi_hw->scan_mutex);

    wifi->wiphy->interface_modes =
        BIT(NL80211_IFTYPE_STATION) | BIT(NL80211_IFTYPE_AP) | BIT(NL80211_IFTYPE_ADHOC);
    wifi->wiphy->max_scan_ssids = 1;
    wifi->wiphy->bands[NL80211_BAND_2GHZ] = &mt3620_band_2ghz;
    wifi->wiphy->bands[NL80211_BAND_5GHZ] = &mt3620_band_5ghz;

    wifi->wiphy->flags =
        WIPHY_FLAG_HAVE_AP_SME | WIPHY_FLAG_AP_PROBE_RESP_OFFLOAD;
    wifi->wiphy->probe_resp_offload =
        NL80211_PROBE_RESP_OFFLOAD_SUPPORT_WPS |
        NL80211_PROBE_RESP_OFFLOAD_SUPPORT_WPS2;
    wifi->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
    wifi->wiphy->cipher_suites = mt3620_wifi_cipher_suites;
    wifi->wiphy->n_cipher_suites = ARRAY_SIZE(mt3620_wifi_cipher_suites);

    wifi->wiphy->regulatory_flags =
        REGULATORY_CUSTOM_REG | REGULATORY_COUNTRY_IE_IGNORE | REGULATORY_DISABLE_BEACON_HINTS;
    wifi->wiphy->reg_notifier = mt3620_wifi_reg_notifier;

    wifi->wiphy->mgmt_stypes = mt3620_wifi_mgmt_stypes;
    wifi->wiphy->wowlan = &mt3620_wowlan_support;
    mt3620_set_vendor_commands(wifi->wiphy);
    
    ret = wiphy_register(wifi->wiphy);
    if (ret < SUCCESS) {
        return ret;
    }


    // Setup our STA interface
    wdev = mt3620_wifi_add_interface(
        wifi->wiphy, "wlan%d", NET_NAME_UNKNOWN, NL80211_IFTYPE_STATION);
    if (IS_ERR(wdev)) {
        ret = PTR_ERR(wdev);
        return ret;
    }


    // AP interface
    wdev = mt3620_wifi_add_interface(wifi->wiphy, "wlan%d",
                     NET_NAME_UNKNOWN, NL80211_IFTYPE_AP);
    if (IS_ERR(wdev)) {
        ret = PTR_ERR(wdev);
        return ret;
    }
    // Setup our IBSS interface
    wdev = mt3620_wifi_add_interface(
        wifi->wiphy, "wlan%d", NET_NAME_UNKNOWN, NL80211_IFTYPE_ADHOC);
    if (IS_ERR(wdev)) {
        ret = PTR_ERR(wdev);
        return ret;
    }
    //setup our APCLI interface

    wdev = mt3620_wifi_add_interface(
        wifi->wiphy, "wlan%d", NET_NAME_UNKNOWN, NL80211_IFTYPE_AP);
    if (IS_ERR(wdev)) {
        ret = PTR_ERR(wdev);
        return ret;
    }

    dev_dbg(wifi->dev, "Wi-Fi device registered");
    printk("Wi-Fi device registered\n");

    return SUCCESS;
}

static int mt3620_wifi_set_hqa_mode_capability(void)
{
    int ret = SUCCESS;
    u16 capability = PLUTON_DEVICE_CAPABILITY_ENABLE_RF_TEST_MODE;
    bool has_capability;

    ret = pluton_remote_api_send_command_to_m4_sync(
        IS_CAPABILITY_ENABLED, &capability, sizeof(u16),
        &has_capability, sizeof(bool));

    if (!ret) {
        g_wifi_hw->has_hqa_mode_capability = has_capability;
    }

    return ret;
}

///
/// Initialize the wifi driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int mt3620_wifi_probe(struct platform_device *pdev)
{
    struct mt3620_wifi *wifi = NULL;
    void *handle = NULL;
    int ret = SUCCESS;
#ifdef WIFI_N9_DBG_LOG_CHANGES
    struct resource *regs = NULL;
#endif
//    u32 version;

    handle = mt3620_hif_api_get_handle(&pdev->dev);
    if (handle == NULL) {
        // Our dependencies haven't started yet, tell the kernel to try
        // again later
        return -EPROBE_DEFER;
    }

    if (pluton_remote_api_is_ready() != 0) {
        return -EPROBE_DEFER;
    }
    
    dev_info(&pdev->dev, "Starting MT3620 Wi-Fi driver\n");

    // Allocate memory for our driver state
    wifi = devm_kzalloc(&pdev->dev, sizeof(*wifi), GFP_KERNEL);
    if (!wifi) {
        return -ENOMEM;
    }
#ifdef    CONFIG_MT3620_PIO_DATA    
    spin_lock_init(&wifi->data_lock);
    INIT_WORK(&wifi->write_work,
             mt3620_tx_bottom_half);
#endif
    wifi->dev = &pdev->dev;

    wifi->hif_api_handle = handle; //Assigning the HIF handle to Wifi device

#ifdef    CONFIG_MT3620_PIO_DATA    

    mt3620_tx_desc_init(wifi);
#endif
    platform_set_drvdata(pdev, wifi);

    // register ourselves as a wifi driver
    ret = mt3620_initialize_cfg80211(wifi);
    if (ret != SUCCESS) {
        dev_err(wifi->dev, "Error initializing IEEE80211 driver");
        return ret;
    }
    
    // check whether capabilities allow entering RF test mode
    ret = mt3620_wifi_set_hqa_mode_capability();
    
    if (ret != SUCCESS) {
        dev_err(wifi->dev, "Error checking RF test capability");
        return ret;
    }

    // Load the sysram sysram offset
#ifdef WIFI_N9_DBG_LOG_CHANGES
    regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);

    wifi->pwifi_sysram_base= devm_ioremap_resource(wifi->dev, regs);

    if (IS_ERR(wifi->pwifi_sysram_base)) {
        return PTR_ERR(wifi->pwifi_sysram_base);
    }
#endif
    return ret;
}

///
/// Teardown the wifi driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int mt3620_wifi_remove(struct platform_device *pdev)
{
    struct mt3620_wifi *wifi = platform_get_drvdata(pdev);

    if (!wifi) {
        return -EINVAL;
    }

    // Free WIPHY driver
    wiphy_unregister(wifi->wiphy);
    wiphy_free(wifi->wiphy);

    g_wifi_hw = NULL;

    return SUCCESS;
}

static const struct of_device_id mt3620_wifi_match[] = {
    {.compatible = "mediatek,mt3620-wifi"}, {/* Sentinel */}};

MODULE_DEVICE_TABLE(of, mt3620_wifi_match);

static struct platform_driver mt3620_wifi_driver = {
    .probe = mt3620_wifi_probe,
    .remove = mt3620_wifi_remove,
    .driver =
    {
        .name = DRIVER_NAME, .of_match_table = mt3620_wifi_match,
    },
};

module_platform_driver(mt3620_wifi_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("MT3620 wifi driver");
MODULE_ALIAS("platform:mt3620-wifi");
