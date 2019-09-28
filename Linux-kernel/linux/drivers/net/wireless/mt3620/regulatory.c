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
#include <linux/inetdevice.h>
#include <linux/kernel.h>
#include <mt3620/mt3620_hifapi.h>
#include <net/cfg80211.h>
#include <net/netlink.h>
#include "regulatory.h"
#include "mt3620_wifi_common.h"

#ifdef CONFIG_MT3620_WIFI_DEBUG

// Set DEBUG to 1 to enable debug log output
#define DEBUG 1

#endif

int set_custom_regulatory_region(int band, int region)
{
    struct mt3620_wifi_set_country_region custom_region;
    custom_region.band = band;
    custom_region.region = region;

    dev_dbg(g_wifi_hw->wifi->dev,
        "Setting %d region to %hhu",
        custom_region.band, 
        custom_region.region);

    return mt3620_hif_api_send_command_to_n9_sync(
        g_wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_COUNTRY_REGION, true, &custom_region,
        sizeof(custom_region), NULL, 0);
}

void print_current_regulatory_region(void)
{
    u8 current_2ghz_region = NO_REG_DOMAIN_MATCH;
    u8 current_5ghz_region = NO_REG_DOMAIN_MATCH;
    u8 band = MT3620_BAND_2GHZ;
    int ret = mt3620_hif_api_send_command_to_n9_sync(
        g_wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_COUNTRY_REGION, false, &band, sizeof(u8),
        &current_2ghz_region, sizeof(u8));
    if (ret != SUCCESS) {
        dev_err(g_wifi_hw->wifi->dev, "Could not get 2.4Ghz region: %u",
            ret);
        return;
    }
    dev_dbg(g_wifi_hw->wifi->dev, "For Band 2.4Ghz: %d\n", current_2ghz_region);

    // Get current 5Ghz region
    band = MT3620_BAND_5GHZ;
    ret = mt3620_hif_api_send_command_to_n9_sync(
        g_wifi_hw->wifi->hif_api_handle,
        WIFI_COMMAND_ID_EXTENSION_COUNTRY_REGION, false, &band, sizeof(u8),
        &current_5ghz_region, sizeof(u8));
    if (ret != SUCCESS) {
        dev_err(g_wifi_hw->wifi->dev, "Could not get 5Ghz region: %u",
            ret);
        return;
    }
    dev_dbg(g_wifi_hw->wifi->dev, "For Band 5Ghz: %d\n", current_5ghz_region);
}

/** @brief Retrieve allowed channel list from regulatory database and set 
 *  channels to N9 firmware - This is done by setting WIFI_COMMAND_ID_EXTENSION_COUNTRY_REGION
 *  to custom region - 8 (for 2.4 Ghz), 12 (for 5Ghz) and then using the command 
 *  WIFI_COMMAND_ID_EXTENSION_CHANNEL_LIST to set the channel list
 */
int mt3620_wifi_apply_country_code_regulatory_rules(void)
{
    const struct ieee80211_reg_rule *reg_rule;	
    u32 freq;

    // 48+2--> 48 - total number of channels and 2 - length of 'bg' and 'a' channel list
    // channel_buf[0] = No_of_channels in BG_BAND
    // channel_buf[1] = No_of_channels in A_BAND
    // channel_buf[2] all the channel number in increasing order (1st BG_Band Channels and then A channels)
    // Example --> 11 5 1 2 3 4 5 6 7 8 9 10 11 36 40 44 48 165
    u8 channel_buf[50];

    int mt3620_2ghz_channel_count = ARRAY_SIZE(mt3620_wifi_2ghz_all_channels);
    int mt3620_5ghz_channel_count = ARRAY_SIZE(mt3620_wifi_5ghz_all_channels);

    int mt3620_2ghz_allowed_channel_count = 0;
    int mt3620_5ghz_allowed_channel_count = 0;

    int channel_buf_total_length;
    int current_index = 2;
    int channel_index = 0;

    int ret = SUCCESS;

    if (g_wifi_hw->in_hqa_mode) {
        dev_err(g_wifi_hw->wifi->dev, "Device busy in RF Testing Mode");
        return -EBUSY;
    }

    // Fill out the allowed 2.4 Ghz channels
    for (channel_index = 0;
        channel_index < mt3620_2ghz_channel_count;
        channel_index++) {
        // Get the active rule for this channel
        freq = MHZ_TO_KHZ(m3620_wifi_channel_to_frequency(
               mt3620_wifi_2ghz_all_channels[channel_index]));
        reg_rule = freq_reg_info(g_wifi_hw->wifi->wiphy, freq);

        // Do not fill it up if the channel is not allowed 
        // under the current regulatory domain
        if (IS_ERR(reg_rule)) {
            continue;
        }

        mt3620_2ghz_allowed_channel_count++;
        channel_buf[current_index] = mt3620_wifi_2ghz_all_channels[channel_index];
        current_index++;
    }

    // Fill out the allowed 5 Ghz channels
    for (channel_index = 0;
        channel_index < mt3620_5ghz_channel_count;
        channel_index++) {
        // Get the active rule for this channel
        freq = MHZ_TO_KHZ(m3620_wifi_channel_to_frequency(
               mt3620_wifi_5ghz_all_channels[channel_index]));
        reg_rule = freq_reg_info(g_wifi_hw->wifi->wiphy, freq);

        // Do not fill it up if the channel is not allowed 
        // under the current regulatory domain
        if (IS_ERR(reg_rule)) {
            continue;
        }

        mt3620_5ghz_allowed_channel_count++;
        channel_buf[current_index] = mt3620_wifi_5ghz_all_channels[channel_index];
        current_index++;
    }

    // Fill out channel buffer headers
    channel_buf[0] = mt3620_2ghz_allowed_channel_count;
    channel_buf[1] = mt3620_5ghz_allowed_channel_count;
    channel_buf_total_length = channel_buf[0] + channel_buf[1] + 2;

    // Set custom regions for 2.4Ghz (8) and 5Ghz (12)
    ret = set_custom_regulatory_region(MT3620_BAND_2GHZ, CUSTOM_REGION_2GHZ);
    if (ret != SUCCESS) {
        dev_err(g_wifi_hw->wifi->dev,
            "Could not set 2.4Ghz region: %u", ret);
            return ret;
    }

    ret = set_custom_regulatory_region(MT3620_BAND_5GHZ, CUSTOM_REGION_5GHZ);
    if (ret != SUCCESS) {
        dev_err(g_wifi_hw->wifi->dev,
            "Could not set 5Ghz region: %u", ret);
            return ret;
    }

    // Now set the allowed channel list
    ret = mt3620_hif_api_send_command_to_n9_sync(
            g_wifi_hw->wifi->hif_api_handle,
            WIFI_COMMAND_ID_EXTENSION_CHANNEL_LIST, true, &channel_buf,
            channel_buf_total_length, NULL, 0);
    if (ret != SUCCESS) {
        dev_err(g_wifi_hw->wifi->dev,
            "Could not set channel list: %u", ret);
    }

    return ret;
}