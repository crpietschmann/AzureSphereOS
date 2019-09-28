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
#pragma once

/** @brief N9's custom region number for 2.4 Ghz
*/
#define CUSTOM_REGION_2GHZ 8

/** @brief N9's custom region number for 5 Ghz
*/
#define CUSTOM_REGION_5GHZ 12

/** @brief The comprehensive list of channels for 2.4 Ghz
*/
static const u8 mt3620_wifi_5ghz_all_channels[] = {
    36,  40,  44,  48,  52,  56,  60,  64,  100, 104, 108, 112,
    116, 120, 124, 128, 132, 136, 140, 149, 153, 157, 161, 165
};

/** @brief The comprehensive list of channels for 5 Ghz
*/
static const u8 mt3620_wifi_2ghz_all_channels[] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

/** @brief Apply regulatory changes for current country code
*/
int mt3620_wifi_apply_country_code_regulatory_rules(void);

/** @brief Print current regulatory region from N9 for debugging purposes
*/
void print_current_regulatory_region(void);

/** @brief Sends the WIFI_COMMAND_ID_EXTENSION_COUNTRY_REGION to set 
 *  region in N9
*/
int set_custom_regulatory_region(int band, int region);

