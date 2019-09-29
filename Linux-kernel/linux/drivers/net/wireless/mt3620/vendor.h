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

#pragma once

#define MT3620_HQA_VENDOR_CMD(cmd, fn)                                         \
	{                                                                      \
		{.vendor_id = MT3620_OUI, .subcmd = cmd},                      \
		    .flags = WIPHY_VENDOR_CMD_NEED_RUNNING |                   \
			     WIPHY_VENDOR_CMD_NEED_NETDEV,                     \
		    .doit = fn                                                 \
	}

#define MT3620_OUI 0x003620
#define EFUSE_BLOCK_SIZE 16
#define EFUSE_MAX_ADDR 0x200
#define EFUSE_BLANK_BLOCKS 30	/* Number of free blocks in blank e-fuse */
#define EFUSE_MAX_OFFSET (EFUSE_MAX_ADDR - EFUSE_BLOCK_SIZE)

/*
 * @MT3620_VENDOR_SUBCMD_UNSPEC: Reserved value 0
 *
 * @MT3620_VENDOR_SUBCMD_WOW_CONFIG: Command to set whether wake-on-WLAN is
 * enabled.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_ENTER: Enter RF testing (HQA) mode. Wireless
 * interface must not be connected to any APs or stations or this command will
 * fail. This command must be issued (and be successful) before using the other
 * HQA vendor subcommands.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_LEAVE: Leave RF testing mode.
 *
 * @MT3620_VENDOR_SUBCMD_GET_THERMAL_VALUE: Command that responds with the
 * current die temperature in degrees Celsius.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_RESET_COUNTERS: Reset transmit and receive packet
 * counters
 *
 * @MT3620_VENDOR_SUBCMD_HQA_GET_RX_ALL_PKT: Get count of all received packets
 *
 * @MT3620_VENDOR_SUBCMD_HQA_GET_RX_UNICAST_DATA: Get count of all received
 * unicast
 * packets to this device's MAC
 *
 * @MT3620_VENDOR_SUBCMD_HQA_GET_RX_OK_DATA: Get count of all successfully
 * received data packets
 *
 * @MT3620_VENDOR_SUBCMD_HQA_GET_RX_OK_OTHER: Get count of all other
 * successfully packets, excluding data packets
 *
 * @MT3620_VENDOR_SUBCMD_HQA_GET_TX: Get count of transmitted packets
 *
 * @MT3620_VENDOR_SUBCMD_HQA_GET_CW_TONE_RSSI: Get RSSI value for a continuous
 * tone transmission.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_GET_EFUSE_TOTAL_BLOCKS: Get number of total e-fuse
 * blocks on the device.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_GET_EFUSE_FREE_BLOCKS: Get number of free e-fuse
 * blocks on the device
 *
 * @MT3620_VENDOR_SUBCMD_HQA_SET_ANTENNA_DIVERSITY: Configure antenna settings.
 * Allowed settings
 * are antenna band (2.4G/5G) and whether TX/RX uses the main antenna or
 * diversity antenna.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_SET_TX_AIFS: Specify the arbitration inter-frame
 * spacing (AIFS) in microseconds. Valid values are 4-100us.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_SET_CHANNEL: Specify the channel for transmit and
 * receive commands to operate on.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_SET_FREQ_OFFSET: Set the crystal calibration trim
 * value for frequency adjustments. This command has not yet been implemented.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_SET_TX_RATE: Set the modulation scheme and data
 * rate for transmissions. Modulation schemes are described in the
 * mt3620_hqa_tx_mod enum and supported rates in the mt3620_hqa_cck_rate and
 * mt3620_hqa_ofdm_ht_rate enums.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_SET_TX_POWER: Set the transmission power adjustment
 * level
 *
 * @MT3620_VENDOR_SUBCMD_HQA_SET_TX_FRAME: Configure fields in packets that are
 * transmitted. This command takes parameters specifying the frame control,
 * duration and bss, source, and destination MAC addresses.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_START_TX: Start transmission. This command takes in
 * the number of packets to send, the desired packet size, whether the packet
 * should contain random data, and if not, the content of the packet. Set number
 * of packets to 0 to repeat transmission until the STOP_TX command is issued.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_STOP_TX: Stop transmission.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_START_RX: Start receiving packets. This command
 * must be called in order to enable the packet counters.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_STOP_RX: Stop receiving packets.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_ENTER_TEST_MODE: Enter continuous transmission
 * testing mode. This command must be issued before using the
 * CW_TONE and CONTI_TX commands that follow.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_EXIT_TEST_MODE: Exit continuous transmission
 * testing mode.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_START_CW_TONE: Start generating and outputting an
 * unmodulated sine-wave tone on the selected channel. Note:
 * this command requires the device to be in the special test mode, which is
 * activated using the ENTER_TEST_MODE command.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_STOP_CW_TONE: Stop transmission of sine-wave tone.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_START_CONTI_TX: Start generating and outputting a
 * continuous stream of packets with no inter-packet gap time.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_STOP_CONTI_TX: Stop generating continuous packet
 * stream.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_SET_BUFFER_MODE: Configure whether the Wi-Fi
 * chipset reads settings from e-fuses or the volatile buffer bin.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_READ_BUFFER_BIN: Read contents of e-fuses/buffer
 * bin. Takes in a 16-bit address offset and returns a 16-byte data
 * corresponding to the e-fuse block at that address.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_WRITE_BUFFER_BIN: Write data to the buffer bin.
 * Takes a 16-bit address offset to determine where to write this data.
 *
 * @MT3620_VENDOR_SUBCMD_HQA_WRITE_BUFFER_TO_EFUSE: Write the contents of the
 * buffer bin to e-fuses. This is a permanent change that cannot be reversed.
 *
 */

enum mt3620_vendor_subcmds {
	MT3620_VENDOR_SUBCMD_UNSPEC = 0,
	MT3620_VENDOR_SUBCMD_WOW_CONFIG = 1,
	MT3620_SET_N9_WDT = 2,
	MT3620_N9_WDT_TRIGGER = 3,
	MT3620_N9_RESET_DONE = 4,
	MT3620_N9_LOG_CTRL = 5,
	MT3620_VENDOR_SUBCMD_HQA_ENTER = 10,
	MT3620_VENDOR_SUBCMD_HQA_LEAVE = 11,
	MT3620_VENDOR_SUBCMD_HQA_GET_THERMAL_VALUE = 12,
	MT3620_VENDOR_SUBCMD_HQA_RESET_COUNTERS = 13,
	MT3620_VENDOR_SUBCMD_HQA_GET_RX_ALL_PKT = 14,
	MT3620_VENDOR_SUBCMD_HQA_GET_RX_UNICAST_DATA = 15,
	MT3620_VENDOR_SUBCMD_HQA_GET_RX_OK_DATA = 16,
	MT3620_VENDOR_SUBCMD_HQA_GET_RX_OK_OTHER = 17,
	MT3620_VENDOR_SUBCMD_HQA_GET_TX = 18,
	MT3620_VENDOR_SUBCMD_HQA_GET_CW_TONE_RSSI = 19,
	MT3620_VENDOR_SUBCMD_HQA_GET_EFUSE_TOTAL_BLOCKS = 20,
	MT3620_VENDOR_SUBCMD_HQA_GET_EFUSE_FREE_BLOCKS = 21,
	MT3620_VENDOR_SUBCMD_HQA_SET_ANTENNA_DIVERSITY = 22,
	MT3620_VENDOR_SUBCMD_HQA_SET_TX_AIFS = 23,
	MT3620_VENDOR_SUBCMD_HQA_SET_CHANNEL = 24,
	MT3620_VENDOR_SUBCMD_HQA_SET_FREQ_OFFSET = 25,
	MT3620_VENDOR_SUBCMD_HQA_SET_TX_RATE = 26,
	MT3620_VENDOR_SUBCMD_HQA_SET_TX_POWER = 27,
	MT3620_VENDOR_SUBCMD_HQA_SET_TX_FRAME = 28,
	MT3620_VENDOR_SUBCMD_HQA_START_TX = 29,
	MT3620_VENDOR_SUBCMD_HQA_STOP_TX = 30,
	MT3620_VENDOR_SUBCMD_HQA_START_RX = 31,
	MT3620_VENDOR_SUBCMD_HQA_STOP_RX = 32,
	MT3620_VENDOR_SUBCMD_HQA_ENTER_TEST_MODE = 33,
	MT3620_VENDOR_SUBCMD_HQA_EXIT_TEST_MODE = 34,
	MT3620_VENDOR_SUBCMD_HQA_START_CW_TONE = 35,
	MT3620_VENDOR_SUBCMD_HQA_STOP_CW_TONE = 36,
	MT3620_VENDOR_SUBCMD_HQA_START_CONTI_TX = 37,
	MT3620_VENDOR_SUBCMD_HQA_STOP_CONTI_TX = 38,
	MT3620_VENDOR_SUBCMD_HQA_SET_BUFFER_MODE = 39,
	MT3620_VENDOR_SUBCMD_HQA_READ_BUFFER_BIN = 40,
	MT3620_VENDOR_SUBCMD_HQA_WRITE_BUFFER_BIN = 41,
	MT3620_VENDOR_SUBCMD_HQA_WRITE_BUFFER_TO_EFUSE = 42,
	MT3620_VENDOR_SUBCMD_READ_EFUSE = 43,
	MT3620_VENDOR_SUBCMD_SET_TEST_MACADDR = 44,
	MT3620_VENDOR_SUBCMD_HQA_BUFFER_BIN_REINIT = 45,
};

enum mt3620_hqa_actions {
	HQA_ACT_SET_RATE = 0,
	HQA_ACT_START_TX = 1,
	HQA_ACT_STOP_TX = 2,
	HQA_ACT_GET_CW_TONE_RSSI = 3,
	HQA_ACT_SET_AIFS = 4,
	HQA_ACT_SET_TX_FRAME = 5,
	HQA_ACT_SET_FREQ_OFFSET = 6, // not used; legacy
	HQA_ACT_START_RX = 7,
	HQA_ACT_STOP_RX = 8,
	HQA_ACT_GET_RX_OK_DATA = 9,
	HQA_ACT_GET_TX_TRANSMITTED = 10,
	HQA_ACT_RESET_TRX_COUNTER = 11,
	HQA_ACT_GET_RX_OK_OTHER = 12,
	HQA_ACT_GET_RX_ALL_PKT = 13,
	HQA_ACT_GET_EFUSE_BLK_NUM = 14,
	HQA_ACT_GET_FREE_EFUSE_BLK = 15,
	HQA_ACT_GET_THERMAL_VALUE = 16,
	HQA_ACT_GET_RX_U2M_DATA = 17,
	HQA_ACT_START_CW_TONE = 18,
	HQA_ACT_STOP_CW_TONE = 19,
	HQA_ACT_SET_CHANNEL = 20,
	HQA_ACT_NUM = 21, // not used
	ENTER_TEST_MODE = 22,
	EXIT_TEST_MODE = 23,
	ENTER_HQA_MODE = 24,
	LEAVE_HQA_MODE = 25,
	HQA_ACT_SET_TX_POWER = 26,
	HQA_ACT_SET_TX_CONTI_START = 27,
	HQA_ACT_SET_TX_CONTI_STOP = 28,
	HQA_ACT_BUFFER_BIN = 29
};

enum mt3620_vendor_attr_hqa_results {
	MT3620_VENDOR_ATTR_INVALID = 0,
	MT3620_VENDOR_ATTR_THERMAL_VALUE = 1,
	MT3620_VENDOR_ATTR_EFUSE_FREE_BLOCKS = 2,
	MT3620_VENDOR_ATTR_EFUSE_TOTAL_BLOCKS = 3,
	MT3620_VENDOR_ATTR_RX_ALL_PACKETS = 4,
	MT3620_VENDOR_ATTR_RX_OK_OTHER_PACKETS = 5,
	MT3620_VENDOR_ATTR_RX_OK_DATA_PACKETS = 6,
	MT3620_VENDOR_ATTR_RX_U2M_DATA_PACKETS = 7,
	MT3620_VENDOR_ATTR_CW_TONE_RSSI = 8,
	MT3620_VENDOR_ATTR_TX_TRANSMITTED = 9,
	MT3620_VENDOR_ATTR_BUFFER_BIN_DATA = 10,
	MT3620_VENDOR_ATTR_EFUSE_DATA = 11,
	MT3620_VENDOR_ATTR_LAST
};

enum mt3620_hqa_tx_mod {
	MT3620_HQA_TX_MOD_CCK = 0,
	MT3620_HQA_TX_MOD_OFDM_HT,
	MT3620_HQA_TX_MOD_LAST
};

enum mt3620_hqa_cck_rate {
	MT3620_HQA_CCK_RATE_1M = 0,
	MT3620_HQA_CCK_RATE_2M,
	MT3620_HQA_CCK_RATE_5M,
	MT3620_HQA_CCK_RATE_11M,
	MT3620_HQA_CCK_RATE_LAST
};

enum mt3620_hqa_ofdm_ht_rate {
	MT3620_HQA_OFDM_RATE_6M = 0,
	MT3620_HQA_OFDM_RATE_9M,
	MT3620_HQA_OFDM_RATE_12M,
	MT3620_HQA_OFDM_RATE_18M,
	MT3620_HQA_OFDM_RATE_24M,
	MT3620_HQA_OFDM_RATE_36M,
	MT3620_HQA_OFDM_RATE_48M,
	MT3620_HQA_OFDM_RATE_54M,
	MT3620_HQA_HTMM_RATE_MCS0,
	MT3620_HQA_HTMM_RATE_MCS1,
	MT3620_HQA_HTMM_RATE_MCS2,
	MT3620_HQA_HTMM_RATE_MCS3,
	MT3620_HQA_HTMM_RATE_MCS4,
	MT3620_HQA_HTMM_RATE_MCS5,
	MT3620_HQA_HTMM_RATE_MCS6,
	MT3620_HQA_HTMM_RATE_MCS7,
	MT3620_HQA_CCKM_RATE_1M,
	MT3620_HQA_CCKM_RATE_2M,
	MT3620_HQA_CCKM_RATE_5M,
	MT3620_HQA_CCKM_RATE_11M,
	MT3620_HQA_RATE_LAST
};

enum mt3620_hqa_tone_freq_offset {
	MT3620_HQA_TONE_FREQ_DC = 0,
	MT3620_HQA_TONE_FREQ_5M,
	MT3620_HQA_TONE_FREQ_10M,
	MT3620_HQA_TONE_FREQ_20M,
	MT3620_HQA_TONE_FREQ_40M,
	MT3620_HQA_TONE_FREQ_LAST
};

enum mt3620_hqa_buffer_operation {
	MT3620_HQA_BUFFER_READ = 0,
	MT3620_HQA_BUFFER_WRITE = 1,
};

enum mt3620_hqa_buffer_mode {
	MT3620_HQA_EFUSE_MODE = 0,
	MT3620_HQA_BUFFER_MODE = 1
};

struct mt3620_hqa_tx_frame_setting {
	u16 frame_control;
	u16 duration;
	u8 bssid_mac_addr[ETH_ALEN];
	u8 src_mac_addr[ETH_ALEN];
	u8 dest_mac_addr[ETH_ALEN];
};

struct mt3620_hqa_info {
	u32 command;
	s32 params[5];
	struct mt3620_hqa_tx_frame_setting tx_setting;
	u8 padding[2];
	u8 efuse_buffer[EFUSE_BLOCK_SIZE];
};

enum mt3620_efuse_operation {
	MT3620_EFUSE_OPERATION_GETFREE = 0,
	MT3620_EFUSE_OPERATION_READ = 1,
	MT3620_EFUSE_OPERATION_WRITE = 2,
};

enum mt3620_hqa_buffer_bin_action {
	MT3620_HQA_WRITE_BUFFER_AND_INIT = 0,
	MT3620_HQA_WRITE_BUFFER_ONLY = 1,
	MT3620_HQA_INIT_BUFFER_ONLY = 2,
};

struct mt3620_efuse_rw_info {
	u8 command : 4;
	u16 offset : 12;
	u8 data[EFUSE_BLOCK_SIZE];
	u8 len;
};

struct mt3620_versioned_cmd_args {
	u32 version;
	const void *data;
	int len;
};

void mt3620_set_vendor_commands(struct wiphy *wiphy);

int mt3620_read_country_code_efuse(struct wiphy *wiphy, char *country_code);
