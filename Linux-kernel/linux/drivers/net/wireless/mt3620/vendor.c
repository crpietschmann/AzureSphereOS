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

#include "vendor.h"
#include "mt3620_wifi_common.h"
#include "regulatory.h"

#define API_VERSION_LEN (sizeof(u32))
#define MAX_HQA_RESPONSE_LEN (EFUSE_BLOCK_SIZE)   // Maximum length of an HQA response

#define COUNTRY_CODE_ALPHA2_EFUSE_1 6
#define COUNTRY_CODE_ALPHA2_EFUSE_2 7
#define COUNTRY_CODE_BASE_OFFSET_EFUSE 0x30

static int supported_rftest_api_versions[] = { 1 };

static int mt3620_create_vendor_nl_reply(struct wiphy *wiphy,
			                 void *reply_buffer,
					 int reply_attrlen,
					 int reply_attrtype,
					 struct sk_buff **reply)
{
	int ret = SUCCESS;

	if(reply == NULL) {
		ret = -EINVAL;
		goto exit;
	}

	*reply = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_attrlen);
	if (*reply == NULL) {
		ret = -ENOMEM;
		goto exit;
	}

	if (nla_put(*reply, reply_attrtype, reply_attrlen, reply_buffer)) {
		kfree_skb(*reply);
		ret = -ENOBUFS;
		goto exit;
	}

exit:
	return ret;
}


static int mt3620_wlan_wow_config(struct wiphy *wiphy,
				  struct wireless_dev *wdev, const void *data,
				  int len)
{
	int ret = SUCCESS;
	u8 wowEnable = *(u8 *)data;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);

	if (wowEnable && wdev->netdev->ip_ptr->ifa_list) {
		wifi_hw->sta_vif->mode_data.sta.wowconf.rEnable.enable =
		    wowEnable;
		wifi_hw->sta_vif->mode_data.sta.wowconf.rMagic.enable =
		    wowEnable;
		wifi_hw->sta_vif->mode_data.sta.wowconf.rArpOffload
		    .wow_arp_enable = wowEnable;
		memcpy(wifi_hw->sta_vif->mode_data.sta.wowconf.rArpOffload
			   .ip_address,
		       &wdev->netdev->ip_ptr->ifa_list->ifa_address, 4);
	} else if (!wowEnable) {
		wifi_hw->sta_vif->mode_data.sta.wowconf.rEnable.enable =
		    wowEnable;
	} else {
		ret = -EINVAL;
		goto exit;
	}

	/* Update WOW config */
	ret = mt3620_hif_api_send_command_to_n9_sync(
	    wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_WOW_ENABLE,
	    true, &wifi_hw->sta_vif->mode_data.sta.wowconf,
	    sizeof(wifi_hw->sta_vif->mode_data.sta.wowconf), NULL, 0);
exit:
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "mt3620_wlan_wow_config: %d\n",
			ret);
	}

	return ret;
}

static int mt3620_read_efuse(struct wiphy *wiphy, u16 offset, u8 *buffer,
			     u32 buffer_size)
{
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	struct mt3620_efuse_rw_info efuse_info;

	int ret = SUCCESS;

	if (buffer_size < EFUSE_BLOCK_SIZE) {
		return -EINVAL;
	}

	if (offset > EFUSE_MAX_OFFSET) {
		return -EINVAL;
	}

	memset(&efuse_info, 0, sizeof(efuse_info));
	efuse_info.command = MT3620_EFUSE_OPERATION_READ;
	efuse_info.offset = offset;

	ret = mt3620_hif_api_send_command_to_n9_sync(
	    wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_RW_EFUSE,
	    false, &efuse_info, sizeof(efuse_info), buffer, EFUSE_BLOCK_SIZE);

	if(ret != SUCCESS){
		memset(buffer, 0, EFUSE_BLOCK_SIZE);
		ret = SUCCESS;
	}

	return ret;
}

static int mt3620_run_hqa_action(struct wiphy *wiphy,
				 struct mt3620_hqa_info *hqa_info,
				 void *reply_buffer, u32 reply_size)
{
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	int ret;
	bool set;
	u32 command_size;

	command_size = sizeof(*hqa_info);

	if (!reply_buffer) {
		set = true;
		reply_size = 0;
	} else {
		set = false;
	}

	ret = mt3620_hif_api_send_command_to_n9_sync(
	    wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_UNTET_HQA_ACTION,
	    set, (void *)hqa_info, command_size, reply_buffer, reply_size);

	return ret;
}

static int mt3620_hqa_get_data(struct wiphy *wiphy,
			       enum mt3620_vendor_subcmds command,
			       enum mt3620_vendor_attr_hqa_results attribute,
			       u32 api_version,
			       int payload_size)
{
	int ret;
	int versioned_payload_size = payload_size + API_VERSION_LEN;

	struct mt3620_hqa_info hqa_info;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	u8 out_buffer[API_VERSION_LEN + MAX_HQA_RESPONSE_LEN] = {0};
	struct sk_buff *reply;

	memcpy(out_buffer, &api_version, API_VERSION_LEN);

	memset(&hqa_info, 0, sizeof(hqa_info));
	hqa_info.command = command;
	ret = mt3620_run_hqa_action(wiphy, &hqa_info, out_buffer + API_VERSION_LEN,
				    sizeof(out_buffer) - API_VERSION_LEN);

	if (ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_create_vendor_nl_reply(wiphy, out_buffer, versioned_payload_size,
					    attribute, &reply);

	if (ret != SUCCESS) {
		goto exit;
	}

	ret = cfg80211_vendor_cmd_reply(reply);
exit:
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "Error entering getting data: %u",
			ret);
	}
	return ret;
}

static int mt3620_hqa_set_param(struct wiphy *wiphy,
				enum mt3620_vendor_subcmds command, s32 param1,
				s32 param2, s32 param3, s32 param4, s32 param5)
{
	int ret;
	struct mt3620_hqa_info hqa_info;

	/* HQA commands are sent in a fixed-sized info structure containing
	 * for 5 signed 32-bit ints, transmit frame settings, and efuse data.
	 * If these fields are unused for a given command then they should be
	 * set to 0.
	 *
	 * The function of each of the 5 integer parameters depends on the HQA 
	 * command that is being run.
	 */

	memset(&hqa_info, 0, sizeof(hqa_info));
	hqa_info.command = command;
	hqa_info.params[0] = param1;
	hqa_info.params[1] = param2;
	hqa_info.params[2] = param3;
	hqa_info.params[3] = param4;
	hqa_info.params[4] = param5;
	ret = mt3620_run_hqa_action(wiphy, &hqa_info, NULL, 0);
	return ret;
}

static int mt3620_write_buffer_bin(struct wiphy *wiphy,
				   u16 offset, 
				   enum mt3620_hqa_buffer_bin_action action,
				   u8* buffer, u32 buffer_len)
{
	int ret;
	struct mt3620_hqa_info hqa_info;

	if(buffer_len != EFUSE_BLOCK_SIZE){
		return -EINVAL;
	}

	memset(&hqa_info, 0, sizeof(hqa_info));
	memcpy(hqa_info.efuse_buffer, buffer, EFUSE_BLOCK_SIZE);

	hqa_info.command = HQA_ACT_BUFFER_BIN;
	hqa_info.params[0] = MT3620_HQA_BUFFER_WRITE;
	hqa_info.params[1] = offset;
	hqa_info.params[2] = MT3620_HQA_BUFFER_MODE;
	hqa_info.params[4] = action;

	ret = mt3620_run_hqa_action(wiphy, &hqa_info, NULL, 0);
	return ret;
}

static int mt3620_parse_rftest_version(const void *data, int len,
				       struct mt3620_versioned_cmd_args *args)
{
	int i, result;
	bool supported = false;
	u32 version;
	const u32 *params;

	if (args == NULL || data == NULL) {
		return -EINVAL;
	}

	if (len < API_VERSION_LEN) {
		return -EINVAL;
	}

	params = (const u32 *)data;
	version = params[0];

	for (i = 0; i < ARRAY_SIZE(supported_rftest_api_versions); i++) {
		if (supported_rftest_api_versions[i] == version) {
			supported = true;
			break;
		}
	}

	if (supported) {
		args->version = version;
		args->data = (const void *)(params + 1);
		args->len = len - API_VERSION_LEN;
		result = SUCCESS;
	} else {
		result = -EINVAL;
	}

	return result;
}

static int mt3620_vendor_cmds_hqa_leave(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);

	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, LEAVE_HQA_MODE, 0, 0, 0, 0, 0);

	if (ret != SUCCESS) {
		goto exit;
	}
	
	wifi_hw->in_hqa_mode = false;
exit:
	if (ret != SUCCESS){
		dev_err(wifi_hw->wifi->dev, "Error leaving HQA mode: %d", ret);
	}

	return ret;
}

static int mt3620_vendor_cmds_hqa_enter(struct wiphy *wiphy,
					struct wireless_dev *wdev,
					const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);

	/* Check if device is allowed to enter test mode */
	if (!wifi_hw->has_hqa_mode_capability) {
		ret = -EPERM;
		goto exit;
	}

	ret = mt3620_parse_rftest_version(data, len, &args);

	if (ret != SUCCESS) {
		goto exit;
	}

	/* Check if any interfaces are connected. Fail out if true */
	if ((wifi_hw->sta_vif && wifi_hw->sta_vif->state != DISCONNECTED) ||
	    (wifi_hw->ap_vif && wifi_hw->ap_vif->state != DISCONNECTED) ||
	    (wifi_hw->ibss_vif && wifi_hw->ibss_vif->state != DISCONNECTED)) {
		dev_err(wifi_hw->wifi->dev,
			"Wifi must be disconnected to enter test mode");
		ret = -EBUSY;
		goto exit;
	}

	/* Device must be in STA mode */
	if (wifi_hw->sta_vif == NULL || !wifi_hw->sta_vif->enabled) {
		dev_err(wifi_hw->wifi->dev,
			"Device must be STA mode for HQA testing");
		ret = -EBUSY;
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, ENTER_HQA_MODE, 0, 0, 0, 0, 0);

exit:
	if (ret == SUCCESS) {
		wifi_hw->in_hqa_mode = true;
	} else {
		dev_err(wifi_hw->wifi->dev, "Error entering HQA mode: %u", ret);
	}

	return ret;
}

static int mt3620_vendor_cmds_hqa_get_thermal_value(struct wiphy *wiphy,
						    struct wireless_dev *wdev,
						    const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret =
	    mt3620_hqa_get_data(wiphy, HQA_ACT_GET_THERMAL_VALUE,
				MT3620_VENDOR_ATTR_THERMAL_VALUE, 
				args.version,
				sizeof(s32));
exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_reset_counters(struct wiphy *wiphy,
						 struct wireless_dev *wdev,
						 const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_RESET_TRX_COUNTER, 0, 0, 0, 0,
			0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_get_rx_all_packets(struct wiphy *wiphy,
						     struct wireless_dev *wdev,
						     const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret =
	    mt3620_hqa_get_data(wiphy, HQA_ACT_GET_RX_ALL_PKT,
				MT3620_VENDOR_ATTR_RX_ALL_PACKETS,
				args.version,
				sizeof(u32));

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_get_rx_unicast_data_packets(
    struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_get_data(wiphy, HQA_ACT_GET_RX_U2M_DATA,
				  MT3620_VENDOR_ATTR_RX_U2M_DATA_PACKETS,
				  args.version,
				  sizeof(u32));

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_get_rx_ok_data_packets(
    struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_get_data(wiphy, HQA_ACT_GET_RX_OK_DATA,
				  MT3620_VENDOR_ATTR_RX_OK_DATA_PACKETS,
				  args.version,
				  sizeof(u32));

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_get_rx_ok_other_packets(
    struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_get_data(wiphy, HQA_ACT_GET_RX_OK_OTHER,
				  MT3620_VENDOR_ATTR_RX_OK_OTHER_PACKETS,
				  args.version,
				  sizeof(u32));

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_get_tx_packets(struct wiphy *wiphy,
						 struct wireless_dev *wdev,
						 const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret =
	    mt3620_hqa_get_data(wiphy, HQA_ACT_GET_TX_TRANSMITTED,
				MT3620_VENDOR_ATTR_TX_TRANSMITTED,
				args.version,
				sizeof(u32));

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_get_cw_tone_rssi(struct wiphy *wiphy,
						   struct wireless_dev *wdev,
						   const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_get_data(wiphy, HQA_ACT_GET_CW_TONE_RSSI,
				  MT3620_VENDOR_ATTR_CW_TONE_RSSI,
				  args.version,
				  sizeof(u32));

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_get_efuse_total_blocks(
    struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_get_data(wiphy, HQA_ACT_GET_EFUSE_BLK_NUM,
				  MT3620_VENDOR_ATTR_EFUSE_TOTAL_BLOCKS,
				  args.version,
				  sizeof(u32));

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_get_efuse_free_blocks(
    struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_get_data(wiphy, HQA_ACT_GET_FREE_EFUSE_BLK,
				  MT3620_VENDOR_ATTR_EFUSE_FREE_BLOCKS,
				  args.version,
				  sizeof(u32));

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_set_antenna_diversity(
    struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	const int num_params = 2;
	const s32 *params;
	s32 band;
	s32 ant_setting;
	u8 cmd_buf[2];
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	band = params[0];
	ant_setting = params[1];

	cmd_buf[0] = (u8)band;
	cmd_buf[1] = (u8)ant_setting;

	ret = mt3620_hif_api_send_command_to_n9_sync(
	    wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_ANT_MODE,
	    true, cmd_buf, 2, NULL, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_set_test_mac_addr(struct wiphy *wiphy,
					        struct wireless_dev *wdev,
					        const void *data, int len)
{
	int ret = SUCCESS;
	struct mt3620_versioned_cmd_args args;
	
	struct mt3620_wifi_set_mac_addr mac;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);

	mutex_lock(&wifi_hw->mutex);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}

	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}
	
	if(args.len != ETH_ALEN) {
		ret = -EINVAL;
		goto exit;
	}

	mac.port = WIFI_PORT_STA;
	memcpy(mac.mac_addr, args.data, ETH_ALEN); 
	ret = mt3620_wifi_set_mac_helper(wifi_hw, &mac);

	if(ret != SUCCESS) {
		goto exit;
	}

exit:
	mutex_unlock(&wifi_hw->mutex);
	return ret;
}

static int mt3620_vendor_cmds_hqa_set_tx_aifs(struct wiphy *wiphy,
					      struct wireless_dev *wdev,
					      const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	const int num_params = 1;
	const s32 *params;
	s32 aifs;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	aifs = params[0];

	/* Arbitration inter-frame spacing supports 4-100us */
	if (aifs < 4 || aifs > 100) {
		ret = -EINVAL;
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_SET_AIFS, aifs, 0, 0, 0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_set_channel(struct wiphy *wiphy,
					      struct wireless_dev *wdev,
					      const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	const int num_params = 1;
	const s32 *params;
	s32 channel;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	channel = params[0];

	if (channel < 0) {
		ret = -EINVAL;
		goto exit;
	}

	dev_dbg(wifi_hw->wifi->dev, "Selecting channel %d", channel);
	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_SET_CHANNEL, channel, 0, 0, 0,
				   0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_set_tx_rate(struct wiphy *wiphy,
					      struct wireless_dev *wdev,
					      const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	const int num_params = 2;
	const s32 *params;
	s32 modulation, rate;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	modulation = params[0];
	rate = params[1];

	if (modulation >= MT3620_HQA_TX_MOD_LAST) {
		ret = -EINVAL;
		goto exit;
	}

	if (rate >= MT3620_HQA_RATE_LAST) {
		ret = -EINVAL;
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_SET_RATE, modulation, rate, 0,
				   0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_set_tx_power(struct wiphy *wiphy,
					       struct wireless_dev *wdev,
					       const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	const int num_params = 1;
	const s32 *params;
	s32 power;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	power = params[0];

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_SET_TX_POWER, (u32)power, 0,
				   0, 0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_set_tx_frame(struct wiphy *wiphy,
					       struct wireless_dev *wdev,
					       const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	struct mt3620_hqa_info hqa_info;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(struct mt3620_hqa_tx_frame_setting)) {
		ret = -EINVAL;
		goto exit;
	}

	memset(&hqa_info, 0, sizeof(struct mt3620_hqa_info));
	hqa_info.command = HQA_ACT_SET_TX_FRAME;
	memcpy(&(hqa_info.tx_setting), args.data, args.len);
	ret = mt3620_run_hqa_action(wiphy, &hqa_info, NULL, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_start_tx(struct wiphy *wiphy,
					   struct wireless_dev *wdev,
					   const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	const int num_params = 4;
	const s32 *params;
	u32 count;
	s32 bytes_to_send, random, content;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;

	count = (u32)params[0];
	bytes_to_send = params[1];
	random = params[3];
	content = params[4];

	/* Max size is 2^16 */
	bytes_to_send &= 0xFFFF;

	/*
	 * Command parameter actually corresponds to
	 * number of additional bytes to send beyond 26.
	 */
	if (bytes_to_send <= 26) {
		bytes_to_send = 0;
	} else {
		bytes_to_send -= 26;
	}

	if (random) {
		random = 1;
	}

	/* Content needs to be a single byte */
	content &= 0xFF;

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_START_TX, (s32)count,
				   bytes_to_send, random, content, 0);
exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_stop_tx(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_STOP_TX, 0, 0, 0, 0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_start_rx(struct wiphy *wiphy,
					   struct wireless_dev *wdev,
					   const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_START_RX, 0, 0, 0, 0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_stop_rx(struct wiphy *wiphy,
					  struct wireless_dev *wdev,
					  const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_STOP_RX, 0, 0, 0, 0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_enter_test_mode(struct wiphy *wiphy,
						  struct wireless_dev *wdev,
						  const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, ENTER_TEST_MODE, 0, 0, 0, 0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_exit_test_mode(struct wiphy *wiphy,
						 struct wireless_dev *wdev,
						 const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, EXIT_TEST_MODE, 0, 0, 0, 0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_start_cw_tone(struct wiphy *wiphy,
						struct wireless_dev *wdev,
						const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	const int num_params = 1;
	const s32 *params;
	s32 freq_offset;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}

	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len / sizeof(s32) != num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	freq_offset = params[0];

	if (freq_offset >= MT3620_HQA_TONE_FREQ_LAST) {
		ret = -EINVAL;
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_START_CW_TONE, freq_offset, 0,
				   0, 0, 0);
exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_stop_cw_tone(struct wiphy *wiphy,
					       struct wireless_dev *wdev,
					       const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_STOP_CW_TONE, 0, 0, 0, 0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_start_continuous_tx(struct wiphy *wiphy,
						      struct wireless_dev *wdev,
						      const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_SET_TX_CONTI_START, 0, 0, 0,
				   0, 0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_stop_continuous_tx(struct wiphy *wiphy,
						     struct wireless_dev *wdev,
						     const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_SET_TX_CONTI_STOP, 0, 0, 0, 0,
				   0);

exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_set_buffer_mode(struct wiphy *wiphy,
						  struct wireless_dev *wdev,
						  const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	const int num_params = 1;
	const s32 *params;
	s32 buffer_mode;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}

	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;

	/* EFuse = 0, Buffer = 1 */
	buffer_mode = params[0];
	if (buffer_mode != 0) {
		buffer_mode = MT3620_HQA_BUFFER_MODE;
	} else {
		buffer_mode = MT3620_HQA_EFUSE_MODE;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_BUFFER_BIN, 0, 0, buffer_mode,
				   0, 0);
exit:
	return ret;
}

static int mt3620_vendor_cmds_hqa_read_buffer_bin(struct wiphy *wiphy,
						  struct wireless_dev *wdev,
						  const void *data, int len)
{
	int ret;
	const int versioned_response_len = EFUSE_BLOCK_SIZE + API_VERSION_LEN;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_hqa_info hqa_info;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);

	struct sk_buff *reply = NULL;
	u8 out_buffer[API_VERSION_LEN + EFUSE_BLOCK_SIZE] = {0};

	const int num_params = 1;
	const s32 *params;
	s32 offset;
	
	if (!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}

	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	offset = params[0];

	/* offset is a 16-bit address, multiple of 16-bytes */
	offset &= 0xFFF0;

	dev_dbg(wifi_hw->wifi->dev, "Reading buffer-bin offset %04X", offset);

	memcpy(out_buffer, &args.version, API_VERSION_LEN);

	memset(&hqa_info, 0, sizeof(hqa_info));
	hqa_info.command = HQA_ACT_BUFFER_BIN;
	hqa_info.params[0] = MT3620_HQA_BUFFER_READ;
	hqa_info.params[1] = offset;
	hqa_info.params[2] = MT3620_HQA_BUFFER_MODE;

	ret = mt3620_run_hqa_action(wiphy, &hqa_info, out_buffer + API_VERSION_LEN,
				    sizeof(out_buffer) - API_VERSION_LEN);

	if (ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_create_vendor_nl_reply(wiphy, out_buffer, versioned_response_len,
					    MT3620_VENDOR_ATTR_BUFFER_BIN_DATA, 
					    &reply);

	if (ret != SUCCESS) {
		goto exit;
	}

	ret = cfg80211_vendor_cmd_reply(reply);

exit:
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "Error reading buffer bin: %u",
			ret);
	}
	return ret;
}

int mt3620_read_country_code_efuse(struct wiphy *wiphy, char *country_code)
{
    // We can only read in 16-byte chunks from the EFUSE
    u8 out_buffer[16];

    int ret = mt3620_read_efuse(wiphy, COUNTRY_CODE_BASE_OFFSET_EFUSE, out_buffer, 
                sizeof(out_buffer));

    if(ret != SUCCESS) {
        return ret;
    }

    // Pick just the country code data from 0x36 and 0x37 location
    country_code[0] = out_buffer[COUNTRY_CODE_ALPHA2_EFUSE_1];
    country_code[1] = out_buffer[COUNTRY_CODE_ALPHA2_EFUSE_2];
    country_code[2] = '\0';

    return SUCCESS;
}

static int mt3620_vendor_cmds_read_efuses(struct wiphy *wiphy,
					 struct wireless_dev *wdev,
					 const void *data, int len)
{
	int ret;
	const int versioned_response_len = EFUSE_BLOCK_SIZE + API_VERSION_LEN;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);

	struct sk_buff *reply;
	u8 out_buffer[API_VERSION_LEN + EFUSE_BLOCK_SIZE] = {0};

	const int num_params = 1;
	const s32 *params;
	s32 offset;

	mutex_lock(&wifi_hw->mutex);
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}
	
	if (args.len != sizeof(s32) * num_params) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	offset = params[0];

	/* offset is a 16-bit address, multiple of 16 bytes */
	offset &= 0xFFF0;

	memcpy(out_buffer, &args.version, API_VERSION_LEN);

	ret = mt3620_read_efuse(wiphy, offset, out_buffer + API_VERSION_LEN, 
			        sizeof(out_buffer) - API_VERSION_LEN);
	
	if(ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "N9 error when reading efuse: %d",
			ret);
		ret = -ENOSYS;
		goto exit;
	}
	
	ret = mt3620_create_vendor_nl_reply(wiphy, out_buffer, versioned_response_len,
					    MT3620_VENDOR_ATTR_BUFFER_BIN_DATA, 
					    &reply);

	if (ret != SUCCESS) {
		goto exit;
	}

	ret = cfg80211_vendor_cmd_reply(reply);
exit:
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "Error reading efuses: %d",
			ret);
	}
	mutex_unlock(&wifi_hw->mutex);
	return ret;	

}

static int mt3620_vendor_cmds_hqa_write_buffer_bin(struct wiphy *wiphy,
						   struct wireless_dev *wdev,
						   const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	const int num_params = 2;
	const s32 *params;
	s32 offset;
	s32 action;

	if(!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	if (args.len != sizeof(s32)*num_params + EFUSE_BLOCK_SIZE) {
		ret = -EINVAL;
		goto exit;
	}

	params = (const s32 *)args.data;
	offset = params[0];
	action = params[1]; 

	/* Offset is a 16-bit address */
	offset &= 0xFFFF;

	ret = mt3620_write_buffer_bin(wiphy, (u16)offset, action, (u8 *)(params + num_params), EFUSE_BLOCK_SIZE);

exit:
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "Error writing buffer bin : %u",
			ret);
	}
	return ret;
}

static int mt3620_vendor_cmds_hqa_buffer_reinit(
    struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if(!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(
	    wiphy, HQA_ACT_BUFFER_BIN, MT3620_HQA_BUFFER_WRITE, 0,
	    MT3620_HQA_BUFFER_MODE, 0, MT3620_HQA_INIT_BUFFER_ONLY);
exit:
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev,
			"Error re-initializing N9 with new buffer bin data: %u",
			ret);
	}
	return ret;
}

static int mt3620_vendor_cmds_hqa_write_buffer_to_efuse(
    struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	int ret;
	struct mt3620_versioned_cmd_args args;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	
	if(!wifi_hw->in_hqa_mode) {
		ret = -EPERM;
		goto exit;
	}
	
	ret = mt3620_parse_rftest_version(data, len, &args);

	if(ret != SUCCESS) {
		goto exit;
	}

	ret = mt3620_hqa_set_param(wiphy, HQA_ACT_BUFFER_BIN,
				   MT3620_HQA_BUFFER_READ, 0,
				   MT3620_HQA_BUFFER_MODE,
				   1, // write value to e-fuses
				   0);
exit:
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "Error writing buffer bin : %u",
			ret);
	}
	return ret;
}

///
/// Call Location : this function will call when the CM4 successfully reset the N9
//					so it should call from the Ca7-Cm4 mbox rx interrupt
/// Work          : it will init the hif,dma and patch,fw download, wifi settings for STA only
/// @wiphy - wiphy device
/// @returns - 0 for success

static int mt3620_n9_reset_done(struct wiphy *wiphy, struct wireless_dev *wdev,
		    const void *data, int data_len)
{
	struct mt3620_wifi_hw *wifi_hw = g_wifi_hw;
	int ret = SUCCESS;
	u8 mode = WIFI_MODE_STA_ONLY;
	u8 onoff = 1;
	struct mt3620_wifi_set_mac_addr mac;

	if(wifi_hw->sta_vif == NULL)
	{
		return -1;
	}
	//init the HIF
	mt3620_hif_open();
	//Add the DMA setting if required

	//start the fw download
	mt3620_hif_fw_download();
	mt3620_hif_enable_interrupt();

	ret = mt3620_hif_api_send_command_to_n9_sync(
	    wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_IOT_SWITCH_MODE, true,
	    &mode, sizeof(u8), NULL, 0);
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "[mt3620_n9_reset_done]Error setting STA mode: %#x", ret);
		goto exit;
	}

	ret = mt3620_hif_api_send_command_to_n9_sync(
	    wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_RADIO_ON_OFF,
	    true, &onoff, sizeof(u8), NULL, 0);
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "[mt3620_n9_reset_done]Error setting radio on: %#x", ret);
		goto exit;
	}

	mac.port = WIFI_PORT_STA;
	memcpy(mac.mac_addr, wifi_hw->sta_vif->ndev->dev_addr, ETH_ALEN);
	if (!wifi_hw->sta_vif->is_mac_set) {
		ret = mt3620_hif_api_send_command_to_n9_sync(
			g_wifi_hw->wifi->hif_api_handle,
			WIFI_COMMAND_ID_IOT_OWN_MAC_ADDRESS, true, &mac,
			sizeof(struct mt3620_wifi_set_mac_addr), NULL, 0);
		if (ret != SUCCESS) {
			dev_err(wifi_hw->wifi->dev, "[mt3620_n9_reset_done]Could not set MAC to %pM: %u", wifi_hw->sta_vif->ndev->dev_addr, ret);
			goto exit;
		}
		wifi_hw->sta_vif->is_mac_set = true;
	}

	//country mode settings
	ret = mt3620_wifi_apply_country_code_regulatory_rules();
	if (ret != SUCCESS) {
		dev_err(wifi_hw->wifi->dev, "[mt3620_n9_reset_done] mt3620_wifi_apply_regulatory_changes failed: %u", ret);
		goto exit;
	}

	wifi_hw->n9_initialized = true;
exit:
	if (ret != SUCCESS) {
			dev_err(wifi_hw->wifi->dev, "[mt3620_n9_reset_done] Failed %d\n",ret);
	}
	return ret;
}

///
/// Enable and set the WDT timer of the N9 [wifi]
///
/// @wiphy - wiphy device
/// @wdev - wireless device
/// @data - parameters passed from the vendor command
///			1nd byte - enable = 1 to enable the WDT timer
///							  = 0 to disable the WDT timer
///			2rd to 5th byte   = length of the WDT timer required when enable=1
///  example of command : vendor 123456 01 0111000000
/// @len - length of the parameters it must be 5
/// @returns - 0 for success

static int mt3620_wifi_set_N9_WDT(struct wiphy *wiphy,
                                                struct wireless_dev *wdev,
                                                const void *data, int len)
{
	u8 enable =  *((u8*)data);
	u32 wdttime = 0;
	struct mt3620_wifi_hw *wifi_hw = wiphy_priv(wiphy);
	int ret = SUCCESS;

	if(len !=5)
	{
		ret = -EINVAL;
		goto exit;
	}
	dev_dbg(wifi_hw->wifi->dev, "[mt3620_wifi_set_N9_WDT] enable=%d wdttime=%d[0x%x]\n",enable,wdttime,wdttime);
	ret = mt3620_hif_api_send_command_to_n9_sync(
		wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_N9WDT_ON_OFF,
		true, &enable, sizeof(enable), NULL, 0);
	if(ret != SUCCESS)
	{
		dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_set_N9_WDT] on_off failed %d\n",ret);
		ret = -EBUSY;
		goto exit;
	}
	if(enable)
	{
		wdttime = *((u32*)(data+1));
		if(wdttime >= 0x7ff)
		{
			dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_set_N9_WDT] wdttimw=0x%x N9 will hit Assert\n",wdttime);
		}
		ret = mt3620_hif_api_send_command_to_n9_no_response(
			wifi_hw->wifi->hif_api_handle, WIFI_COMMAND_ID_EXTENSION_N9WDT_LENGTH,
			true, &wdttime, sizeof(wdttime));
		if(ret != SUCCESS)
		{
			dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_set_N9_WDT] length set failed %d\n",ret);
			ret = -EBUSY;
			goto exit;
		}
	}
exit:
	if (ret != SUCCESS) {
			dev_err(wifi_hw->wifi->dev, "[mt3620_wifi_set_N9_WDT] Failed %d\n",ret);
	}
	return ret;
}

///
/// Call Location : this function will call when we receive the WDT trigger Notification from the CM4
///                 So it should call from the Ca7-CM4 mailbox rx interrupt
/// Work          : this functin should block all the Tx from the upper layer and hif layer
///					indicate the disconnect in case of STA to the Host
///					this function will prepare the wifi driver for N9 ,Hif ,DMA reset
/// @wiphy - wiphy device
/// @returns - 0 for success

static int mt3620_n9_WDT_trigger(struct wiphy *wiphy, struct wireless_dev *wdev,
		    const void *data, int data_len)
{

	dev_dbg(g_wifi_hw->wifi->dev, "[mt3620_n9_WDT_trigger] indicate disconnect\n");
	//stop sending any command to hif
	g_wifi_hw->n9_initialized = false;
	//reset the mac flag
	g_wifi_hw->sta_vif->is_mac_set = false;
	//send the disconnect to upper layer
	mt3620_wifi_disconnect_if_connected(g_wifi_hw->sta_vif);

	//we can also stop the cfg layer but it can also stop the vend command interface
	//so we will add this feature when we mbox is up and we dont need the vend cmd interface
	// in that case we have to enable the cfg layer at the end of mt3620_n9_reset_done function
	//TODO

	//write a function for the mt3620_hif_close to protect the hif if required
	//it is not required at this time.
	return 0;
}

static const struct wiphy_vendor_command mt3620_vendor_commands[] = {
    {
	.info =
	    {
		.vendor_id = MT3620_OUI,
		.subcmd = MT3620_VENDOR_SUBCMD_WOW_CONFIG,
	    },
	.flags = WIPHY_VENDOR_CMD_NEED_NETDEV | WIPHY_VENDOR_CMD_NEED_RUNNING,
	.doit = mt3620_wlan_wow_config,
    },
	{
		.info = {
			.vendor_id = MT3620_OUI,
			.subcmd = MT3620_SET_N9_WDT,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = mt3620_wifi_set_N9_WDT,
	},
	{
		.info = {
			.vendor_id = MT3620_OUI,
			.subcmd = MT3620_N9_WDT_TRIGGER,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = mt3620_n9_WDT_trigger,
	},
	{
		.info = {
			.vendor_id = MT3620_OUI,
			.subcmd = MT3620_N9_RESET_DONE,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = mt3620_n9_reset_done,
	},
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_ENTER,
			  mt3620_vendor_cmds_hqa_enter),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_LEAVE,
			  mt3620_vendor_cmds_hqa_leave),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_THERMAL_VALUE,
			  mt3620_vendor_cmds_hqa_get_thermal_value),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_RESET_COUNTERS,
			  mt3620_vendor_cmds_hqa_reset_counters),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_RX_ALL_PKT,
			  mt3620_vendor_cmds_hqa_get_rx_all_packets),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_RX_UNICAST_DATA,
			  mt3620_vendor_cmds_hqa_get_rx_unicast_data_packets),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_RX_OK_DATA,
			  mt3620_vendor_cmds_hqa_get_rx_ok_data_packets),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_RX_OK_OTHER,
			  mt3620_vendor_cmds_hqa_get_rx_ok_other_packets),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_TX,
			  mt3620_vendor_cmds_hqa_get_tx_packets),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_CW_TONE_RSSI,
			  mt3620_vendor_cmds_hqa_get_cw_tone_rssi),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_EFUSE_TOTAL_BLOCKS,
			  mt3620_vendor_cmds_hqa_get_efuse_total_blocks),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_GET_EFUSE_FREE_BLOCKS,
			  mt3620_vendor_cmds_hqa_get_efuse_free_blocks),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_SET_ANTENNA_DIVERSITY,
			  mt3620_vendor_cmds_hqa_set_antenna_diversity),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_SET_TX_AIFS,
			  mt3620_vendor_cmds_hqa_set_tx_aifs),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_SET_CHANNEL,
			  mt3620_vendor_cmds_hqa_set_channel),
    //  TODO: Xtal trimming not yet functioning
    //  MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_SET_FREQ_OFFSET,
    //			  mt3620_vendor_cmds_hqa_set_freq_offset),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_SET_TX_RATE,
			  mt3620_vendor_cmds_hqa_set_tx_rate),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_SET_TX_POWER,
			  mt3620_vendor_cmds_hqa_set_tx_power),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_SET_TX_FRAME,
			  mt3620_vendor_cmds_hqa_set_tx_frame),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_START_TX,
			  mt3620_vendor_cmds_hqa_start_tx),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_STOP_TX,
			  mt3620_vendor_cmds_hqa_stop_tx),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_START_RX,
			  mt3620_vendor_cmds_hqa_start_rx),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_STOP_RX,
			  mt3620_vendor_cmds_hqa_stop_rx),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_ENTER_TEST_MODE,
			  mt3620_vendor_cmds_hqa_enter_test_mode),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_EXIT_TEST_MODE,
			  mt3620_vendor_cmds_hqa_exit_test_mode),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_START_CW_TONE,
			  mt3620_vendor_cmds_hqa_start_cw_tone),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_STOP_CW_TONE,
			  mt3620_vendor_cmds_hqa_stop_cw_tone),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_START_CONTI_TX,
			  mt3620_vendor_cmds_hqa_start_continuous_tx),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_STOP_CONTI_TX,
			  mt3620_vendor_cmds_hqa_stop_continuous_tx),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_SET_BUFFER_MODE,
			  mt3620_vendor_cmds_hqa_set_buffer_mode),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_READ_BUFFER_BIN,
			  mt3620_vendor_cmds_hqa_read_buffer_bin),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_WRITE_BUFFER_BIN,
			  mt3620_vendor_cmds_hqa_write_buffer_bin),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_WRITE_BUFFER_TO_EFUSE,
			  mt3620_vendor_cmds_hqa_write_buffer_to_efuse),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_READ_EFUSE,
			  mt3620_vendor_cmds_read_efuses),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_SET_TEST_MACADDR,
			  mt3620_vendor_cmds_set_test_mac_addr),
    MT3620_HQA_VENDOR_CMD(MT3620_VENDOR_SUBCMD_HQA_BUFFER_BIN_REINIT,
			  mt3620_vendor_cmds_hqa_buffer_reinit)
};

void mt3620_set_vendor_commands(struct wiphy *wiphy)
{
	wiphy->vendor_commands = mt3620_vendor_commands;
	wiphy->n_vendor_commands = ARRAY_SIZE(mt3620_vendor_commands);
}
