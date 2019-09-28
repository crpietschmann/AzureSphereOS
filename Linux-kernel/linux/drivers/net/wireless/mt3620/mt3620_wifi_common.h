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


/** @brief The max number of keys
*/
#define WLAN_MAX_KEY_COUNT 4


/** @brief This value depends upon the hifproc structure n9_wifi_data_header
* Block size defined by the M4 for N9 communication
*/
#define N9_DATA_HEADER_LEN 12 


/** @brief 64
* this is a duplicate,if there is any change in original one , should also update in this
*/
#define N9_TX_BLOCK_SIZE  64 


/** @brief Max length of sequence number
*/
#define MAX_PN_NUMBER_LEN 16

#define CH_ACTIVE_SCAN 0
#define CH_PASSIVE_SCAN  1

/** @brief Port types*/
enum mt3620_wifi_vif_port {
	// Port for our STA client
	WIFI_PORT_STA = 0,
	// Port for our AP
	WIFI_PORT_AP = 1,
	// Port for our IBSS
	WIFI_PORT_IBSS = 2,
};


/** @brief Type of interfaces supported
*/
enum mt3620_wifi_interface {
	// Port for our STA client
	WIFI_INTERFACE_STA = 0,
	// Port for our AP
	WIFI_INTERFACE_AP = 1,
	//IBSS
	WIFI_INTERFACE_IBSS = 2,
	//APCLI
	WIFI_INTERFACE_APCLI = 3,
};


/** @brief Cached key info
*/
struct mt3620_wifi_key_info {
	// Cipher algorithm
	u32 cipher;
	// Key data
	u8 key[WLAN_KEY_LEN_TKIP];
	// Key length
	u8 key_len;
	// Sequence counter (IV) data
	u8 seq[WLAN_KEY_LEN_TKIP];
	// Sequence counter length
	u8 seq_len;
};


/** @brief Enable/Disable wow
*/
typedef struct {
    uint8_t enable; /**< 0: disable, 1:enable */
} wifi_wow_enable_t;


/** @brief Wake up host by magic packet.
*/
typedef struct {
    uint8_t bss_index; /**< BSS Index (BIT(0) enable for BSSID0, BIT(1) enable for BSSID1, BIT(2) enable for BSSID2, BIT(3) enable for BSSID3) */
    uint8_t enable; /**< 0: disable, 1:enable the magic packet filter */
} wifi_wow_magic_packet_t;


/** @brief @brief ARP offloading.
*/
typedef struct {
    uint8_t wow_arp_enable;
    uint8_t ip_address[4]; /**< e.g.192.168.0.11-->ip_address[]={192, 168, 0, 11} */
} wifi_wow_arp_offload_t;

/** @brief Re-key offloading.
*/
typedef struct {
    uint8_t wow_rekey_enable;
	uint8_t kck[16];
    uint8_t kek[16]; /**< 64 bytes Pairwise Transient Key */
} wifi_wow_rekey_offload_t;


/** @brief WoW Configuration structure
*/
typedef struct {
   wifi_wow_enable_t rEnable;
   wifi_wow_arp_offload_t rArpOffload;
   wifi_wow_rekey_offload_t rRekey;
   wifi_wow_magic_packet_t rMagic;
} wifi_wow_configure_t;


/** @brief N9 debug levels
*/
enum mt3620_wifi_n9_debug_level {
	N9_DEBUG_LEVEL_NONE = 0,
	N9_DEBUG_LEVEL_ERROR = 1,
	N9_DEBUG_LEVEL_WARNING = 2,
	N9_DEBUG_LEVEL_TRACE = 3,
	N9_DEBUG_LEVEL_INFO = 4,
	N9_DEBUG_LEVEL_LOUD = 5,
};


/** @brief Max memory pool size
*/
#define MEM_POOL_MAX_SIZE  50


/** @brief Transmit descriptor*/
struct mt3620_tx_desc {
		u32 *ar[MEM_POOL_MAX_SIZE];
		u32 get;
		u32 put;
		u32 msg_count;
};


/** @brief Our basic device state structure
*/
struct mt3620_wifi {
	// The underlying linux kernel device
	struct device *dev;
	// Pointer to our WIPHY device
	struct wiphy *wiphy;
	// Our hif API handle
	void *hif_api_handle;

#ifdef CONFIG_MT3620_PIO_DATA
	// Bottom half of TX data handler
	struct work_struct write_work;
	//Active VIF
	struct mt3620_wifi_vif *act_vif;

	spinlock_t data_lock;
	struct mt3620_tx_desc mt3620_txdesc;
#endif
};


/** @brief CRC check status codes
*/
enum mt3620_wifi_crc_status { FAILURE = -1, SUCCESS = 0, NOT_SUPPORTED = 1 };


/** @brief Data structures that are specific to Wi-Fi mailbox commands
 * 
*/
#define IFNAMESIZE 16


/** @brief Generic parameter format for things that fit in an int
* These definitions all come from the M4 firmware, but they're data formats
* @brief sent by the N9
*/
struct iw_param {
	// Parameter value
	s32 value;
	// Hardware should not use auto select
	u8 fixed;
	// Disable the feature
	u8 disabled;
	// Flags that modify this value
	u16 flags;
};


/** @brief Pointer to user space data
*/
struct iw_point {
	// Data pointer
	void *pointer;
	// Size of data
	u16 length;
	// Flags
	u16 flags;
};


/** @brief Encoded frequency data
*/
struct iw_freq {
	// Mantissa
	s32 m;
	// Exponent
	s16 e;
	// List index
	u8 i;
	// Flags
	u8 flags;
};


/** @brief Quality of link data
*/
struct iw_quality {
	// Link quality
	u8 qual;
	// Signal level
	u8 level;
	// Signal noise
	u8 noise;
	// Flags on what was updated
	u8 updated;
};


/** @brief Socket address private structure
*/
struct sockaddr_1 {
	// Address family
	u16 sa_family;
	// Octet data of address
	u8 sa_data[14];
};


/** @brief Incoming Wi-Fi firmware events use this union
*/
union iwreq_data {
	// Generic payload
	char name[IFNAMESIZE];
	// Extended network name
	struct iw_point essid;
	// Network ID
	struct iw_param nwid;
	// Frequency or channel
	struct iw_freq freq;
	// Signal level threshold
	struct iw_param sens;
	// Default bit rate
	struct iw_param bitrate;
	// Default transmit power
	struct iw_param txpower;
	// RTS threshold
	struct iw_param rts;
	// Fragmentation threshold
	struct iw_param frag;
	// Operation mode
	u32 mode;
	// Retry limits and lifetime
	struct iw_param retry;
	// Encoding settings
	struct iw_param encoding;
	// PM duration / timeout
	struct iw_param power;
	// Quality statistics
	struct iw_quality qual;
	// Access point address
	struct sockaddr_1 ap_addr;
	// Destination address
	struct sockaddr_1 addr;
	// Other small parameters
	struct iw_param param;
	// Other large parameters
	struct iw_point data;
};

#define IWEVDISASSOC 0x8B15
#define IWEVCUSTOM 0x8C02
#define IWEVASSOCREQIE 0x8C07
#define IWEVREASSOCREQIE 0x8C0A

/** @brief Flags set in the iwreq_data structure, specifying message types
*/
enum iwreq_data_flags {
	// Associated
	RT_ASSOC_EVENT_FLAG = 0x101,
	// Disassociated
	RT_DISASSOC_EVENT_FLAG = 0x102,
	// Associate req / resp IEs
	RT_REQ_IE_EVENT_FLAG = 0x103,
	RT_RESP_IE_EVENT_FLAG = 0x104,
	// Association info
	RT_ASSOCINFO_EVENT_FLAG = 0x105,
	// Custom events that fires when we start / stop our interface
	RT_INTERFACE_DOWN = 0x107,
	RT_INTERFACE_UP = 0x108,
	// Association event
	IW_ASSOC_EVENT_FLAG = 0x200,
	// Disassociation event
	IW_DISASSOC_EVENT_FLAG = 0x201,
	// Deauth event
	IW_DEAUTH_EVENT_FLAG = 0x202,
	// Link is up and ready for use
	IW_STA_LINKUP_EVENT_FLAG = 0x20f,
	// MIC error
	IW_MIC_ERROR_EVENT_FLAG = 0x0209,
	// Custom event that fires when we lose our link
	IW_STA_LINKDOWN_EVENT_FLAG = 0x210,
	// Requested scan has completed
	IW_STA_SCAN_COMPLETED_EVENT_FLAG = 0x211,
};


/** @brief Wifi power modes
*/
enum mt3620_wifi_power_mode {
	// Constantly awake mode
	POWER_MODE_CAM = 0,
	// Power saving mode
	POWER_MODE_FAST_PSP = 2
};


/** @brief PHY modes - combinations of supported wireless standards
*/
enum mt3620_wifi_phy_mode {
	WIFI_PHY_11BG_MIXED = 0,
	WIFI_PHY_11B = 1,
	WIFI_PHY_11A = 2,
	WIFI_PHY_11ABG_MIXED = 3,
	WIFI_PHY_11G = 4,
	WIFI_PHY_11ABGN_MIXED = 5,
	WIFI_PHY_11N_2_4G = 6,
	WIFI_PHY_11GN_MIXED = 7,
	WIFI_PHY_11AN_MIXED = 8,
	WIFI_PHY_11BGN_MIXED = 9,
	WIFI_PHY_11AGN_MIXED = 10,
	WIFI_PHY_11N_5G = 11
};


/** @brief Wireless mode protocol struct
*/
struct mt3620_wifi_wireless_mode {
	u8 port;
	u8 mode;
};


/** @brief Struct to set current channel
*/
struct mt3620_wifi_set_channel {
	u8 port;
	u8 channel;
};


/** @brief Struct to set current MAC address
*/
struct mt3620_wifi_set_mac_addr {
	u8 port;
	u8 mac_addr[ETH_ALEN];
};


/** @brief Struct to set current bssid
*/
struct mt3620_wifi_set_bssid {
	u8 port;
	u8 bssid[ETH_ALEN];
};


/** @brief Struct to get current ssid
*/
struct mt3620_wifi_get_ssid {
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
};


/** @brief Struct to set current ssid
*/
struct mt3620_wifi_set_ssid {
	u8 port;
	u8 ssid_len;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
};


/** @brief Extra IE types
*/
enum mt3620_wifi_extra_ie_type {
	WIFI_EXTRA_WPS_BEACON_IE = 0,
	WIFI_EXTRA_WPS_PROBE_REQ_IE = 1,
	WIFI_EXTRA_WPS_PROBE_RSP_IE = 2,
	WIFI_EXTRA_WPS_ASSOC_REQ_IE = 3,
	WIFI_EXTRA_WPS_ASSOC_RSP_IE = 4
};


/** @brief An IE is a 802.11 Information Element.
*  Each 802.11 packet has a variable number of IEs of various types in it.
*  CFG80211 my ask us to include extra IEs in specific messages.
*  This structure tells the N9 about the IE we want added to requests.
*/
struct mt3620_wifi_extra_ie {
	// N9 port to apply to
	u8 port;
	// Extra IE Type
	u8 type;
	// Reserved / padding
	u8 reserved[2];
	// Length of IE block
	u32 ie_len;
	// IE payload
	u8 ie[0];
};


/** @brief Reserved sizes for qos TX headers
*/
#define QOS_RESERVED_SIZE 42


/** @brief Reserved sizes for non qos TX headers
*/
#define NON_QOS_RESERVED_SIZE 38


/** @brief Highest 2.4 Ghz channel number
*/
#define MAX_2GHZ_CHANNEL 14


/** @brief Scan channel data
*/
struct mt3620_wifi_channel_desc {
	// First channel number
	u8 first_channel;
	// Number of consecutive channels
	u8 num_of_channel;
	// 0 - Active, 1 - Passive
	u8 channel_properties;
	// Reserved for alignment
	u8 reserved;
};


/** @brief Scan channel list
*/
struct mt3620_wifi_channel_list {
	// Number of B/G band items
	u8 num_bg_band;
	// Number of A band items
	u8 num_a_band;
	// List of channels, B/G band followed by A band
	struct mt3620_wifi_channel_desc channel[0];
};


/** @brief Scan request
*/
struct mt3620_wifi_scan_request {
	// 0 full scan, 1 partial scan
	u8 partial;
	// 0 active (passive in regulatory), 1 passive, 2, force active in all
	// channels
	u8 scan_mode;
	u8 partial_bcn_count;
	// 0 - scan once.  Non 0 - scan after X becaon intervals
	u8 periodic_bcn_count;
	// Should we use bssid?
	u8 enable_bssid;
	// BSSID value
	u8 bssid[ETH_ALEN];
	// Should we use SSID?
	u8 enable_ssid;
	// SSID value
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	// Channel list
	struct mt3620_wifi_channel_list channels;
};


/** @brief 802.11 auth modes we support
*/
enum mt3620_wifi_auth_mode {
	// No auth
	WIFI_AUTH_MODE_OPEN = 0,
	// shared auth
	WIFI_AUTH_SHARED = 1,
	// WPA PSK
	WIFI_AUTH_MODE_WPA_PSK = 4,
	// WPA2 PSK
	WIFI_AUTH_MODE_WPA2_PSK = 7,
	// WPA / WPA2 mixed PSK
	WIFI_AUTH_MODE_WPA_PSK_WPA2_PSK = 9,
};


/** @brief N9 data packet to change the auth mode
*/
struct mt3620_wifi_set_auth_mode {
	// N9 Interface port
	u8 port;
	// Auth mode value
	u8 auth_mode;
};


/** @brief 802.11 encryption types we support
*/
enum mt3620_wifi_encryption_mode {
	// WEP
	WIFI_ENCRYPT_TYPE_WEP = 0,
	// No encryption
	WIFI_ENCRYPT_TYPE_NONE = 1,
	// TKIP
	WIFI_ENCRYPT_TYPE_TKIP = 4,
	// AES (WPA)
	WIFI_ENCRYPT_TYPE_AES = 6,
	//AES and TKIP
	WIFI_ENCRYPT_TYPE_AES_TKIP=8,
};


/** @brief N9 data packet to change the encryption mode
*/
struct mt3620_wifi_set_encryption_mode {
	// N9 Interface port
	u8 port;
	// Encryption mode value
	u8 encryption_mode;
};


/** @brief Cipher types for keys
*/
enum mt3620_cipher_type {
	WIFI_CIPHER_NONE = 0,
	WIFI_CIPHER_TKIP = 3,
	WIFI_CIPHER_AES = 4,
};


/** @brief Struct for a WPA key
*/
struct mt3620_wifi_wpa_key {
	// 128 bit key
	u8 key[16];
	// RX MIC key
	u8 rx_mic[8];
	// TX MIC key
	u8 tx_mic[8];
	// TX TSC
	u8 tx_tsc[6];
	// RX TSC
	u8 rx_tsc[6];
	// Cipher type
	u8 cipher_type;
	// Length of key
	u8 key_len;
	// BSSID this key is for
	u8 bssid[ETH_ALEN];
	// Indicates pairwise / group when reporting an error
	u8 type;
};


/** @brief Payload for setting a WPA key
*/
struct mt3620_wifi_set_wpa_key {
	// Key index
	u8 index;
	// 1 if this is a pairwise key
	u8 is_pairwise;
	// Key data
	struct mt3620_wifi_wpa_key key_data;
	// Mac address for this key
	u8 mac_addr[ETH_ALEN];
	//starting seq number
	u8 start_seq[MAX_PN_NUMBER_LEN];
};


/** @brief Struct for a WEP key
*/
struct mt3620_wifi_wep_key {
	// Key data
	u8 key[32];
	// Key length
	u8 key_len;
};


/** @brief Stuct for setting WEP keys
*/
struct mt3620_wifi_set_wep_keys {
	// Interface port
	u8 port;
	// TX key index
	u8 default_key_index;
	// Key data
	struct mt3620_wifi_wep_key keys[WLAN_MAX_KEY_COUNT];
};


/** @brief Struct for disconnecting a client
*/
struct mt3620_wifi_disconnect_station {
	// Interface port
	u8 port;
	// Mac address
	u8 mac_addr[ETH_ALEN];
};


/** @brief Info about a connected station
*/
struct mt3620_wifi_station_info {
	// Last seen TX rate
	u16 last_tx_rate;
	// Padding
	u16 reserved;
	// Last seen RX rate
	u16 last_rx_rate;
	// Padding
	u16 reserved2;
	// Last received RSSI
	s8 last_rssi;
	// Padding
	u8 reserved3;
	// Average RSSI over 8 times
	s16 average_rssi;
	// Mac address
	u8 mac_addr[ETH_ALEN];
	// Current power save mode
	u8 power_save_mode;
	// Negotiation bandwidth
	u8 bandwidth;
	// Keep alive status
	u8 keep_alive;
	// Padding
	u8 reserved4[3];
};


/** @brief Maximum number of connected stations the hardware supports
*/
#define MAX_STATION_COUNT 16


/** @brief List of connected stations
*/
struct __attribute__((__packed__)) mt3620_wifi_station_list {
	// Number of entries
	u8 count;
	// List of entries
	struct mt3620_wifi_station_info stations[MAX_STATION_COUNT];
};


/** @brief List of supported management messages we support monitoring (just probe
* requests for now)
*/
static const struct ieee80211_txrx_stypes
    mt3620_wifi_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	    [NL80211_IFTYPE_AP] =
		{
		    .tx = 0, .rx = BIT(IEEE80211_STYPE_PROBE_REQ >> 4),
		},
};


/** @brief TKIP key structure - 16 bytes of key, 8 bytes of TX MIC, 8 bytes of RX MIC
*/
#define WLAN_TKIP_KEY_LEN 16
#define WLAN_TKIP_MIC_LEN 8
#define WLAN_TKIP_TX_MIC_OFFSET 16
#define WLAN_TKIP_RX_MIC_OFFSET 24


/** @brief reserve one block for commands even under high loads
*/
#define N9_TX_RESERVED_SPACE 128


/** @brief Macro to help translate channel numbers to frequency numbers, handling both
* @brief our supported bands
*/
#define m3620_wifi_channel_to_frequency(channel)                               \
	ieee80211_channel_to_frequency(channel, channel <= MAX_2GHZ_CHANNEL    \
						    ? NL80211_BAND_2GHZ        \
						    : NL80211_BAND_5GHZ)


/** @brief Valid bands for setting our country / region domain
*/
enum mt3620_wifi_country_region_band {
	MT3620_BAND_2GHZ = 0,
	MT3620_BAND_5GHZ = 1,
};


/** @brief Set the active country / region for a band
*/
struct mt3620_wifi_set_country_region {
	// 2Ghz or 5Ghz
	u8 band;
	// Region code
	u8 region;
};


/** In the current N9 firmware Req IE data is prefixed with the ASCII string
* @brief "ASSOCINFO_ReqIEs="
*/
#define REQ_IE_OFFSET_SIZE 17


/** @brief Number of bytes at the end of a probe response that are N9 specific
*/
#define PROBE_RESP_N9_DATA_SIZE 2


/** @brief Value for no matching channel
*/
#define NO_CHANNEL 0


/** @brief Successful return value
*/
#define SUCCESS 0


/** @brief There is no matching domain
*/
#define NO_REG_DOMAIN_MATCH 0xFF


/** @brief Regulatory domain information
*/
struct mt3620_wifi_regulatory_domain_info {
	// FW domain code
	u8 code;
	// Number of channels
	u8 channel_count;
	// Array of channels
	const u8 *channels;
};

/** @brief Current VIF connection state
*/
enum mt3620_wifi_connect_state {
	// Not connected
	DISCONNECTED = 0,
	// Active connection attempt in progress
	CONNECTING = 1,
	// Connected
	CONNECTED = 2
};


/** Private data on AP mode interfaces
*/
struct mt3620_wifi_vif_ap_data {
	// Generation ID for connected stations
	u32 generation;
};


/** @brief Private data on STA mode interfaces
*/
struct mt3620_wifi_vif_station_data {
	// Current BSSID we're connected to
	u8 bssid[ETH_ALEN];
	// Are we connected to a AP in WMM / QoS mode
	bool qos;
	// Association request IEs
	u8 *req_ie;
	// Data length of req_ie
	size_t req_ie_len;
	// Association response IEs
	u8 *resp_ie;
	// Data length of resp_ie
	size_t resp_ie_len;
	// Target SSID
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	// Target SSID length
	u8 ssid_len;
    wifi_wow_configure_t wowconf;
};


/** @brief Private data attached per interface
*/
struct mt3620_wifi_vif {
	// Channel we're associated with
	u8 channel;
	// The underlying linux kernel device
	struct device *dev;
	// Is this VIF enabled?
	bool enabled;
	// Have we set our mac address on the N9?
	bool is_mac_set;
	// Cached key info
	struct mt3620_wifi_key_info keys[WLAN_MAX_KEY_COUNT];
	// Our network device
	struct net_device *ndev;
	// Network statistics
	struct net_device_stats net_stats;
	// N9 port for this interface
	enum mt3620_wifi_vif_port port;
	// Current state of the connection
	enum mt3620_wifi_connect_state state;
	//scan_in_progress_flag
	bool scan_in_progress;
	// Our wireless device
	struct wireless_dev wdev;
	// Index of WEP TX key
	u8 wep_tx_key_index;
	// Data per interface mode
	union {
		struct mt3620_wifi_vif_ap_data ap;
		struct mt3620_wifi_vif_station_data sta;
	} mode_data;
};


/** @brief Our private structure for 80211 wifi hardware
*/
struct mt3620_wifi_hw {
	// Completion structure for our connect scan event
	struct completion *connect_scan;
	// MACs for our STA and AP clients
	struct mac_address mac_addresses[4];
	// Mutex for locking initialization methods
	struct mutex mutex;
	// Have we initialized the N9 firmware yet?
	bool n9_initialized;
	// Number of active interfaces
	u32 num_vif;
	// The current scan request
	struct cfg80211_scan_request *scan_request;
	// Pointer to our master driver data
	struct mt3620_wifi *wifi;
	// Pointer to our AP mode VIF data
	struct mt3620_wifi_vif *ap_vif;
	// Pointer to our station mode VIF data
	struct mt3620_wifi_vif *sta_vif;
	// Pointer to our IBSS mode VIF data
	struct mt3620_wifi_vif *ibss_vif;
	// Whether our device is allowed to enter HQA (test) mode
	bool has_hqa_mode_capability;
	// Whether our device is in HQA (test) mode
	bool in_hqa_mode;
	// Whether the device is in ibss mode
	bool ibss_start;
	// Mutex for scan complete message methods
	struct mutex scan_mutex;
};

/** @brief Set mac address
 */
int mt3620_wifi_set_mac_helper(struct mt3620_wifi_hw *wifi_hw, 
	                       struct mt3620_wifi_set_mac_addr *mac);

/** @brief Disconnect from Wi-Fi if connected
*/
void mt3620_wifi_disconnect_if_connected(struct mt3620_wifi_vif *vif);


/** @brief Apply regulatory changes
*/
int mt3620_wifi_apply_country_code_regulatory_rules(void);


/** @brief Global variable for mt3620 driver containing the structure
*  with 80211 Wi-Fi hardware parameters
*/
extern struct mt3620_wifi_hw *g_wifi_hw;

