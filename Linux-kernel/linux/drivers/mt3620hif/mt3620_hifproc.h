// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 hif Proc driver
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

// Successful return code
#define SUCCESS 0

// Number of buffers to allocate to the M4 RX system
#define M4_MIN_RX_BUFFERS 16
#define M4_NUM_RX_BUFFERS 32
// RX buffer size
#define M4_RX_BUFFER_SIZE 1664

// Block size defined by the M4 for N9 communication
#define N9_TX_BLOCK_SIZE  64 //128

// constants for routing data to the N9
#define N9_CMD_PQ 0x8000
#define N9_DATA_PQ 0x8800

#define N9_PACKET_ID_CMD 0xA0
#define N9_PACKET_ID_EVENT 0xE000

// Must be in sync with M4 - extra status data on the end of a N9 data packet
#define N9_DATA_EXTRA_LENGTH 4

/* ==============  TOP WFSOC CR ================ */
#define TOP_WFSOC             (0x011c)
#define TOP_WFSOC_SET_RESET             (1)
#define TOP_WFSOC_CLEAR_RESET             (0)



#define PDA_BUSY_WAIT_ITERATION             (100000)
#define PDA_BUSY_WAIT_INTERVAL              (33) // 1ms
/* PDA - Patch Decryption Accelerator */


#define PDA_SOURCE_CON          (0x63000110)  //MT3620
#define PDA_BASE                (0x64000000)  //MT3620
#define PDA_TAR_ADDR         (PDA_BASE)
#define PDA_TAR_LEN          (PDA_BASE + 0x4)
#define PDA_SOURCE_SEL_BIT              BIT(28)

#define PDA_TOP_WRAPPER_PORT    (0x61000000) //MT3620
#define PDA_PKT_HDR_SIZE        (12) // 3 DW
#define PDA_CONFG_HEADER_3DW    BIT(1)
#define PDA_DWLD_STATE      (0x0008)
#define PDA_CONFG           (0x000c)



/* ==============   CONNSYS FW side CR ================ */
//#define SDIO_GEN3_BASE          (0x60000000) //MT3620

#define SDIO_GEN3_CMD_SETUP     (0x0)
//#define SDIO_GEN3_DMA_SETUP     (SDIO_GEN3_BASE+0x100)

#define SDIO_GEN3_CMD3_DATA     (0x8)
#define SDIO_GEN3_CMD5_DATA     (0xC)
#define SDIO_GEN3_CMD7_DATA     (0x10)
#define SDIO_GEN3_CMD11_DATA    (0x14)

#define SDIO_GEN3_CMD52_DATA    (0x4)
#define SDIO_GEN3_CMD53_DATA    (0x1000)



#define CONNSYS_POLLING_DELAY_TIME 10
#define CFG_MAX_HIF_RX_LEN_NUM                  0
#define CFG_RPT_OWN_RX_PACKET_LEN_EN            0
#define CFG_RX0_RPT_PKT_LEN                     4 /* 0:16, 1~15 */
#define CFG_RX1_RPT_PKT_LEN                     10
#define DEFAULT_N9_PSE_PAGE_SIZE                128
#define DEFAULT_N9_PSE_PAGE_QUOTA               287

#define NUM_OF_WIFI_HIF_RX_PORT                 2
#define NUM_OF_WIFI_HIF_TX_PORT                 2
#define WIFI_HIF_RX_PORT0_IDX                   0   /* in-band command port */
#define WIFI_HIF_RX_PORT1_IDX                   1   /* data port */
#define WIFI_HIF_TX_PORT_IDX                    0   

#define WIFI_HIF_RX_CS_OFFLOAD_STATUS_LEN       4
#define WIFI_HIF_HEADER_LEN                 12
#define NUM_OF_WIFI_TXQ                     16
#define WIFI_HIF_RX_FIFO_MAX_LEN            16

#define D2H_SW_INT_SET  (0xFFFFFF << 8)
#define LEN_SDIO_TX_TERMINATOR  4 /*HW design spec*/
#define WCIR 0x0000
#define CHIP_ID_MASK (0xffff)
#define GET_CHIP_ID(p) (((p) & CHIP_ID_MASK))
#define REVISION_ID_MASK (0xf << 16)
#define GET_REVISION_ID(p) (((p) & REVISION_ID_MASK) >> 16)
#define POR_INDICATOR (1 << 20)
#define GET_POR_INDICATOR(p) (((p) & POR_INDICATOR) >> 20)
#define W_FUNC_RDY (1 << 21)
#define GET_W_FUNC_RDY(p) (((p) & W_FUNC_RDY) >> 21)
#define DEVICE_STATUS_MASK (0xff << 24)
#define GET_DEVICE_STATUS(p) (((p) & DEVICE_STATUS_MASK) >> 24)

#define WHLPCR 0x0004
#define W_INT_EN_SET (1 << 0)
#define W_INT_EN_CLR (1 << 1)
#define W_FW_OWN_REQ_SET (1 << 8)
#define GET_W_FW_OWN_REQ_SET(p) (((p) & W_FW_OWN_REQ_SET) >> 8)
#define W_FW_OWN_REQ_CLR (1 << 9)
#define WSDIOCSR 0x0008
#define WHCR 0x000C
#define W_INT_CLR_CTRL (1 << 1)
#define RECV_MAILBOX_RD_CLR_EN (1 << 2)
#define RPT_OWN_RX_PACKET_LEN (1 << 3)
#define MAX_HIF_RX_LEN_NUM_MASK (0x3f << 8)
#define MAX_HIF_RX_LEN_NUM_OFFSET (8)

#define MAX_HIF_RX_LEN_NUM(p) (((p) & 0x3f) << 8)
#define GET_MAX_HIF_RX_LEN_NUM(p) (((p) & MAX_HIF_RX_LEN_NUM_MASK) >> 8)
#define RX_ENHANCE_MODE (1 << 16)


#define WHISR 0x0010
#define TX_DONE_INT (1 << 0)
#define RX0_DONE_INT (1 << 1)
#define RX1_DONE_INT (1 << 2)
#define ABNORMAL_INT (1 << 6)
#define FW_OWN_BACK_INT (1 << 7)
#define D2H_SW_INT (0xffffff << 8)
#define D2H_SW_INT_MASK (0xffffff << 8)
#define GET_D2H_SW_INT(p) (((p) & D2H_SW_INT_MASK) >> 8)
#define WHIER 0x0014
#define TX_DONE_INT_EN (1 << 0)
#define RX0_DONE_INT_EN (1 << 1)
#define RX1_DONE_INT_EN (1 << 2)
#define ABNORMAL_INT_EN (1 << 6)
#define FW_OWN_BACK_INT_EN (1 << 7)
#define D2H_SW_INT_EN_MASK (0xffffff << 8)
#define D2H_SW_INT_EN(p) (((p) & 0xffffff) << 8)
#define GET_D2H_SW_INT_EN(p) (((p) & D2H_SW_INT_EN_MASK) >> 8)
#define WHIER_DEFAULT (TX_DONE_INT_EN | RX0_DONE_INT_EN | RX1_DONE_INT_EN\
						| ABNORMAL_INT_EN\
						| D2H_SW_INT_EN_MASK)
#define WASR 0x0020
#define TX1_OVERFLOW (1 << 1)
#define RX0_UNDERFLOW (1 << 8)
#define RX1_UNDERFLOW (1 << 9)
#define WASR_WASR2 (1 << 16)
#define WSICR 0x0024
#define WTDR1 0x0034
#define WRDR0 0x0050
#define WRDR1 0x0054
#define H2DSM0R 0x0070
#define H2DSM1R 0x0074
#define D2HRM0R 0x0078
#define D2HRM1R 0x007c
#define WRPLR 0x0090
#define RX0_PACKET_LENGTH_MASK (0xffff)
#define GET_RX0_PACKET_LENGTH(p) (((p) & RX0_PACKET_LENGTH_MASK))
#define RX1_PACKET_LENGTH_MASK (0xffff << 16)
#define GET_RX1_PACKET_LENGTH(p) (((p) & RX1_PACKET_LENGTH_MASK) >> 16)
#define WTMDR 0x00b0
#define WTMCR 0x00b4
#define WTMDPCR0 0x00b8
#define WTMDPCR1 0x00bc
#define WPLRCR 0x00d4
#define WTQCR7 0x014c
#define RX0_RPT_PKT_LEN_MASK (0x3f)
#define RX0_RPT_PKT_LEN_OFFSET 0
#define RX0_RPT_PKT_LEN(p) (((p) & 0x3f))
#define GET_RPT_PKT_LEN(p) (((p) & RX0_RPT_PKT_LEN_MASK))
#define RX1_RPT_PKT_LEN_MASK (0x3f << 8)
#define RX1_RPT_PKT_LEN_OFFSET 8
#define RX1_RPT_PKT_LEN(p) (((p) & 0x3f) << 8)
#define GET_RX1_RPT_PKT_LEN(p) (((p) & RX1_RPT_PKT_LEN_MASK) >> 8)
#define WSR 0x00D8

/* TXCFFA: the real amount of free pages that HIF source port can use for this time. 
 * Other TXQ_CNT: the amount of finished packet for a certain queue.
 */
#define WIFI_TXQ_CNT_IDX_0_TXCAC0       0
#define WIFI_TXQ_CNT_IDX_1_TXCAC1       1
#define WIFI_TXQ_CNT_IDX_2_TXCAC2       2
#define WIFI_TXQ_CNT_IDX_3_TXCAC3       3
#define WIFI_TXQ_CNT_IDX_4_TXCAC4       4
#define WIFI_TXQ_CNT_IDX_5_TXCAC5       5
#define WIFI_TXQ_CNT_IDX_6_TXCAC6       6
#define WIFI_TXQ_CNT_IDX_7_TXCBMC       7  
#define WIFI_TXQ_CNT_IDX_8_TXCBCN       8
#define WIFI_TXQ_CNT_IDX_9_TXCAC10      9 
#define WIFI_TXQ_CNT_IDX_10_TXCAC11     10
#define WIFI_TXQ_CNT_IDX_11_TXCAC12     11
#define WIFI_TXQ_CNT_IDX_12_TXCAC13     12
#define WIFI_TXQ_CNT_IDX_13_TXCAC14     13
#define WIFI_TXQ_CNT_IDX_14_TXCFFA      14
#define WIFI_TXQ_CNT_IDX_15_TXCCPU      15

#define LEN_INT_ENHANCE_MODE    (sizeof(enhance_mode_data_struct_t)) 
#define LEN_RX_ENHANCE_MODE     (4 + sizeof(enhance_mode_data_struct_t)) // HW design spec


#define HIF_MAX_RX_PKT_SIZE 1664 //(0x640) // The max RX pkt size by upper layer (lwip)
#define MY_HIF_BLOCK_SIZE  64 //128


#define CFG_HIF_3628_CONTINOUS_ALLOCATE_FAIL_PRINT_CNT_VAL (16)


#ifdef MTK_HIF_GDMA_ENABLE
#define CFG_WIFI_HIF_GDMA_EN                 1
#else
#define CFG_WIFI_HIF_GDMA_EN                 0
#endif

/* ==================== HW Definition ============ */
#define ALIGN_4BYTE(size)       (((size+3)/4) * 4)
#define LEN_SDIO_TX_TERMINATOR  4 /*HW design spec*/
#define LEN_SDIO_RX_TERMINATOR  4
#define SDIO_HOST_REGISTER_VALUE_MAX    0x014C
#define LEN_SDIO_TX_AGG_WRAPPER(len)    ALIGN_4BYTE((len) + LEN_SDIO_TX_TERMINATOR)




#define HIF_STATUS_SUCCESS  (0)
#define HIF_STATUS_FAIL     (-1)
#define HIF_DEBUG_MODE_EN          0


enum SDIO_GEN3_RW_TYPE {
    SDIO_GEN3_READ,
    SDIO_GEN3_WRITE
};

enum SDIO_GEN3_TRANS_MODE {
    SDIO_GEN3_BYTE_MODE,
    SDIO_GEN3_BLOCK_MODE
};

enum SDIO_GEN3_OP_MODE {
    SDIO_GEN3_FIXED_PORT_MODE,
    SDIO_GEN3_INCREMENT_MODE
};



enum SDIO_GEN3_FUNCTION {
    SDIO_GEN3_FUNCTION_0,
    SDIO_GEN3_FUNCTION_WIFI,
    SDIO_GEN3_FUNCTION_BT,
};



typedef union _mt3620_hif_gen3_cmd52_info
{
    struct{
        uint32_t data : 8;             /* data for write, dummy for read */
        uint32_t reserved_8:1;         /* stuff */
        uint32_t addr : 17;            /* register address */
        uint32_t reserved_26_27: 2;    /* raw flag / stuff */
        uint32_t func_num: 3;          /* function number */
        uint32_t rw_flag: 1;           /* read / write flag */
    } field;
    uint32_t word;
} mt3620_hif_gen3_cmd52_info;

typedef union _mt3620_hif_gen3_cmd53_info
{
    struct {
        uint32_t count : 9;            /* block count for block mode, byte count for byte mode  */
        uint32_t addr : 17;            /* register address */
        uint32_t op_mode:1;            /* 1 for increment mode, 0 for port mode */
        uint32_t block_mode: 1;        /* 1 for block mode, 0 for byte mode */
        uint32_t func_num: 3;          /* function number */
        uint32_t rw_flag: 1;           /* read / write flag */
    } field;
    uint32_t word;
} mt3620_hif_gen3_cmd53_info;

#define WIFI_PROFILE_PDA_HEADER_LEN     (12)

// copy from N9 wifi_uni_mac_7636/rom/include/iot/rt_bora.h
#define MAX_LEN_OF_SSID                 (32)

typedef struct _CIPHER_KEY 
{
    uint8_t   Key[16];            // right now we implement 4 keys, 128 bits max
    uint8_t   RxMic[8];           // make alignment 
    uint8_t   TxMic[8];
    uint8_t   TxTsc[6];           // 48bit TSC value
    uint8_t   RxTsc[6];           // 48bit TSC value
    uint8_t   CipherAlg;          // 0-none, 1:WEP64, 2:WEP128, 3:TKIP, 4:AES, 5:CKIP64, 6:CKIP128
    uint8_t   KeyLen; 
    uint8_t   BssId[6];
    uint8_t   Type;               // Indicate Pairwise/Group when reporting MIC error
} CIPHER_KEY;

typedef struct _CH_DESC_S
{
    unsigned char FirstChannel; 
    unsigned char NumOfCh;
    unsigned char ChannelProp; //0: Active, 1: Passive
    unsigned char Reserve; // 4-align and could be used to extend ChannelProp
} CH_DESC_T, *P_CH_DESC_T;

typedef struct _CH_LIST_S {
    unsigned char NumBGBandTriplet;
    unsigned char NumABandTriplet;
    CH_DESC_T Triplet[]; // BGBand Triplet followed by Aband Triplet
} CH_LIST_T, *P_CH_LIST_T;


typedef struct syscfg
{
    uint8_t OpMode;
    uint8_t CountryRegion;
    uint8_t CountryRegionABand;
    uint8_t CountryCode[4];
    uint8_t RadioOff;
    uint8_t DbgLevel;
    uint8_t RTSThreshold[2];
    uint8_t FragThreshold[2];

    uint8_t STA_LocalAdminMAC;
    uint8_t STA_IpAddr[4];
    uint8_t STA_MacAddr[6];
    uint8_t STA_Ssid[32];
    uint8_t STA_SsidLen;
	uint8_t STA_BssType;
	uint8_t STA_Channel;
	uint8_t STA_BW;
	uint8_t STA_WirelessMode;
	uint8_t STA_BADecline;
	uint8_t STA_AutoBA;
	uint8_t STA_HT_MCS;
	uint8_t STA_HT_BAWinSize;
	uint8_t STA_HT_GI;
	uint8_t STA_HT_PROTECT;
	uint8_t STA_HT_EXTCHA;
	uint8_t STA_WmmCapable;
	uint8_t STA_ListenInterval;
	uint8_t STA_AuthMode;
	uint8_t STA_EncrypType;
	uint8_t STA_WpaPsk[64];
	uint8_t STA_WpaPskLen;
	//uint8_t STA_Password[32];	
	uint8_t STA_PMK[32];
	uint8_t STA_PairCipher;
	uint8_t STA_GroupCipher;
	uint8_t STA_DefaultKeyId;
	//CIPHER_KEY STA_SharedKey[4];
	//uint8_t STA_SharedKeyLen[4];
	//uint8_t STA_SharedKeyIdx;
	uint8_t STA_PSMode;
	uint8_t STA_KeepAlivePeriod;

    uint8_t AP_LocalAdminMAC;
    uint8_t AP_IpAddr[4];
    uint8_t AP_MacAddr[6];
    uint8_t AP_Ssid[32];
    uint8_t AP_SsidLen;
	uint8_t AP_Channel;
	uint8_t AP_BW;
	uint8_t AP_WirelessMode;
	uint8_t AP_AutoBA;
	uint8_t AP_HT_MCS;
	uint8_t AP_HT_BAWinSize;
	uint8_t AP_HT_GI;
	uint8_t AP_HT_PROTECT;
	uint8_t AP_HT_EXTCHA;
	uint8_t AP_WmmCapable;
	uint8_t AP_DtimPeriod;
	uint8_t AP_HideSSID;
	uint8_t AP_AutoChannelSelect;
	uint8_t AP_AuthMode;
	uint8_t AP_EncrypType;
	uint8_t AP_WpaPsk[64];
	uint8_t AP_WpaPskLen;
	//uint8_t AP_Password[32];
	uint8_t AP_PMK[32];
	uint8_t AP_PairCipher;
	uint8_t AP_GroupCipher;
	uint8_t AP_DefaultKeyId;
	//CIPHER_KEY AP_SharedKey[4];
	//uint8_t AP_SharedKeyLen[4];
	//uint8_t AP_SharedKeyIdx;

    // "scan channel table" and "regulatory table"
    unsigned char bg_band_entry_num;
    CH_DESC_T bg_band_triple[10];
    unsigned char a_band_entry_num;
    CH_DESC_T a_band_triple[10];
	uint8_t AP_Bcn_disEn;
    uint8_t forwarding_zero_copy;
	uint8_t Valid_Mbss;
	uint8_t AP_Ssid1[32];
	uint8_t AP_SsidLen1;
	uint8_t AP_Ssid2[32];
	uint8_t AP_SsidLen2;
	uint8_t ConfigFree_Ready;	
	uint8_t ConfigFree_Enable;
	uint8_t Sta_Fast_Link;
		
		
} syscfg_t;





#define DATA_MODE_BIT_SHFT_ENCRYPT_MODE       (0)    //bit0
#define DATA_MODE_MASK_ENCRYPT_MODE           (0x01 << DATA_MODE_BIT_SHFT_ENCRYPT_MODE)    //bit0


#define DATA_MODE_BIT_SHFT_KEY_INDEX          (1)    //bit[2:1]
#define DATA_MODE_MASK_KEY_INDEX              (0x03 << DATA_MODE_BIT_SHFT_KEY_INDEX)    //bit[2:1]

#define DATA_MODE_BIT_SHFT_RESET_IV            (3)   //bit3
#define DATA_MODE_MASK_RESET_IV                (0x1 << DATA_MODE_BIT_SHFT_RESET_IV)

#define DATA_MODE_BIT_SHFT_NEED_ACK           (31)    //bit31
#define DATA_MODE_MASK_NEED_ACK               (0x01 << DATA_MODE_BIT_SHFT_NEED_ACK)    //bit31

#define LEN_FW_SCATTER                      MAX_BUF_SIZE_MT3620 //(1024*4)
#define PQ_TO_PDA                           0xC000
#define LEN_FW_DOWNLOAD_EVENT               (sizeof(INIT_WIFI_EVENT_T) + sizeof(INIT_EVENT_CMD_RESULT))

#define MAX_BUF_SIZE_MT3620 (0x400)

#define SDIO_MAX_RW_SIZE 	(128)

#define LEN_4_BYTE_CRC  4



#define WIFI_PROFILE_LEN                (0x800) // 2K
#define WIFI_PROFILE_ADDR               (0x020B4800)//(0x020B2000)
#define WIFI_PROFILE_DATA_MODE          (0x80000000) // plain (non-encrypted)
#define WIFI_PROFILE_KEY_INDEX          (0x0)


#define FEATURE_MASK_ENCRYPT_MODE    (0x01 << 0)    //bit0
#define FEATURE_MASK_KEY_INDEX       (0x03 << 1)    //bit[2:1]

#define READ_DMA_STATUS_MAX_COUNT     1000

#define GLB_CH  34
#define MT3620_DMA_WIFI_CHANNEL  12
#define DMA_START(_n)			( 0x0008 + (0x0100 * (_n)))
#define DMA_RESET(_n)			( 0x000C + (0x0100 * (_n)))

#define DMA_CON(_n)				( 0x0018 + (0x0100 * (_n)))
#define DMA_SRC(_n)				( 0x001C + (0x0100 * (_n))) 
#define DMA_DST(_n)				( 0x0020 + (0x0100 * (_n))) 
#define DMA_CONNECT(_n)			( 0x0034 + (0x0100 * (_n))) 
#define DMA_COUNT(_n)			( 0x0024 + (0x0100 * (_n)))
#define DMA_INT_FLAG(_n)        (0x0000 + (0x0100 * (_n)))
#define DMA_CH_EN_SET(ch)		(((ch > 31)?(0x0024):(0x0020)) + (0x0100 * (GLB_CH)))

/*
 * Command type table
 */
#define PQ_TO_PDA               0xC000
#define P1_Q0                   0x8000
#define P1_Q1                   0x8800
#define PKT_ID_CMD              0xA0
#define PKT_ID_EVENT            0xE000


enum MT_CMD_TYPE {
	MT_TARGET_ADDRESS_LEN_REQ = 0x01,
	MT_FW_START_REQ = 0x02,
	INIT_CMD_ACCESS_REG = 0x3,
	MT_HIF_LOOPBACK = 0x20,
	CMD_CH_PRIVILEGE = 0x20,
	CMD_ACCESS_REG = 0xC2,
	EXT_CID = 0xED,
	MT_FW_SCATTER = 0xEE,
	MT_RESTART_DL_REQ = 0xEF,
};

typedef struct _INIT_CMD_WIFI_START {
    uint32_t     u4Override;
    uint32_t     u4Address;
} INIT_CMD_WIFI_START, *P_INIT_CMD_WIFI_START;

typedef struct _fw_dl_data_t
{
    uint32_t ilm_addr;
    uint32_t ilm_len;
    uint8_t  ilm_encrypt;
    uint8_t  ilm_encrypt_key_index;
    uint32_t dlm_addr;
    uint32_t dlm_len;
    uint8_t  dlm_encrypt;
    uint8_t  dlm_encrypt_key_index;
    uint8_t  *image;
} fw_dl_data_t;


typedef struct _tailer_format_tag
{
    uint32_t addr;
    uint8_t  chip_info;
    uint8_t  feature_set;
    uint8_t  ram_version[10];
    uint8_t  ram_built_date[16];
    uint32_t len;

} tailer_format_t;

typedef struct _fw_image_tailer_tag
{
    tailer_format_t ilm_info;
    tailer_format_t dlm_info;
} fw_image_tailer_t;



//================== Inband Command ====================================================//
typedef struct _INIT_WIFI_CMD_T {
    uint8_t      ucCID;
    uint8_t      ucPktTypeID;    /* Must be 0xA0 (CMD Packet) */
    uint8_t      ucReserved;
    uint8_t      ucSeqNum;
//    UINT32     u4Reserved;     /* add one DW to compatible with normal TXD format. */
// ----------Ext Cmd ----------------
    uint8_t      ucPacketOffset;     /* for N9 tx zero copy */
    uint8_t      ucExtenCID;         /* Extend CID */
    uint8_t      ucD2B2Rev;          /* padding fields, hw may auto modify this field */
    uint8_t      ucExtCmdOption;     /* Extend CID option */	
// ----------Ext Cmd ----------------
    uint8_t      aucBuffer[];        /* for KEIL porting, don't use zero-length [0] */
} INIT_WIFI_CMD_T, *P_INIT_WIFI_CMD_T;

typedef struct _INIT_HIF_TX_HEADER_T { /* size=12 bytes  */
    uint16_t     u2TxByteCount;
    uint16_t     u2PQ_ID;        /* Must be 0x8000 (Port1, Queue 0) */
    INIT_WIFI_CMD_T rInitWifiCmd;
} INIT_HIF_TX_HEADER_T, *P_INIT_HIF_TX_HEADER_T;

#define LEN_INBAND_CMD_HDR_ROM      sizeof(INIT_HIF_TX_HEADER_T)

typedef enum _ENUM_INIT_EVENT_ID
{
    INIT_EVENT_ID_CMD_RESULT = 1,
    INIT_EVENT_ID_ACCESS_REG,
    INIT_EVENT_ID_PENDING_ERROR,
    INIT_EVENT_ID_PATCH_SEMA_CTRL
} ENUM_INIT_EVENT_ID, *P_ENUM_INIT_EVENT_ID;


typedef struct _INIT_EVENT_CMD_RESULT {
    uint8_t      ucStatus;           /* 0: success, others: failure */
    uint8_t      aucReserved[3];
} INIT_EVENT_CMD_RESULT, *P_INIT_EVENT_CMD_RESULT, INIT_EVENT_PENDING_ERROR, *P_INIT_EVENT_PENDING_ERROR;
#define WIFI_EVENT_HDR_LEN (sizeof (INIT_WIFI_EVENT_T) + sizeof(INIT_EVENT_CMD_RESULT))

#define LEN_INBAND_EVENT_HDR_ROM      sizeof(INIT_WIFI_EVENT_T)


typedef struct _INIT_CMD_DOWNLOAD_CONFIG {
    uint32_t     u4Address;
    uint32_t     u4Length;
    uint32_t     u4DataMode;
} INIT_CMD_DOWNLOAD_CONFIG, *P_INIT_CMD_DOWNLOAD_CONFIG;


typedef struct _INIT_WIFI_EVENT_T {

    /* DW#0 */
    uint16_t     u2RxByteCount;
    uint16_t     u2PacketType;   /* Must be filled with 0xE000 (EVENT Packet) */
    /* DW#1 */
    uint8_t      ucEID;
    uint8_t      ucSeqNum;
    uint8_t      aucReserved[2];
    /* DW#2 */
    uint16_t      aucBuffer[];   /* for KEIL porting, don't use zero-length [0] */
} INIT_WIFI_EVENT_T, *P_INIT_WIFI_EVENT_T;

#define WIFI_FW_ADDR_IN_FLASH   (0x90080000)


int32_t mt3620_hif_fifo_read(uint32_t addr, uint8_t *buf, size_t size);
int32_t mt3620_hif_fifo_write(uint8_t *buf, size_t size);

int32_t mt3620_hif_cr_write(uint32_t addr, uint32_t value);
int32_t mt3620_hif_cr_read(uint32_t addr, uint32_t *value);


// Our basic device state structure
struct mt3620_hif_proc {
	
	// The underlying linux kernel device
	struct device *dev;
	// Base register address of the M7 sysram that we'll get data buffers on
	void __iomem *phif_sysram_base;

	struct list_head hif_management_list;
	
	spinlock_t hif_lock;

	int read_irq;
	// Bottom half of rd IRQ handler
	struct work_struct read_work;
	// IRQ that triggers when our data has been trasmitted
	int write_irq;
	// Bottom half of wr IRQ handler
	struct tasklet_struct write_tasklet;
	// Hardware version number

      // Software flag to record the Ownership status. Default should be "true", so that the first get_ownership will
      // be executed.
      bool is_fw_own;
      spinlock_t own_lock;
};

extern struct mt3620_hif_proc *g_pmt3620_hif_proc;


 struct mt3620_hif_func {
    uint32_t num;		/* function number */
    uint32_t blksize;	    /* current block size */
    uint32_t use_dma;
	//void*   task_handle;
} ;


typedef struct{
    uint32_t             WHISR_reg_val;
    union {
        struct {
            uint16_t             free_page_num[NUM_OF_WIFI_TXQ];
        } u;
        uint32_t                 WTSR_val[8];
    } tx_info;
    union {
        struct {
            uint16_t             valid_len_num[NUM_OF_WIFI_HIF_RX_PORT];
            uint16_t             each_rx_len[NUM_OF_WIFI_HIF_RX_PORT][WIFI_HIF_RX_FIFO_MAX_LEN];
        } u;
        uint32_t                 rx_status_raw[17];
    } rx_info;
    uint32_t                     receive_mail_box_0;
    uint32_t                     receive_mail_box_1;
} enhance_mode_data_struct_t;


typedef struct{
    uint32_t send_pkt_cnt_by_tx_port[NUM_OF_WIFI_HIF_TX_PORT];
    uint32_t total_send_pkt_cnt;
    uint32_t send_page_cnt_by_tx_port[NUM_OF_WIFI_HIF_TX_PORT];
    uint32_t total_send_page_cnt;
    uint32_t total_drop_pkt_cnt;
    
    uint32_t free_page_cnt_by_wifi_txq[NUM_OF_WIFI_TXQ];
    uint32_t total_free_page_cnt;    
    int32_t available_page_cnt;
    int32_t current_page_cnt;
    int32_t max_page_cnt;
    uint32_t reserve_quota_page_cnt;
    uint32_t page_sz;
    spinlock_t hif_tx_flow_lock;
}wifi_hif_tx_flow_control_t;


typedef struct _hif_info{
    uint32_t rx_packet_cnt;
    uint32_t rx_invalid_sz_packet_cnt;
    uint32_t rx_max_invalid_sz;
    uint32_t rx_error_cnt;
    uint32_t rx_allocate_fail_cnt;    
}hif_info;


typedef struct _hif_stat{
    uint32_t number_of_int;
    uint32_t number_of_abnormal_int;
    uint32_t num_of_tx_int;
    uint32_t num_of_rx_int;
    uint32_t num_of_tx_wait;
    uint32_t number_of_fw_own_back;
    uint32_t process_start_time;
    uint32_t process_end_time;
    hif_info rx_port[NUM_OF_WIFI_HIF_RX_PORT];
}hif_stat_t;






// Wifi command details
struct n9_wifi_command {
	// Command being sent
	u8 cmd;
	// Packet type (must be 0xA0 today)
	u8 packet_type;
	// 1 if this is a get request
	u8 get;
	// sequence number for the request
	u8 sequence_number;
	// Offset used for N9 TX copy
	u8 packet_offset;
	// Extended command ID
	u8 extended_cmd;
	// Padding / hardware use only
	u8 padding;
	// Option data for extended command
	u8 extended_cmd_option;
	// pointer to our data buffer at the end of this struct
	u8 data_buffer[0];
};

// N9 wifi data header
struct n9_wifi_data_header {
	// Total size of data
	u16 byte_count;
	// Used by the N9 - must be 0x8000
	u16 port_queue_id;
	// Command being sent
	u8 cmd;
	// Packet type (must be 0xA0 today)
	u8 packet_type;
	// Interface number
	u8 interface;
	// Reserved
	u8 reserved[2];
	// Offset used for N9 TX copy
	u8 packet_offset;
	// Reserved
	u8 reserved2[2];
	// pointer to our data buffer at the end of this struct
	u8 data_buffer[0];
};

// Command header for N9 messages
struct n9_command_header {
	// Total size of data
	u16 byte_count;
	// Used by the N9 - must be 0x8000
	u16 port_queue_id;
	// Wifi command details
	struct n9_wifi_command wifi_command;
};

// Response result data
struct n9_command_result {
	// 0 for success
	u8 status;
	// Padding
	u8 reserved[3];
};

// Header for packets that originate from the N9
struct n9_event_header {
	// Total size of data
	u16 byte_count;
	// Packet type (must be 0xE000 today)
	u16 packet_type;
	// Type of event
	u8 event_id;
	// sequence number for the event
	u8 sequence_number;
	// Interface number
	u8 interface;
	// Padding
	u8 reserved;
	// pointer to our data buffer at the end of this struct
	u8 data_buffer[0];
};

// Header for packets that originate from the N9 on the data channel
struct n9_data_header {
	// Total size of data
	u16 byte_count;
	// Packet type (must be 0xE000 today)
	u16 packet_type;
	// Type of event
	u8 event_id;
	// sequence number for the event
	u8 sequence_number;
	// Interface number
	u8 interface;
	// Padding
	u8 reserved;
	// Offset into the data_buffer where the packet begins
	u8 packet_offset;
	// Reserved / padding
	u8 reserved2[3];
	// pointer to our data buffer at the end of this struct
	u8 data_buffer[0];
};




// What kind of response we're going to route for a received message
enum mt3620_hif_response_type {
	NO_RESPONSE, // No action needed
	CALLBACK,    // Invoke a callback
	COMPLETION,  // Copy data and invoke a completion event
};

// Response data for a completion callback
struct mt3620_hif_completion_data {
	// Completion object
	struct completion *completion;
	// Data buffer to copy response to
	void *buffer;
	// Size of buffer
	u32 buffer_size;
	// Size of the needed buffer if the caller didn't provide enough space
	u32 *needed_buffer_size;
	// Pointer to hold status from message
	u8 *status;
};

// Data on how to route responses
union mt3620_hif_response_data {
	// Callback when we get a response on this sequence number
	mt3620_hif_api_callback callback;
	// Data for completion based responses
	struct mt3620_hif_completion_data completion;
};



struct mt3620_hif_management_info {
	// What type of response we're going to issue
	enum mt3620_hif_response_type response_type;
	// Our response data
	union mt3620_hif_response_data response_data;
	// List pointer
	struct list_head list;
	// Sequence number - used to execute callbacks
	u8 sequence_number;
	//n9 command id
	enum n9_commands cmd;
	//extended command
	//u8 extended_cmd;
	// Size of the inner data packet
	u16 inner_packet_size;

	// Pointer to the inner data packet (specific to the command)
	u8 *inner_packet;
	// Physical address of structure
	dma_addr_t hwaddr;
};



///
/// Shuts down Remote API system
///
void mt3620_hif_api_shutdown(void);

///
/// Initialization of Remote API
///
/// @returns - 0 for success
int mt3620_hif_api_init(void);



uint8_t mt3620_hif_get_ownership(void);
uint8_t mt3620_hif_set_ownership(void);

void mt3620_hif_api_handle_n9_message(struct n9_event_header *header);
void mt3620_hif_api_handle_n9_data(struct sk_buff *skb, u16 packet_size);
void mt3620_hif_api_handle_tx_update(u32 free_tx_space);

