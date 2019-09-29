// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Security Monitor API
 *
 * Copyright (c) 2018 Microsoft Corporation. All rights reserved.
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

#include <linux/arm-smccc.h>

#define SECURITY_MONITOR_ENTITY (ARM_SMCCC_OWNER_TRUSTED_OS)

#define SECURITY_MONITOR_FUNCTION(async, function) \
	ARM_SMCCC_CALL_VAL(async, ARM_SMCCC_SMC_32, SECURITY_MONITOR_ENTITY, function)

// ARM_SMCCC_STD_CALL is the yielding call where as FAST is the synchronous call
#define SECURITY_MONITOR_API_ASYNC (ARM_SMCCC_STD_CALL)
#define SECURITY_MONITOR_API_SYNC  (ARM_SMCCC_FAST_CALL)

/*
 * API: version
 */

#define SECURITY_MONITOR_CURRENT_VERSION (1)

#define SECURITY_MONITOR_API_GET_VERSION \
 	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 1)

/*
 * API: ping
 */

#define SECURITY_MONITOR_API_PING \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 2)

/*
 * API: query flash
 *
 * args[0] = result pointer
 *
 * result = error
 */

#define SECURITY_MONITOR_API_QUERY_FLASH \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 3)

struct security_monitor_query_flash_arguments {
	u32 result_pointer;
};

struct security_monitor_query_flash_result {
	u32 info_length;
	
	u32 length;

	u32 min_erase_length;
	u32 preferred_erase_length;
	u32 max_erase_length;

	u32 min_write_length;
	u32 preferred_write_length;
	u32 max_write_length;	
};

/*
 * Add simple static assert that will be used to validate compatibility structs
 */
#ifndef static_assert
#define static_assert(x, y) \
    typedef int __security_monitor_static_assert[(x) ? 0 : -1]
#endif // static_assert

/*
 * Make sure security_monitor_query_flash_result only increases in length for compatibilty with SK calls
 */
static_assert(sizeof(struct security_monitor_query_flash_result) >= 32, "security_monitor_query_flash_result size changed");
static_assert(offsetof(struct security_monitor_query_flash_result, max_write_length) == 28, "security_monitor_query_flash_result layout changed");

/*
 * API: erase flash
 *
 * args[0] = start_offset
 * args[1] = length
 *
 * result = error
 */

#define SECURITY_MONITOR_API_ERASE_FLASH \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 4)

struct security_monitor_erase_flash_arguments {
	u32 start_offset;
	u32 length;
};

/*
 * API: flash write
 */

#define SECURITY_MONITOR_API_WRITE_FLASH \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 5)

struct security_monitor_write_flash_arguments {
	u32 start_offset;
	u32 length;
	u32 data_pointer;
};

/*
 * API: update image
 *
 * args[0] = cmd
 * args[1] = input_param
 * args[2] = input_length
 * args[3] = output_param
 * args[4] = output_length
 *
 * result = error
 */

struct security_monitor_generic_command_arguments {
	u32 cmd;
	u32 input_param;
	u32 input_length;
	u32 output_param;
	u32 output_length;
};

#define SECURITY_MONITOR_API_VERIFY_IMAGE_BY_FLASH_ADDRESS_CMD \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 6)

#define SECURITY_MONITOR_API_RESET_CMD \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 7)

struct security_monitor_verify_image_by_flash_address_arguments {
	u32 flash_address;
};

#define SECURITY_MONITOR_API_WRITE_LOG \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 8)

struct security_monitor_write_log_arguments {
	u32 log_data;
	u32 log_data_size;
};

#define SECURITY_MONITOR_API_GET_LOG_DATA_SIZE \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 9)

#define SECURITY_MONITOR_API_GET_LOG_DATA \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 10)
	
#define SECURITY_MONITOR_API_DERIVE_KEY_CMD \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 11)

struct security_monitor_derive_key_data {
	struct {
		uint8_t client_uid[16];
		uint32_t generation_delta;
	} input_params;
	struct {
		uint8_t key[32];
		uint32_t instance;
		uint32_t generation;
	} output_params;
};

#define SECURITY_MONITOR_API_GET_PERIPHERAL_COUNT \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 12)

#define SECURITY_MONITOR_API_LIST_PERIPHERALS \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 13)

#define SECURITY_MONITOR_API_GET_WIFI_FIRMWARE_LOCATION \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 14)

struct security_monitor_get_wifi_firmware_params {
	struct security_monitor_get_wifi_firmware_params_out {
		u32 wifi_firmware_address;
		u32 wifi_firmware_length;
	} output;

	struct security_monitor_get_wifi_firmware_params_in {
		// The size of a C++ empty struct is 1 byte, but in C it is 0 bytes, so add a
		// padding byte.
		uint8_t padding;
	} input;
};

#define SECURITY_MONITOR_API_GET_APPLICATION_IMAGE_COUNT \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 15)

#define SECURITY_MONITOR_API_LIST_ALL_APPLICATION_IMAGES \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 16)

#define SECURITY_MONITOR_API_SHOULD_IMAGE_BE_UPDATED \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 17)

#define SECURITY_MONITOR_API_INVALIDATE_IMAGE \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 18)

#define SECURITY_MONITOR_API_OPEN_IMAGE_FOR_STAGING \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 19)

#define SECURITY_MONITOR_API_WRITE_BLOCK_TO_STAGE_IMAGE \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 20)

#define SECURITY_MONITOR_API_COMMIT_IMAGE_STAGING \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 21)

#define SECURITY_MONITOR_API_ABORT_IMAGE_STAGING \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 22)

#define SECURITY_MONITOR_API_INSTALL_STAGED_IMAGES \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 23)

#define SECURITY_MONITOR_API_GET_COMPONENT_COUNT \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 24)

#define SECURITY_MONITOR_API_GET_COMPONENT_SUMMARY \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 25)

#define SECURITY_MONITOR_API_GET_COMPONENT_IMAGES \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 26)

#define SECURITY_MONITOR_API_STAGE_COMPONENT_MANIFESTS \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 27)

#define SECURITY_MONITOR_API_COUNT_OF_MISSING_IMAGES_TO_DOWNLOAD \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 28)

#define SECURITY_MONITOR_API_GET_MISSING_IMAGES_TO_DOWNLOAD \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 29)

#define SECURITY_MONITOR_API_SET_PERIPHERAL_MAPPING \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 30)

#define SECURITY_MONITOR_API_SET_PIN_MAPPING \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 31)

#define SECURITY_MONITOR_API_GET_ABI_TYPE_COUNT \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 32)

#define SECURITY_MONITOR_API_GET_ABI_VERSIONS \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 33)

#define SECURITY_MONITOR_API_SET_RTC_CURRENT_TIME \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 34)

#define SECURITY_MONITOR_API_SET_RTC_ALARM \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 35)

#define SECURITY_MONITOR_API_GET_UPDATE_CERT_STORE_IMAGE_INFO \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 36)

#define SECURITY_MONITOR_API_STAGE_BASE_MANIFESTS \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 37)

#define SECURITY_MONITOR_API_COUNT_OF_MISSING_BASE_IMAGES_TO_DOWNLOAD \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 38)

#define SECURITY_MONITOR_API_GET_MISSING_BASE_IMAGES_TO_DOWNLOAD \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 39)

#define SECURITY_MONITOR_API_IO_CORE_CONTROL \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 40)

#define SECURITY_MONITOR_API_GET_ROLLBACK_INFO \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 41)

#define SECURITY_MONITOR_API_RECORD_TELEMETRY_EVENT_DATA \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 42)

#define SECURITY_MONITOR_API_GET_TELEMETRY_DATA \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 43)

#define SECURITY_MONITOR_API_RESET_TELEMETRY \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 44)

#define SECURITY_MONITOR_API_RETAIN_TELEMETRY \
	SECURITY_MONITOR_FUNCTION(SECURITY_MONITOR_API_SYNC, 45)

enum security_monitor_io_core_control_action {
	SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_START_CORE = 0,
	SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_STOP_CORE = 1,
	SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_SET_FLAGS = 2,
	SECURITY_MONITOR_IO_CORE_CONTROL_ACTION_CONFIGURE_COMMUNICATION_BUFFER = 3,
};

enum security_monitor_io_core_communication_buffer_type {
	SECURITY_MONITOR_IO_CORE_COMMUNICATION_BUFFER_FROM_IO_CORE = 0,
	SECURITY_MONITOR_IO_CORE_COMMUNICATION_BUFFER_TO_IO_CORE = 1,
};

struct security_monitor_io_core_control {
	struct {
		uint32_t core_id;
		uint32_t action;
		union {
			struct {
				uint32_t address;
				uint32_t size;
				uint32_t flags;
			} start_core_params;
			struct {
				uint32_t flags;
			} set_flags_params;
			struct {
				uint32_t type;
				uint32_t address;
				uint32_t size;
			} configure_communication_buffer;
		};
	} input_params;
	struct {
		uint32_t result;
	} output_params;
};

