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

// Structure for output from querying flash; includes information
// about flash size along with the available size of erase and write
// operations.
struct azure_sphere_sm_flash_info
{
	// Total size of the flash, in bytes.
	u32 length;

	struct
	{
		// Size of the smallest supported flash erase operation.
		u32 min_length;
		// Size of the *preferred* flash erase operation.
		u32 preferred_length;
		// Size of the largest supported flash erase operation.
		u32 max_length;
	} erase;

	struct
	{
		// Size of the smallest supported flash write operation.
		u32 min_length;
		// Size of the *preferred* flash write operation.
		u32 preferred_length;
		// Size of the largest supported flash write operation.
		u32 max_length;
	} write;
};

// Internal representation of RTC time
struct azure_sphere_rtc_time
{
	u32 time_sec;	// seconds
	u32 time_min;	// minutes
	u32 time_hour;   // hours
	u32 time_mday;   // day (of month)
	u32 time_mon;	// month
	u32 time_year;   // year
	u32 time_wday;   // day (of week)
	u32 time_yday;   // day (of year)
	bool time_isdst; // daylight savings in effect
};

// Internal representation of RTC alarm
struct azure_sphere_rtc_wake_alarm
{
	bool enabled;					   // alarm enabled state
	bool pending;					   // alarm pending
	struct azure_sphere_rtc_time time; // alarm time
};

///
/// Gets the version number of the Security Monitor. The version number is
/// incremented each time the interface changes, and will never be 0.
///
/// @returns - version number
u32 azure_sphere_sm_get_version(void);

///
/// Queries the Security Monitor for information about the device's flash.
///
/// @flash_info - on success, receives information about the flash.
/// @returns - 0 on success; a negative error code otherwise.
int azure_sphere_sm_query_flash(struct azure_sphere_sm_flash_info *flash_info);

///
/// Erases the specified range from the device's flash storage.
///
/// @start_offset - the starting location for erasing, expressed as a byte
/// 	offset from the start of flash; must be aligned to the erasure length.
/// @length - the number of bytes to erase; must be an exact multiple of one
/// 	of the supported erasure operations. azure_sphere_sm_query_flash may
///  	be called to query the supported sizes.
/// @returns - 0 on success; a negative error code otherwise.
int azure_sphere_sm_erase_flash(u32 start_offset, u32 length);

///
/// Erases the specified range from the device's flash storage.
///
/// @start_offset - the starting location for writing, expressed as a byte
/// 	offset from the start of flash; must be aligned to the write length.
/// @length - the number of bytes to write; must be an exact multiple of one
/// 	of the supported write operations. azure_sphere_sm_query_flash may
///  	be called to query the supported sizes.
/// @data - the bytes to write.
/// @returns - 0 on success; a negative error code otherwise.
int azure_sphere_sm_write_flash(u32 start_offset, u32 length, const void *data);

///
/// Send generic command to Security Monitor
///
/// @cmd - Command id.
/// @input_param - Input param structure.
/// @input_length - The length of input param in bytes
/// @output_param - Output param structure.
/// @output_length - The length of output param in bytes
/// @returns - 0 for success
int azure_sphere_sm_generic_command_from_user(u32 cmd, void *input_param, u32 input_length, void *output_param, u32 output_length);

///
/// Verify image.
///
/// @flash_address - the flash address of the image to verify.
/// @returns - 0 on success; a negative error code otherwise.
int azure_sphere_sm_verify_image_by_flash_address(u32 flash_address);

///
/// Reset device
///
/// @returns - noreturn on success; a negative error code otherwise.
int azure_sphere_sm_reset(void);

///
/// Write log data to Security Monitor log storage
///
/// @log_data - pointer to the log data
/// @log_data_size - size of the log data
/// @returns - WriteLogReturnCode
int azure_sphere_sm_write_log_from_user(const void __user *log_data, u32 log_data_size);

///
/// Get log data size
///
/// @storage_type - storage to get the size of
/// @returns - log data size in bytes
int azure_sphere_sm_get_log_data_size(u32 storage_type);

///
/// Get log data
///
/// @storage_type - storage to get the data of
/// @offset - the start offset for the data to receive in the buffer
/// @length - size of the log data within the buffer to get
/// @log_buffer - the log buffer
/// @returns - GetLogDataReturnCode
int azure_sphere_sm_get_log_data(u32 storage_type, u32 offset, u32 length, void __user *log_buffer);

///
/// Get the number of peripherals of the specified type
///
/// @peripheral_type - the type of peripheral to query
/// @returns - number of uarts
int azure_sphere_sm_get_peripheral_count(uint16_t peripheral_type);

///
/// Fill a buffer with information about peripherals that are available
///
/// @peripheral_type - the type of peripheral to query
/// @uart_data - output parameter to fill with data
/// @length - the size of the buffer
/// @entry_length - the expected size of an entry in the output buffer
/// @returns any error code
int azure_sphere_sm_list_peripherals(uint16_t peripheral_type, void __user *uart_data, u32 length, u32 entry_length);

///
/// Derive Application Key
///
/// @client_uid - the UID to generate the key for
/// @generation_delta - the distance from the current generation to the target generation
/// @key - pointer to the generated key (AZURE_SPHERE_SM_DERIVE_KEY_LENGTH bytes)
/// @instance - a pointer to receive the generated key's instance id
/// @generation - a pointer to receive the actually used key generation
/// @returns - 0 on success

#define AZURE_SPHERE_SM_DERIVE_KEY_LENGTH 32
int azure_sphere_sm_derive_key(void *client_uid, u32 generation_delta, void *key, u32 *instance, u32 *generation);

///
/// Get the N9 Wi-fi firmware address and length
///
/// @wifi_firmware_address - The address of the n9 Wi-Fi firmware image.
/// @wifi_firmware_length  - The length of the Wi-Fi firmware image.
/// @returns - 0 on success; a negative error code otherwise.
int azure_sphere_sm_get_n9_firmware_location(u32 *wifi_firmware_address, u32 *wifi_firmware_length);

///
/// Sets the current time in the RTC.
///
/// @time - Current date & time to set.
/// @returns - 0 for success
int azure_sphere_set_rtc_current_time(struct azure_sphere_rtc_time *time);

///
/// Sets the alarm time and enable state in the RTC.
///
/// @time - Alarm date & time and enable state to set.
/// @returns - 0 for success
int azure_sphere_set_rtc_alarm(struct azure_sphere_rtc_wake_alarm *alarm);

///
/// Starts an IO core
///
/// @core_id - The IO Core instance to control
/// @physical_address - The physical address of the executable to run
/// @size - The size of the executable
/// @flags - Debug flags
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_io_core_start(int core, uintptr_t physical_address, loff_t size, uint32_t flags);

///
/// Stops an IO core
///
/// @core_id - The IO Core instance to control
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_io_core_stop(int core);

///
/// Sets debug flags of an IO core
///
/// @core_id - The IO Core instance to control
/// @returns - 0 for success, non-zero for failure
/// @flags - Debug flags
int azure_sphere_sm_io_core_set_flags(int core, uint32_t flags);

///
/// Record a telemetry event
///
/// @id - Telemetry ID to record
/// @event_timestamp - Time of the telemetry event
/// @payload_length - Number of bytes of payload data.
/// @payload - Payload data.  May be null if payload_length is 0
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_record_telemetry_event_data(uint16_t id, uint32_t event_timestamp, uint8_t payload_length, const void __user *payload);

///
/// Get telemetry data
///
/// @offset - Offset into aggregate to begin copying.  Pass 0 to create the aggregate.
/// @buffer - Buffer to store the aggregate in.  Should be exactly 4096.
/// @buffer_size - Length of the output_buffer, in bytes.
/// @returns - Length of data written to the output buffer, or <= 0 on failure
int azure_sphere_sm_get_telemetry(uint16_t offset, void __user *buffer, uint16_t buffer_size);

///
/// Reset or retain telemetry data
///
/// @retain - true to retain, false to reset
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_reset_retain_telemetry(bool retain);

/// Configures a communication buffer for an IO Core
///
/// @core - The IO Core instance to control
/// @type - The type of the communication buffer
/// @physical_address - The physical address of the communication buffer
/// @size - The size of the communication buffer
/// @returns - 0 for success, non-zero for failure
int azure_sphere_sm_io_core_configure_communication_buffer(int core, uint32_t type, uintptr_t physical_address, loff_t size);

/*
 * Add simple static assert that will be used to validate compatibility structs
 */
#ifndef static_assert
#define static_assert(x, y) typedef int __security_monitor_static_assert[(x) ? 0 : -1]
#endif // static_assert
