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
#ifndef _UAPI__LINUX_AZURE_SPHERE_SECURITY_MONITOR_H
#define _UAPI__LINUX_AZURE_SPHERE_SECURITY_MONITOR_H

#include <linux/ioctl.h>
#include <linux/types.h>

// security monitor API commands
typedef enum {
    AZURE_SPHERE_SMAPI_GET_APPLICATION_IMAGE_COUNT = 0,
    AZURE_SPHERE_SMAPI_LIST_ALL_APPLICATION_IMAGES = 1,
    AZURE_SPHERE_SMAPI_SHOULD_IMAGE_BE_UPDATED = 2,
    AZURE_SPHERE_SMAPI_INVALIDATE_IMAGE = 3,
    AZURE_SPHERE_SMAPI_OPEN_IMAGE_FOR_STAGING = 4,
    AZURE_SPHERE_SMAPI_WRITE_BLOCK_TO_STAGE_IMAGE = 5,
    AZURE_SPHERE_SMAPI_COMMIT_IMAGE_STAGING = 6,
    AZURE_SPHERE_SMAPI_ABORT_IMAGE_STAGING = 7,
    AZURE_SPHERE_SMAPI_INSTALL_STAGED_IMAGES = 8,
    AZURE_SPHERE_SMAPI_GET_COMPONENT_COUNT = 9,
    AZURE_SPHERE_SMAPI_GET_COMPONENT_SUMMARY = 10,
    AZURE_SPHERE_SMAPI_GET_COMPONENT_IMAGES = 11,
    AZURE_SPHERE_SMAPI_STAGE_COMPONENT_MANIFESTS = 12,
    AZURE_SPHERE_SMAPI_COUNT_OF_MISSING_IMAGES_TO_DOWNLOAD = 13,
    AZURE_SPHERE_SMAPI_GET_MISSING_IMAGES_TO_DOWNLOAD = 14,
    AZURE_SPHERE_SMAPI_SET_PERIPHERAL_MAPPING = 15,
    AZURE_SPHERE_SMAPI_SET_PIN_MAPPING = 16,
    AZURE_SPHERE_SMAPI_GET_ABI_TYPE_COUNT = 17,
    AZURE_SPHERE_SMAPI_GET_ABI_VERSIONS = 18,
    AZURE_SPHERE_SMAPI_GET_UPDATE_CERT_STORE_IMAGE_INFO = 19,
    AZURE_SPHERE_SMAPI_STAGE_BASE_MANIFESTS = 20,
    AZURE_SPHERE_SMAPI_COUNT_OF_MISSING_BASE_IMAGES_TO_DOWNLOAD = 21,
    AZURE_SPHERE_SMAPI_GET_MISSING_BASE_IMAGES_TO_DOWNLOAD = 22,
    AZURE_SPHERE_SMAPI_IO_CORE_CONTROL = 23,
    AZURE_SPHERE_SMAPI_GET_SOFTWARE_ROLLBACK_INFO = 24,
    AZURE_SPHERE_SMAPI_RECORD_TELEMETRY_EVENT_DATA = 25,
    AZURE_SPHERE_SMAPI_GET_TELEMETRY = 26,
    AZURE_SPHERE_SMAPI_RESET_TELEMETRY = 27,
    AZURE_SPHERE_SMAPI_RETAIN_TELEMETRY = 28,

    AZURE_SPHERE_SMAPI_MAX
} azure_sphere_smapi_command;

// capabilities a caller may hold
typedef enum {
    AZURE_SPHERE_CAP_NONE = 0,
    AZURE_SPHERE_CAP_UPDATE_IMAGE = 1 << 0,
    AZURE_SPHERE_CAP_QUERY_IMAGE_INFO = 1 << 1,
    AZURE_SPHERE_CAP_UPDATE_SECURITY_STATE = 1 << 2,
    // 1 << 3 is reserved
    AZURE_SPHERE_CAP_SFS = 1 << 4,
    AZURE_SPHERE_CAP_ATTESTATION_RUNTIME = 1 << 5,
    AZURE_SPHERE_CAP_POSTCODE = 1 << 6,
    AZURE_SPHERE_CAP_PERIPHERAL_PIN_MAPPING = 1 << 7,
    AZURE_SPHERE_CAP_RECORD_TELEMETRY = 1 << 8,
    AZURE_SPHERE_CAP_MANAGE_LOG_AND_TELEMETRY = 1 << 9,
    AZURE_SPHERE_CAP_ALL = ~AZURE_SPHERE_CAP_NONE
} azure_sphere_capability_t;

// Request to write log data
struct azure_sphere_write_log_data {
    // Data size
    uint32_t size;
    // Data to write
    const void *data;
};

typedef enum {
    AZURE_SPHERE_DEFAULT_LOG_STORAGE = 0,
    AZURE_SPHERE_TELEMETRY_LOG_STORAGE = 1
} azure_sphere_log_storage_type;

struct azure_sphere_get_log_data {
    // Offset into the data
    uint32_t offset;
    // Buffer to load to
    void *buffer;
    // Size of buffer
    uint32_t buffer_size;
    // Total size to read
    uint32_t total_size;
    // Storage to read from
    azure_sphere_log_storage_type storage_type;
};

struct azure_sphere_smapi_command_request {
    // Command ID
    azure_sphere_smapi_command command;
    // Input blob
    void *input_params;
    // Size of input
    uint32_t input_params_length;
    // Output buffer
    void *output_params;
    // Size of output
    uint32_t output_params_length;
};

struct azure_sphere_derive_key_request {
    // Indication of whether an image-specific or component-specific key should be derived
    uint32_t key_type;
    // generation delta of the key to be derived (0: current, 1: previous etc.)
    uint32_t generation_delta;
};

struct azure_sphere_derive_key_result {
    // Result status code
    uint32_t status_code;
    // instance of key material
    uint32_t instance;
    // actual generation of returned key (including generation delta of request)
    uint32_t generation;
    // derived key
    uint8_t key[32];
};

struct azure_sphere_derive_key {
    // input data
    struct azure_sphere_derive_key_request request;
    // output data
    struct azure_sphere_derive_key_result result;
};

enum azure_sphere_peripheral_type {
    AZURE_SPHERE_GPIO = 0,
    AZURE_SPHERE_UART = 1,
    AZURE_SPHERE_I2C = 2,
    AZURE_SPHERE_SPI_MASTER = 3,
    AZURE_SPHERE_SPI_SLAVE = 4,
    AZURE_SPHERE_CAN = 5,
    AZURE_SPHERE_I2S = 6,
    AZURE_SPHERE_PWM = 7,
    AZURE_SPHERE_SDIO = 8,
    AZURE_SPHERE_ETHERNET = 9,
    AZURE_SPHERE_ADC = 10,
    AZURE_SPHERE_DAC = 11,
    AZURE_SPHERE_JTAG = 12,
    AZURE_SPHERE_WIFI = 13,
    AZURE_SPHERE_EINT = 14,
    AZURE_SPHERE_INVALID = 0xFF
};

struct azure_sphere_peripheral_count {
    // type of peripheral
    uint16_t peripheral_type;
    // Result
    uint32_t status_code;
    // output count
    uint32_t count;
};

struct azure_sphere_list_peripherals {
    // type of peripheral
    uint16_t peripheral_type;
    // buffer size
    uint32_t buffer_size;
    // list of peripheral information
    void *info;
    // entry size
    uint32_t entry_size;
};

struct azure_sphere_enable_peripheral_driver {
    // type of peripheral
    uint16_t peripheral_type;
    // logical index of peripheral instance
    uint16_t peripheral_index;
    // enable or disable?
    bool enable;
};

struct azure_sphere_security_set_process_details {
    // Process component ID
    uint8_t component_id[16];
    // Is job control allowed from this process?
    bool job_control_allowed;
    // Process DAA tenant id
    char daa_tenant_id[64];
    // Capabilities for this process
    azure_sphere_capability_t capabilities;
};

typedef enum {
    AZURE_SPHERE_IO_CORE_CONTROL_RUN = 0,
    AZURE_SPHERE_IO_CORE_CONTROL_STOP = 1,
    AZURE_SPHERE_IO_CORE_CONTROL_START_COMMUNICATION = 2,
} azure_sphere_io_core_control_request;

typedef enum {
    AZURE_SPHERE_IO_CORE_CONTROL_FLAG_ENABLE_SWD = 1,
    AZURE_SPHERE_IO_CORE_CONTROL_FLAG_LOADER_BREAKPOINT = 2,
} azure_sphere_io_core_control_flags;

struct azure_sphere_io_core_control {
    // request
    uint16_t request;
    // the core number
    uint16_t core;
    // file descriptor of executable to run
    int fd;
    // flags
    uint32_t flags;
    // component_id
    uint8_t component_id[16];
};

struct azure_sphere_telemetry_record_event_data {
    // Telemetry ID
    uint16_t id;
    // Number of bytes of payload data
    uint8_t payload_length;
    // Time of the telemetry event
    uint32_t event_timestamp;
    // Payload data
    const void* payload;
};

struct azure_sphere_get_telemetry {
    // Offset into the data
    uint16_t offset;
    // Size of buffer
    uint16_t buffer_size;
    // Buffer to load to
    void *buffer;
};

struct azure_sphere_reset_retain_telemetry {
    bool retain; // true if retain, false if reset
};

// 0x01 is reserved
#define SECURITY_MONITOR_WRITE_LOG _IOWR('p', 0x02, struct azure_sphere_write_log_data)
#define SECURITY_MONITOR_GET_LOG_DATA _IOWR('p', 0x03, struct azure_sphere_get_log_data)
#define SECURITY_MONITOR_SMAPI_COMMAND _IOWR('p', 0x04, struct azure_sphere_smapi_command_request)
#define SECURITY_MONITOR_DERIVE_KEY _IOWR('p', 0x05, struct azure_sphere_derive_key)
#define SECURITY_MONITOR_GET_PERIPHERAL_COUNT _IOWR('p', 0x06, struct azure_sphere_peripheral_count)
#define SECURITY_MONITOR_LIST_PERIPHERALS _IOWR('p', 0x07, struct azure_sphere_list_peripherals)
// 0x08 is reserved
#define SECURITY_MONITOR_ENABLE_PERIPHERAL_DRIVER _IOWR('p', 0x09, struct azure_sphere_enable_peripheral_driver)
#define SECURITY_MONITOR_IO_CORE_CONTROL _IOWR('p', 0x0A, struct azure_sphere_io_core_control)
#define SECURITY_MONITOR_RECORD_TELEMETRY_EVENT_DATA _IOW('p', 0x0B, struct azure_sphere_telemetry_record_event_data)
#define SECURITY_MONITOR_GET_TELEMETRY _IOWR('p', 0x0C, struct azure_sphere_get_telemetry)
#define SECURITY_MONITOR_RESET_RETAIN_TELEMETRY _IOW('p', 0x0D, struct azure_sphere_reset_retain_telemetry)

#endif /* _UAPI__LINUX_AZURE_SPHERE_SECURITY_MONITOR_H */
