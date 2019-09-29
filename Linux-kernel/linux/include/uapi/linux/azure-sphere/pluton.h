// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Pluton API
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
#ifndef _UAPI__LINUX_AZURE_SPHERE_PLUTON_H
#define _UAPI__LINUX_AZURE_SPHERE_PLUTON_H

#include <linux/ioctl.h>
#include <linux/types.h>

// Result for get security state message
struct azure_sphere_get_security_state_result {
    // Security state of device
    uint32_t state;
    // ECC General public key
    uint8_t general_public_key[64];
    // Attestation public key
    uint8_t attestation_public_key[64];
};

struct azure_sphere_ecc256_public_key {
    uint32_t X[8];
    uint32_t Y[8];
};

struct azure_sphere_ecdsa256_signature {
    uint8_t R[32];
    uint8_t S[32];
};

struct azure_sphere_sha256_digest {
    uint8_t data[32];
};

struct azure_sphere_measurement {
    // What type of measurement is this?
    uint32_t type;

    // Value of the measurement
    struct azure_sphere_sha256_digest value;
};

struct azure_sphere_measurement_list {
    uint32_t MeasurementCount;
    struct azure_sphere_measurement Measurements[20];
};

/// <summary>
/// Message used for sending tenant id to M4
/// </summary>
struct azure_sphere_tenant_id {
    char data[64];
};

struct azure_sphere_attestation_request {
    // Nonce for this request
    struct azure_sphere_sha256_digest nonce;
};

struct azure_sphere_sign_with_tenant_key {
    struct azure_sphere_tenant_id tenant_id;
    struct azure_sphere_sha256_digest digest;
};

struct azure_sphere_attestation_result {

    /// <summary>
    /// Was the request successful?
    /// </summary>
    bool successful;

    /// <summary>
    /// Attestation device ID
    /// </summary>
    struct azure_sphere_ecc256_public_key device_id;

    /// <summary>
    /// Attestation signature
    /// </summary>
    struct azure_sphere_ecdsa256_signature signature;

    /// <summary>
    /// List of measurements
    /// </summary>
    struct azure_sphere_measurement_list measurements;
};

struct azure_sphere_attestation_command {
    // Input data
    struct azure_sphere_attestation_request request;
    // Output data
    struct azure_sphere_attestation_result result;
};

struct azure_sphere_status_code_result {
    // status code returned by attestation runtime operation
    uint32_t status_code;
};

struct azure_sphere_boot_mode_flags {
    uint32_t boot_mode_flags;
};

/// <summary>
/// Message used for getting is capability enabled from M4
/// </summary>
struct azure_sphere_is_capability_enabled {
    /// <summary>
    /// Input data: the capability to be checked
    /// </summary>
    uint16_t capability;

    /// <summary>
    /// Output data: is the capability enabled
    /// </summary>
    bool o_is_enabled;
};

/// <summary>
/// Message used for getting enabled capabilities from M4
/// </summary>
struct azure_sphere_get_enabled_capabilities {
    /// <summary>
    /// Input data: the capability to be checked
    /// </summary>
     uint16_t o_enabled_capabilities[64];
};

struct azure_sphere_manufacturing_state {
   // Manufacturing state
   uint32_t completion_state;
};

// IDs 2, 3 are deprecated but reserved.  Do not use these values.
#define PLUTON_GET_SECURITY_STATE _IOWR('p', 0x01, struct azure_sphere_get_security_state_result)
#define PLUTON_GENERATE_CLIENT_AUTH_KEY _IOWR('p', 0x06, uint32_t)
#define PLUTON_COMMIT_CLIENT_AUTH_KEY _IOWR('p', 0x07, uint32_t)
#define PLUTON_GET_TENANT_PUBLIC_KEY _IOWR('p', 0x08, struct azure_sphere_ecc256_public_key)
#define PLUTON_PROCESS_ATTESTATION _IOWR('p', 0x09, struct azure_sphere_attestation_command)
#define PLUTON_SIGN_WITH_TENANT_ATTESTATION_KEY \
    _IOWR('p', 0x0A, struct azure_sphere_ecdsa256_signature)
#define PLUTON_SET_POSTCODE _IOWR('p', 0x0B, uint32_t)
#define PLUTON_GET_BOOT_MODE_FLAGS _IOWR('p', 0x0C, struct azure_sphere_boot_mode_flags)
#define PLUTON_IS_CAPABILITY_ENABLED _IOWR('p', 0x0D, struct azure_sphere_is_capability_enabled)
#define PLUTON_GET_ENABLED_CAPABILITIES _IOR('p', 0x0E, struct azure_sphere_get_enabled_capabilities)
#define PLUTON_SET_MANUFACTURING_STATE _IOW('p', 0x0F, struct azure_sphere_manufacturing_state) 
#define PLUTON_GET_MANUFACTURING_STATE _IOR('p', 0x10, struct azure_sphere_manufacturing_state)

#endif /* _UAPI__LINUX_AZURE_SPHERE_PLUTON_H */
