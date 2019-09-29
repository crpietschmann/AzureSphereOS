// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Security Monitor security policy
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cred.h>
#include <linux/sched.h>
#include <linux/security.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <azure-sphere/security.h>

#include "caller_security.h"
#include "sm_user.h"

typedef struct
{
    azure_sphere_smapi_command cmd;
    azure_sphere_capability_t caps;
} azure_sphere_sm_cap_lookup_t;

// ----- Required capabilities for security monitor API cmds
static azure_sphere_sm_cap_lookup_t azure_sphere_sm_cmd_required_capabilities[] = {
    {.cmd = AZURE_SPHERE_SMAPI_GET_APPLICATION_IMAGE_COUNT, .caps = AZURE_SPHERE_CAP_QUERY_IMAGE_INFO},
    {.cmd = AZURE_SPHERE_SMAPI_LIST_ALL_APPLICATION_IMAGES, .caps = AZURE_SPHERE_CAP_QUERY_IMAGE_INFO},
    {.cmd = AZURE_SPHERE_SMAPI_SHOULD_IMAGE_BE_UPDATED, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_INVALIDATE_IMAGE, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_OPEN_IMAGE_FOR_STAGING, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_WRITE_BLOCK_TO_STAGE_IMAGE, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_COMMIT_IMAGE_STAGING, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_ABORT_IMAGE_STAGING, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_INSTALL_STAGED_IMAGES, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_GET_COMPONENT_COUNT, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_GET_COMPONENT_SUMMARY, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_GET_COMPONENT_IMAGES, .caps = AZURE_SPHERE_CAP_QUERY_IMAGE_INFO},
    {.cmd = AZURE_SPHERE_SMAPI_STAGE_COMPONENT_MANIFESTS, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_COUNT_OF_MISSING_IMAGES_TO_DOWNLOAD, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_GET_MISSING_IMAGES_TO_DOWNLOAD, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_SET_PERIPHERAL_MAPPING, .caps = AZURE_SPHERE_CAP_PERIPHERAL_PIN_MAPPING},
    {.cmd = AZURE_SPHERE_SMAPI_SET_PIN_MAPPING, .caps = AZURE_SPHERE_CAP_PERIPHERAL_PIN_MAPPING},
    {.cmd = AZURE_SPHERE_SMAPI_GET_ABI_TYPE_COUNT, .caps = AZURE_SPHERE_CAP_QUERY_IMAGE_INFO},
    {.cmd = AZURE_SPHERE_SMAPI_GET_ABI_VERSIONS, .caps = AZURE_SPHERE_CAP_QUERY_IMAGE_INFO},
    {.cmd = AZURE_SPHERE_SMAPI_GET_UPDATE_CERT_STORE_IMAGE_INFO, .caps = AZURE_SPHERE_CAP_QUERY_IMAGE_INFO},
    {.cmd = AZURE_SPHERE_SMAPI_STAGE_BASE_MANIFESTS, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_COUNT_OF_MISSING_BASE_IMAGES_TO_DOWNLOAD, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_GET_MISSING_BASE_IMAGES_TO_DOWNLOAD, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
    {.cmd = AZURE_SPHERE_SMAPI_GET_SOFTWARE_ROLLBACK_INFO, .caps = AZURE_SPHERE_CAP_UPDATE_IMAGE},
};

///
/// Returns the set of Azure Sphere capabilities required to execute the given command
///
/// @cmd - the command
/// @returns - the required capabilities
static azure_sphere_capability_t azure_sphere_get_smapi_cmd_required_capabilities(azure_sphere_smapi_command cmd) {
    u32 i;
    for (i = 0; i < ARRAY_SIZE(azure_sphere_sm_cmd_required_capabilities); i++) {
        azure_sphere_sm_cap_lookup_t *cmd_caps = &azure_sphere_sm_cmd_required_capabilities[i];
        if (cmd_caps->cmd == cmd) {
            return cmd_caps->caps;
        }
    }
    // Unless otherwise specified, commands require all capabilities
    return AZURE_SPHERE_CAP_ALL;
}


bool azure_sphere_caller_has_capabilities_for_smapi_cmd(azure_sphere_smapi_command cmd) {
    const azure_sphere_capability_t cmd_req_caps = azure_sphere_get_smapi_cmd_required_capabilities(cmd);
    return azure_sphere_capable(cmd_req_caps);
}
