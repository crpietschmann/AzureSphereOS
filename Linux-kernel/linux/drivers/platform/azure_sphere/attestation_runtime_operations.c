// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere DAA operations driver
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

#ifdef CONFIG_AZURE_SPHERE_SECUIRTY_MONITOR_DEBUG
// Set DEBUG to 1 to enable debug log output
#define DEBUG 1
#endif

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <uapi/linux/azure-sphere/pluton.h>
#include <azure-sphere/pluton_remoteapi.h>
#include <linux/slab.h>
#include <linux/cred.h>
#include <azure-sphere/security.h>

#include "caller_security.h"
#include "sm_user.h"
#include "attestation_runtime_operations.h"
#include "pluton.h"

///
/// PLUTON_GENERATE_CLIENT_AUTH_KEY message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_generate_client_auth_key(void __user *arg, struct pluton_file_data *data, bool async) {
    u32 ret = 0;
    struct azure_sphere_status_code_result output_data;
    
    if (!azure_sphere_capable(AZURE_SPHERE_CAP_ATTESTATION_RUNTIME)) {
        dev_err(g_sm_user->dev, "%s sender not authorized to generate keypair\n", __FUNCTION__);
            ret = -EINVAL;
            goto exit;
    }

    ret = pluton_send_mailbox_message(GENERATE_CLIENT_AUTH_KEY, 
        NULL, 0, &output_data, sizeof(output_data), 0, sizeof(output_data), data, async);

exit:
    if (!ret) {
        ret = output_data.status_code;
    }

    return ret;
}

///
/// PLUTON_COMMIT_CLIENT_AUTH_KEY message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_commit_client_auth_key(void __user *arg, struct pluton_file_data *data, bool async) {
    u32 ret = 0;
    struct azure_sphere_status_code_result output_data;
    
    if (!azure_sphere_capable(AZURE_SPHERE_CAP_ATTESTATION_RUNTIME)) {
        dev_err(g_sm_user->dev, "%s sender not authorized to generate keypair\n", __FUNCTION__);
            ret = -EINVAL;
            goto exit;
    }

    ret = pluton_send_mailbox_message(COMMIT_CLIENT_AUTH_KEY, 
        NULL, 0, &output_data, sizeof(output_data), 0, sizeof(output_data), data, async);

exit:
    if (!ret) {
        ret = output_data.status_code;
    }

    return ret;
}

///
/// PLUTON_GET_TENANT_PUBLIC_KEY message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_get_tenant_public_key(void __user *arg, struct pluton_file_data *data, bool async) {
    u32 ret = 0;
    struct azure_sphere_tenant_id tenant_id;
    struct azure_sphere_ecc256_public_key output_data;
    
    memset(&tenant_id, 0, sizeof(tenant_id));
    ret = copy_from_user(&tenant_id, arg, sizeof(tenant_id));
    if (unlikely(ret)) {
        goto exit;
    }

    ret = pluton_send_mailbox_message(GET_TENANT_PUBLIC_KEY, 
        &tenant_id, sizeof(tenant_id), 
        &output_data, sizeof(output_data), 
        0, sizeof(output_data),
        data, async);

exit:
    // don't send back key data on err
    if (!ret) {
        ret = copy_to_user(arg, &output_data, sizeof(output_data));
    }

    return ret;
}

///
/// PLUTON_PROCESS_ATTESTATION message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_process_attestation(void __user *arg, struct pluton_file_data *data, bool async) {
    u32 ret = 0;
    struct azure_sphere_attestation_command *command_data = NULL;

    if (!azure_sphere_capable(AZURE_SPHERE_CAP_ATTESTATION_RUNTIME)) {
        dev_err(g_sm_user->dev, "%s sender not authorized to process attestation\n", __FUNCTION__);
            ret = -EINVAL;
            goto exit;
    }

    command_data = devm_kmalloc(g_sm_user->dev, sizeof(struct azure_sphere_attestation_command), GFP_KERNEL);
    if (!command_data) {
        ret = -ENOMEM;
        goto exit;
    }

    ret = copy_from_user(command_data, arg, sizeof(struct azure_sphere_attestation_command));
    if (unlikely(ret)) {
        goto exit;
    }

    ret = pluton_send_mailbox_message(PROCESS_ATTESTATION, 
        &command_data->request, sizeof(struct azure_sphere_attestation_request),
        command_data, sizeof(*command_data),
        offsetof(struct azure_sphere_attestation_command, result), sizeof(struct azure_sphere_attestation_result), 
        data, async);

exit:
    // no data sent back on err
    if (command_data) {
        if (!ret) {
            ret = copy_to_user(arg, command_data, sizeof(struct azure_sphere_attestation_command));
        }

        devm_kfree(g_sm_user->dev, command_data);
    }
    

    return ret;
}

///
/// PLUTON_SIGN_WITH_TENANT_ATTESTATION_KEY message handler
///
/// @arg - ioctl buffer
/// @data - file data for FD
/// @async - is the FD in async mode
/// @returns - 0 for success
int pluton_sign_with_tenant_attestation_key(void __user *arg, struct pluton_file_data *data, bool async) {
    u32 ret = 0;
    struct azure_sphere_task_cred *tsec;
    struct azure_sphere_sign_with_tenant_key request;
    struct azure_sphere_ecdsa256_signature signature;

    // no runtime permission check

    ret = copy_from_user(&request.digest, arg, sizeof(request.digest));
    if (unlikely(ret)) {
        return ret;
    }

    // copy out the tenant id
    tsec = current->cred->security;
    memcpy(&request.tenant_id, tsec->daa_tenant_id, sizeof(request.tenant_id));


    ret = pluton_send_mailbox_message(SIGN_WITH_TENANT_ATTESTATION_KEY, 
        &request, sizeof(request), &signature, sizeof(signature), 0, sizeof(signature), data, async);

    // no data sent back on err
    if (!ret) {
        ret = copy_to_user(arg, &signature, sizeof(signature));
    }

    return ret;
}