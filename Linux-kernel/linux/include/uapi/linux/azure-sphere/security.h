// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Linux Security Module
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
#ifndef __UAPI_AZURE_SPHERE_SECURITY_H
#define __UAPI_AZURE_SPHERE_SECURITY_H
#include <linux/types.h>

// Newer kernels have a good UUID framework - switch to that
// when available
struct azure_sphere_guid {
    u32 data1;
    u16 data2;
    u16 data3;
    u8 data4[8];
};

// exposed through /proc/<pid>/attr/exec
struct azure_sphere_task_cred {
    union {
        u8     raw_bytes[16];
        struct azure_sphere_guid guid;
    } component_id;
    char   daa_tenant_id[64];
    bool   is_app_man : 1;
    bool   job_control_allowed : 1;
    unsigned int : 0;
    u32 capabilities;
};

#endif