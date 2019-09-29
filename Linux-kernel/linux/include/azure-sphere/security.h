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
#ifndef _AZURE_SPHERE_SECURITY_H
#define _AZURE_SPHERE_SECURITY_H
#include <linux/types.h>
#include <uapi/linux/azure-sphere/security_monitor.h>
#include <uapi/linux/azure-sphere/security.h>

struct task_struct;

// Check if the caller is holding a given azure sphere capability
bool azure_sphere_capable(azure_sphere_capability_t cap);

// Get component id of p
bool azure_sphere_get_component_id(struct azure_sphere_guid *component_id, struct task_struct *p);

#endif