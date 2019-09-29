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

#include <linux/device.h>
#include <linux/lsm_hooks.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/kernel.h>
#include <linux/binfmts.h>
#include <linux/types.h>
#include <linux/security.h>
#include <linux/file.h>
#include <linux/dcache.h>
#include <linux/cred.h>
#include <linux/uaccess.h>
#include <linux/mman.h>

#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <azure-sphere/security.h>
#include "lsm.h"

static int azure_sphere_task_setpgid(struct task_struct *p, pid_t pgid)
{
    struct azure_sphere_task_cred *tsec = p->cred->security;

    if (!tsec->is_app_man && !tsec->job_control_allowed) {
        return -ENOSYS;
    }

    return 0;
}

static int azure_sphere_cred_alloc_blank(struct cred *cred, gfp_t gfp)
{
	struct azure_sphere_task_cred *tsec;

	tsec = kzalloc(sizeof(struct azure_sphere_task_cred), gfp);
	if (!tsec)
		return -ENOMEM;

	cred->security = tsec;
	return 0;
}

static void azure_sphere_cred_free(struct cred *cred)
{
	struct azure_sphere_task_cred *tsec = cred->security;
	cred->security = 0;
	kfree(tsec);
}

static int azure_sphere_cred_prepare(struct cred *new, const struct cred *old, gfp_t gfp)
{
	const struct azure_sphere_task_cred *old_tsec;
	struct azure_sphere_task_cred *tsec;

	old_tsec = old->security;

	tsec = kmemdup(old_tsec, sizeof(struct azure_sphere_task_cred), gfp);
	if (!tsec)
		return -ENOMEM;

	new->security = tsec;
	return 0;
}

static void azure_sphere_cred_transfer(struct cred *new, const struct cred *old)
{
	const struct azure_sphere_task_cred *old_tsec = old->security;
	struct azure_sphere_task_cred *tsec = new->security;

	*tsec = *old_tsec;
}

static void azure_sphere_cred_init_security(void)
{
	struct cred *cred = (struct cred *) current->real_cred;
	struct azure_sphere_task_cred *tsec;

	tsec = kzalloc(sizeof(struct azure_sphere_task_cred), GFP_KERNEL);
	if (!tsec)
		panic("Failed to initialize initial task security object.\n");

	tsec->is_app_man = true;
    tsec->capabilities = AZURE_SPHERE_CAP_ALL;
    cred->security = tsec;
}	

bool azure_sphere_capable(azure_sphere_capability_t cap)
{
    const struct cred *cred;
    const struct azure_sphere_task_cred *tsec;
    bool ret = false;

    cred = get_task_cred(current);
    tsec = cred->security;
    if (!cred->security) {
        put_cred(cred);
        return false;
    }

    ret = ((tsec->capabilities & cap) == cap);

    put_cred(cred);
    return ret;
}

bool azure_sphere_get_component_id(struct azure_sphere_guid *component_id, struct task_struct *p)
{
    const struct cred *cred;
    const struct azure_sphere_task_cred *tsec;

    cred = get_task_cred(p);
    tsec = cred->security;
    if (!cred->security) {
        put_cred(cred);
        return false;
    }

    *component_id = tsec->component_id.guid;

    put_cred(cred);

    return true;
}

static int azure_sphere_security_getprocattr(struct task_struct *p, char *name, char **value)
{
    const struct cred *cred;
    const struct azure_sphere_task_cred *tsec;
    int ret = 0;

    cred = get_task_cred(p);
    tsec = cred->security;

    //if no security entry then fail
    if (!tsec) {
        put_cred(cred);
        return -ENOENT;
    }

    if (strcmp(name, "exec") == 0) {
        *value = kmalloc(sizeof(*tsec), GFP_KERNEL);
        if (*value == NULL) {
            ret = -ENOMEM;
        } else {
            memcpy(*value, tsec, sizeof(*tsec));
            ret = sizeof(*tsec);
        }
    } else if (strcmp(name, "current") == 0) {
        int tenant_id_strlen = strnlen(tsec->daa_tenant_id, sizeof(tsec->daa_tenant_id));
        int len = 5 + 36 + 1 + 5 + tenant_id_strlen + 1 + 15 + 1; // "CID: " + <guid> + "\n" + "TID: " + <tenant id> + "\n" + "CAPS: 00000000\n" + NULL
        *value = kmalloc(len, GFP_KERNEL);
        if (*value == NULL) {
            ret = -ENOMEM;
        } else {
            ret = snprintf(*value, len, "CID: %08X-%04hX-%04hX-%02hhX%02hhX-%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX\nTID: %.*s\nCAPS: %08X\n", 
                tsec->component_id.guid.data1, tsec->component_id.guid.data2, tsec->component_id.guid.data3, 
                tsec->component_id.guid.data4[0], tsec->component_id.guid.data4[1], tsec->component_id.guid.data4[2], tsec->component_id.guid.data4[3],
                tsec->component_id.guid.data4[4], tsec->component_id.guid.data4[5], tsec->component_id.guid.data4[6], tsec->component_id.guid.data4[7],
                tenant_id_strlen, tsec->daa_tenant_id, tsec->capabilities);
        }
    } else {
        ret = -ENOTSUPP;
    }

    put_cred(cred);
    return ret;    
}

static int azure_sphere_security_setprocattr(struct task_struct *p, char *name, void *value, size_t size) {
    struct azure_sphere_security_set_process_details *data = value;
    struct cred *cred;
    struct azure_sphere_task_cred *tsec;
    int ret;

    // Can only set in binary format
    if (strcmp(name, "exec") != 0) {
        return -EINVAL;
    }

    if (value == NULL || size < sizeof(*data)) {
        return -EINVAL;
    }

    if (p != current) {
        // You can only change the current process
        return -EPERM;
    }

    cred = prepare_creds();
    if (!cred) {
        return -ENOMEM;
    }
    tsec = cred->security;

    //if no security entry then fail
    if (!tsec) {
        ret = -ENOENT;
        goto error;
    }

    if (!tsec->is_app_man) {
        ret = -EPERM;
        goto error;
    }


    memcpy(&tsec->component_id, data->component_id, sizeof(tsec->component_id));
    memset(&tsec->daa_tenant_id, 0, sizeof(tsec->daa_tenant_id));
    memcpy(&tsec->daa_tenant_id, data->daa_tenant_id, strnlen(data->daa_tenant_id, sizeof(tsec->daa_tenant_id) - 1));
    tsec->is_app_man = false;
    tsec->job_control_allowed = data->job_control_allowed;
    tsec->capabilities = data->capabilities;

    return commit_creds(cred);

error:
    abort_creds(cred);
    return ret;
}

#ifdef CONFIG_AZURE_SPHERE_MMAP_EXEC_PROTECTION
int azure_sphere_mmap_file(struct file *file, unsigned long reqprot, unsigned long prot, unsigned long flags) {
    // if attempting write and execute at the same time then deny
    if((reqprot & (PROT_WRITE | PROT_EXEC)) == (PROT_WRITE | PROT_EXEC))
        return -EPERM;

    //if requesting exec on shared memory then fail
    if((reqprot & PROT_EXEC) && (flags & MAP_SHARED))
        return -EPERM;

    //all good
    return 0;
}

int azure_sphere_file_mprotect(struct vm_area_struct *vma, unsigned long reqprot, unsigned long prot) {
    // if attempting write and execute at the same time then deny
    if((reqprot & (PROT_WRITE | PROT_EXEC)) == (PROT_WRITE | PROT_EXEC))
        return -EPERM;

    // check the current VMA flags, if swapping between write and execute then fail
    if((vma->vm_flags & VM_WRITE) && (reqprot & PROT_EXEC)) {
        return -EPERM;
    }
    else if((vma->vm_flags & VM_EXEC) && (reqprot & PROT_WRITE)) {
        return -EPERM;
    }
    else if((vma->vm_flags & VM_SHARED) && (reqprot & PROT_EXEC)) {
        return -EPERM;
    }
    return 0;
}
#endif

static struct security_hook_list azure_sphere_hooks[] = {
    LSM_HOOK_INIT(task_setpgid, azure_sphere_task_setpgid),
    LSM_HOOK_INIT(cred_alloc_blank, azure_sphere_cred_alloc_blank),
    LSM_HOOK_INIT(cred_free, azure_sphere_cred_free),
    LSM_HOOK_INIT(cred_prepare, azure_sphere_cred_prepare),
    LSM_HOOK_INIT(cred_transfer, azure_sphere_cred_transfer),
    LSM_HOOK_INIT(getprocattr, azure_sphere_security_getprocattr),
    LSM_HOOK_INIT(setprocattr, azure_sphere_security_setprocattr),
#ifdef CONFIG_AZURE_SPHERE_MMAP_EXEC_PROTECTION
    LSM_HOOK_INIT(mmap_file, azure_sphere_mmap_file),
    LSM_HOOK_INIT(file_mprotect, azure_sphere_file_mprotect),
#endif
};

static int __init azure_sphere_lsm_init(void)
{
    if (!security_module_enable("AzureSphere")) {
        printk(KERN_INFO "Azure Sphere LSM disabled by boot time parameter");
		return 0;
	}

    // Setup init perms
    azure_sphere_cred_init_security();

    security_add_hooks(azure_sphere_hooks, ARRAY_SIZE(azure_sphere_hooks));

    return 0;
}

security_initcall(azure_sphere_lsm_init);
