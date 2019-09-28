// SPDX-License-Identifier: GPL-2.0
/*
 * LittleFS Linux File System driver
 * 
 * Copyright (C) 2018 Microsoft
 * 
 * LittleFS is a file system designed for small devices with limited resources.
 * This driver wraps the core LittleFS code into a Linux compatible FS driver.
 *
 * LittleFS Copyright (c) 2017, Arm Limited. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/fs.h>
#include <linux/quota.h>
#include <linux/slab.h>
#include <linux/capability.h>

#include "littlefs.h"
#include "core/lfs.h"

static struct littlefs_quota_info *littlefs_alloc_quota_info(kuid_t uid)
{
	struct littlefs_quota_info *ret = NULL;

	ret = kzalloc(sizeof(*ret), GFP_KERNEL);
	if (ret != NULL) {
		ret->uid = uid;
	}

	return ret;
}

static struct littlefs_quota_info *littlefs_get_quota_for_next_uid(struct littlefs_sb_info *c, kuid_t uid)
{
	struct littlefs_quota_info *item = NULL;

	list_for_each_entry(item, &c->quota_list, list)
	{
		if (uid_gte(item->uid, uid)) {
			return item;
		}
	}

	return NULL;
}

static struct littlefs_quota_info *littlefs_get_quota_for_uid(struct littlefs_sb_info *c, kuid_t uid, bool allocate)
{
	struct littlefs_quota_info *ret = NULL;
	struct littlefs_quota_info *item = NULL;
	struct list_head *insert_point = &c->quota_list;

	// Find the item in the list
	// Keep the list sorted by UID to make enumeration easier
	list_for_each_entry(item, &c->quota_list, list)
	{
		if (uid_eq(item->uid, uid)) {
			// It's our item
			ret = item;
			break;
		} else if (uid_gt(item->uid, uid)) {
			// It goes here
			insert_point = &item->list;
			break;
		}
	}

	// Allocate a new one
	if (allocate && ret == NULL) {
		ret = littlefs_alloc_quota_info(uid);
		if (ret) {
			list_add_tail(&ret->list, insert_point);
		}
	}

	return ret;
}

static int littlefs_quota_charge(struct littlefs_sb_info *c, kuid_t uid, int delta_in_bytes)
{
	struct littlefs_quota_info *info;
	uint32_t abs_amount = delta_in_bytes < 0 ? delta_in_bytes * -1 : delta_in_bytes;
	int charge_amount = 2 * ALIGN(abs_amount, c->cfg.block_size);
	if (delta_in_bytes < 0) {
		charge_amount *= -1;
	}

	info = littlefs_get_quota_for_uid(c, uid, true);
	if (info == NULL) {
		return -ENOMEM;
	}

	// Only check if charge is positive (you can always reclaim space),
	// not holding CAP_SYS_RESOURCE which lets you bypass quotas,
	// and enforcement is enabled
	if (charge_amount > 0 && 
		!capable(CAP_SYS_RESOURCE) && 
		info->flags & QC_SPC_HARD) {
		// Check against quota
		if ((info->used_bytes + charge_amount) > info->limit_bytes) {
			return -EDQUOT;
		}
	}

	// Update accounting
	info->used_bytes += charge_amount;

	return 0;
}

int littlefs_quota_charge_file(struct littlefs_sb_info *c, kuid_t uid, int old_size, int new_size)
{
	old_size = ALIGN(old_size, c->cfg.block_size);
	new_size = ALIGN(new_size, c->cfg.block_size);

	if (old_size != new_size) {
		return littlefs_quota_charge(c, uid, new_size - old_size);
	}

	return 0;
}

int littlefs_quota_charge_dir(struct littlefs_sb_info *c, kuid_t uid, int delta)
{
	return littlefs_quota_charge(c, uid, delta * c->cfg.block_size);
}

static int littlefs_quota_block_callback(void *d, lfs_block_t b) 
{
	return 0;
}

static int littlefs_quota_entry_callback(void *d, lfs_dir_t *dir, lfs_entry_t *e)
{
	kuid_t uid = {0};
	struct lfs_attr attr;
	struct littlefs_sb_info *c = d;
	int ret = 0;

	attr.type = LITTLEFS_ATTR_UID;
	attr.size = sizeof(uid);
	attr.buffer = &uid;

	ret = lfs_dir_getattrs(&c->lfs, dir, e, &attr, 1);
	if (ret) {
		return ret;
	}

	if (e->d.type & LFS_TYPE_DIR) {
		littlefs_quota_charge_dir(c, uid, 1);
	} else if (e->d.type & LFS_TYPE_REG) {
		uint32_t size = e->d.u.file.size;
		if ((0x70 & e->d.type) == LFS_STRUCT_INLINE) {
			// Calculation for inline file size, needed since the data in entry isn't accurate yet
			size = e->d.elen | ((e->d.alen & 0xc0) << 2);
		}
		littlefs_quota_charge_file(c, uid, 0, size);
	}

	return 0;
}

static void littlefs_copy_dqblk_data(struct littlefs_quota_info *info, struct qc_dqblk *dqblk)
{
	memset(dqblk, 0, sizeof(*dqblk));
	dqblk->d_fieldmask |= QC_SPACE;
	dqblk->d_space = info->used_bytes;

	if (info->flags & QC_SPC_HARD) {
		dqblk->d_fieldmask |= QC_SPC_HARD;
		dqblk->d_spc_hardlimit = info->limit_bytes;
	}
}

static int littlefs_get_nextdqblk(struct super_block *sb, struct kqid *id, struct qc_dqblk *dqblk)
{
	struct littlefs_quota_info *info;
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(sb);
	int ret = 0;
	
	if (id->type != USRQUOTA) {
		return -EINVAL;
	}

	mutex_lock(&c->sem);

	info = littlefs_get_quota_for_next_uid(c, id->uid);
	if (info == NULL) {
		ret = -ESRCH;
	} else {
		id->uid = info->uid;
		littlefs_copy_dqblk_data(info, dqblk);
	}

	mutex_unlock(&c->sem);

	return ret;
}

static int littlefs_get_dqblk(struct super_block *sb, struct kqid id, struct qc_dqblk *dqblk)
{
	struct littlefs_quota_info *info;
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(sb);
	int ret = 0;
	
	if (id.type != USRQUOTA) {
		return -EINVAL;
	}

	mutex_lock(&c->sem);

	info = littlefs_get_quota_for_uid(c, id.uid, false);
	if (info == NULL) {
		ret = -ESRCH;
	} else {
		littlefs_copy_dqblk_data(info, dqblk);
	}

	mutex_unlock(&c->sem);

	return ret;
}

static int littlefs_set_dqblk(struct super_block *sb, struct kqid id, struct qc_dqblk *dqblk)
{
	struct littlefs_quota_info *info;
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(sb);
	int ret = 0;
	
	if (id.type != USRQUOTA) {
		return -EINVAL;
	}

	mutex_lock(&c->sem);

	info = littlefs_get_quota_for_uid(c, id.uid, true);
	if (info == NULL) {
		ret = -ENOMEM;
	} else {
		if (dqblk->d_fieldmask & QC_SPC_HARD) {
			info->flags |= QC_SPC_HARD;
			info->limit_bytes = dqblk->d_spc_hardlimit;
		} else {
			info->flags &= ~(QC_SPC_HARD);
			info->limit_bytes = 0;
		}
	}

	mutex_unlock(&c->sem);

	return ret;
}

static struct quotactl_ops littlefs_quotactl_ops = {
	.get_dqblk	= littlefs_get_dqblk,
	.set_dqblk = littlefs_set_dqblk,
	.get_nextdqblk = littlefs_get_nextdqblk,
};

int littlefs_init_quota(struct super_block *sb)
{
	struct lfs_traverse_callbacks callbacks = {
		.block_callback = littlefs_quota_block_callback,
		.entry_callback = littlefs_quota_entry_callback
	};

	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(sb);
	int ret;

	INIT_LIST_HEAD(&c->quota_list);

	// build out current usage
	ret = lfs_traverse(&c->lfs, callbacks, c);

	sb->s_qcop = &littlefs_quotactl_ops;
	sb->s_quota_types = QTYPE_MASK_USR;

	return ret;
}