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

#ifndef _LITTLEFS_H
#define _LITTLEFS_H

#include <linux/list.h>
#include <linux/types.h>
#include <linux/mutex.h>

#include "core/lfs.h"

struct littlefs_quota_info {
    kuid_t uid;
    uint32_t used_bytes;
    uint32_t limit_bytes;
    uint32_t flags;

    struct list_head list;
};

struct littlefs_sb_info {
	struct mtd_info *mtd;
    struct lfs_config cfg;

    lfs_t lfs;

	atomic_t highest_ino;

    struct mutex sem;

	/* OS-private pointer for getting back to master superblock info */
	void *os_priv;

    struct list_head quota_list;
};

struct littlefs_inode_info {
    union {
        lfs_dir_t dir;
        lfs_file_t file;
    } lfs;

    struct lfs_attr attrs[3];
};

#define LITTLEFS_ATTR_UID 0xC0
#define LITTLEFS_ATTR_GID 0xC1
#define LITTLEFS_ATTR_MODE 0xC2

#define LITTLEFS_SB_INFO(sb) (sb->s_fs_info)
#define LITTLEFS_INODE_INFO(in) (in->i_private)

#define ITIME(sec) ((struct timespec){sec, 0})

void littlefs_dirty_inode(struct inode *inode, int flags);
void littlefs_evict_inode(struct inode *inode);
struct inode *littlefs_get_inode(struct super_block *sb, struct inode *dir, const char *name, umode_t type);

int littlefs_core_read(const struct lfs_config *c, lfs_block_t block,
		lfs_off_t off, void *buffer, lfs_size_t size);
int littlefs_core_prog(const struct lfs_config *c, lfs_block_t block,
		lfs_off_t off, const void *buffer, lfs_size_t size);
int littlefs_core_erase(const struct lfs_config *c, lfs_block_t block);
int littlefs_core_sync(const struct lfs_config *c);

int littlefs_init_quota(struct super_block *sb);
int littlefs_quota_charge_file(struct littlefs_sb_info *c, kuid_t uid, int old_size, int new_size);
int littlefs_quota_charge_dir(struct littlefs_sb_info *c, kuid_t uid, int delta);

#endif