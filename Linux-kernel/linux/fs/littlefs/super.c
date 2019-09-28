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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/super.h>
#include <linux/statfs.h>

#include "littlefs.h"
#include "core/lfs.h"

static void littlefs_put_super(struct super_block *sb)
{
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(sb);

	mutex_lock(&c->sem);
	lfs_unmount(&c->lfs);
	mutex_unlock(&c->sem);

	mtd_sync(c->mtd);
}

static int littlefs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(dentry->d_sb);
	lfs_ssize_t sz;

	buf->f_type = LITTLEFS_SUPER_MAGIC;
	buf->f_bsize = c->cfg.block_size;
	buf->f_blocks = c->cfg.block_count;
	buf->f_files = 0;
	buf->f_ffree = 0;
	buf->f_namelen = LFS_NAME_MAX;
	buf->f_fsid.val[0] = LITTLEFS_SUPER_MAGIC;
	buf->f_fsid.val[1] = c->mtd->index;

	mutex_lock(&c->sem);
	sz = lfs_fs_size(&c->lfs);
	mutex_unlock(&c->sem);

	buf->f_bavail = buf->f_bfree = c->cfg.block_count - sz;
	
	return 0;
}

static const struct super_operations littlefs_super_operations =
{
	.put_super =	littlefs_put_super,
	.statfs =	littlefs_statfs,
	.evict_inode =	littlefs_evict_inode,
	.dirty_inode =	littlefs_dirty_inode,
};

int littlefs_do_fill_super(struct super_block *sb, void *data, int silent)
{
	struct littlefs_sb_info *c;
	struct inode *root_i;
	int ret;
	uint32_t flash_size, unit_size;

	c = LITTLEFS_SB_INFO(sb);
	
	// Only support NOR flash for now
	if (c->mtd->type != MTD_NORFLASH)
		return -EINVAL;

	flash_size = c->mtd->size;
	// Always try to write / read at least 256 bytes for optimization
	unit_size = c->mtd->writesize >= 256 ? c->mtd->writesize : 256;
	
	c->cfg.read_size = unit_size;
	c->cfg.prog_size = unit_size;
	c->cfg.block_size = c->mtd->erasesize;
	c->cfg.block_count = flash_size / c->cfg.block_size;
	c->cfg.lookahead = 128;

	c->cfg.read  = littlefs_core_read;
	c->cfg.prog  = littlefs_core_prog;
	c->cfg.erase = littlefs_core_erase;
	c->cfg.sync  = littlefs_core_sync;

	c->cfg.context = c;

	// mount the filesystem
	ret = lfs_mount(&c->lfs, &c->cfg);

	// reformat if we can't mount the filesystem
	// this should only happen on the first boot
	if (ret == LFS_ERR_CORRUPT) {
		ret = lfs_format(&c->lfs, &c->cfg);
		if (ret) {
			return ret;
		}
		ret = lfs_mount(&c->lfs, &c->cfg);
		if (ret) {
			return ret;
		}
	} else if (ret) {
		return ret;
	}

	root_i = littlefs_get_inode(sb, NULL, "/", S_IFDIR | S_IRWXU | S_IXGRP | S_IXOTH);
	if (IS_ERR(root_i)) {
		ret = PTR_ERR(root_i);
		return ret;
	}

	sb->s_root = d_make_root(root_i);
	if (!sb->s_root)
		return -ENOMEM;

	sb->s_maxbytes = 0xFFFFFFFF;
	sb->s_blocksize = PAGE_SIZE;
	sb->s_blocksize_bits = PAGE_SHIFT;
	sb->s_magic = LITTLEFS_SUPER_MAGIC;

	ret = littlefs_init_quota(sb);
	return ret;
}

static int littlefs_fill_super(struct super_block *sb, void *data, int silent)
{
	struct littlefs_sb_info *c;
	int ret;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->mtd = sb->s_mtd;
	c->os_priv = sb;
	sb->s_fs_info = c;

	mutex_init(&c->sem);

	sb->s_op = &littlefs_super_operations;
	sb->s_flags = sb->s_flags | MS_NOATIME;
	ret = littlefs_do_fill_super(sb, data, silent);
	return ret;
}

static struct dentry *littlefs_mount(struct file_system_type *fs_type,
			int flags, const char *dev_name, void *data)
{
	return mount_mtd(fs_type, flags, dev_name, data, littlefs_fill_super);
}

static void littlefs_kill_sb(struct super_block *sb)
{
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(sb);
	kill_mtd_super(sb);
	kfree(c);
}

static struct file_system_type littlefs_type = {
	.owner =	THIS_MODULE,
	.name =		"littlefs",
	.mount =	littlefs_mount,
	.kill_sb =	littlefs_kill_sb,
};
MODULE_ALIAS_FS("littlefs");

static int __init init_littlefs(void)
{
	int ret;
	ret = register_filesystem(&littlefs_type);
	if (ret) {
		pr_err("error: Failed to register filesystem\n");
		return ret;
	}
	return 0;
}

static void __exit exit_littlefs(void)
{
	unregister_filesystem(&littlefs_type);
}

module_init(init_littlefs);
module_exit(exit_littlefs);

MODULE_DESCRIPTION("Little File System");
MODULE_LICENSE("GPL");
