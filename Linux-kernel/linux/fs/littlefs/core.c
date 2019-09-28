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

#include "littlefs.h"
#include "core/lfs.h"

// Read a region in a block. Negative error codes are propogated
// to the user.
int littlefs_core_read(const struct lfs_config *c, lfs_block_t block,
		lfs_off_t off, void *buffer, lfs_size_t size)
{
	struct littlefs_sb_info *sb;
	size_t target, ret_len;

	sb = c->context;

	if (block >= c->block_count) {
		return -EINVAL;
	}

	// Compute target address in MTD space
	target = (c->block_size * block) + off;

	return mtd_read(sb->mtd, target, size, &ret_len, buffer);
}

// Program a region in a block. The block must have previously
// been erased. Negative error codes are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int littlefs_core_prog(const struct lfs_config *c, lfs_block_t block,
		lfs_off_t off, const void *buffer, lfs_size_t size)
{
	struct littlefs_sb_info *sb;
	size_t target, ret_len;

	sb = c->context;

	if (block >= c->block_count) {
		return -EINVAL;
	}

	// Compute target address in MTD space
	target = (c->block_size * block) + off;

	return mtd_write(sb->mtd, target, size, &ret_len, buffer);
}

// Erase a block. A block must be erased before being programmed.
// The state of an erased block is undefined. Negative error codes
// are propogated to the user.
// May return LFS_ERR_CORRUPT if the block should be considered bad.
int littlefs_core_erase(const struct lfs_config *c, lfs_block_t block)
{
	struct littlefs_sb_info *sb;
	struct erase_info instr;
	size_t target;
	
	sb = c->context;

	// Compute target address in MTD space
	target = (c->block_size * block);

	memset(&instr, 0, sizeof(instr));

	instr.mtd = sb->mtd;
	instr.addr = target;
	instr.len = c->block_size;
	instr.callback = NULL;

	return mtd_erase(sb->mtd, &instr);
}

// Sync the state of the underlying block device. Negative error codes
// are propogated to the user.
int littlefs_core_sync(const struct lfs_config *c)
{	
	// Nothing to do here
	return 0;
}