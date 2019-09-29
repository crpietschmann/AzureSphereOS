// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Execute In Place File System (ASXipFS)
 * 
 * Copyright (C) 2018 Microsoft
 * 
 * ASXipFS is designed for read only file systems that use
 * execute in place (XIP) techniques to limit RAM usage on
 * compatible MTD devices.
 * 
 * The file system supports some basic extended attributes
 * and security settings but does not have generic xattr support.
 * Changes were also made to support CoW scenarios, specifically to
 * enable GDB software breakpoints to work with XIP running apps.
 * 
 * This code is based off of CramFS and related XIP patches.
 * A new file system type is forked since CramFS is deprecated and
 * the XIP patches aren't in the mainline.  This work is extended from
 * the following patches and contributions:
 *
 * Compressed rom filesystem for Linux.
 *
 * Copyright (C) 1999 Linus Torvalds.
 *
 * This file is released under the GPL.
 *
 * Linear Addressing code
 *
 * Copyright (C) 2000 Shane Nay.
 *
 * Allows you to have a linearly addressed cramfs filesystem.
 * Saves the need for buffer, and the munging of the buffer.
 * Savings a bit over 32k with default PAGE_SIZE, BUFFER_SIZE
 * etc.  Usefull on embedded platform with ROM :-).
 *
 * Downsides- Currently linear addressed cramfs partitions
 * don't co-exist with block cramfs partitions.
 *
 * 28-Dec-2000: XIP mode for linear cramfs
 * Copyright (C) 2000 Robert Leslie <rob@mars.org>
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
#ifndef __ASXIPFS_H
#define __ASXIPFS_H

#include <linux/types.h>
#include <linux/magic.h>

/*
 * Width of various bitfields in struct asxipfs_inode.
 */
#define ASXIPFS_MODE_WIDTH 16
#define ASXIPFS_UID_WIDTH 16
#define ASXIPFS_SIZE_WIDTH 24
#define ASXIPFS_GID_WIDTH 8
#define ASXIPFS_NAMELEN_WIDTH 6
#define ASXIPFS_OFFSET_WIDTH 26

/*
 * Since inode.namelen is a unsigned 6-bit number, the maximum asxipfs
 * path length is 63 << 2 = 252.
 */
#define ASXIPFS_MAXPATHLEN (((1 << ASXIPFS_NAMELEN_WIDTH) - 1) << 2)

/*
 * Reasonably terse representation of the inode data.
 */
struct asxipfs_inode {
	__u32 mode:ASXIPFS_MODE_WIDTH, uid:ASXIPFS_UID_WIDTH;
	/* SIZE for device files is i_rdev */
	__u32 size:ASXIPFS_SIZE_WIDTH, gid:ASXIPFS_GID_WIDTH;
	/* NAMELEN is the length of the file name, divided by 4 and
           rounded up.  (asxipfs doesn't support hard links.) */
	/* OFFSET: For symlinks and non-empty regular files, this
	   contains the offset (divided by 4) of the file data.  
       For non-empty directories it is the offset
	   (divided by 4) of the inode of the first file in that
	   directory.  For anything else, offset is zero. */
	__u32 namelen:ASXIPFS_NAMELEN_WIDTH, offset:ASXIPFS_OFFSET_WIDTH;
};

struct asxipfs_info {
	__u32 crc;
	__u32 edition;
	__u32 blocks;
	__u32 files;
};

/*
 * Superblock information at the beginning of the FS.
 */
struct asxipfs_super {
	__u32 magic;			/* 0x28cd3d45 - random number */
	__u32 size;			/* length in bytes */
	__u32 flags;			/* feature flags */
	__u32 future;			/* reserved for future use */
	__u8 signature[16];		/* ????? unused for now */
	struct asxipfs_info fsid;	/* unique filesystem info */
	__u8 name[16];			/* user-defined name */
	struct asxipfs_inode root;	/* root inode data */
};

/*
 * Feature flags
 */
#define ASXIPFS_FLAG_FSID_VERSION_2	0x00000001	/* fsid version #2 */
#define ASXIPFS_FLAG_SORTED_DIRS		0x00000002	/* sorted dirs */
#define ASXIPFS_FLAG_CAPABILITIES	0x00000004	/* file capabilities */
#define ASXIPFS_FLAG_WRONG_SIGNATURE	0x00000200	/* reserved */
#define ASXIPFS_FLAG_SHIFTED_ROOT_OFFSET	0x00000400	/* shifted root fs */
#define ASXIPFS_FLAG_XATTR			0x00000800 /* extended attribute support */

/*
 * Valid values in super.flags.  Currently we refuse to mount
 * if (flags & ~ASXIPFS_SUPPORTED_FLAGS).  Maybe that should be
 * changed to test super.future instead.
 * Note: we still have the XATTR flag for compatibility on loads
 * although we are no longer supporting XATTR as it is unknown
 * if we are just setting it on current images
 */
#define ASXIPFS_SUPPORTED_FLAGS	( 0x000000ff \
				| ASXIPFS_FLAG_WRONG_SIGNATURE \
				| ASXIPFS_FLAG_SHIFTED_ROOT_OFFSET \
				| ASXIPFS_FLAG_XATTR )


#endif
