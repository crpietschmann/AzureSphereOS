// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere Flash driver
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/nand.h>
#include <azure-sphere/security_monitor.h>

static int azure_sphere_flash_read(struct mtd_info *, loff_t, size_t, size_t *, u_char *);
static int azure_sphere_flash_write(struct mtd_info *, loff_t, size_t, size_t *, const u_char *);
static int azure_sphere_flash_erase(struct mtd_info *, struct erase_info *);
static void azure_sphere_flash_nop(struct mtd_info *);
static int azure_sphere_read_oob(struct mtd_info *, loff_t, struct mtd_oob_ops *);
static int azure_sphere_write_oob(struct mtd_info *, loff_t, struct mtd_oob_ops *);
static struct mtd_info *azure_sphere_flash_probe(struct map_info *map);
static unsigned long azure_sphere_flash_unmapped_area(struct mtd_info *, unsigned long,
					  unsigned long, unsigned long);

// This structure is used to register this flash chip support code with the MTD
// infrastructure.
static struct mtd_chip_driver azure_sphere_flash_chipdrv = {
	.probe	= azure_sphere_flash_probe,
	.name	= "azure_sphere_flash",
	.module	= THIS_MODULE
};

///
/// Probes for a flash device accessible on Azure Sphere; if found, it
/// returns information about that device.
///
/// @map - provides information about the MTD device mapping.
/// @returns - a filled-out info struct describing the parameters of the
/// 	discovered flash device, and pointers to the functions that will
/// 	be invoked to handle I/O operations on it. Returns NULL on failure.
static struct mtd_info *azure_sphere_flash_probe(struct map_info *map)
{
	struct mtd_info *mtd;
	int err;
	struct azure_sphere_sm_flash_info flash_info;

	printk(KERN_INFO "Probing for flash devices via SK client\n");

	err = azure_sphere_sm_query_flash(&flash_info);
	if (err != 0) {
		printk(KERN_WARNING "Failed to query flash devices: %#x\n", err);
		return NULL;
	}

	printk(KERN_INFO "Discovered flash: len=%#x\n", flash_info.length);

	if (flash_info.length == 0) {
		printk(KERN_WARNING "Flash device has 0 length");
		return NULL;
	}

	mtd = kzalloc(sizeof(*mtd), GFP_KERNEL);
	if (!mtd)
		return NULL;

	map->fldrv = &azure_sphere_flash_chipdrv;
	mtd->priv = map;
	mtd->name = map->name;
	mtd->type = MTD_NORFLASH;
	mtd->size = map->size;
	mtd->_erase = azure_sphere_flash_erase;
	mtd->_get_unmapped_area = azure_sphere_flash_unmapped_area;
	mtd->_read = azure_sphere_flash_read;
	mtd->_write = azure_sphere_flash_write;
	mtd->_panic_write = azure_sphere_flash_write;
	mtd->_sync = azure_sphere_flash_nop;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->_write_oob = azure_sphere_write_oob;
	mtd->_read_oob = azure_sphere_read_oob;
	mtd->ecc_strength = 0;
	mtd->writesize = flash_info.write.min_length;
	mtd->writebufsize = flash_info.write.max_length;
	mtd->erasesize = flash_info.erase.preferred_length;

	__module_get(THIS_MODULE);
	return mtd;
}

///
/// Provides a direct pointer for reading the indicated offset in flash.
/// Used to allow NOMMU mmap() to directly map the device.
///
/// @mtd - provides info about the MTD device in question.
/// @len - the length of the needed flash range.
/// @offset - the location of the needed flash range, expressed as a byte
/// 	offset from the beginning of flash.
/// @flags - flags for the request.
/// @returns - a pointer to the range of flash.
static unsigned long azure_sphere_flash_unmapped_area(struct mtd_info *mtd,
					  unsigned long len,
					  unsigned long offset,
					  unsigned long flags)
{
	struct map_info *map = mtd->priv;
	return (unsigned long)map->virt + offset;
}

///
/// Reads from the flash device, placing the read data in the provided buffer.
///
/// @mtd - provides info about the MTD device in question.
/// @from - the location in flash from which to start reading, expressed as a
/// 	byte offset from the start of flash.
/// @len - the number of bytes to read.
/// @retlen - receives the number of bytes read.
/// @buf - on success, receives the bytes read.
/// @returns - 0 on success, a negative error code otherwise.
static int azure_sphere_flash_read(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	struct map_info *map = mtd->priv;

	map_copy_from(map, buf, from, len);
	*retlen = len;
	return 0;
}

///
/// Writes to the flash device.
///
/// @mtd - provides info about the MTD device in question.
/// @to - the location in flash at which to start writing, expressed as a byte
/// 	offset from the start of flash.
/// @len - the number of bytes to write.
/// @retlen - receives the number of bytes written.
/// @buf - pointer to a buffer containing the bytes to write.
/// @returns - 0 on success, a negative error code otherwise.
static int azure_sphere_flash_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	int result;
	u32 remaining;
	u32 max_write_len;
	const u8 *cursor;

	/*
	 * Validate offset and length fit in 32 bits.
	 */

	u32 to32 = (u32)to;
	u32 len32 = (u32)len;

	if (to32 != to) {
		printk(KERN_ERR "%s: write dest invalid\n", __func__);
		return -EINVAL;
	}

	if (len32 != len) {
		printk(KERN_ERR "%s: write len invalid\n", __func__);
		return -EINVAL;
	}

	/*
	 * Write!
	 */

	remaining = len32;
	max_write_len = mtd->writebufsize;
	cursor = (const u8 *)buf;

	while (remaining > 0) {
		int chunk_len = remaining;
		int unit_offset = to32 % max_write_len;

		if (chunk_len > max_write_len)
			chunk_len = max_write_len;

		if (unit_offset + chunk_len > max_write_len)
			chunk_len = max_write_len - unit_offset;

		result = azure_sphere_sm_write_flash(to32, chunk_len, cursor);
		if (result < 0) {
			printk(KERN_ERR "%s: write failed: %#x\n", __func__, result);
			return -EIO;
		}

		to32 += chunk_len;
		cursor += chunk_len;
		remaining -= chunk_len;
	}

	*retlen = len32;
	return 0;
}

///
/// Erases a range of bytes on the flash device.
///
/// @mtd - provides info about the MTD device in question.
/// @instr - parameters describing where to erase, how much to erase, etc.
/// @returns - 0 on success, a negative error code otherwise.
static int azure_sphere_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int result;

	/*
	 * Validate offset and length fit in 32 bits.
	 */

	u32 addr32 = (u32)instr->addr;
	u32 len32 = (u32)instr->len;

	if (addr32 != instr->addr) {
		printk(KERN_ERR "%s: erase addr invalid\n", __func__);
		return -EINVAL;
	}

	if (len32 != instr->len) {
		printk(KERN_ERR "%s: erase len invalid\n", __func__);
		return -EINVAL;
	}

	/*
	 * Erase!
	 */

	result = azure_sphere_sm_erase_flash(addr32, len32);

	if (result == 0) {
		instr->state = MTD_ERASE_DONE;
	} else {
		printk(KERN_ERR "%s: erase failed: %#x\n", __func__, result);
		instr->state = MTD_ERASE_FAILED;
		result = -EIO;	
	}
	
	mtd_erase_callback(instr);

	return result;
}

///
/// No-op operation handler for the flash device.
///
/// @mtd - provides info about the MTD device in question.
static void azure_sphere_flash_nop(struct mtd_info *mtd)
{
	/* Nothing to see here */
}

///
/// NAND OOB read. We do not support any OOB so just return the data requested
///
static int azure_sphere_read_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops)
{
	return mtd_read(mtd, to, ops->len, &ops->retlen, ops->datbuf);
}

///
/// NAND OOB write. We do not support any OOB so just write the data
///
static int azure_sphere_write_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops)
{
	return mtd_write(mtd, to, ops->len, &ops->retlen, ops->datbuf);
}

///
/// Initializes this driver.
///
/// @returns - 0 on success, a negative error code otherwise.
static int __init azure_sphere_flash_init(void)
{
	register_mtd_chip_driver(&azure_sphere_flash_chipdrv);
	return 0;
}

///
/// Tears down this driver.
static void __exit azure_sphere_flash_exit(void)
{
	unregister_mtd_chip_driver(&azure_sphere_flash_chipdrv);
}

module_init(azure_sphere_flash_init);
module_exit(azure_sphere_flash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Azure Sphere Team <azuresphereoss@microsoft.com>");
MODULE_DESCRIPTION("MTD chip driver for Azure Sphere");
