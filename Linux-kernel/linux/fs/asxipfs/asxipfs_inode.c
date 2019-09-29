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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/blkdev.h>
#include <linux/nsproxy.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/vfs.h>
#include <linux/mutex.h>
#include <linux/pfn_t.h>
#include <linux/mnt_namespace.h>
#include <linux/uaccess.h>
#include <asm/tlbflush.h>
#include <asm/io.h>
#include <linux/xattr.h>
#include <linux/parser.h>
#include <azure-sphere/security_monitor.h>

#include "asxipfs.h"

/*
 * asxipfs super-block data in memory
 */
struct asxipfs_sb_info {
	unsigned long magic;
	unsigned long size;
	unsigned long blocks;
	unsigned long files;
	unsigned long flags;
	unsigned int set_owner;
	unsigned int owner;
	unsigned int set_group;
	unsigned int group;
	unsigned int mode;
	bool set_mode;
	bool set_suid_bit;
	bool set_sgid_bit; 
	bool no_security_options;
	unsigned long linear_phys_addr;
	void __iomem *linear_virt_addr;
};

static inline struct asxipfs_sb_info *ASXIPFS_SB(struct super_block *sb)
{
	return sb->s_fs_info;
}

static const struct super_operations asxipfs_ops;
static const struct inode_operations asxipfs_dir_inode_operations;
static const struct file_operations asxipfs_directory_operations;
static const struct address_space_operations asxipfs_aops;

static DEFINE_MUTEX(read_mutex);


/* These macros may change in future, to provide better st_ino semantics. */
#define OFFSET(x)	((x)->i_ino)
#define LINEAR(x)	((x)->linear_phys_addr)

#define ASXIPFS_INODE_IS_XIP(x)	((x)->i_mode & S_ISVTX)

enum {
	Opt_physaddr, Opt_owner, Opt_group, Opt_suid, Opt_sgid, Opt_nosec, Opt_mode, Opt_err, Opt_size,
};

static const match_table_t tokens = {
	{Opt_physaddr, "physaddr=%s"},
	{Opt_size, "size=%s"},
	{Opt_owner, "owner=%u"},
	{Opt_group, "group=%u"},
	{Opt_suid, "suid=%u"},
	{Opt_sgid, "sgid=%u"},
	{Opt_nosec, "nosec"},
	{Opt_mode, "mode=%o"},
	{Opt_err, NULL},
};

static int asxipfs_parse_options(char *options, struct asxipfs_sb_info *sbi)
{
	char *p;
	substring_t args[MAX_OPT_ARGS];
	int option;

	if (!options)
		return 1;

	while ((p = strsep(&options, ",")) != NULL) {
		int token;
		if (!*p)
			continue;

		args[0].to = args[0].from = NULL;
		token = match_token(p, tokens, args);
		switch (token) {
		case Opt_physaddr:
			if (kstrtou32(args[0].from, 0, &option)) {
				printk(KERN_ERR "asxipfs: invalid physical address parameter %s\n", args[0].from);
				return 0;
			}
			sbi->linear_phys_addr = option;
			break;
		case Opt_size:
			if (kstrtou32(args[0].from, 0, &option)) {
				printk(KERN_ERR "asxipfs: invalid size parameter %s\n", args[0].from);
				return 0;
			}
			sbi->size = option;
			break;
		case Opt_owner:
			if (match_int(&args[0], &option)){
				printk(KERN_ERR "asxipfs: invalid owner parameter\n");
				return 0;
			}
			sbi->set_owner = 1;
			sbi->owner = option;
			break;
		case Opt_group:
			if (match_int(&args[0], &option)){
				printk(KERN_ERR "asxipfs: invalid group parameter\n");
				return 0;
			}
			sbi->set_group = 1;
			sbi->group = option;
			break;
		case Opt_suid:
			if (match_int(&args[0], &option)) {
				printk(KERN_ERR "asxipfs: invalid suid parameter\n");
				return 0;
			}
			if (option < 0 || option > 1) {
				printk(KERN_ERR "asxipfs: suid parameter must be 0 or 1\n");
				return 0;
			}
			sbi->set_suid_bit = true;
			break;
		case Opt_sgid:
			if (match_int(&args[0], &option)) {
				printk(KERN_ERR "asxipfs: invalid sgid parameter\n");
				return 0;
			}
			if (option < 0 || option > 1) {
				printk(KERN_ERR "asxipfs: sgid parameter must be 0 or 1\n");
				return 0;
			}
			sbi->set_sgid_bit = true;
			break;
		case Opt_nosec:
			sbi->no_security_options = true;
			break;
		case Opt_mode:
			if(match_octal(&args[0], &option)) {
				printk(KERN_ERR "asxipfs: invalid mode parameter\n");
				return 0;
			}
			sbi->set_mode = true;
			sbi->mode = option;
			break;
		default:
			printk(KERN_ERR "asxipfs: unrecognized mount option \"%s\" or missing value\n", p);
			return 0;
		}
	}

	return 1;
}

static int asxipfs_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct file *file = vma->vm_file;
	struct address_space *mapping = file->f_mapping;
	struct inode *inode = mapping->host;
	struct super_block *sb = inode->i_sb;
	struct asxipfs_sb_info *sbi = ASXIPFS_SB(sb);
	struct page *page;
	unsigned long vaddr = (unsigned long)vmf->virtual_address;
	unsigned long address = 0;
	pgoff_t size;
	int error;
	unsigned long pfn;

	// See if it's too big
	size = (i_size_read(inode) + PAGE_SIZE - 1) >> PAGE_SHIFT;
	if (vmf->pgoff >= size)
		return VM_FAULT_SIGBUS;

 repeat:
	page = find_get_page(mapping, vmf->pgoff);
	if (page) {
		if (!lock_page_or_retry(page, vma->vm_mm, vmf->flags)) {
			put_page(page);
			return VM_FAULT_RETRY;
		}
		if (unlikely(page->mapping != mapping)) {
			unlock_page(page);
			put_page(page);
			goto repeat;
		}
	}

	address  = PAGE_ALIGN(sbi->linear_phys_addr + OFFSET(inode));
	address += vmf->pgoff << PAGE_SHIFT;
	pfn = __phys_to_pfn(address);

	if (vmf->cow_page) {
		// Copy on write - move data from XIP to new page
		struct page *new_page = vmf->cow_page;
		u32 blkptr_offset = PAGE_ALIGN(OFFSET(inode)) +
				(vmf->pgoff << PAGE_SHIFT);

		void *data = (void *)(sbi->linear_virt_addr + blkptr_offset);
		void *vto;

		// Copy data to new page
		vto = kmap_atomic(new_page);
		memcpy(vto, data, PAGE_SIZE);
		kunmap_atomic(vto);

		vmf->page = page;
		return VM_FAULT_LOCKED;
	}

	if (page) {
		// Clean up old page if needed
		unmap_mapping_range(mapping, vmf->pgoff << PAGE_SHIFT,
							PAGE_SIZE, 0);
		delete_from_page_cache(page);
		unlock_page(page);
		put_page(page);
	}

	// Install page mapped directly to physical memory
	i_mmap_lock_read(mapping);
	error = vm_insert_mixed(vma, vaddr, pfn_to_pfn_t(pfn));
	i_mmap_unlock_read(mapping);

	if (error == -ENOMEM)
		return VM_FAULT_OOM;
	/* -EBUSY is fine, somebody else faulted on the same PTE */
	if ((error < 0) && (error != -EBUSY))
		return VM_FAULT_SIGBUS;
	return VM_FAULT_NOPAGE;
}

int asxipfs_access(struct vm_area_struct *vma, unsigned long addr,
			      void *buf, int len, int write)
{
	struct inode *inode;
	u32 blkptr_offset;
	void *data;
	struct super_block *sb;
	struct asxipfs_sb_info *sbi;
	int offset = addr - vma->vm_start;

	// We only work on file mappings
	if (vma->vm_file == NULL)
		return -EINVAL;

	// You can't read beyond the mapping
	if (addr + len > vma->vm_end)
		return -EINVAL;

	inode = vma->vm_file->f_inode;
	sb = inode->i_sb;
	sbi = ASXIPFS_SB(sb);

	// We compute the location in the file by computing the inode offset +
	// the offset in pages of the vma instance.  This tells us where the page starts
	// in mapped space
	blkptr_offset = PAGE_ALIGN(OFFSET(inode)) + (vma->vm_pgoff << PAGE_SHIFT);
	data = (void *)(sbi->linear_virt_addr + blkptr_offset);

	if (write) {
		// Write is not supported (a CoW will happen)
		return -EINVAL;
	}

	if (((vma->vm_pgoff << PAGE_SHIFT) + offset + len) > PAGE_ALIGN(inode->i_size)) {
		// Over sized mapping, read back zeroes
		memset(buf, 0, len);
	} else {
		memcpy_fromio(buf, data + offset, len);
	}

	return len;
}

static const struct vm_operations_struct asxipfs_vm_ops = {
	.access		= asxipfs_access,
	.fault		= asxipfs_fault
};

static int asxipfs_mmap(struct file *file, struct vm_area_struct *vma)
{
	if ((vma->vm_flags & VM_SHARED) && (vma->vm_flags & VM_MAYWRITE)) {
		return -EINVAL;
	}

	/*
	 * Don't dump addresses that are not real memory to a core file.
	 */
	vma->vm_ops = &asxipfs_vm_ops;
	vma->vm_flags |= VM_MIXEDMAP | VM_DONTDUMP;

#ifdef DEBUG_asxipfs_XIP
	{
		unsigned long address, length;
		struct inode *inode = file->f_inode;
		struct super_block *sb = inode->i_sb;
		struct asxipfs_sb_info *sbi = ASXIPFS_SB(sb);

		address  = PAGE_ALIGN(sbi->linear_phys_addr + OFFSET(inode));
		address += vma->vm_pgoff << PAGE_SHIFT;

		length = vma->vm_end - vma->vm_start;
		length = PAGE_ALIGN(length);

		printk("asxipfs_mmap: mapped %s at 0x%08lx, flags 0x%08lx, length %lu to vma 0x%08lx"
			", page_prot 0x%08x\n",
			file->f_path.dentry->d_name.name, address, vma->vm_flags, length,
			vma->vm_start, (unsigned int)pgprot_val(vma->vm_page_prot));
	}
#endif

	return 0;
}

static struct file_operations asxipfs_linear_xip_fops = {
	.llseek		= generic_file_llseek,
	.read_iter		= generic_file_read_iter,
	.mmap		= asxipfs_mmap,
};

static unsigned long asxipfsino(const struct asxipfs_inode *cino, unsigned int offset)
{
	if (!cino->offset)
		return offset + 1;
	if (!cino->size)
		return offset + 1;

	return cino->offset << 2;
}

static struct inode *get_asxipfs_inode(struct super_block *sb,
	const struct asxipfs_inode *asxipfs_inode, unsigned int offset)
{
	struct inode *inode;
	static struct timespec zerotime;
	struct asxipfs_sb_info *sbi;

	inode = iget_locked(sb, asxipfsino(asxipfs_inode, offset));

	sbi = ASXIPFS_SB(sb);

	if (!inode)
		return ERR_PTR(-ENOMEM);
	if (!(inode->i_state & I_NEW))
		return inode;

	switch (asxipfs_inode->mode & S_IFMT) {
	case S_IFREG:
		inode->i_fop = &generic_ro_fops;
		// err out 
		if(!(LINEAR(sbi) && (asxipfs_inode->mode & S_ISVTX))) {
			printk(KERN_ERR "Non-linear or non-XIP asxipfs is not allowed.\n");
			return ERR_PTR(-EINVAL);
		}
		inode->i_fop = &asxipfs_linear_xip_fops;
		inode->i_data.a_ops = &asxipfs_aops;
		break;
	case S_IFDIR:
		inode->i_op = &asxipfs_dir_inode_operations;
		inode->i_fop = &asxipfs_directory_operations;
		break;
	case S_IFLNK:
		inode->i_op = &page_symlink_inode_operations;
		inode_nohighmem(inode);
		inode->i_data.a_ops = &asxipfs_aops;
		break;
	default:
		init_special_inode(inode, asxipfs_inode->mode,
				old_decode_dev(asxipfs_inode->size));
	}

	inode->i_mode = asxipfs_inode->mode | (sbi->set_suid_bit ? S_ISUID : 0) | (sbi->set_sgid_bit ? S_ISGID : 0);

	if (sbi->no_security_options) {
		// Strip off SGID / SUID
		inode->i_mode = inode->i_mode & ~(S_ISUID | S_ISGID);
	}
	if(sbi->set_mode) {
		/* first reset the file permission bits to zero, then set to requested level */
		inode->i_mode = inode->i_mode & ~(S_IRWXU | S_IRWXG | S_IRWXO);
		inode->i_mode |= sbi->mode;
	}

	i_uid_write(inode, sbi->set_owner ? sbi->owner : asxipfs_inode->uid);
	i_gid_write(inode, sbi->set_group ? sbi->group : asxipfs_inode->gid);


	/* if the lower 2 bits are zero, the inode contains data */
	if (!(inode->i_ino & 3)) {
		inode->i_size = asxipfs_inode->size;
		inode->i_blocks = (asxipfs_inode->size - 1) / 512 + 1;
	}

	/* Struct copy intentional */
	inode->i_mtime = inode->i_atime = inode->i_ctime = zerotime;
	/* inode->i_nlink is left 1 - arguably wrong for directories,
	   but it's the best we can do without reading the directory
	   contents.  1 yields the right result in GNU find, even
	   without -noleaf option. */

	unlock_new_inode(inode);

	return inode;
}

static void *asxipfs_read(struct super_block *sb, unsigned int offset, unsigned int len)
{
	struct asxipfs_sb_info *sbi = ASXIPFS_SB(sb);

	if (!len)
		return NULL;

	if (offset + len < offset || offset + len > sbi->size)
		return NULL;

	return (void *)(sbi->linear_virt_addr + offset);
}

static void asxipfs_kill_sb(struct super_block *sb)
{
	struct asxipfs_sb_info *sbi = ASXIPFS_SB(sb);
	if (sbi->linear_virt_addr)
		iounmap(sbi->linear_virt_addr);

	//When using XIP, there is no block device.  
	//Therefore, don't call kill_block_super.  Just call
	//generic_shutdown_super, which is noramally called by
	//kill block super.
	generic_shutdown_super(sb);
	kfree(sbi);
}

static int asxipfs_remount(struct super_block *sb, int *flags, char *data)
{
	sync_filesystem(sb);
	*flags |= MS_RDONLY;
	return 0;
}

static int asxipfs_fill_super(struct super_block *sb, void *data, int silent)
{
	struct asxipfs_super super;
	unsigned long root_offset;
	struct asxipfs_sb_info *sbi = NULL;
	struct inode *root;

	sb->s_flags |= MS_RDONLY;

	sbi = kzalloc(sizeof(struct asxipfs_sb_info), GFP_KERNEL);
	if (!sbi)
		return -ENOMEM;

	sb->s_fs_info = sbi;

	/*
	 * The physical location of the asxipfs image is specified as
	 * a mount parameter.  This parameter is mandatory for obvious
	 * reasons.  Some validation is made on the phys address but this
	 * is not exhaustive and we count on the fact that someone using
	 * this feature is supposed to know what he/she's doing.
	 * Also, the size must be passed in.
	 */

	if (asxipfs_parse_options(data, sbi) == 0) {
		printk(KERN_ERR "asxipfs: failed to read parameters\n");
		return -EINVAL;
	}

	if (sbi->size == 0) {
		printk(KERN_ERR "asxipfs: failed to pass in size param\n");
		return -EINVAL;
	}

	if (sbi->linear_phys_addr == 0) {
		printk(KERN_ERR "asxipfs: failed to pass in physaddr param\n");
		return -EINVAL;
	}

	if (sbi->linear_phys_addr & (PAGE_SIZE-1)) {
		printk(KERN_ERR "asxipfs: physical address 0x%lx"
			" isn't aligned to a page boundary\n",
			sbi->linear_phys_addr);
		return -EINVAL;
	}
	if (sbi->linear_phys_addr == 0) {
		printk(KERN_ERR "asxipfs: physical address for"
			" image can't be 0\n");
		return -EINVAL;
	}

	if (azure_sphere_sm_verify_image_by_flash_address(sbi->linear_phys_addr) != 0) {
		printk(KERN_ERR "asxipfs: image verification failed\n");
		return -EINVAL;
	}

	/* Map only one page for now.  Will remap it when fs size is known.*/
	sbi->linear_virt_addr =
		ioremap(sbi->linear_phys_addr, PAGE_SIZE);

	if (!sbi->linear_virt_addr) {
		printk(KERN_ERR "asxipfs: ioremap of the image failed\n");
		return -EINVAL;
	}


	/* Invalidate the read buffers on mount: think disk change.. */
	mutex_lock(&read_mutex);

	/* Read the first block and get the superblock from it */
	data = asxipfs_read(sb, 0, sizeof(super));
	if (!data) {
		mutex_unlock(&read_mutex);
		return -EINVAL;
	}

	memcpy(&super, data, sizeof(super));
	mutex_unlock(&read_mutex);

	/* Do sanity checks on the superblock */
	if (super.magic != ASXIPFS_MAGIC) {
		pr_err("wrong magic\n");
		return -EINVAL;
	}

	/* get feature flags first */
	if (super.flags & ~ASXIPFS_SUPPORTED_FLAGS) {
		pr_err("unsupported filesystem features\n");
		return -EINVAL;
	}

	/* Check that the root inode is in a sane state */
	if (!S_ISDIR(super.root.mode)) {
		pr_err("root is not a directory\n");
		return -EINVAL;
	}

	super.root.mode |= (S_IRUSR | S_IXUSR | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

	root_offset = super.root.offset << 2;
	if (super.flags & ASXIPFS_FLAG_FSID_VERSION_2) {
		if (super.size > sbi->size) {
			pr_err("filesystem size exceeds mapping (%u > %lu)", super.size, sbi->size);
			return -EINVAL;
		} else {
			sbi->size = super.size;
		}
		sbi->blocks = super.fsid.blocks;
		sbi->files = super.fsid.files;
	} else {
		// sbi->size is filled from mount parameters
		sbi->blocks = 0;
		sbi->files = 0;
	}
	sbi->magic = super.magic;
	sbi->flags = super.flags;
	if (root_offset == 0)
		pr_info("empty filesystem");
	else if (!(super.flags & ASXIPFS_FLAG_SHIFTED_ROOT_OFFSET) &&
		 ((root_offset != sizeof(struct asxipfs_super)) &&
		  (root_offset != 512 + sizeof(struct asxipfs_super))))
	{
		pr_err("bad root offset %lu\n", root_offset);
		return -EINVAL;
	}

	/* Set it all up.. */
	sb->s_op = &asxipfs_ops;
	root = get_asxipfs_inode(sb, &super.root, 0);
	if (IS_ERR(root))
		return PTR_ERR(root);
	sb->s_root = d_make_root(root);
	if (!sb->s_root)
		return -ENOMEM;

	if(!LINEAR(sbi)) {
		// not good
		return -EINVAL;
	}

	/* Remap the whole filesystem now */
	iounmap(sbi->linear_virt_addr);

	sbi->linear_virt_addr =
		ioremap(sbi->linear_phys_addr, sbi->size);
	if (!sbi->linear_virt_addr) {
		printk(KERN_ERR "asxipfs: ioremap of the image failed\n");
		return -EINVAL;
	}

	return 0;
}

static int asxipfs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	struct super_block *sb = dentry->d_sb;
	struct asxipfs_sb_info *sbi = ASXIPFS_SB(sb);
	u64 id = 0;

	// we will always be using xip
	if (!LINEAR(sbi)) {
		printk(KERN_ERR "asxipfs: statfs found non-linear sb\n");
		return -EINVAL;
	}

	buf->f_type = ASXIPFS_MAGIC;
	buf->f_bsize = PAGE_SIZE;
	buf->f_blocks = ASXIPFS_SB(sb)->blocks;
	buf->f_bfree = 0;
	buf->f_bavail = 0;
	buf->f_files = ASXIPFS_SB(sb)->files;
	buf->f_ffree = 0;
	buf->f_fsid.val[0] = (u32)id;
	buf->f_fsid.val[1] = (u32)(id >> 32);
	buf->f_namelen = ASXIPFS_MAXPATHLEN;
	return 0;
}

/*
 * Read a asxipfs directory entry.
 */
static int asxipfs_readdir(struct file *file, struct dir_context *ctx)
{
	struct inode *inode = file_inode(file);
	struct super_block *sb = inode->i_sb;
	char *buf;
	unsigned int offset;

	/* Offset within the thing. */
	if (ctx->pos >= inode->i_size)
		return 0;
	offset = ctx->pos;
	/* Directory entries are always 4-byte aligned */
	if (offset & 3)
		return -EINVAL;

	buf = kmalloc(ASXIPFS_MAXPATHLEN, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	while (offset < inode->i_size) {
		struct asxipfs_inode *de;
		unsigned long nextoffset;
		char *name;
		ino_t ino;
		umode_t mode;
		int namelen;

		mutex_lock(&read_mutex);
		de = asxipfs_read(sb, OFFSET(inode) + offset, sizeof(*de)+ASXIPFS_MAXPATHLEN);
		if (!de) {
			mutex_unlock(&read_mutex);
			break;
		}
		name = (char *)(de+1);

		/*
		 * Namelengths on disk are shifted by two
		 * and the name padded out to 4-byte boundaries
		 * with zeroes.
		 */
		namelen = de->namelen << 2;
		memcpy(buf, name, namelen);
		ino = asxipfsino(de, OFFSET(inode) + offset);
		mode = de->mode;
		mutex_unlock(&read_mutex);
		nextoffset = offset + sizeof(*de) + namelen;
		for (;;) {
			if (!namelen) {
				kfree(buf);
				return -EIO;
			}
			if (buf[namelen-1])
				break;
			namelen--;
		}
		if (!dir_emit(ctx, buf, namelen, ino, mode >> 12))
			break;

		ctx->pos = offset = nextoffset;
	}
	kfree(buf);
	return 0;
}

/*
 * Lookup and fill in the inode data..
 */
static struct dentry *asxipfs_lookup(struct inode *dir, struct dentry *dentry, unsigned int flags)
{
	unsigned int offset = 0;
	struct inode *inode = NULL;
	int sorted;

	mutex_lock(&read_mutex);
	sorted = ASXIPFS_SB(dir->i_sb)->flags & ASXIPFS_FLAG_SORTED_DIRS;
	while (offset < dir->i_size) {
		struct asxipfs_inode *de;
		char *name;
		int namelen, retval;
		int dir_off = OFFSET(dir) + offset;

		de = asxipfs_read(dir->i_sb, dir_off, sizeof(*de)+ASXIPFS_MAXPATHLEN);
		if (!de) {
			inode = ERR_PTR(-EIO);
			break;
		}
		name = (char *)(de+1);

		/* Try to take advantage of sorted directories */
		if (sorted && (dentry->d_name.name[0] < name[0]))
			break;

		namelen = de->namelen << 2;
		offset += sizeof(*de) + namelen;

		/* Quick check that the name is roughly the right length */
		if (((dentry->d_name.len + 3) & ~3) != namelen)
			continue;

		for (;;) {
			if (!namelen) {
				inode = ERR_PTR(-EIO);
				goto out;
			}
			if (name[namelen-1])
				break;
			namelen--;
		}
		if (namelen != dentry->d_name.len)
			continue;
		retval = memcmp(dentry->d_name.name, name, namelen);
		if (retval > 0)
			continue;
		if (!retval) {
			inode = get_asxipfs_inode(dir->i_sb, de, dir_off);
			break;
		}
		/* else (retval < 0) */
		if (sorted)
			break;
	}
out:
	mutex_unlock(&read_mutex);
	if (IS_ERR(inode))
		return ERR_CAST(inode);
	d_add(dentry, inode);
	return NULL;
}

static int asxipfs_readpage(struct file *file, struct page *page)
{
	struct inode *inode = page->mapping->host;
	u32 maxblock;
	int bytes_filled;
	void *pgdata;
	struct super_block *sb;

	maxblock = (inode->i_size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	bytes_filled = 0;
	pgdata = kmap(page);

	sb = inode->i_sb;

	/* asxipfs doesn't handle compressed files */
	if(!ASXIPFS_INODE_IS_XIP(inode)) {
		pr_err("Found compressed block\n");
		goto err;
	}

	if (page->index < maxblock) {
		u32 blkptr_offset = PAGE_ALIGN(OFFSET(inode)) +
				page->index * PAGE_SIZE;
		void *data;

		mutex_lock(&read_mutex);
		data = asxipfs_read(sb, blkptr_offset, PAGE_SIZE);
		if (data != NULL)
			memcpy(page_address(page),
				data,
				PAGE_SIZE);
		mutex_unlock(&read_mutex);
		if (data == NULL)
			goto err;
		bytes_filled = PAGE_SIZE;
	}

	memset(pgdata + bytes_filled, 0, PAGE_SIZE - bytes_filled);
	flush_dcache_page(page);
	kunmap(page);
	SetPageUptodate(page);
	unlock_page(page);
	return 0;

err:
	kunmap(page);
	ClearPageUptodate(page);
	SetPageError(page);
	unlock_page(page);
	return 0;
}

static sector_t asxipfs_bmap(struct address_space *mapping, sector_t block)
{
	struct inode *inode = mapping->host;
	u32 maxblock;
	struct super_block *sb;
	unsigned long addr;
	u32 blkptr_offset;
	struct asxipfs_sb_info *sbi;

	/* asxipfs doesn't handle compressed files */
	if(!ASXIPFS_INODE_IS_XIP(inode)) {
		pr_err("Found compressed block\n");
		return -EINVAL;
	}

	maxblock = (inode->i_size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	if (block > maxblock) {
		return -EINVAL;
	}

	sb = inode->i_sb;
	sbi = ASXIPFS_SB(sb);
	blkptr_offset = PAGE_ALIGN(OFFSET(inode)) +
			block * PAGE_SIZE;
	addr = sbi->linear_phys_addr + blkptr_offset;

	return addr;
}

static const struct address_space_operations asxipfs_aops = {
	.readpage = asxipfs_readpage,
	.bmap = asxipfs_bmap,
};

/*
 * Our operations:
 */

/*
 * A directory can only readdir
 */
static const struct file_operations asxipfs_directory_operations = {
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.iterate_shared	= asxipfs_readdir,
};

static const struct inode_operations asxipfs_dir_inode_operations = {
	.lookup		= asxipfs_lookup,
};

static const struct super_operations asxipfs_ops = {
	.remount_fs	= asxipfs_remount,
	.statfs		= asxipfs_statfs,
};

static struct dentry *asxipfs_mount(struct file_system_type *fs_type,
	int flags, const char *dev_name, void *data)
{
	return mount_nodev(fs_type, flags, data, asxipfs_fill_super);
}

static struct file_system_type asxipfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= "asxipfs",
	.mount 		= asxipfs_mount,
	.kill_sb	= asxipfs_kill_sb,
};

MODULE_ALIAS_FS("asxipfs");

static int __init init_asxipfs_fs(void)
{
	int rv;
	rv = register_filesystem(&asxipfs_fs_type);
	return rv;
}

static void __exit exit_asxipfs_fs(void)
{
	unregister_filesystem(&asxipfs_fs_type);
}

module_init(init_asxipfs_fs)
module_exit(exit_asxipfs_fs)
MODULE_LICENSE("GPL");
