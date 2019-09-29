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
#include <linux/pagemap.h>

#include "littlefs.h"

static int littlefs_get_dir_attrs(const char *path, struct littlefs_sb_info *c, struct littlefs_inode_info *f)
{
	if (!strcmp("/", path)) {
		return lfs_fs_getattrs(&c->lfs, &f->attrs[0], ARRAY_SIZE(f->attrs));
	} else {
		return lfs_getattrs(&c->lfs, path, &f->attrs[0], ARRAY_SIZE(f->attrs));
	}
}

static int littlefs_set_dir_attrs(const char *path, struct littlefs_sb_info *c, struct littlefs_inode_info *f)
{
	if (!strcmp("/", path)) {
		return lfs_fs_setattrs(&c->lfs, &f->attrs[0], ARRAY_SIZE(f->attrs));
	} else {
		return lfs_setattrs(&c->lfs, path, &f->attrs[0], ARRAY_SIZE(f->attrs));
	}
}

void littlefs_dirty_inode(struct inode *inode, int flags)
{
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	struct littlefs_inode_info *f = LITTLEFS_INODE_INFO(inode);
	struct dentry *d;
	char buf[LFS_NAME_MAX], *path;
	umode_t mode;
	kuid_t uid;
	kgid_t gid;
	bool dirty = false;

	if (!f) {
		return;
	}

	if (!(inode->i_state & I_DIRTY_DATASYNC)) {
		// Not done yet
		return;
	}

	if (!(inode->i_mode & S_IFDIR)) {
		// Only need to proactively set dir mode bits
		return;
	}

	d = d_find_alias(inode);

	if (!d) {
		return;
	}

	path = dentry_path_raw(d, buf, sizeof(buf));

	dput(d);

	if (IS_ERR(path)) {
		// path too long, should never happen at this point
		return;
	}

	mode = inode->i_mode;
	uid = inode->i_uid;
	gid = inode->i_gid;

	mutex_lock(&c->sem);

	// See if we need to set anything
	littlefs_get_dir_attrs(path, c, f);
	
	if (mode != inode->i_mode) {
		inode->i_mode = mode;
		dirty = true;
	}
	if (!uid_eq(uid, inode->i_uid)) {
		inode->i_uid = uid;
		dirty = true;
	}
	if (!gid_eq(gid, inode->i_gid)) {
		inode->i_gid = gid;
		dirty = true;
	}

	if (!dirty) {
		goto exit;
	}

	littlefs_set_dir_attrs(path, c, f);

exit:
	mutex_unlock(&c->sem);
}

void littlefs_evict_inode(struct inode *inode)
{
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	struct littlefs_inode_info *p = inode->i_private;
	umode_t mode = inode->i_mode;

	truncate_inode_pages_final(&inode->i_data);
	clear_inode(inode);
	
	if (p) {
		mutex_lock(&c->sem);
		if (mode & S_IFDIR) {
			lfs_dir_close(&c->lfs, &p->lfs.dir);
		} else if (mode & S_IFREG) {
			lfs_file_close(&c->lfs, &p->lfs.file);
		}
		mutex_unlock(&c->sem);

		kfree(p);

		inode->i_private = NULL;
	}
}


static int littlefs_rename(struct inode *old_dir_i, struct dentry *old_dentry,
			 struct inode *new_dir_i, struct dentry *new_dentry,
			 unsigned int flags)
{
	int ret;
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(old_dir_i->i_sb);
	char buf[LFS_NAME_MAX], *path;
	char victim_buf[LFS_NAME_MAX], *victim_path;
	struct littlefs_inode_info *victim_f = NULL;

	if (flags & ~RENAME_NOREPLACE)
		return -EINVAL;

	path = dentry_path_raw(old_dentry, buf, sizeof(buf));
	victim_path = dentry_path_raw(new_dentry, victim_buf, sizeof(victim_buf));

	if (IS_ERR(path)) {
		return PTR_ERR(path);
	}

	if (IS_ERR(victim_path)) {
		return PTR_ERR(victim_path);
	}

	mutex_lock(&c->sem);

	if (d_really_is_positive(new_dentry)) {
		victim_f = LITTLEFS_INODE_INFO(d_inode(new_dentry));

		if (!victim_f) {
			ret = -ENOENT;
			goto unlock;
		}

		if (d_is_dir(new_dentry)) {
			if (d_inode(new_dentry)->i_nlink > 2) {
				ret = -ENOTEMPTY;
				victim_f = NULL;
				goto unlock;
			}

			// Close the old dir
			lfs_dir_close(&c->lfs, &victim_f->lfs.dir);
			littlefs_quota_charge_dir(c, d_inode(new_dentry)->i_uid, -1);
		} else {
			// Close the old file
			lfs_file_close(&c->lfs, &victim_f->lfs.file);
			littlefs_quota_charge_file(c, d_inode(new_dentry)->i_uid, d_inode(new_dentry)->i_size, 0);
		}

		d_inode(new_dentry)->i_private = NULL;
	}

	ret = lfs_remove(&c->lfs, victim_path);

	if (ret) {
		goto unlock;
	}

	// We remove before rename due to some issues with renaming inline files
	ret = lfs_rename(&c->lfs, path, victim_path);
	
	if (ret) {
		goto unlock;
	}

	if (d_is_dir(old_dentry) && !victim_f)
		inc_nlink(new_dir_i);

	if (d_is_dir(old_dentry))
		drop_nlink(old_dir_i);

unlock:
	mutex_unlock(&c->sem);

	if (victim_f) {
		kfree(victim_f);
	}

	return ret;
}

static int littlefs_unlink(struct inode *dir_i, struct dentry *dentry)
{
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(dir_i->i_sb);
	struct inode *inode = d_inode(dentry);
	struct littlefs_inode_info *f = LITTLEFS_INODE_INFO(inode);
	char buf[LFS_NAME_MAX], *path;
	int ret;

	path = dentry_path_raw(dentry, buf, sizeof(buf));

	if (IS_ERR(path)) {
		return PTR_ERR(path);
	}

	mutex_lock(&c->sem);

	if (!f) {
		ret = -ENOENT;
		goto out;
	}

	if (inode->i_mode & S_IFDIR) {
		lfs_dir_close(&c->lfs, &f->lfs.dir);
		drop_nlink(dir_i);
		drop_nlink(inode); // future call will mark as dirty

		ret = littlefs_quota_charge_dir(c, inode->i_uid, -1);
		if (ret) {
			goto out;
		}
	} else if (inode->i_mode & S_IFREG) {
		lfs_file_close(&c->lfs, &f->lfs.file);

		ret = littlefs_quota_charge_file(c, inode->i_uid, inode->i_size, 0);
		if (ret) {
			goto out;
		}
	}

	ret = lfs_remove(&c->lfs, path);

	// Free private data
	kfree(inode->i_private);
	inode->i_private = NULL;	
	
	inode_dec_link_count(inode); // decrement inode count, mark as dirty
out:
	mutex_unlock(&c->sem);

	return ret;
}

static int littlefs_create(struct inode *dir_i, struct dentry *dentry,
			umode_t mode, bool excl)
{
	struct littlefs_inode_info *dir_f;
	struct littlefs_sb_info *c;
	struct inode *inode;
	char buf[LFS_NAME_MAX], *path;

	c = LITTLEFS_SB_INFO(dir_i->i_sb);
	dir_f = LITTLEFS_INODE_INFO(dir_i);

	if (!dir_f) {
		return -ENOENT;
	}

	path = dentry_path(dentry, buf, sizeof(buf));
	inode = littlefs_get_inode(dir_i->i_sb, dir_i, path, mode);

	if (IS_ERR(inode)) {
		return PTR_ERR(inode);
	}

	d_instantiate(dentry, inode);
	return 0;
}

static int littlefs_readdir(struct file *file, struct dir_context *ctx)
{
	struct inode *inode = file_inode(file);
	struct littlefs_inode_info *f = LITTLEFS_INODE_INFO(inode);
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	struct dentry *d;
	struct lfs_info info;
	char buf[LFS_NAME_MAX], *path;
	unsigned long curofs = 0;
	int ret = 0;
	unsigned int type = 0;

	if (ctx->pos == INT_MAX)
		return 0;

	if (!dir_emit_dots(file, ctx))
		return 0;

	mutex_lock(&c->sem);


	if (!f)  {
		ret = -ENOENT;
		goto unlock;
	}

	d = d_find_alias(inode);

	if (!d) {
		ret = -ENOENT;
		goto unlock;
	}

	path = dentry_path_raw(d, buf, sizeof(buf));

	dput(d);

	if (IS_ERR(path)) {
		ret = PTR_ERR(path);
		goto unlock;
	}

	// We need to do this or th results can get out of sync.  
	// Looks like a caching bug in littlefs we can investigate as in all
	// cases the right data makes it to flash.
	lfs_dir_close(&c->lfs, &f->lfs.dir);
	lfs_dir_open(&c->lfs, &f->lfs.dir, path);
	lfs_dir_rewind(&c->lfs, &f->lfs.dir);

	while ((ret = lfs_dir_read(&c->lfs, &f->lfs.dir, &info)) > 0) {
		// Resume
		if (curofs++ < ctx->pos) {
			continue;
		}
		
		switch (info.type) {
			case LFS_TYPE_DIR:
				type = DT_DIR;
				break;
			case LFS_TYPE_REG:
				type = DT_REG;
				break;
		}

		if (!dir_emit(ctx, info.name, strlen(info.name), 0, type))
			break;
		ctx->pos++;
	}

	ctx->pos = INT_MAX;

unlock:
	mutex_unlock(&c->sem);

	return ret;
}

int littlefs_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
{
	struct inode *inode = filp->f_mapping->host;
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	struct littlefs_inode_info *f = LITTLEFS_INODE_INFO(inode);
	int ret;

	if (!f) {
		return -ENOENT;
	}

	ret = filemap_write_and_wait_range(inode->i_mapping, start, end);
	if (ret)
		return ret;

	if (inode->i_mode & S_IFREG) {
		mutex_lock(&c->sem);

		ret = lfs_file_sync(&c->lfs, &f->lfs.file);

		mutex_unlock(&c->sem);
	}

	return 0;
}

static struct dentry *littlefs_lookup(struct inode *dir_i, struct dentry *target,
				   unsigned int flags)
{
	struct littlefs_sb_info *c;
	struct lfs_info info;
	struct inode *inode = NULL;
	char buf[LFS_NAME_MAX], *path;
	int err;

	if (target->d_name.len > LFS_NAME_MAX)
		return ERR_PTR(-ENAMETOOLONG);

	c = LITTLEFS_SB_INFO(dir_i->i_sb);

	path = dentry_path_raw(target, buf, sizeof(buf));

	if (IS_ERR(path)) {
		return ERR_PTR(PTR_ERR(path));
	}

	mutex_lock(&c->sem);
	err = lfs_stat(&c->lfs, path, &info);
	mutex_unlock(&c->sem);

	if (err == LFS_ERR_NOENT) {
		// Does not exist, fall through to return an empty inode
	} else if (err) {
		pr_err("LittleFs lookup err: %d\n", err);
	} else {
		inode = littlefs_get_inode(dir_i->i_sb, dir_i, path, info.type == LFS_TYPE_REG ? S_IFREG : S_IFDIR);
	}

	return d_splice_alias(inode, target);
}

static int littlefs_mkdir(struct inode *dir_i, struct dentry *dentry, umode_t mode)
{
	struct littlefs_inode_info *f;
	struct littlefs_sb_info *c;
	struct inode *inode;
	int ret;
	char buf[LFS_NAME_MAX], *path;
	
	if (dentry->d_name.len > LFS_NAME_MAX)
		return -ENAMETOOLONG;

	path = dentry_path_raw(dentry, buf, sizeof(buf));

	if (IS_ERR(path)) {
		return PTR_ERR(path);
	}

	mode |= S_IFDIR;

	c = LITTLEFS_SB_INFO(dir_i->i_sb);
	f = LITTLEFS_INODE_INFO(dir_i);

	if (!f) {
		return -ENOENT;
	}

	mutex_lock(&c->sem);

	ret = littlefs_quota_charge_dir(c, current_fsuid(), 1);
	if (ret) {
		mutex_unlock(&c->sem);
		goto out;
	}

	ret = lfs_mkdir(&c->lfs, path);
	mutex_unlock(&c->sem);

	if (ret) {
		goto out;
	}

	inode = littlefs_get_inode(dir_i->i_sb, dir_i, path, mode);

	if (IS_ERR(inode)) {
		ret = PTR_ERR(inode);
		goto out;
	}

	dir_i->i_mtime = dir_i->i_ctime = ITIME(get_seconds());
	inc_nlink(dir_i);

	d_instantiate(dentry, inode);

 out:
	return ret;
}

static int littlefs_rmdir (struct inode *dir_i, struct dentry *dentry)
{
	return littlefs_unlink(dir_i, dentry);
}

static int littlefs_dir_setattr(struct dentry *dentry, struct iattr *iattr) 
{
	struct inode *inode = d_inode(dentry);
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	int error = 0;

	error = setattr_prepare(dentry, iattr);
	if (error)
		return error;

	if (iattr->ia_valid & ATTR_UID) {
		mutex_lock(&c->sem);
		
		// Charge new owner
		error = littlefs_quota_charge_dir(c, iattr->ia_uid, 1);
		if (error) {
			mutex_unlock(&c->sem);
			return error;
		}

		// Credit old owner
		littlefs_quota_charge_dir(c, inode->i_uid, -1);
		
		mutex_unlock(&c->sem);
	}

	setattr_copy(inode, iattr);
	mark_inode_dirty(inode);
	return 0;
}

static int littlefs_file_setattr(struct dentry *dentry, struct iattr *iattr) 
{
	struct inode *inode = d_inode(dentry);
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	struct littlefs_inode_info *f = LITTLEFS_INODE_INFO(inode);
	int error = 0;

	if (!f) {
		return -ENOENT;
	}

	error = setattr_prepare(dentry, iattr);
	if (error)
		return error;

	if (iattr->ia_valid & ATTR_UID) {
		mutex_lock(&c->sem);
		
		// Charge new owner
		error = littlefs_quota_charge_file(c, iattr->ia_uid, 0, inode->i_size);
		if (error) {
			mutex_unlock(&c->sem);
			return error;
		}

		// Credit old owner
		littlefs_quota_charge_file(c, inode->i_uid, inode->i_size, 0);
		
		mutex_unlock(&c->sem);
	}

	if (iattr->ia_valid & ATTR_SIZE) {
		mutex_lock(&c->sem);
		error = littlefs_quota_charge_file(c, inode->i_uid, inode->i_size, iattr->ia_size);
		if (error) {
			mutex_unlock(&c->sem);
			return error;
		}
		error = lfs_file_truncate(&c->lfs, &f->lfs.file, iattr->ia_size);
		if (error) {
			mutex_unlock(&c->sem);
			return error;
		}
		truncate_setsize(inode, iattr->ia_size);
		mutex_unlock(&c->sem);
	}
	setattr_copy(inode, iattr);
	mark_inode_dirty(inode);
	return 0;
}

const struct file_operations littlefs_dir_operations =
{
	.read =		generic_read_dir,
	.iterate_shared=littlefs_readdir,
	.fsync =	littlefs_fsync,
	.llseek =	generic_file_llseek,
};


const struct inode_operations littlefs_dir_inode_operations =
{
	.create =	littlefs_create,
	.lookup =	littlefs_lookup,
	.unlink =	littlefs_unlink,
	.mkdir =	littlefs_mkdir,
	.rmdir =	littlefs_rmdir,
	.rename =	littlefs_rename,
	.setattr =	littlefs_dir_setattr,
};

const struct file_operations littlefs_file_operations =
{
	.llseek =	generic_file_llseek,
	.open =		generic_file_open,
	.read_iter =	generic_file_read_iter,
	.write_iter =	generic_file_write_iter,
	.mmap =		generic_file_readonly_mmap,
	.fsync =	littlefs_fsync,
	.splice_read =	generic_file_splice_read,
};

const struct inode_operations littlefs_file_inode_operations =
{
	.setattr = littlefs_file_setattr,
};

static int littlefs_do_readpage_nolock(struct inode *inode, struct page *pg)
{
	struct littlefs_inode_info *f = LITTLEFS_INODE_INFO(inode);
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	unsigned char *pg_buf;
	int ret = 0;
	lfs_off_t off =  pg->index << PAGE_SHIFT;

	BUG_ON(!PageLocked(pg));

	if (!f) {
		ret = -ENOENT;
		goto out;
	}

	pg_buf = kmap(pg);
	if (lfs_file_seek(&c->lfs, &f->lfs.file, off, LFS_SEEK_SET) != off) {
		ret = -ENOMEM;
	} else {
		lfs_file_read(&c->lfs, &f->lfs.file, pg_buf, PAGE_SIZE);
	}

out:
	if (ret) {
		ClearPageUptodate(pg);
		SetPageError(pg);
	} else {
		SetPageUptodate(pg);
		ClearPageError(pg);
	}

	flush_dcache_page(pg);
	kunmap(pg);

	return ret;
}

static int littlefs_do_readpage_unlock(struct inode *inode, struct page *pg)
{
	int ret = littlefs_do_readpage_nolock(inode, pg);
	unlock_page(pg);
	return ret;
}

static int littlefs_readpage (struct file *filp, struct page *pg)
{
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(pg->mapping->host->i_sb);
	int ret;

	mutex_lock(&c->sem);
	ret = littlefs_do_readpage_unlock(pg->mapping->host, pg);
	mutex_unlock(&c->sem);
	return ret;
}

static int littlefs_write_begin(struct file *filp, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned flags,
			struct page **pagep, void **fsdata)
{
	struct page *pg;
	struct inode *inode = mapping->host;
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	pgoff_t index = pos >> PAGE_SHIFT;
	uint32_t pageofs = index << PAGE_SHIFT;
	int ret = 0;

	pg = grab_cache_page_write_begin(mapping, index, flags);
	if (!pg)
		return -ENOMEM;
	*pagep = pg;

	if (pageofs > inode->i_size) {
		mutex_lock(&c->sem);
		ret = littlefs_quota_charge_file(c, inode->i_uid, inode->i_size, pageofs);
		mutex_unlock(&c->sem);
		if (ret)
			goto out_page;

		inode->i_size = pageofs;
		inode->i_blocks = (inode->i_size + 511) >> 9;
	}

	if (!PageUptodate(pg)) {
		mutex_lock(&c->sem);
		ret = littlefs_do_readpage_nolock(inode, pg);
		mutex_unlock(&c->sem);

		if (ret)
			goto out_page;
	}
	return ret;

out_page:
	unlock_page(pg);
	put_page(pg);
	return ret;
}

static int littlefs_write_end(struct file *filp, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned copied,
			struct page *pg, void *fsdata)
{
	/* Actually commit the write from the page cache page we're looking at.
	 * For now, we write the full page out each time. It sucks, but it's simple
	 */
	struct inode *inode = mapping->host;
	struct littlefs_inode_info *f = LITTLEFS_INODE_INFO(inode);
	struct littlefs_sb_info *c = LITTLEFS_SB_INFO(inode->i_sb);
	unsigned start = pos & (PAGE_SIZE - 1);
	lfs_off_t off = pos;
	int ret = 0;
	lfs_ssize_t writtenlen = 0;

	BUG_ON(!PageUptodate(pg));

	if (!f) {
		ret = -ENOENT;
		SetPageError(pg);
		goto end;
	}

	kmap(pg);

	mutex_lock(&c->sem);
	if (inode->i_size < pos + copied) {
		ret = littlefs_quota_charge_file(c, inode->i_uid, inode->i_size, pos + copied);
	}

	if (!ret) {
		if (lfs_file_seek(&c->lfs, &f->lfs.file, off, LFS_SEEK_SET) != off) {
			ret = -ENOMEM;
		} else {
			writtenlen = lfs_file_write(&c->lfs, &f->lfs.file, page_address(pg) + start, copied);
			ret = lfs_file_sync(&c->lfs, &f->lfs.file);
		}
	}
	mutex_unlock(&c->sem);

	kunmap(pg);

	if (writtenlen < 0) {
		/* There was an error writing. */
		ret = writtenlen;
		SetPageError(pg);
	} else if (writtenlen) {
		if (inode->i_size < pos + writtenlen) {
			inode->i_size = pos + writtenlen;
			inode->i_blocks = (inode->i_size + 511) >> 9;
		}
	}

end:
	unlock_page(pg);
	put_page(pg);
	return writtenlen > 0 ? writtenlen : ret;
}

const struct address_space_operations littlefs_file_address_operations =
{
	.readpage =	    littlefs_readpage,
	.write_begin =	littlefs_write_begin,
	.write_end =	littlefs_write_end,
};

struct littlefs_inode_search_info {
	const char *name;
	umode_t type;
};

static int littlefs_inode_test(struct inode *inode, void *data)
{
	struct littlefs_inode_search_info *d = data;
	struct dentry *dentry;
	char buf[LFS_NAME_MAX], *path;
	int match;

	if (!inode->i_private) {
		return 0;
	}

	dentry = d_find_alias(inode);
	if (!dentry) {
		return 0;
	}

	path = dentry_path_raw(dentry, buf, sizeof(buf));

	dput(dentry);

	if (IS_ERR(path)) {
		return 0;
	}

	match = (d->type & S_IFMT) == (inode->i_mode & S_IFMT) && !strcmp(d->name, path);

	return match;
}

static int littlefs_inode_set(struct inode *inode, void *data)
{
	return 0;
}

struct inode *littlefs_get_inode(struct super_block *sb, struct inode *dir, const char *name, umode_t type)
{
	struct littlefs_inode_info *f;
	struct littlefs_sb_info *c;
	struct inode *inode;
	struct lfs_info info;
	int ret;
	struct littlefs_inode_search_info data;
	int ino;

	memset(&data, 0, sizeof(data));
	data.name = name;
	data.type = type;

	c = LITTLEFS_SB_INFO(sb);

	ino = atomic_inc_return(&c->highest_ino);

	inode = iget5_locked(sb, ino,
				 littlefs_inode_test, littlefs_inode_set,
				 &data);
	if (!inode)
		return ERR_PTR(-ENOMEM);
	if (!(inode->i_state & I_NEW))
		return inode;

	inode->i_ino = ino;

	mutex_lock(&c->sem);

	inode->i_atime = ITIME(0);
	inode->i_mtime = ITIME(0);
	inode->i_ctime = ITIME(0);

	inode->i_size = 0;
	inode->i_blocks = 0;

	f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f) {
		ret = -ENOMEM;
		goto error;
	}

	inode->i_private = f;
	f->attrs[0].type = LITTLEFS_ATTR_UID;
	f->attrs[0].size = sizeof(inode->i_uid);
	f->attrs[0].buffer = &inode->i_uid;

	f->attrs[1].type = LITTLEFS_ATTR_GID;
	f->attrs[1].size = sizeof(inode->i_gid);
	f->attrs[1].buffer = &inode->i_gid;

	f->attrs[2].type = LITTLEFS_ATTR_MODE;
	f->attrs[2].size = sizeof(inode->i_mode);
	f->attrs[2].buffer = &inode->i_mode;

	switch (type & S_IFMT) {
	case S_IFDIR:
	{
		int count = 0;
		set_nlink(inode, 2); /* parent and '.' */

		ret = lfs_dir_open(&c->lfs, &f->lfs.dir, name);
		if (ret) {
			goto error;
		}

		littlefs_get_dir_attrs(name, c, f);

		if (inode->i_mode == 0) {
			// New file
			inode_init_owner(inode, dir, type);
			ret = littlefs_set_dir_attrs(name, c, f);
		}

		// read items in dir
		while ( (ret = lfs_dir_read(&c->lfs, &f->lfs.dir, &info)) > 0) {
			if (count++ < 2) {
				continue;
			}
			if (info.type == LFS_TYPE_DIR) {
				inc_nlink(inode);
			}
		}

		if (ret < 0) {
			goto error;
		}

		if (inode->i_ino == 1)
			inc_nlink(inode);

		inode->i_op = &littlefs_dir_inode_operations;
		inode->i_fop = &littlefs_dir_operations;
		break;
	}
	case S_IFREG:
		ret = lfs_file_open(&c->lfs, &f->lfs.file, name, LFS_O_CREAT | LFS_O_RDWR);
		if (ret) {
			goto error;
		}

		lfs_file_getattrs(&c->lfs, &f->lfs.file, &f->attrs[0], ARRAY_SIZE(f->attrs)); // load attrs from disk
		lfs_file_setattrs(&c->lfs, &f->lfs.file, &f->attrs[0], ARRAY_SIZE(f->attrs)); // setup file attrs to be used when file is written

		if (inode->i_mode == 0) {
			// New file
			inode_init_owner(inode, dir, type);
		}

		inode->i_size = lfs_file_size(&c->lfs, &f->lfs.file);
		inode->i_blocks = (inode->i_size + 511) >> 9;

		inode->i_op = &littlefs_file_inode_operations;
		inode->i_fop = &littlefs_file_operations;
		inode->i_mapping->a_ops = &littlefs_file_address_operations;
		inode->i_mapping->nrpages = 0;
		break;

	case S_IFBLK:
	case S_IFCHR:
	case S_IFSOCK:
	case S_IFIFO:
	case S_IFLNK:
	default:
		pr_err("%s(): Invalid mode %o for ino %lu\n",
			__func__, inode->i_mode, (unsigned long)inode->i_ino);
		ret = -EINVAL;
		goto error;
	}

	mutex_unlock(&c->sem);

	unlock_new_inode(inode);
	return inode;

error:
	mutex_unlock(&c->sem);
	iget_failed(inode);
	return ERR_PTR(ret);
}