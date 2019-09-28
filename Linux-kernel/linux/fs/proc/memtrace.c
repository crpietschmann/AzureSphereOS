// SPDX-License-Identifier: GPL-2.0
/*
 * Basic memory statistics support on procfs
 *
 * Copyright (C) 2018 Microsoft
 * 
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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/atomic.h>
#include "internal.h"

static int memtrace_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;

/*
 * display in kilobytes.
 */
#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);

	/*
	 * Tagged format, for easy grepping and expansion.
	 */
	seq_printf(m,
		"MemTotal:       %8lu kB\n"
		"MemFree:        %8lu kB\n"
		"MemCurrent:     %8lu kB\n"
		"MemPeak:        %8lu kB\n"
		,
		K(i.totalram),
		K(i.freeram),
		K(i.totalram - i.freeram),
		K(get_peak_alloc())
		);

	return 0;
#undef K
}

static int memtrace_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, memtrace_proc_show, NULL);
}

static const struct file_operations memtrace_proc_fops = {
	.open		= memtrace_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_memtrace_init(void)
{
	proc_create("memtrace", 0, NULL, &memtrace_proc_fops);
	return 0;
}
fs_initcall(proc_memtrace_init);
