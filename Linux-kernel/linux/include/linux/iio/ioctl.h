// SPDX-License-Identifier: GPL-2.0
/*
 * Custom IOCTL for IIO without sysfs
 *
 * Copyright (c) 2019 Microsoft Corporation. All rights reserved.
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
 */

#ifndef _IIO_IOCL_H
#define _IIO_IOCL_H


#ifdef CONFIG_IIO_IOCTL

/**
 * ioctl handler that is chained from iio_ioctl to handle
 * interaction from applications when sysfs is not available (due to size
 * constraints)
 **/
long iio_nosysfs_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#else

long iio_nosysfs_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

#endif

#endif
