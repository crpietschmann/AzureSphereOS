// SPDX-License-Identifier: GPL-2.0
/*
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 */
#ifndef _LINUX_AZSPIO_H
#define _LINUX_AZSPIO_H

#include <linux/socket.h>

#define AZSPIO_COMPONENT_ID_LENGTH 16

struct sockaddr_azspio {
	__kernel_sa_family_t sa_family; // AF_AZSPIO
	__u16 sa_port; // Must be zero.
	__u8 sa_component_id[AZSPIO_COMPONENT_ID_LENGTH];
};

struct azspio_fw_entry {
	struct sockaddr_azspio src;
	struct sockaddr_azspio dst;
};

#define SIOCAZSPIOFWADD (SIOCPROTOPRIVATE + 0)
#define SIOCAZSPIOFWDEL (SIOCPROTOPRIVATE + 1)

#endif /* _LINUX_AZSPIO_H */
