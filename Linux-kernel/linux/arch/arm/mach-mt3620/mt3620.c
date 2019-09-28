// SPDX-License-Identifier: GPL-2.0
/*
 * Support for the MT3620 SoC
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
 
 */
#include <linux/init.h>
#include <asm/mach/arch.h>

static const char *const mt3620_dt_match[] __initconst = {
	"mediatek,mt3620",
	NULL
};

DT_MACHINE_START(MT3620_DT, "MediaTek MT3620")
	.dt_compat = mt3620_dt_match,
MACHINE_END
