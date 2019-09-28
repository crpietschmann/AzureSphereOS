// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 MediaTek Inc.
 * Author: Simon-bs <simon-bs.wu@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef _MTK3620_DMA_H_
#define _MTK3620_DMA_H_
struct mtk_chan {
	struct virt_dma_chan vc;
	struct list_head node;
	struct dma_slave_config	cfg;
	struct mtk_desc *desc;
	void __iomem *channel_base;
	unsigned int dma_ch, irq_number;
	char irq_name[20];
	struct tasklet_struct task;
	spinlock_t lock;
};

struct mtk_dma_cb {
	unsigned int info;
	unsigned int src;
	unsigned int dst;
	unsigned int length;
	unsigned int next;
};

struct mtk_sg {
	struct mtk_dma_cb *cb;
	dma_addr_t cb_addr;
};

struct mtk_desc {
	struct mtk_chan *c;
	struct virt_dma_desc vd;
	enum dma_transfer_direction dir;
	unsigned int sg_num, sg_idx;
	struct mtk_sg *sg;
	//cyclic
	struct mtk_sg *control_block_cyclic;
};
#endif
