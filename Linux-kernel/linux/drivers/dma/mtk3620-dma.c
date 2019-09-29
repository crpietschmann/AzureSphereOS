// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017 MediaTek Inc.
 * MTK DMAengine support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sizes.h>
#include <asm/io.h>
#include <linux/jiffies.h>
#include "virt-dma.h"
#include "mtk3620-dma.h"

#define DRV_NAME            "ca7dma-mtk"

/*---------------------------------------------------------------------------*/
#define INT_FLAG_B          BIT(0)
#define INT_EN_B            BIT(0)

#define EN_B                BIT(0)

#define HARD_RST_B          BIT(1)
#define WARM_RST_B          BIT(0)

#define PAUSE_B             BIT(1)
#define STOP_B              BIT(0)
#define STOP_CLR_B          (0)

#define FLUSH_B             BIT(0)
#define FLUSH_CLR_B         (0)
#define HW_FLUSH_B          BIT(1)

#define DMA_BYTE            0x0
#define DMA_SHORT           0x1
#define DMA_LONG            0x2

#define DMA_HANDSHAKE       BIT(0)
#define DMA_DIR_DEV_TO_MEM  BIT(2)

// 0: src-side ; 1:dst-side
#define WRAP_SEL(x)         ((x) << 20)
#define WRAP_EN_B           BIT(15)
#define CON_BURST_LEN(x)    ((x) << 16)
#define CON_SLOW_CNT(x)     ((x) << 5)
#define CON_SLOW_EN         BIT(2)

#define CON_RSIZE(x)        ((x) << 28)
#define CON_WSIZE(x)        ((x) << 24)
#define RADDR_FIX_EN        BIT(4)
#define WADDR_FIX_EN        BIT(3)

#define VDMA_RX_CON_RSIZE(x) ((x) << 0)
#define VDMA_TX_CON_WSIZE(x) ((x) << 0)
#define VDMA_TX_WPT_WRAP_B  BIT(16)
#define VDMA_RX_RPT_WRAP_B  BIT(16)

#define GLB_CFG_OFFSET      0x2200
#define CH_EN_STS0          0x18
#define CH_EN_STS1          0x1C
#define CH_EN_SET0          0x20
#define CH_EN_SET1          0x24
#define CH_EN_CLR0          0x28
#define CH_EN_CLR1          0x2C
#define GLB_STA0            0x30
#define GLB_STA1            0x34
#define GLB_STA2            0x38
#define TO_CNT_DIV          0x40
#define SET_DREQ_0          0x70
#define SET_DREQ_1          0x74
#define CLR_DREQ_0          0x78
#define CLR_DREQ_1          0x7C
/*---------------------------------------------------------------------------*/



struct mtk_dmadev {
	struct dma_device ddev;
	spinlock_t lock;
	void __iomem *base;
	struct clk *clk;
	unsigned int dma_num;
	unsigned int m2m_ch_s, m2m_ch_e, m2m_ch_num;
	unsigned int peri_ch_s, peri_ch_e, peri_ch_num;
	unsigned int vff_ch_s, vff_ch_e, vff_ch_num;
	unsigned int ch_irq_sts;
	int ch_irq[];
};

enum dma_reg_offset{
	DMA_INT_FLAG        = 0x00,
	DMA_INT_EN          = 0x04,
	DMA_EN              = 0x08,
	DMA_RST             = 0x0C,
	DMA_STOP            = 0x10,
	//DMA_FLUSH           = 0x14,
	DMA_CON             = 0x18,
	DMA_SRC_ADDR        = 0x1C,
	DMA_DST_ADDR        = 0x20,
	DMA_LEN1            = 0x24,
	DMA_LEN2            = 0x28,
	DMA_JUMP_ADDR       = 0x2C,
	DMA_CONNECT         = 0x34,
	DMA_INT_BUF_SIZE    = 0x38,
	DMA_DCM_EN          = 0x48,
	DMA_DEBUG_STATUS    = 0x50,
};

enum vdma_reg_offset{
	VDMA_INT_FLAG       = 0x00,
	VDMA_INT_EN         = 0x04,
	VDMA_EN             = 0x08,
	VDMA_RST            = 0x0C,
	VDMA_STOP           = 0x10,
	VDMA_FLUSH          = 0x14,
	VDMA_CON            = 0x18,
	VDMA_MEM_ADDR       = 0x1C,
	VDMA_PORT_ADDR      = 0x20,
	VDMA_FIFO_LEN       = 0x24,
	VDMA_THRE           = 0x28,
	VDMA_WPT            = 0x2C,
	VDMA_RPT            = 0x30,
	VDMA_RX_FLOW_CTRL   = 0x34,
	VDMA_INT_BUF_SIZE   = 0x38,
	VDMA_VALID_SIZE     = 0x3C,
	VDMA_LEFT_SIZE      = 0x40,
	VDMA_TOVALUE        = 0x44,
	VDMA_DCM_EN         = 0x48,
	VDMA_DEBUG_STATUS   = 0x50,
};

enum dma_slave_ip {
	SLAVE_ID_I2C      = 0,
	SLAVE_ID_SPI      = 1,
	SLAVE_ID_UART     = 2,
	SLAVE_ID_I2S      = 3,
	SLAVE_ID_UART_CA7 = 4,
	SLAVE_ID_ADC      = 5,
	SLAVE_ID_WFSYS    = 6,
	SLAVE_ID_NONE
};

#define MTK3620_CA7DMA_CHANIO(base, n)  ((base) + 0x100*(n))
#define MAX_DMA_LEN                     ((SZ_64K - 1)&0xFFFFFFF0)
#define MAX_TIMEOUT_SEC                 (10)
#define MASK_MSB_16_BIT                 (0x0000ffff)
#define MASK_MSB_29_BIT                 (0x00000007)
#define MASK_FOR_8_BYTE_ALIGN           (0xfff8)
#define MAX_DMA_ADC_LEN                 ((SZ_64K - 1) & 0x0000fff8)
#define MAX_DMA_I2C_LEN                 (SZ_64K - 1)
#define MAX_DMA_SPI_LEN                 (SZ_64K - 1)
#define MAX_DMA_UART_LEN                ((SZ_64K - 1) & 0x0000fff8l)
/*---------------------------------------------------------------------------*/
#define DMA_FULL_CH_S                   13   /* FULL DMA start channel */
#define DMA_FULL_CH_E                   14   /* FULL DMA end channel */
#define DMA_HALF_CH_S                    0   /* HALF DMA start channel */
#define DMA_HALF_CH_E                   12   /* HALF DMA end channel */
#define DMA_VFIFO_CH_S                  15   /* VFIFO start channel */
#define DMA_VFIFO_CH_E                  33   /* VFIFO end channel */

#define DMA_CHANNEL_AMOUNT              34
/*---------------------------------------------*/
#define VFF_THRE_INT_FLAG_B             BIT(0)
#define VFF_THRE_INT_FLAG_CLR_B         BIT(0)
#define VFF_TO_INT_FLAG_B               BIT(1)
#define VFF_TO_INT_FLAG_CLR_B           BIT(1)
#define VFF_FLUSH_INT_FLAG_B            BIT(2)
#define VFF_FLUSH_INT_FLAG_CLR_B        BIT(2)
#define VFF_STOP_INT_FLAG_B             BIT(3)
#define VFF_STOP_INT_FLAG_CLR_B         BIT(3)

#define VFF_INT_EN_CLR                  (0x0)
#define VFF_THRE_INT_EN_B               BIT(0)
#define VFF_TO_INT_EN_B                 BIT(1)
#define VFF_FLUSH_INT_EN_B              BIT(2)
#define VFF_STOP_INT_EN_B               BIT(3)
/*---------------------------------------------*/

void __iomem *DMA_CHANNEL_BASE;
typedef void (* VOID_FUNC)(void);
static void mtk_dma_error_lisr(void)
{
	printk(KERN_ERR "DMA LISR not defined\n");
}

VOID_FUNC DMA_VFIFO_LISR_FUNC[DMA_VFIFO_CH_E - DMA_VFIFO_CH_S + 1] =
{
	mtk_dma_error_lisr
};

VOID_FUNC DMA_VFIFO_TO_LISR_FUNC[DMA_VFIFO_CH_E - DMA_VFIFO_CH_S + 1] =
{
	mtk_dma_error_lisr
};

void mtk_dma_vfifo_register_callback(unsigned int ch, VOID_FUNC callback_func)
{
	DMA_VFIFO_LISR_FUNC[ch - DMA_VFIFO_CH_S] = callback_func;
}

void mtk_dma_vfifo_register_to_callback(unsigned int ch, VOID_FUNC callback_func)
{
	DMA_VFIFO_TO_LISR_FUNC[ch - DMA_VFIFO_CH_S] = callback_func;
}

static void mtk_set_dma_base_adress(void __iomem *addr)
{
	DMA_CHANNEL_BASE = 	addr;

	printk(KERN_DEBUG "[%s] DMA_CHANNEL_BASE = 0x%p, addr = 0x%p\n", __func__, DMA_CHANNEL_BASE, addr);
}

static void __iomem *mtk_get_dma_base_adress(void)
{
	printk(KERN_DEBUG "[%s] DMA_CHANNEL_BASE = 0x%p\n", __func__, DMA_CHANNEL_BASE);

	return (DMA_CHANNEL_BASE);
}

void mtk_set_i2s_dma_warm_rst(unsigned int ch, unsigned int warm_rst)
{
	void __iomem *channel_base;
	unsigned int reg = 0;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	if(warm_rst){
		writel( WARM_RST_B, channel_base + VDMA_RST);

		printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_RST: 0x%08x(0x%08x)\n", \
			__func__, ch, (unsigned int)(channel_base + VDMA_RST), readl(channel_base + VDMA_RST));

		do{
			reg = readl(channel_base + VDMA_EN);
		}while( (reg & 0x1) != 0 );

		writel(0, channel_base + VDMA_RST);

		printk(KERN_DEBUG "[%s] (Warm Reset flow-After) ch(%u)\tVDMA_RST: 0x%08x(0x%08x)\n", \
			__func__, ch, (unsigned int)(channel_base + VDMA_RST), readl(channel_base + VDMA_RST));
	}
}

void mtk_set_i2s_dma_clk(unsigned int ch, unsigned int en)
{
	unsigned int reg = 0;
	void __iomem *channel_base;
	void __iomem *glb_cfg_base;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	if(en){
		/* clk enable */
		glb_cfg_base = channel_base + GLB_CFG_OFFSET;

		if (ch < 32){
			writel((1 << ch), (glb_cfg_base + CH_EN_SET0));

			reg = readl((glb_cfg_base + CH_EN_STS0));

			printk(KERN_DEBUG "[%s] ch(%u)\tCH_EN_STS0: 0x%08x(0x%08x)\n", \
				__func__, ch, (unsigned int)(glb_cfg_base + CH_EN_STS0), readl(glb_cfg_base + CH_EN_STS0));

			if ((reg & (1 << ch)) == 0) {
				printk("Couldn't enable the DMA %u clock\n", ch);
			}
		}else{
			writel((1 << (ch-32)), glb_cfg_base + CH_EN_SET1);

			reg = readl((glb_cfg_base + CH_EN_STS1));

			printk(KERN_DEBUG "[%s] ch(%u)\tCH_EN_STS1: 0x%08x(0x%08x)\n", \
				__func__, ch, (unsigned int)(glb_cfg_base + CH_EN_STS1), readl(glb_cfg_base + CH_EN_STS1));

			if ((reg & (1 << (ch-32))) == 0) {
				printk("Couldn't enable the DMA %u clock\n", ch);
			}
		}
	}else{
		/* clk disable */
		glb_cfg_base = channel_base + GLB_CFG_OFFSET;

		if (ch < 32){
			writel((1 << ch), (glb_cfg_base + CH_EN_CLR0));

			reg = readl((glb_cfg_base + CH_EN_STS0));

			printk(KERN_DEBUG "[%s] ch(%u)\tCH_EN_STS0: 0x%08x(0x%08x)\n", \
				__func__, ch, (unsigned int)(glb_cfg_base + CH_EN_STS0), readl(glb_cfg_base + CH_EN_STS0));

			if ((reg & (1 << ch)) == 0) {
				printk("Disable the DMA %u clock\n", ch);
			}
		}else{
			writel((1 << (ch-32)), glb_cfg_base + CH_EN_CLR1);

			reg = readl((glb_cfg_base + CH_EN_STS1));

			printk(KERN_DEBUG "[%s] ch(%u)\tCH_EN_STS1: 0x%08x(0x%08x)\n", \
				__func__, ch, (unsigned int)(glb_cfg_base + CH_EN_STS1), readl(glb_cfg_base + CH_EN_STS1));

			if ((reg & (1 << (ch-32))) == 0) {
				printk("Disable the DMA %u clock\n", ch);
			}
		}
	}
}

void mtk_set_i2s_dma_enable_int(unsigned int ch, unsigned int int_en)
{
	void __iomem *channel_base;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	writel( int_en, channel_base + VDMA_INT_EN);

	printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_INT_EN: 0x%08x(0x%08x)\n", \
		__func__, ch, (unsigned int)(channel_base + VDMA_INT_EN), readl(channel_base + VDMA_INT_EN));
}

void mtk_set_i2s_dma_disable_int(unsigned int ch)
{
	void __iomem *channel_base;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	writel( 0, channel_base + VDMA_INT_EN);

	printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_INT_EN: 0x%08x(0x%08x)\n", \
		__func__, ch, (unsigned int)(channel_base + VDMA_INT_EN), readl(channel_base + VDMA_INT_EN));
}


void mtk_set_i2s_dma_param(unsigned int ch, unsigned int mem_addr, unsigned int port_addr, unsigned int fifolen, unsigned int int_en)
{
	unsigned int con = 0;
	void __iomem *channel_base;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	/* clk enable */
	mtk_set_i2s_dma_clk(ch, 1);

	/* warm reset */
	mtk_set_i2s_dma_warm_rst(ch, 1);

	/* mem addr */
	writel(mem_addr, channel_base + VDMA_MEM_ADDR);

	/* port addr */
	writel(port_addr, channel_base + VDMA_PORT_ADDR);

	/* len */
	writel(fifolen, channel_base + VDMA_FIFO_LEN);

	/* con */
	con = readl(channel_base + VDMA_CON);
	con |= (DMA_LONG << 0);
	writel(con, channel_base + VDMA_CON);

	/* interrupt enable */
	mtk_set_i2s_dma_enable_int(ch, int_en);

	/* write pointer */
	writel(0, channel_base + VDMA_WPT);

	/* read pointer */
	writel(0, channel_base + VDMA_RPT);

	/* kick dma */
	writel(EN_B, channel_base + VDMA_EN);

	printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_MEM_ADDR(0x%08x)\tVDMA_PORT_ADDR(0x%08x)\tVDMA_FIFO_LEN(0x%08x)\tVDMA_CON(0x%08x)\tVDMA_WPT(0x%08x)\tVDMA_RPT(0x%08x)\tVDMA_EN(0x%08x)\n", \
		__func__, ch, readl(channel_base + VDMA_MEM_ADDR), readl(channel_base + VDMA_PORT_ADDR), \
		readl(channel_base + VDMA_FIFO_LEN), readl(channel_base + VDMA_CON), readl(channel_base + VDMA_WPT), \
		readl(channel_base + VDMA_RPT), readl(channel_base + VDMA_EN));

}

void mtk_set_i2s_dma_stop(unsigned int ch, unsigned int stop)
{
	void __iomem *channel_base;
	unsigned int reg = 0;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	writel( ( (stop != 0)?(STOP_B):(STOP_CLR_B) ), channel_base + VDMA_STOP);

	if(stop){

		do{
			reg = readl(channel_base + VDMA_EN);
			printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_EN: 0x%08x(0x%08x)\n", __func__, ch, \
				(unsigned int)(channel_base + VDMA_EN), readl(channel_base + VDMA_EN));
		}while( (reg & 0x1) != 0 );

		writel(0, channel_base + VDMA_STOP);
		printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_STOP: 0x%08x(0x%08x)\n", __func__, ch, \
			(unsigned int)(channel_base + VDMA_STOP), readl(channel_base + VDMA_STOP));
	}

	mtk_set_i2s_dma_warm_rst(ch, 1);

	mtk_set_i2s_dma_clk(ch, 0);
}

void mtk_set_i2s_dma_thres(unsigned int ch, unsigned int threshold)
{
	void __iomem *channel_base;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	/* threshold interrupt (tx_vff_left_size >= tx_vff_thre) ; (rx_vff_valid_size >= rx_vff_thre) */
	writel(threshold, channel_base + VDMA_THRE);

	printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_THRE: 0x%08x(0x%08x)\n", __func__, ch, \
		(unsigned int)(channel_base + VDMA_THRE), readl(channel_base + VDMA_THRE));
}

void mtk_set_i2s_dma_to(unsigned int ch, unsigned int tovalue)
{
	void __iomem *channel_base;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	/* time-out criterion*/
	writel(tovalue, channel_base + VDMA_TOVALUE);

	printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_TOVALUE: 0x%08x(0x%08x)\n", __func__, ch, \
		(unsigned int)(channel_base + VDMA_TOVALUE), readl(channel_base + VDMA_TOVALUE));
}

static unsigned int mtk_dma_vfifo_flow_ctl(unsigned int swptr, unsigned int ffsize, unsigned int length_byte)
{
	unsigned int wrap = 0;

	wrap = (swptr & 0x00010000);
	swptr &= 0x0000ffff;

	/* Handle ring buffer case for software pointer */
	if((swptr+length_byte) >= ffsize)
	{
		swptr = (swptr + length_byte - ffsize);
		if ((wrap) != 0x00010000)
		{
			swptr &= 0x0000ffff;
			wrap = 0x00010000;
		}
		else
		{
			swptr &= 0x0000ffff;
			wrap &= 0x00000000;
		}
	}else{
		swptr += length_byte;
	}
	swptr |= wrap;

	return (swptr);
}

void mtk_set_i2s_dma_swptr(unsigned int ch, enum dma_transfer_direction direction, unsigned int length_byte)
{
	void __iomem *channel_base;
	unsigned int swptr_cr = 0, swptr_process = 0, ffsize = 0;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	ffsize = readl(channel_base + VDMA_FIFO_LEN);

	switch(direction){
		case DMA_MEM_TO_DEV: /* TX */
			/* FIFO Flow control for TX-side */
			swptr_cr = readl(channel_base + VDMA_WPT);

			swptr_process = mtk_dma_vfifo_flow_ctl(swptr_cr, ffsize, length_byte);

			/* write pointer */
			writel(swptr_process, channel_base + VDMA_WPT);

			printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_WPT: 0x%08x(0x%08x)\n", __func__, ch, \
				(unsigned int)(channel_base + VDMA_WPT), readl(channel_base + VDMA_WPT));
			break;

		case DMA_DEV_TO_MEM: /* RX */
			/* FIFO Flow control for RX-side */
			swptr_cr = readl(channel_base + VDMA_RPT);

			swptr_process = mtk_dma_vfifo_flow_ctl(swptr_cr, ffsize, length_byte);

			/*read pointer */
			writel(swptr_process, channel_base + VDMA_RPT);

			printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_RPT: 0x%08x(0x%08x)\n", __func__, ch, \
				(unsigned int)(channel_base + VDMA_RPT), readl(channel_base + VDMA_RPT));
			break;

		default:
			printk("\t\t[%s] Not support transfer direction(ch: %u, direction: %d, length_byte: 0x%08x)!!\n", __func__, ch, direction, length_byte);
			return;
			break;
	}
}

void mtk_set_i2s_dma_dreq(unsigned int ch)
{
	void __iomem *channel_base, *glb_cfg_base;
	unsigned int reg = 0;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	glb_cfg_base = channel_base + GLB_CFG_OFFSET;

	if (ch < 32){
		writel((0 << ch), (glb_cfg_base + SET_DREQ_0));

		reg = readl((glb_cfg_base + SET_DREQ_0));

		writel((1 << ch), (glb_cfg_base + SET_DREQ_0));

		reg = readl((glb_cfg_base + SET_DREQ_0));

		if ((reg & (1 << ch)) == 0) {
			printk("Couldn't set the DMA %u DREQ manually\n", ch);
		}
		writel((0 << ch), (glb_cfg_base + SET_DREQ_0));

		reg = readl((glb_cfg_base + SET_DREQ_0));

		printk(KERN_DEBUG "[%s] GLB-SET_DREQ_0: 0x%p(0x%08x)\n", __func__, (glb_cfg_base + SET_DREQ_0), \

		readl(glb_cfg_base + SET_DREQ_0));
	}else{
		writel((0 << (ch-32)), glb_cfg_base + SET_DREQ_1);

		reg = readl((glb_cfg_base + SET_DREQ_1));

		writel((1 << (ch-32)), glb_cfg_base + SET_DREQ_1);

		reg = readl((glb_cfg_base + SET_DREQ_1));

		if ((reg & (1 << (ch-32))) == 0) {
			printk("Couldn't set the DMA %u DREQ manually\n", ch);
		}

		writel((0 << (ch-32)), glb_cfg_base + SET_DREQ_1);

		reg = readl((glb_cfg_base + SET_DREQ_1));

		printk(KERN_DEBUG "[%s] GLB-SET_DREQ_1: 0x%p(0x%08x)\n", __func__, (glb_cfg_base + SET_DREQ_1), \

		readl(glb_cfg_base + SET_DREQ_1));
	}
}

void mtk_clr_i2s_dma_dreq(unsigned int ch)
{
	void __iomem *channel_base, *glb_cfg_base;
	unsigned int reg = 0;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	glb_cfg_base = channel_base + GLB_CFG_OFFSET;

	if (ch < 32){
		writel((0 << ch), (glb_cfg_base + CLR_DREQ_0));

		reg = readl((glb_cfg_base + CLR_DREQ_0));

		writel((1 << ch), (glb_cfg_base + CLR_DREQ_0));

		reg = readl((glb_cfg_base + CLR_DREQ_0));

		if ((reg & (1 << ch)) == 0) {
			printk("Couldn't clear the DMA %u DREQ manually\n", ch);
		}

		writel((0 << ch), (glb_cfg_base + CLR_DREQ_0));

		reg = readl((glb_cfg_base + CLR_DREQ_0));

		printk(KERN_DEBUG "[%s] GLB-CLR_DREQ_0: 0x%p(0x%08x)\n", __func__, (glb_cfg_base + CLR_DREQ_0), \
			readl(glb_cfg_base + CLR_DREQ_0));
	}else{
		writel((0 << (ch-32)), glb_cfg_base + CLR_DREQ_1);

		reg = readl((glb_cfg_base + CLR_DREQ_1));

		writel((1 << (ch-32)), glb_cfg_base + CLR_DREQ_1);

		reg = readl((glb_cfg_base + CLR_DREQ_1));

		if ((reg & (1 << (ch-32))) == 0) {
			printk("Couldn't clear the DMA %u DREQ manually\n", ch);
		}

		writel((0 << (ch-32)), glb_cfg_base + CLR_DREQ_1);

		reg = readl((glb_cfg_base + CLR_DREQ_1));

		printk(KERN_DEBUG "[%s] GLB-CLR_DREQ_1: 0x%p(0x%08x)\n", __func__, (glb_cfg_base + CLR_DREQ_1), \
			readl(glb_cfg_base + CLR_DREQ_1));
	}
}

unsigned int mtk_read_i2s_dma_hwptr(unsigned int ch, enum dma_transfer_direction direction)
{
	void __iomem *reg;
	void __iomem *channel_base;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	switch(direction){
		case DMA_MEM_TO_DEV: /* TX */
		reg = ( channel_base + VDMA_RPT);

		printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_RPT(0x%08x)\tVDMA_INT_EN(0x%08x)\tVDMA_INT_FLAG(0x%08x)\n", \
			__func__, ch, readl( reg ), readl(channel_base + VDMA_INT_EN ), \
		readl(channel_base + VDMA_INT_FLAG ));

		break;

		case DMA_DEV_TO_MEM: /* RX */
		reg = ( channel_base + VDMA_WPT);

		printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_WPT(0x%08x)\tVDMA_INT_EN(0x%08x)\tVDMA_INT_FLAG(0x%08x)\n", \
			__func__, ch, readl( reg ), readl(channel_base + VDMA_INT_EN ), \
			readl(channel_base + VDMA_INT_FLAG ));
		break;

		default:
			printk("\t\t[%s] Not support transfer direction(ch: %u, direction: %d)!!\n", __func__, ch, direction);
		break;
	}

	return readl(reg);
}

void mtk_dump_i2s_dma_vffinfo(unsigned int ch)
{
	void __iomem *channel_base, *glb_cfg_base;

	channel_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), ch);

	glb_cfg_base = MTK3620_CA7DMA_CHANIO(mtk_get_dma_base_adress(), 34);

	printk(KERN_DEBUG "[%s] CH_EN_STS0(0x%08x)\tCH_EN_STS1(0x%08x)\tGLB_STA0(0x%08x)\tGLB_STA1(0x%08x)\tGLB_STA2(0x%08x)\n\n", \
		__func__, readl(glb_cfg_base + CH_EN_STS0), \
		readl(glb_cfg_base + CH_EN_STS1), readl(glb_cfg_base + GLB_STA0), \
		readl(glb_cfg_base + GLB_STA1), readl(glb_cfg_base + GLB_STA2));

	printk(KERN_DEBUG "[%s] ch(%u)\tVDMA_INT_FLAG(0x%08x)\tVDMA_INT_EN(0x%08x)\tVDMA_EN(0x%08x)\n", \
		__func__, ch, readl(channel_base + VDMA_INT_FLAG), \
		readl(channel_base + VDMA_INT_EN), readl(channel_base + VDMA_EN));

	printk(KERN_DEBUG "[%s] VDMA_RST(0x%08x)\tVDMA_STOP(0x%08x)\tVDMA_FLUSH(0x%08x)\tVDMA_CON(0x%08x)\n", \
		__func__, readl(channel_base + VDMA_RST), \
		readl(channel_base + VDMA_STOP), readl(channel_base + VDMA_FLUSH), \
		readl(channel_base + VDMA_CON));

	printk(KERN_DEBUG "[%s] VDMA_MEM_ADDR(0x%08x)\tVDMA_PORT_ADDR(0x%08x)\tVDMA_FIFO_LEN(0x%08x)\tVDMA_WPT(0x%08x)\tVDMA_RPT(0x%08x)\n", \
		__func__, readl(channel_base + VDMA_MEM_ADDR), readl(channel_base + VDMA_PORT_ADDR), \
		readl(channel_base + VDMA_FIFO_LEN), readl(channel_base + VDMA_WPT), \
		readl(channel_base + VDMA_RPT));

	printk(KERN_DEBUG "[%s] VDMA_INT_BUF_SIZE(0x%08x)\tVDMA_VALID_SIZE(0x%08x)\tVDMA_LEFT_SIZE(0x%08x)\n", \
		__func__, readl(channel_base + VDMA_INT_BUF_SIZE), readl(channel_base + VDMA_VALID_SIZE), \
		readl(channel_base + VDMA_LEFT_SIZE));

	printk(KERN_DEBUG "[%s] VDMA_THRE(0x%08x)\tVDMA_RX_FLOW_CTRL(0x%08x)\tVDMA_TOVALUE(0x%08x)\tVDMA_DCM_EN(0x%08x)\tVDMA_DEBUG_STATUS(0x%08x)\t\n",
		__func__, readl(channel_base + VDMA_THRE), readl(channel_base + VDMA_RX_FLOW_CTRL), \
		readl(channel_base + VDMA_TOVALUE), readl(channel_base + VDMA_DCM_EN), readl(channel_base + VDMA_DEBUG_STATUS));
}

EXPORT_SYMBOL(mtk_set_i2s_dma_param);
EXPORT_SYMBOL(mtk_set_i2s_dma_disable_int);
EXPORT_SYMBOL(mtk_set_i2s_dma_warm_rst);
EXPORT_SYMBOL(mtk_set_i2s_dma_enable_int);
EXPORT_SYMBOL(mtk_set_i2s_dma_disable_int);
EXPORT_SYMBOL(mtk_set_i2s_dma_clk);
EXPORT_SYMBOL(mtk_set_i2s_dma_dreq);
EXPORT_SYMBOL(mtk_clr_i2s_dma_dreq);
EXPORT_SYMBOL(mtk_set_i2s_dma_stop);
EXPORT_SYMBOL(mtk_set_i2s_dma_thres);
EXPORT_SYMBOL(mtk_set_i2s_dma_to);
EXPORT_SYMBOL(mtk_set_i2s_dma_swptr);
EXPORT_SYMBOL(mtk_read_i2s_dma_hwptr);
EXPORT_SYMBOL(mtk_dump_i2s_dma_vffinfo);
EXPORT_SYMBOL(mtk_dma_vfifo_register_callback);
EXPORT_SYMBOL(mtk_dma_vfifo_register_to_callback);
/*---------------------------------------------------------------------------*/
static bool mtk_dma_filter_fn(struct dma_chan *chan, void *param);
static struct of_dma_filter_info mtk_dma_info = {
	.filter_fn = mtk_dma_filter_fn,
};

static inline struct mtk_dmadev *to_mtk_dma_dev(struct dma_device *d)
{
	return container_of(d, struct mtk_dmadev, ddev);
}

static inline struct mtk_chan *to_mtk_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct mtk_chan, vc.chan);
}

static inline struct mtk_desc *to_mtk_dma_desc(struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct mtk_desc, vd.tx);
}

static void mtk_dma_chan_write(struct mtk_chan *c, unsigned int reg, unsigned int val)
{
	writel(val, c->channel_base + reg);
}

static unsigned int mtk_dma_chan_read(struct mtk_chan *c, unsigned int reg)
{
	return readl(c->channel_base + reg);
}

static void mtk_dma_glb_cfg_write(struct mtk_chan *c, unsigned int reg, unsigned int val)
{
	struct mtk_dmadev *mtkd = to_mtk_dma_dev(c->vc.chan.device);
	void __iomem *glb_cfg_base;

	glb_cfg_base = MTK3620_CA7DMA_CHANIO(mtkd->base, mtkd->dma_num);

	dev_dbg(mtkd->ddev.dev, "[%s]DMA mtkd->base:0x%p, glb_cfg_base:0x%p\n",
		__func__, mtkd->base, glb_cfg_base);

	writel(val, glb_cfg_base + reg);

	dev_dbg(mtkd->ddev.dev, "[%s]DMA mtkd->base:0x%p, reg:0x%08x\n",
		__func__, mtkd->base, readl(glb_cfg_base + reg));
}

static unsigned int mtk_dma_glb_cfg_read(struct mtk_chan *c, unsigned int reg)
{
	struct mtk_dmadev *mtkd = to_mtk_dma_dev(c->vc.chan.device);
	void __iomem *glb_cfg_base;
	unsigned int return_value = 0;

	glb_cfg_base = MTK3620_CA7DMA_CHANIO(mtkd->base, mtkd->dma_num);

	dev_dbg(mtkd->ddev.dev, "[%s]DMA mtkd->base:0x%p, glb_cfg_base:0x%p\n",
		__func__, mtkd->base, glb_cfg_base);

	return_value = (unsigned int)readl(glb_cfg_base + reg);

	return (unsigned int)return_value;
}

static int mtk_dma_clk_enable(struct mtk_chan *c)
{
	struct mtk_dmadev *mtkd = to_mtk_dma_dev(c->vc.chan.device);
	unsigned int channel;
	unsigned int reg =  0;

	channel = c->dma_ch;

	dev_dbg(mtkd->ddev.dev, "[%s] channel:%u, ver:0x%08x\n",
		__func__, channel, mtk_dma_glb_cfg_read(c, 0));

	if(channel < 32){
		mtk_dma_glb_cfg_write(c, CH_EN_SET0, (1 << channel));

		reg = mtk_dma_glb_cfg_read(c, CH_EN_STS0);
	}else if(channel >=32 && channel < DMA_CHANNEL_AMOUNT){
		mtk_dma_glb_cfg_write(c, CH_EN_SET1, (1 << (channel-32)));

		reg = mtk_dma_glb_cfg_read(c, CH_EN_STS1);
	}

	dev_dbg(mtkd->ddev.dev, "[%s] channel:%u, reg:0x%08x\n",
		__func__, channel, reg);

	if ((reg & ( (channel < 32)?(1 << channel):(1 << (channel-32)) )) == 0) {
		printk("Couldn't enable the DMA %u clock\n", channel);
	}

	return 0;
}

static void mtk_dma_desc_free(struct virt_dma_desc *vd)
{
	struct dma_chan *chan = vd->tx.chan;
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	unsigned int count;

	dev_dbg(chan->device->dev, "[%s]DMA ch%d\n", __func__, c->dma_ch);

	spin_lock(&c->lock);

	/* Check "free descriptor" function and descriptor content */
	if(c->cfg.slave_id == SLAVE_ID_I2S || c->cfg.slave_id == SLAVE_ID_ADC){
		for (count = 0; count < c->desc->sg_num; count++) {
			if (c->desc->control_block_cyclic[count].cb != NULL){
				kfree(c->desc->control_block_cyclic[count].cb);
				c->desc->control_block_cyclic[count].cb = NULL;
			}
		}
		if (c->desc->control_block_cyclic != NULL) {
			c->desc->control_block_cyclic= NULL;
		}
	}else{
		for (count = 0; count < c->desc->sg_num; count++) {
			if (c->desc->sg[count].cb != NULL){
				kfree(c->desc->sg[count].cb);
				c->desc->sg[count].cb = NULL;
			}
		}
		if (c->desc->sg != NULL) {
			kfree(c->desc->sg);
			c->desc->sg = NULL;
		}
	}

	if (c->desc != NULL) {
		kfree(c->desc);
		c->desc = NULL;
	}

	spin_unlock(&c->lock);
}

static bool mtk_dma_reset(struct mtk_chan *c)
{
	unsigned long timeout;

	dev_dbg(c->vc.chan.device->dev, "[%s]DMA ch%d\n", __func__, c->dma_ch);

	mtk_dma_chan_write(c, DMA_RST, WARM_RST_B);

	timeout = jiffies + HZ * MAX_TIMEOUT_SEC;

	while (time_before(jiffies, timeout)) {
		if (mtk_dma_chan_read(c, DMA_EN) == 0){
			mtk_dma_chan_write(c, DMA_RST, 0);

			return true;
		}
	};

	dev_err(c->vc.chan.device->dev, "DMA warm resets error(ch = %d)!!\n",
		c->dma_ch);

	return false;
}

static bool mtk_dma_stop(struct mtk_chan *c)
{
	unsigned long timeout;

	dev_dbg(c->vc.chan.device->dev, "[%s]DMA ch%d\n", __func__, c->dma_ch);

	/*set stop as 1 -> wait until en is 0 -> set stop as 0*/
	mtk_dma_chan_write(c, DMA_STOP, STOP_B);

	timeout = jiffies + HZ * MAX_TIMEOUT_SEC;

	while (time_before(jiffies, timeout)) {
		if (mtk_dma_chan_read(c, DMA_EN) == 0){
			mtk_dma_chan_write(c, DMA_STOP, STOP_CLR_B);

			return true;
		}
	};

	dev_err(c->vc.chan.device->dev, "DMA stops error(ch = %d)!!\n",
		c->dma_ch);

	return false;
}

static void mtk_set_vchan_completion(struct mtk_chan *c)
{
	dev_dbg(c->vc.chan.device->dev, "[%s]DMA ch%d\n", __func__, c->dma_ch);

	if (!list_empty(&c->vc.desc_issued)) {
		dev_dbg(c->vc.chan.device->dev,
			"\t\tRemove DMA%d virt descriptor on issue list.\n",
			c->dma_ch);

		list_del(&c->desc->vd.node);

		vchan_cookie_complete(&c->desc->vd);
	} else {
		dev_dbg(c->vc.chan.device->dev,
			"\t\tVchan's desc_issued is empty!!\n");
	}
}

static bool mtk_set_vff_dma_param_check(struct mtk_chan *c)
{
	struct mtk_desc *desc_list = c->desc;
	unsigned int size, ptr, tmp;

	if (c->vc.chan.completed_cookie <= DMA_MIN_COOKIE)
		return false;

	dev_dbg(c->vc.chan.device->dev,
		"\t[%s]\tSkip initialize VFF setting!!\n", __func__);

	if (c->cfg.direction == DMA_DEV_TO_MEM) { /* RX-side */
		if (mtk_dma_chan_read(c, VDMA_VALID_SIZE))
			mtk_set_vchan_completion(c);
	} else { /* TX-side */
		size = c->desc->sg[c->desc->sg_idx].cb->length & MASK_MSB_16_BIT;

		if (size) {
			ptr = mtk_dma_chan_read(c, VDMA_WPT);

			tmp = (ptr & (VDMA_TX_WPT_WRAP_B - 1)) + size;

			if (tmp >= mtk_dma_chan_read(c, VDMA_FIFO_LEN)) {
				tmp -= mtk_dma_chan_read(c, VDMA_FIFO_LEN);
				ptr = tmp | ((ptr & VDMA_TX_WPT_WRAP_B)^(VDMA_TX_WPT_WRAP_B)); /* xor */
			} else {
				ptr += size;
			}
			mtk_dma_chan_write(c, VDMA_WPT, ptr);
			/* Align 8-byte */
			mtk_dma_chan_write(c, VDMA_THRE,
				(desc_list->sg[desc_list->sg_idx].cb->length >> 16) -
				(ptr & MASK_MSB_29_BIT));

			/* int_en */
			mtk_dma_chan_write(c, VDMA_INT_EN, INT_EN_B);

			/* Flush un-align 8 bytes case */
			//if (size < 8){mtk_dma_flush(c);} // DMA will flush automatically

		} else {
			dev_dbg(c->vc.chan.device->dev,
			"\t[%s]\tError happens on TX side!!\n", __func__);
		}
	}
	dev_dbg(c->vc.chan.device->dev,
		"[%s](%s) vff_addr: 0x%08x, port_addr: 0x%08x, vff_len: 0x%08x, cb_len: 0x%x, thres: 0x%08x, wptr: 0x%08x, rptr: 0x%08x\n",
		__func__,
		(c->cfg.direction == DMA_MEM_TO_DEV ? "TX" : "RX"),
		mtk_dma_chan_read(c, VDMA_MEM_ADDR),
		mtk_dma_chan_read(c, VDMA_PORT_ADDR),
		mtk_dma_chan_read(c, VDMA_FIFO_LEN),
		(desc_list->sg[desc_list->sg_idx].cb->length),
		mtk_dma_chan_read(c, VDMA_THRE),
		mtk_dma_chan_read(c, VDMA_WPT),
		mtk_dma_chan_read(c, VDMA_RPT));

	return true;
}

static void mtk_set_vff_dma_param(struct mtk_chan *c)
{
	struct mtk_desc *desc_list = c->desc;
	unsigned int dma_con = 0, trans_size = 0;

	dev_dbg(c->vc.chan.device->dev, "[%s]DMA ch%d\n", __func__, c->dma_ch);

	if (!c || !c->desc || !c->desc->sg) {
		dev_err(c->vc.chan.device->dev, "\t[%s]\tnull mtk_chan pointer\n",
		__func__);
		return;
	}

	if (mtk_set_vff_dma_param_check(c))
		return;

	/* clk enable */
	mtk_dma_clk_enable(c);
	/* warm reset DMA channel */
	mtk_dma_reset(c);

	switch (c->cfg.direction) {
		case DMA_MEM_TO_DEV:
			/* TX */
			/* src (mem) */
			mtk_dma_chan_write(c, VDMA_MEM_ADDR,
				desc_list->sg[desc_list->sg_idx].cb->src);
			/* dst (dev) */
			mtk_dma_chan_write(c, VDMA_PORT_ADDR,
					c->cfg.dst_addr);
			/* len */
			mtk_dma_chan_write(c, VDMA_FIFO_LEN,
				(desc_list->sg[desc_list->sg_idx].cb->length >> 16));
			/* write pointer */
			mtk_dma_chan_write(c, VDMA_WPT,
				(desc_list->sg[desc_list->sg_idx].cb->length & MASK_MSB_16_BIT));
			/* int_en */
			mtk_dma_chan_write(c, VDMA_INT_EN, INT_EN_B);
			/* thres (tx_vff_left_size >= tx_vff_thre) ; (tx_vff_valid_size < tx_vff_thre) */
			mtk_dma_chan_write(c, VDMA_THRE,
				(desc_list->sg[desc_list->sg_idx].cb->length >> 16) -
				(desc_list->sg[desc_list->sg_idx].cb->length & MASK_MSB_29_BIT));

			if (c->cfg.slave_id == SLAVE_ID_UART || c->cfg.slave_id == SLAVE_ID_ADC ){
				mtk_dma_chan_write(c, VDMA_THRE, 1);
			}

			/* con */
			/* HP default enable */
			dma_con = mtk_dma_chan_read(c, VDMA_CON);

			if (c->cfg.slave_id == SLAVE_ID_I2C){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_SPI){
				trans_size = DMA_LONG;
			}else if (c->cfg.slave_id == SLAVE_ID_UART){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_I2S){
				trans_size = DMA_LONG;
			}else if (c->cfg.slave_id == SLAVE_ID_UART_CA7){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_ADC){
				trans_size = DMA_LONG;
			}

			mtk_dma_chan_write(c, VDMA_CON, (dma_con | VDMA_TX_CON_WSIZE(trans_size)) );
			break;

		case DMA_DEV_TO_MEM:
			/* RX */
			/* src (dev) */
			mtk_dma_chan_write(c, VDMA_PORT_ADDR,
					c->cfg.src_addr);
			/* dst (mem) */
			mtk_dma_chan_write(c, VDMA_MEM_ADDR,
				desc_list->sg[desc_list->sg_idx].cb->dst);
			/* len */
			mtk_dma_chan_write(c, VDMA_FIFO_LEN,
				(desc_list->sg[desc_list->sg_idx].cb->length & MASK_MSB_29_BIT) ?
				/* 8 bytes unalign case */ //in order to align 8-byte
				((desc_list->sg[desc_list->sg_idx].cb->length + 0x08)& MASK_FOR_8_BYTE_ALIGN) :
				/* 8 bytes align case */
				(desc_list->sg[desc_list->sg_idx].cb->length) );
			/* read pointer */
			mtk_dma_chan_write(c, VDMA_RPT, 0);
			/* int_en */
			mtk_dma_chan_write(c, VDMA_INT_EN, INT_EN_B);
			/* thres (rx_vff_valid_size >= rx_vff_thre) */
			mtk_dma_chan_write(c, VDMA_THRE,
				(desc_list->sg[desc_list->sg_idx].cb->length & MASK_FOR_8_BYTE_ALIGN));

			if (c->cfg.slave_id == SLAVE_ID_UART || c->cfg.slave_id == SLAVE_ID_ADC){
				mtk_dma_chan_write(c, VDMA_THRE, 4);
			}

			//mtk_dma_chan_write(c, VDMA_RX_FLOW_CTRL, 0x80);

			/* con */
			dma_con = mtk_dma_chan_read(c, VDMA_CON); // HP default enable

			if (c->cfg.slave_id == SLAVE_ID_I2C){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_SPI){
				trans_size = DMA_LONG;
			}else if (c->cfg.slave_id == SLAVE_ID_UART){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_I2S){
				trans_size = DMA_LONG;
			}else if (c->cfg.slave_id == SLAVE_ID_UART_CA7){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_ADC){
				trans_size = DMA_LONG;
			}

			mtk_dma_chan_write(c, VDMA_CON, (dma_con | VDMA_RX_CON_RSIZE(trans_size)) );

			break;

		default:
			dev_err(c->vc.chan.device->dev,
				"\t\tNot support transfer direction(ch: %d, dir: %d)!!\n",
				c->dma_ch, c->cfg.direction);
			return;
	}
	dev_dbg(c->vc.chan.device->dev,
		"[%s](%s) vff_addr: 0x%08x, port_addr: 0x%08x, vff_len: 0x%08x, xfer_len: 0x%x, thres: 0x%08x\n",
		__func__,
		(c->cfg.direction == DMA_MEM_TO_DEV ? "TX" : "RX"),
		mtk_dma_chan_read(c, VDMA_MEM_ADDR),
		mtk_dma_chan_read(c, VDMA_PORT_ADDR),
		mtk_dma_chan_read(c, VDMA_FIFO_LEN),
		(desc_list->sg[desc_list->sg_idx].cb->length),
		mtk_dma_chan_read(c, VDMA_THRE));

	/* kick dma */
	mtk_dma_chan_write(c, VDMA_EN, EN_B);
}

static void mtk_set_cyclic_dma_param(struct mtk_chan *c)
{
	struct mtk_desc *desc_list = c->desc;
	unsigned int dma_con = 0, trans_size = 0;

	dev_dbg(c->vc.chan.device->dev, "[%s]DMA ch%d\n", __func__, c->dma_ch);
	if (!c || !c->desc || !c->desc->control_block_cyclic) {
	dev_err(c->vc.chan.device->dev, "\t[%s]\tnull mtk_chan pointer\n",
		__func__);
	return;
	}

	//if (mtk_set_vff_dma_param_check(c))
	//	return;

	/* clk enable */
	mtk_dma_clk_enable(c);
	/* warm reset DMA channel */
	mtk_dma_reset(c);

	switch (c->cfg.direction) {
		case DMA_MEM_TO_DEV:
			/* TX */
			/* src (mem) */
			mtk_dma_chan_write(c, VDMA_MEM_ADDR,
				desc_list->control_block_cyclic[desc_list->sg_idx].cb->src);
			/* dst (dev) */
			mtk_dma_chan_write(c, VDMA_PORT_ADDR,
				desc_list->control_block_cyclic[desc_list->sg_idx].cb->dst);
			/* len */
			mtk_dma_chan_write(c, VDMA_FIFO_LEN,
				(desc_list->control_block_cyclic[desc_list->sg_idx].cb->length * desc_list->sg_num));
			/* write pointer */
			mtk_dma_chan_write(c, VDMA_WPT, 0);
			/* int_en */
			mtk_dma_chan_write(c, VDMA_INT_EN, INT_EN_B);
			/* thres (tx_vff_left_size >= tx_vff_thre) ; (tx_vff_valid_size < tx_vff_thre) */
			mtk_dma_chan_write(c, VDMA_THRE, 4);

			/* con */
			/* HP default enable */
			dma_con = mtk_dma_chan_read(c, VDMA_CON);

			if (c->cfg.slave_id == SLAVE_ID_I2C){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_SPI){
				trans_size = DMA_LONG;
			}else if (c->cfg.slave_id == SLAVE_ID_UART){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_I2S){
				trans_size = DMA_LONG;
			}else if (c->cfg.slave_id == SLAVE_ID_UART_CA7){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_ADC){
				trans_size = DMA_LONG;
			}

			mtk_dma_chan_write(c, VDMA_CON, (dma_con | VDMA_TX_CON_WSIZE(trans_size)) );

			break;

		case DMA_DEV_TO_MEM:
			/* RX */
			/* src (dev) */
			mtk_dma_chan_write(c, VDMA_PORT_ADDR,
				desc_list->control_block_cyclic[desc_list->sg_idx].cb->src);
			/* dst (mem) */
			mtk_dma_chan_write(c, VDMA_MEM_ADDR,
				desc_list->control_block_cyclic[desc_list->sg_idx].cb->dst);
			/* len */
			mtk_dma_chan_write(c, VDMA_FIFO_LEN,
				(desc_list->control_block_cyclic[desc_list->sg_idx].cb->length * desc_list->sg_num));

			/* read pointer */
			mtk_dma_chan_write(c, VDMA_RPT, 0);
			/* int_en */
			mtk_dma_chan_write(c, VDMA_INT_EN, INT_EN_B);
			/* thres (rx_vff_valid_size >= rx_vff_thre) */
			mtk_dma_chan_write(c, VDMA_THRE,
				desc_list->control_block_cyclic[desc_list->sg_idx].cb->length);
			//mtk_dma_chan_write(c, VDMA_RX_FLOW_CTRL, 0x80);

			/* con */
			 /* HP default enable */
			dma_con = mtk_dma_chan_read(c, VDMA_CON);

			if (c->cfg.slave_id == SLAVE_ID_I2C){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_SPI){
				trans_size = DMA_LONG;
			}else if (c->cfg.slave_id == SLAVE_ID_UART){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_I2S){
				trans_size = DMA_LONG;
			}else if (c->cfg.slave_id == SLAVE_ID_UART_CA7){
				trans_size = DMA_BYTE;
			}else if (c->cfg.slave_id == SLAVE_ID_ADC){
				trans_size = DMA_LONG;
			}

			mtk_dma_chan_write(c, VDMA_CON, (dma_con | VDMA_RX_CON_RSIZE(trans_size)) );

			break;

		default:
			dev_err(c->vc.chan.device->dev,
				"\t\tNot support transfer direction(ch: %d, dir: %d)!!\n",
				c->dma_ch, c->cfg.direction);
			return;
	}
	dev_dbg(c->vc.chan.device->dev,
		"[%s](%s) vff_addr: 0x%08x, port_addr: 0x%08x, vff_len: 0x%08x, xfer_len: 0x%x, thres: 0x%08x\n",
		__func__,
		(c->cfg.direction == DMA_MEM_TO_DEV ? "TX" : "RX"),
		mtk_dma_chan_read(c, VDMA_MEM_ADDR),
		mtk_dma_chan_read(c, VDMA_PORT_ADDR),
		mtk_dma_chan_read(c, VDMA_FIFO_LEN),
		(desc_list->control_block_cyclic[desc_list->sg_idx].cb->length),
		mtk_dma_chan_read(c, VDMA_THRE));

	/* kick dma */
	mtk_dma_chan_write(c, VDMA_EN, EN_B);
}

static void mtk_set_peri_dma_param(struct mtk_chan *c)
{
	struct mtk_desc *desc_list = c->desc;
	unsigned int dma_con = 0, trans_size = 0;

	dev_dbg(c->vc.chan.device->dev, "[%s]\n", __func__);

	if (!c || !c->desc || !c->desc->sg) {
		dev_err(c->vc.chan.device->dev, "\t[%s]\tnull mtk_chan pointer\n",
		__func__);
	return;
	}

	/* clk enable */
	mtk_dma_clk_enable(c);
	/* Warm reset DMA channel */
	mtk_dma_reset(c);

	switch (c->cfg.direction) {
		case DMA_MEM_TO_DEV:
			/* TX*/
			/* src (mem) */
			mtk_dma_chan_write(c, DMA_SRC_ADDR,
				desc_list->sg[desc_list->sg_idx].cb->src);
			/* dst (dev) */
			mtk_dma_chan_write(c, DMA_DST_ADDR, c->cfg.dst_addr);

			/* len */
			mtk_dma_chan_write(c, DMA_LEN1,
				desc_list->sg[desc_list->sg_idx].cb->length);
			/* int_en */
			mtk_dma_chan_write(c, DMA_INT_EN, INT_EN_B);

			/* con */
			dma_con = mtk_dma_chan_read(c, DMA_CON);
			if (c->cfg.slave_id == SLAVE_ID_I2C){
				trans_size = DMA_BYTE;
				mtk_dma_chan_write(c, DMA_CONNECT, DMA_HANDSHAKE);
			}else if (c->cfg.slave_id == SLAVE_ID_SPI){
				trans_size = DMA_LONG;
				mtk_dma_chan_write(c, DMA_CONNECT, DMA_HANDSHAKE);
			}else if (c->cfg.slave_id == SLAVE_ID_UART){
				trans_size = DMA_BYTE;
			}
			mtk_dma_chan_write(c, DMA_CON, (dma_con | WADDR_FIX_EN | CON_WSIZE(trans_size)) );

			break;

		case DMA_DEV_TO_MEM:
			/* RX */
			/* src (dev) */
			mtk_dma_chan_write(c, DMA_SRC_ADDR, c->cfg.src_addr);
			/* dst (mem) */
			mtk_dma_chan_write(c, DMA_DST_ADDR,
			desc_list->sg[desc_list->sg_idx].cb->dst);
			/* len */
			mtk_dma_chan_write(c, DMA_LEN1,
			desc_list->sg[desc_list->sg_idx].cb->length);
			/* int_en */
			mtk_dma_chan_write(c, DMA_INT_EN, INT_EN_B);

			/* con */
			dma_con = mtk_dma_chan_read(c, DMA_CON);
			if (c->cfg.slave_id == SLAVE_ID_I2C){
				trans_size = DMA_BYTE;
				mtk_dma_chan_write(c, DMA_CONNECT, DMA_HANDSHAKE | DMA_DIR_DEV_TO_MEM);
			}else if (c->cfg.slave_id == SLAVE_ID_SPI){
				trans_size = DMA_LONG;
				mtk_dma_chan_write(c, DMA_CONNECT, DMA_HANDSHAKE);
			}else if (c->cfg.slave_id == SLAVE_ID_UART){
				trans_size = DMA_BYTE;
			}
			mtk_dma_chan_write(c, DMA_CON, (dma_con | RADDR_FIX_EN | CON_RSIZE(trans_size)) );

			break;

		default:
			dev_err(c->vc.chan.device->dev,
			"\t\tNot support transfer direction(ch: %d, dir: %d)!!\n",
			c->dma_ch, c->cfg.direction);
			return;
	}

	dev_dbg(c->vc.chan.device->dev,
		"[%s](%s) src_addr: 0x%08x, dst_addr: 0x%08x, len: 0x%08x, con: 0x%08x\n",
		__func__,
		(c->cfg.direction == DMA_MEM_TO_DEV ? "TX" : "RX"),
		mtk_dma_chan_read(c, DMA_SRC_ADDR),
		mtk_dma_chan_read(c, DMA_DST_ADDR),
		mtk_dma_chan_read(c, DMA_LEN1),
		mtk_dma_chan_read(c, DMA_CON));

	/* kick dma */
	mtk_dma_chan_write(c, DMA_EN, EN_B);
}

static void mtk_dma_set_m2m_param(struct mtk_chan *c)
{
	unsigned int src, dst, length;
	//struct mtk_dmadev *mtkd = to_mtk_dma_dev(c->vc.chan.device);
	printk(KERN_INFO "[%s]\n", __func__);

	if (!c || !c->desc || !c->desc->sg) {
		printk(KERN_INFO "[%s] null mtk_chan pointer\n", __func__);
		return;
	}

	src = c->desc->sg->cb->src;
	dst = c->desc->sg->cb->dst;
	length = c->desc->sg->cb->length;

	printk(KERN_INFO "[%s] src:0x%08x, dst:0x%08x, len:0x%08x\n",
		__func__, src, dst, length);

	/* clk enable */
	mtk_dma_clk_enable(c);
	/* Warm reset DMA channel */
	mtk_dma_reset(c);

	/* src addr */
	mtk_dma_chan_write(c, DMA_SRC_ADDR, c->desc->sg->cb->src);

	/* dst addr */
	mtk_dma_chan_write(c, DMA_DST_ADDR, c->desc->sg->cb->dst);
	/* len */
	mtk_dma_chan_write(c, DMA_LEN1, c->desc->sg->cb->length);
	/* con */
	mtk_dma_chan_write(c, DMA_CON, c->desc->sg->cb->info);
	/* int_en */
	mtk_dma_chan_write(c, DMA_INT_EN, INT_EN_B);

	/* kick dma */
	mtk_dma_chan_write(c, DMA_EN, EN_B);
}

static void mtk_dma_tasklet(unsigned long data)
{
	struct mtk_chan *c = (struct mtk_chan *)data;

	//mtk_dma_flush(c); // DMA will flush automatically

	if ((mtk_dma_chan_read(c, VDMA_RPT) ==
		mtk_dma_chan_read(c, VDMA_WPT))
		&& (mtk_dma_chan_read(c, VDMA_INT_BUF_SIZE) == 0)) {

		mtk_set_vchan_completion(c);

		dev_dbg(c->vc.chan.device->dev,
			"\t[%s]\tVDMA ch%d TX completes.\n", __func__, c->dma_ch);
	} else {
		dev_err(c->vc.chan.device->dev,
			"\t[%s]\tVDMA ch%d TX error happens!!\n", __func__, c->dma_ch);
	}
}

static void mtk_peri_dma_irq(struct mtk_chan *c)
{
	unsigned int irq_flag = mtk_dma_chan_read(c, DMA_INT_FLAG);

	if (irq_flag == 0)
		return;

	dev_dbg(c->vc.chan.device->dev, "[%s]DMA ch%d\n", __func__, c->dma_ch);

	if (irq_flag & INT_FLAG_B) {	/* Transaction Done, and W1C */
		mtk_dma_chan_write(c, DMA_INT_FLAG, INT_FLAG_B);

		mtk_set_vchan_completion(c);

		dev_dbg(c->vc.chan.device->dev,
			"\t[%s]\tPERI DMA ch%d %s completes.\n",
			__func__, c->dma_ch,
			(c->cfg.direction == DMA_MEM_TO_DEV ? "TX" : "RX"));
	}
}

static void mtk_vff_dma_irq(struct mtk_chan *c)
{
	unsigned int irq_flag = mtk_dma_chan_read(c, VDMA_INT_FLAG);

	if (irq_flag == 0)
		return;

	dev_dbg(c->vc.chan.device->dev, "[%s]VDMA ch%d\n", __func__, c->dma_ch);

	switch (c->cfg.direction) {
		case DMA_MEM_TO_DEV: /* TX-side */
			mtk_dma_chan_write(c, VDMA_INT_FLAG, irq_flag);	/* W1C */

			if (c->cfg.slave_id == SLAVE_ID_UART){
				if ((mtk_dma_chan_read(c, VDMA_RPT) ==
					mtk_dma_chan_read(c, VDMA_WPT))
					&& (mtk_dma_chan_read(c, VDMA_INT_BUF_SIZE) == 0)) {
					mtk_dma_stop(c);
					mtk_set_vchan_completion(c);
				}
			}
			else {
				tasklet_schedule(&c->task);
			}

			break;
		case DMA_DEV_TO_MEM: /* RX-side */
			mtk_dma_chan_write(c, VDMA_INT_FLAG, irq_flag); /* W1C */

			mtk_dma_chan_write(c, VDMA_INT_EN, 0);
			mtk_set_vchan_completion(c);

			dev_dbg(c->vc.chan.device->dev,
				"\t[%s]\tVDMA ch%d RX completes.\n", __func__, c->dma_ch);

			break;

		default:
			dev_err(c->vc.chan.device->dev,
				"\t[%s]\tNot support transfer direction(ch: %d, dir: %d)!!\n",
				__func__, c->dma_ch, c->cfg.direction);
			return;
	}
}

static void mtk_m2m_dma_irq(struct mtk_chan *c)
{
	unsigned int irq_flag = mtk_dma_chan_read(c, DMA_INT_FLAG);
	unsigned int ch = 0, reg = 0;

	if (irq_flag == 0)
		return;

	ch = c->dma_ch;
	dev_dbg(c->vc.chan.device->dev, "[%s]DMA ch%d\n", __func__, ch);

	reg = mtk_dma_chan_read(c, DMA_INT_FLAG);

	printk(KERN_DEBUG "[%s] DMA_INT_FLAG: 0x%08x(0x%08x)\n", __func__,
		(unsigned int)(c->channel_base + DMA_INT_FLAG), reg);

	if (reg & INT_FLAG_B) {
		//mtkd->ch_irq_sts |= (0x1 << (c->vc.chan.chan_id));
		mtk_dma_chan_write(c, DMA_INT_FLAG, INT_FLAG_B);

		dev_dbg(c->vc.chan.device->dev,
			"\t[%s]\tDMA ch%d completes.\n",
			__func__, c->dma_ch);

		vchan_cookie_complete(&c->desc->vd);
	}
}

static void mtk_cyclic_dma_irq(struct mtk_chan *c)
{
	struct mtk_dmadev *dev;
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);
	struct mtk_desc *desc_list = c->desc;
	unsigned int thresh = 0, length = 0, offset = 0;
	unsigned int reg = 0, ch = 0, flag = 0, inten= 0, pread = 0;


	dev_dbg(c->vc.chan.device->dev, "[%s]VDMA ch%d\n", __func__, c->dma_ch);

	dev = to_mtk_dma_dev(c->vc.chan.device);

	flag = 0;
	inten = 0;

	ch = c->dma_ch;
	flag = mtk_dma_chan_read(c, VDMA_INT_FLAG);
	inten = mtk_dma_chan_read(c, VDMA_INT_EN);
	pread = mtk_dma_chan_read(c, VDMA_RPT);

	if (((flag & VFF_THRE_INT_FLAG_B) == VFF_THRE_INT_FLAG_B) && ((inten & VFF_THRE_INT_EN_B) == VFF_THRE_INT_EN_B))
	{
		if (vd->tx.callback != NULL)
		{
			vd->tx.callback(vd->tx.callback_param);
		}
		reg = mtk_dma_chan_read(c, VDMA_INT_FLAG);

		printk(KERN_DEBUG "[%s] (Before)VDMA_INT_FLAG(THRE): 0x%08x(0x%08x)\n",
			__func__, (unsigned int)(c->channel_base+ VDMA_INT_FLAG), reg);

		// Update the read pointer and the read-wrap bit

		// threashold (our notification boundary) is the length of one frame. 
		thresh = desc_list->control_block_cyclic[desc_list->sg_idx].cb->length;
		length = thresh * desc_list->sg_num;

		// pread contains both the offset, and a "wrap" flag.  So we just 
		// pull off the offset in the low 16 bits
		offset = pread & 0xFFFFU; 

		// Update to the new offset 
		offset += thresh;

		// Check for wrap, if we wrapped, then toggle the "wrap" flag.
		if (offset >= length){
			offset -= length;
			pread ^= VDMA_RX_RPT_WRAP_B;
		}

		// mask-out the old offset, and or-in the new one. 
		pread &= ~0xFFFFU;
		pread |= offset & 0xFFFFU;

		// write the register back to the hardware
		mtk_dma_chan_write(c, VDMA_RPT, pread);

		// flag the interrupt as serviced
		mtk_dma_chan_write(c, VDMA_INT_FLAG, VFF_THRE_INT_FLAG_B);

		reg = mtk_dma_chan_read(c, VDMA_INT_FLAG);
		printk(KERN_DEBUG "[%s] (After)VDMA_INT_FLAG(THRE): 0x%08x(0x%08x)\n",
			__func__, (unsigned int)(c->channel_base + VDMA_INT_FLAG), reg);
	}

	//tasklet_schedule(&mtkd->tasklet);
}

static irqreturn_t mtk_dma_callback(int irq, void *data)
{
	struct mtk_dmadev *mtkd = (struct mtk_dmadev *)data;
	struct mtk_chan *c, *next;
	unsigned long flags;

	dev_dbg(mtkd->ddev.dev, "[%s]\n", __func__);

	list_for_each_entry_safe(c, next, &mtkd->ddev.channels,
		vc.chan.device_node) {

		if (c->channel_base == NULL){
			continue;
		}

		spin_lock_irqsave(&c->vc.lock, flags);

		if(c->dma_ch >= mtkd->m2m_ch_s && c->dma_ch <= mtkd->m2m_ch_e){ /* m2m */
			mtk_m2m_dma_irq(c);
		}else{
			switch (c->cfg.slave_id) {
				case SLAVE_ID_I2C:
				case SLAVE_ID_SPI:
					mtk_peri_dma_irq(c);
					break;
				case SLAVE_ID_UART:
				case SLAVE_ID_UART_CA7:
					mtk_vff_dma_irq(c);
					break;
				case SLAVE_ID_ADC:
				case SLAVE_ID_I2S:
					mtk_cyclic_dma_irq(c);
					break;
			}
		}
		spin_unlock_irqrestore(&c->vc.lock, flags);
	}
	return IRQ_HANDLED;
}

static int mtk_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct mtk_dmadev *mtkd = to_mtk_dma_dev(chan->device);
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	int ret = 0;
	unsigned long flags;

	dev_dbg(mtkd->ddev.dev, "[%s]\n", __func__);

	c->dma_ch = chan->chan_id;

	c->channel_base = MTK3620_CA7DMA_CHANIO(mtkd->base, c->dma_ch);

//	ret = request_irq(c->irq_number, mtk_dma_callback,
//		IRQF_TRIGGER_NONE, (const char *)c->irq_name, (void *)mtkd);
	ret = request_irq(c->irq_number, mtk_dma_callback,
		IRQF_SHARED | IRQF_TRIGGER_NONE, (const char *)c->irq_name, (void *)mtkd);

	if (ret) {
		dev_err(mtkd->ddev.dev, "Fail to request IRQ\n");
	}

	tasklet_init(&c->task, mtk_dma_tasklet, (unsigned long)c);
	//tasklet_init(&mtkd->tasklet, mtk3620_dma_do_tasklet, (unsigned long) mtkd);

	spin_lock_irqsave(&c->vc.lock, flags);

	dma_cookie_init(chan);

	spin_unlock_irqrestore(&c->vc.lock, flags);

	dev_dbg(mtkd->ddev.dev, "[%s]DMA ch%d, ch_base(0x%x), irq = %d\n",
		__func__, c->dma_ch, (unsigned int)c->channel_base, c->irq_number);

	return ret;
}

static void mtk_dma_free_chan_resources(struct dma_chan *chan)
{
	struct mtk_dmadev *mtkd = to_mtk_dma_dev(chan->device);
	struct mtk_chan *c = to_mtk_dma_chan(chan);

	free_irq(c->irq_number, mtkd);

	tasklet_kill(&c->task);

	c->channel_base = NULL;

	vchan_free_chan_resources(&c->vc);

	dev_dbg(mtkd->ddev.dev, "[%s]DMA ch%d.\n",
		__func__, c->dma_ch);
}

static size_t mtk_dma_desc_size(struct mtk_chan *c)
{
	size_t size = 0;

	if (c->cfg.direction == DMA_MEM_TO_DEV)
		size = mtk_dma_chan_read(c, DMA_LEN1); /* TX */
	else
		size = mtk_dma_chan_read(c, DMA_LEN1); /* RX */

	return size;
}

static enum dma_status mtk_dma_tx_status(	struct dma_chan *chan,
												dma_cookie_t cookie,
												struct dma_tx_state *txstate)
{
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	struct virt_dma_desc *vd;
	enum dma_status ret;
	unsigned long flags;
	unsigned int size, ptr, tmp;
	//dma_addr_t dma;

	ret = dma_cookie_status(chan, cookie, txstate);

	dev_dbg(chan->device->dev, "[%s]DMA ch%d is %s, cookie = %d\n", __func__,
		c->dma_ch, ((ret == 0) ? "completion" : "on process"), cookie);

	spin_lock_irqsave(&c->vc.lock, flags);

	if (c->cfg.slave_id == SLAVE_ID_UART || c->cfg.slave_id == SLAVE_ID_ADC) {
		size = mtk_dma_chan_read(c, VDMA_VALID_SIZE);

		if (size == 0) {
			/* reset reside for UART */
			txstate->residue = mtk_dma_chan_read(c, VDMA_FIFO_LEN);
			spin_unlock_irqrestore(&c->vc.lock, flags);
			return ret;
		}

		if (c->cfg.direction == DMA_DEV_TO_MEM){ // RX
			txstate->residue = mtk_dma_chan_read(c, VDMA_FIFO_LEN) - size;

			ptr = mtk_dma_chan_read(c, VDMA_RPT);
			tmp = (ptr & (VDMA_RX_RPT_WRAP_B - 1)) +
				size;

			if (tmp >= mtk_dma_chan_read(c, VDMA_FIFO_LEN)) {
				tmp -= mtk_dma_chan_read(c, VDMA_FIFO_LEN);
				ptr = tmp |
				((ptr & VDMA_RX_RPT_WRAP_B)^(VDMA_RX_RPT_WRAP_B));
			} else {
				ptr += size;
			}
			mtk_dma_chan_write(c, VDMA_RPT, ptr);
		} else {
			txstate->residue = mtk_dma_chan_read(c, VDMA_LEFT_SIZE);
		}
	} else {

		if (ret != DMA_COMPLETE && (txstate)){
			vd = vchan_find_desc(&c->vc, cookie);
			if(vd) {
				txstate->residue = mtk_dma_desc_size(c);
			}
			else {
				txstate->residue = 0;
			}
		}
	}

	spin_unlock_irqrestore(&c->vc.lock, flags);

	dev_dbg(chan->device->dev, "\t[%s]\tResidue size = 0x%x\n",
		__func__, txstate->residue);

	return ret;
}

static void mtk_dma_start_desc(struct mtk_chan *c)
{
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);
	struct mtk_desc *d;
	struct mtk_dmadev *dev;

	if (!vd) {
		c->desc = NULL;
		return;
	}

	c->desc = d = to_mtk_dma_desc(&vd->tx);

	dev = to_mtk_dma_dev(c->vc.chan.device);

	dev_dbg(c->vc.chan.device->dev, "[%s]DMA (ch%d,m2m_ch_s:%d,m2m_ch_e:%d)",
		__func__, c->dma_ch, dev->m2m_ch_s, dev->m2m_ch_e);

	if (c->dma_ch >= dev->m2m_ch_s && c->dma_ch <= dev->m2m_ch_e){ /* m2m */
		mtk_dma_set_m2m_param(c);
	}else {
		switch (c->cfg.slave_id) { /* slave id */
			case SLAVE_ID_I2C:
			case SLAVE_ID_SPI:
				mtk_set_peri_dma_param(c);
				break;
			case SLAVE_ID_UART:
			case SLAVE_ID_UART_CA7:
				mtk_set_vff_dma_param(c);
				break;
			case SLAVE_ID_ADC:
			case SLAVE_ID_I2S:
				mtk_set_cyclic_dma_param(c);
				break;
		}
	}
}

static inline size_t mtk_dma_frames_for_length(size_t len, size_t max_len)
{
	return DIV_ROUND_UP(len, max_len);
}

static inline size_t mtk_dma_count_frames_for_sg(
	struct mtk_chan *c,
	struct scatterlist *sgl,
	unsigned int sg_len)
{
	size_t frames = 0;
	struct scatterlist *sgent;
	unsigned int i, max_len, len;

	switch (c->cfg.slave_id) { /* slave id */
		case SLAVE_ID_I2C:
			max_len = MAX_DMA_I2C_LEN;
			break;
		case SLAVE_ID_SPI:
			max_len = MAX_DMA_SPI_LEN;
			break;
		case SLAVE_ID_UART:
		case SLAVE_ID_UART_CA7:
			max_len = MAX_DMA_UART_LEN;
			break;
		case SLAVE_ID_I2S:
			//max_len
			break;
		case SLAVE_ID_ADC:
			max_len = MAX_DMA_ADC_LEN;
			break;
	}

	for_each_sg(sgl, sgent, sg_len, i){
	/* for  UART-TX or I2C-RX */
	if (((c->cfg.slave_id == SLAVE_ID_UART) && (c->cfg.direction == DMA_MEM_TO_DEV)) ||
		(c->cfg.slave_id == SLAVE_ID_SPI) ||
		(c->cfg.slave_id == SLAVE_ID_I2C) || ((c->cfg.slave_id == SLAVE_ID_ADC) && (c->cfg.direction == DMA_DEV_TO_MEM))){

		len = sg_dma_len(sgent) & 0xffffl;
	}
		frames += mtk_dma_frames_for_length(len, max_len);
	}
	return frames;
}

static struct dma_async_tx_descriptor *mtk_dma_prep_dma_memcpy(
	struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
	size_t len, unsigned long flags)
{
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	struct mtk_desc *d;
	struct mtk_dma_cb *control_block;
	struct mtk_sg *sg;

	unsigned int info;
	size_t max_len = MAX_DMA_LEN;
	size_t frames;

	printk(KERN_INFO "[%s]\n", __func__);

	if(!src || !dst || !len)
		return NULL;

	frames = mtk_dma_frames_for_length(len, max_len);

	if (!frames)
		return NULL;

	/* Allocate and setup the descriptor */
	d = kzalloc(sizeof(*d) + frames * sizeof(struct mtk_sg), GFP_KERNEL);

	if (!d)
		return NULL;

	/* Avoid memory leakage (find a place to free them) */
	control_block = (struct mtk_dma_cb *) kzalloc(sizeof(struct mtk_dma_cb), GFP_KERNEL);
	if (!control_block)
		return NULL;

	sg = (struct mtk_sg *) ((void *) d + sizeof(*d));
	sg->cb = control_block;

	d->c = c;
	d->dir = DMA_MEM_TO_MEM;
	d->sg = sg;
	d->sg_idx= 0;
	d->sg_num = frames;

	control_block->info = info;
	control_block->src = src;
	control_block->dst = dst;
	control_block->length = min_t(u32, len, max_len);
	control_block->next = 0;

	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static struct dma_async_tx_descriptor *mtk_dma_prep_slave_sg(
					struct dma_chan *chan,
					struct scatterlist *sgl,
					unsigned int sglen,
					enum dma_transfer_direction	dir,
					unsigned long tx_flags,
					void *context)
{
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	struct scatterlist *sg;
	struct mtk_desc *d;
	struct mtk_dma_cb *control_block;
	unsigned int count, frames, max_len = 0;

	dev_dbg(chan->device->dev, "[%s]DMA ch%d", __func__, c->dma_ch);

	frames = mtk_dma_count_frames_for_sg(c, sgl, sglen);

	switch (c->cfg.slave_id) { /* slave id */
		case SLAVE_ID_I2C:
			max_len = MAX_DMA_I2C_LEN;
			break;
		case SLAVE_ID_SPI:
			max_len = MAX_DMA_SPI_LEN;
			break;
		case SLAVE_ID_UART:
		case SLAVE_ID_UART_CA7:
			max_len = MAX_DMA_UART_LEN;
			break;
		case SLAVE_ID_I2S:
			//max_len
			break;
		case SLAVE_ID_ADC:
			max_len = MAX_DMA_ADC_LEN;
			break;
	}

	if ((frames == 0) || (frames > 1)) {	/* Only support frames = 1. */
		dev_err(chan->device->dev,
			"\t\tSG length(%d) > max dma support length(%d).\n",
		sg_dma_len(sgl), max_len);
		return NULL;
	}

	/* allocate and setup the descriptor */
	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return NULL;

	/* c->desc = d; */
	d->c = c;
	d->sg_idx = 0;
	d->sg_num = frames;

	d->sg = kcalloc(d->sg_num, sizeof(*d->sg), GFP_KERNEL);

	if (!d->sg) {
		kfree(d);
		return NULL;
	}

	sg = sgl;

	for (count = 0; count < frames; count++) {
		control_block = kzalloc(sizeof(struct mtk_dma_cb), GFP_KERNEL);

		if (!control_block)
			goto error_sg;


		/* DMA_DEV_TO_MEM, DMA_RX */
		if (c->cfg.direction == DMA_DEV_TO_MEM) {
			control_block->src = c->cfg.src_addr;

			control_block->dst = sg_dma_address(sg);
		} else {	/* DMA_MEM_TO_DEV, DMA_TX */
			control_block->src = sg_dma_address(sg);

			control_block->dst = c->cfg.dst_addr;
		}
		control_block->next = 0;

		d->sg[count].cb = control_block;

		control_block->length = sg_dma_len(sg);

		sg = sg_next(sg);
	}
	return vchan_tx_prep(&c->vc, &d->vd, tx_flags);

error_sg:
	mtk_dma_desc_free(&d->vd);
	return NULL;
}


static struct dma_async_tx_descriptor *mtk_dma_prep_dma_cyclic(
								struct dma_chan *chan,
								dma_addr_t buf_addr,
								size_t buf_len,
								size_t period_len,
								enum dma_transfer_direction direction,
								unsigned long flags)
{
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	enum dma_slave_buswidth dev_width;
	struct mtk_desc *d;
	struct mtk_dma_cb *control_block;
	dma_addr_t dev_addr;
	unsigned int frame;

	
	dev_dbg(chan->device->dev, "[%s]DMA ch%d",
		__func__, c->dma_ch);

	if (!buf_len || !period_len) {
		dev_err(chan->device->dev, "[%s] Invalid buffer/period len\n",
			__func__);
		return NULL;
	}

	/* Only support cycle transfer when buf_len is multiple of period_len. */
	if (buf_len % period_len) {
		dev_err(chan->device->dev, "[%s] buf_len is not multiple of period_len\n", __func__);
		return NULL;
	}


	/* Grab configuration */
	if (!is_slave_direction(direction)) {
		dev_err(chan->device->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	if (direction == DMA_DEV_TO_MEM) { /* RX */
		dev_addr = c->cfg.src_addr;
		dev_width = c->cfg.src_addr_width;
	} else { /* TX */
		dev_addr = c->cfg.dst_addr;
		dev_width = c->cfg.dst_addr_width;
	}

	/* Now allocate and setup the descriptor. */
	d = kzalloc(sizeof(*d), GFP_NOWAIT);
	if (!d)
		return NULL;

	d->c = c;
	d->dir = direction;
	d->sg_idx = 0;
	d->sg_num = buf_len / period_len;

	d->control_block_cyclic = kcalloc(d->sg_num, sizeof(*d->control_block_cyclic), GFP_KERNEL);
	if (!d->control_block_cyclic) {
		kfree(d);
		return NULL;
	}

	/* Create a control block for each frame and link them together. */
	for (frame = 0; frame < d->sg_num; frame++) {
		/* Allocate memory for control blocks */
		//struct mtk_dma_cb *control_block = &d->control_block_base[frame];
		control_block = kzalloc(sizeof(struct mtk_dma_cb), GFP_KERNEL);

		if (!control_block)
			goto error_cyclic;

		/* Setup adresses */
		if (d->dir == DMA_DEV_TO_MEM) { /* DMA_DEV_TO_MEM - RX */
			control_block->src = dev_addr;
			control_block->dst = buf_addr + frame * period_len;
		} else { /* DMA_MEM_TO_DEV - TX */
			control_block->src = buf_addr + frame * period_len;
			control_block->dst = dev_addr;
		}

		/* Length of a frame */
		control_block->length = period_len;
		//d->size += control_block->length;

		control_block->next = 0;

		d->control_block_cyclic[frame].cb = control_block;
	}
	return vchan_tx_prep(&c->vc, &d->vd, flags);
error_cyclic:
	mtk_dma_desc_free(&d->vd);

	return NULL;
}

static void mtk_dma_issue_pending(struct dma_chan *chan)
{
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	unsigned long flags;

	dev_dbg(chan->device->dev, "[%s]DMA ch%d issues.\n",
		__func__, c->dma_ch);

	spin_lock_irqsave(&c->vc.lock, flags);

	if (vchan_issue_pending(&c->vc) && !c->desc)
		mtk_dma_start_desc(c);

	spin_unlock_irqrestore(&c->vc.lock, flags);
}

static int mtk_dma_slave_config(struct dma_chan *chan,
                                struct dma_slave_config *cfg)
{
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	int ret = 0;

	dev_dbg(chan->device->dev, "[%s]DMA ch%d", __func__, c->dma_ch);

	if (cfg->slave_id >= SLAVE_ID_NONE) {
		dev_err(chan->device->dev,
			"\t\tCA7DMA not support this slave(slave id = %d)!!\n",
			c->cfg.slave_id);

		ret = -EPERM;

		goto exit;
	}

	if (cfg->slave_id == SLAVE_ID_I2S || cfg->slave_id == SLAVE_ID_ADC){
		if ((cfg->direction == DMA_DEV_TO_MEM &&
			cfg->src_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES) ||
			(cfg->direction == DMA_MEM_TO_DEV &&
			cfg->dst_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES) ||
			!is_slave_direction(cfg->direction)) {
			return -EINVAL;
		}
	}

	memcpy(&c->cfg, cfg, sizeof(c->cfg));

	dev_dbg(chan->device->dev,
		"[%s] Slave of CA7DMA(ch%d) is %d(%s)!!\n",
		__func__,
		c->dma_ch,
		c->cfg.slave_id,
		c->cfg.direction == DMA_MEM_TO_DEV ? "TX" : "RX");
exit:
	return ret;
}

static int mtk_dma_terminate_all(struct dma_chan *chan)
{
	struct mtk_chan *c = to_mtk_dma_chan(chan);
	struct mtk_dmadev *d = to_mtk_dma_dev(c->vc.chan.device);
	unsigned long flags;
	LIST_HEAD(head);

	dev_dbg(chan->device->dev, "[%s]DMA ch%d",
		__func__, c->dma_ch);

	spin_lock_irqsave(&c->vc.lock, flags);

	/* Prevent this channel being scheduled */
	spin_lock(&d->lock);

	list_del_init(&c->node);

	spin_unlock(&d->lock);

	if (c->desc) {
		mtk_dma_stop(c);
	}
	vchan_get_all_descriptors(&c->vc, &head);

	spin_unlock_irqrestore(&c->vc.lock, flags);

	vchan_dma_desc_free_list(&c->vc, &head);

	return 0;
}

static int mtk_dma_pause(struct dma_chan *chan)
{
	/* Pause/Resume only allowed with cyclic mode */
	return -EINVAL;
}

static int mtk_dma_resume(struct dma_chan *chan)
{
	/* Pause/Resume only allowed with cyclic mode */
	return -EINVAL;
}

static struct mtk_chan *mtk_dma_chan_init(struct mtk_dmadev *mtkd)
{
	struct mtk_chan *c;

	c = devm_kzalloc(mtkd->ddev.dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return NULL;

	c->vc.desc_free = mtk_dma_desc_free;

	vchan_init(&c->vc, &mtkd->ddev);

	spin_lock_init(&c->lock);

	INIT_LIST_HEAD(&c->node);

	return c;
}

static void mtk_dma_free(struct mtk_dmadev *mtkd)
{
	while (!list_empty(&mtkd->ddev.channels)) {
		struct mtk_chan *c = list_first_entry(&mtkd->ddev.channels,
		struct mtk_chan, vc.chan.device_node);

		list_del(&c->vc.chan.device_node);

		tasklet_kill(&c->vc.task);

		devm_kfree(mtkd->ddev.dev, c);
	}
}

static const struct of_device_id mtk3620_ca7dma_of_match[] = {
	{ .compatible = "mediatek,ca7dma", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mtk3620_ca7dma_of_match);

static int mtk_dma_probe(struct platform_device *pdev)
{
	struct mtk_dmadev *mtkd;
	struct resource *res;

	unsigned int dma_num = 0;
	unsigned int m2m_ch_s = 0, m2m_ch_e = 0, m2m_ch_num = 0;
	unsigned int peri_ch_s = 0, peri_ch_e = 0, peri_ch_num = 0;
	unsigned int vff_ch_s = 0, vff_ch_e = 0, vff_ch_num = 0;
	int rc, i;
	struct mtk_chan *c;
	const char *s;

	/* Set the dma mask bits */
	rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	/* Check DMA Probe */
	dev_dbg(&pdev->dev, "[%s] Starting mt3620 CA7DMA driver", __func__);

	/* DMA channel num from DTS */
	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-dma-num",
			&dma_num)) {
		dev_err(&pdev->dev, "Failed to get dma channel count\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "DMA channels amount: %d \n", dma_num);
	/*DMA-M2M */
	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-m2m-ch-s",
			&m2m_ch_s)) {
		dev_err(&pdev->dev, "Failed to get m2m channel starting num\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "DMA channels (m2m_ch_s #%d) \n", m2m_ch_s);

	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-m2m-ch-e",
			&m2m_ch_e)) {
		dev_err(&pdev->dev, "Failed to get m2m channel end num\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "DMA channels (m2m_ch_e #%d) \n", m2m_ch_e);

	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-m2m-ch-num",
			&m2m_ch_num)) {
		dev_err(&pdev->dev, "Failed to get m2m dma channel count\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "m2m DMA channels amount: %d \n", m2m_ch_num);
	/* DMA-PERI */
	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-peri-ch-s",
			&peri_ch_s)) {
		dev_err(&pdev->dev, "Failed to get peri channel starting num\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "DMA channels (peri_ch_s #%d) \n", peri_ch_s);

	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-peri-ch-e",
			&peri_ch_e)) {
		dev_err(&pdev->dev, "Failed to get peri channel end num\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "DMA channels (peri_ch_e #%d) \n", peri_ch_e);

	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-peri-ch-num",
			&peri_ch_num)) {
		dev_err(&pdev->dev, "Failed to get peri dma channel count\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "peri DMA channels amount: %d \n", peri_ch_num);
	/* DMA-VFF */
	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-vff-ch-s",
			&vff_ch_s)) {
		dev_err(&pdev->dev, "Failed to get vff channel starting num\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "DMA channels (vff_ch_s #%d) \n", vff_ch_s);

	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-vff-ch-e",
			&vff_ch_e)) {
		dev_err(&pdev->dev, "Failed to get vff channel end num\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "DMA channels (vff_ch_e #%d) \n", vff_ch_e);

	if (of_property_read_u32(pdev->dev.of_node,
			"mediatek,ca7dma-vff-ch-num",
			&vff_ch_num)) {
		dev_err(&pdev->dev, "Failed to get vff dma channel count\n");
		return -EINVAL;
	}
	dev_dbg(&pdev->dev, "vff DMA channels amount: %d \n", vff_ch_num);


	/* Allocate memory and initialize the DMA engine */
	mtkd = devm_kzalloc(&pdev->dev, sizeof(*mtkd)*(dma_num), GFP_KERNEL);
	if (!mtkd)
		return -ENOMEM;

	/* Request and map I/O memory */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	/* Get DMA base address from DTS */
	mtkd->base = devm_ioremap_resource(&pdev->dev, res);

	dev_dbg(&pdev->dev, "DMA mtkd->base: 0x%p \n", mtkd->base);
	/* EXPORT API for I2S */
	mtk_set_dma_base_adress(mtkd->base);
	if (IS_ERR(mtkd->base))
		return PTR_ERR(mtkd->base);

	/* memory transfer */
	dma_cap_set(DMA_MEMCPY, mtkd->ddev.cap_mask);
	/* Slave */
	dma_cap_set(DMA_SLAVE, mtkd->ddev.cap_mask);
	dma_cap_set(DMA_SLAVE, mtk_dma_info.dma_cap);
	/* Cyclic */
	dma_cap_set(DMA_CYCLIC, mtkd->ddev.cap_mask);
	dma_cap_set(DMA_CYCLIC, mtk_dma_info.dma_cap);

	/* Must be probed for DMA */
	mtkd->ddev.device_alloc_chan_resources = mtk_dma_alloc_chan_resources;
	mtkd->ddev.device_free_chan_resources = mtk_dma_free_chan_resources;
	mtkd->ddev.device_tx_status = mtk_dma_tx_status;
	mtkd->ddev.device_issue_pending = mtk_dma_issue_pending;

	/* For memory to memory copy */
	if (dma_has_cap(DMA_MEMCPY, mtkd->ddev.cap_mask)) {
		mtkd->ddev.device_prep_dma_memcpy = mtk_dma_prep_dma_memcpy;
	}
	/* For memory to memory scatter gather */
	if (dma_has_cap(DMA_SG, mtkd->ddev.cap_mask)){
		//mtkd->ddev.device_prep_dma_sg = mtk_prep_memcpy_sg;
	}
	/* For Slave */
	if (dma_has_cap(DMA_SLAVE, mtkd->ddev.cap_mask)) {
		mtkd->ddev.device_prep_slave_sg = mtk_dma_prep_slave_sg;
	}
	/* For Cyclic */
	if (dma_has_cap(DMA_CYCLIC, mtkd->ddev.cap_mask)) {
		mtkd->ddev.device_prep_dma_cyclic = mtk_dma_prep_dma_cyclic;
	}

	mtkd->ddev.device_config = mtk_dma_slave_config;

	/* Additional flag. */
	/* pauses a given channel (DMA_PUASE) */
	mtkd->ddev.device_pause = mtk_dma_pause;
	/* resumes a given channel (DMA_RESUME) */
	mtkd->ddev.device_resume = mtk_dma_resume;
	/* abort all transfers on a given channel */
	mtkd->ddev.device_terminate_all = mtk_dma_terminate_all;

	mtkd->ddev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	mtkd->ddev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	mtkd->ddev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) | BIT(DMA_MEM_TO_MEM);
	mtkd->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	mtkd->ddev.dev = &pdev->dev;

	INIT_LIST_HEAD(&mtkd->ddev.channels);

	spin_lock_init(&mtkd->lock);

	platform_set_drvdata(pdev, mtkd);

	mtkd->dma_num = dma_num;
	mtkd->m2m_ch_s = m2m_ch_s;
	mtkd->m2m_ch_e = m2m_ch_e;
	mtkd->m2m_ch_num = m2m_ch_num;
	mtkd->peri_ch_s = peri_ch_s;
	mtkd->peri_ch_e = peri_ch_e;
	mtkd->peri_ch_num = peri_ch_num;
	mtkd->vff_ch_s = vff_ch_s;
	mtkd->vff_ch_e = vff_ch_e;
	mtkd->vff_ch_num = vff_ch_num;

	for(i = 0; i < (m2m_ch_num+peri_ch_num+vff_ch_num); i++){
		c = mtk_dma_chan_init(mtkd);

		if (!c) {
			dev_err(&pdev->dev, "[%s]\tmtk_dma_chan_init function fails(i = %d)!!\n",
			__func__, i);
			goto err_no_dma;
		}

		c->irq_number = platform_get_irq(pdev, i);

		mtkd->ch_irq[i] = c->irq_number;

		dev_dbg(&pdev->dev,"[%s] i = %d, irq_num = %d\n",
			__func__, i, mtkd->ch_irq[i]);

		if (mtkd->ch_irq[i] < 0) {
			dev_err(&pdev->dev, "[%s] Cannot claim IRQ\n", __func__);

			rc = mtkd->ch_irq[i];

			goto err_no_dma;
		}
		if (of_property_read_string_index
			(pdev->dev.of_node, "interrupt-names", i, &s))
			continue;
		else {
			strcpy(c->irq_name, s);
		}
	}

	dev_dbg(&pdev->dev, "[%s] Initialized %d DMA channels\n",
		__func__, (mtkd->dma_num));

	/* Device-tree DMA controller registration */
	rc = of_dma_controller_register(pdev->dev.of_node,
		of_dma_simple_xlate, &mtk_dma_info);

	if (rc) {
		dev_err(&pdev->dev, "Failed to register DMA controller\n");
		goto err_no_dma;
	}
	dev_dbg(&pdev->dev, "Register DMA controller successfully...\n");

	/* Register the DMA engine with the core */
	rc = dma_async_device_register(&mtkd->ddev);

	if (rc) {
		dev_err(&pdev->dev,
			"Failed to register slave DMA engine device: %d\n", rc);
		goto err_no_dma;
	}

	dev_dbg(&pdev->dev, "Load  CA7DMA-MTK engine driver(phys: 0x%x, mmio: 0x%p, size: 0x%x)\n",
				res->start, mtkd->base, (res->end - res->start + 1));
	return 0;

err_no_dma:
	dev_err(&pdev->dev, "Fail Load CA7DMA-MTK engine driver !!\n");

	mtk_dma_free(mtkd);

	devm_kfree(&pdev->dev, mtkd);

	return rc;
}

static int mtk_dma_remove(struct platform_device *pdev)
{
	struct mtk_dmadev *mtkd = platform_get_drvdata(pdev);

	if (pdev->dev.of_node)
	of_dma_controller_free(pdev->dev.of_node);

	/* Disable DMA clock */
	//mtk_dma_clk_disable(mtkd);

	dma_async_device_unregister(&mtkd->ddev);

	mtk_dma_free(mtkd);

	return 0;
}

static struct platform_driver mtk_dma_driver = {
	.probe	= mtk_dma_probe,
	.remove	= mtk_dma_remove,
	.driver = {
		.name = "ca7dma-mtk",
		.of_match_table = of_match_ptr(mtk3620_ca7dma_of_match),
	},
};

static bool mtk_dma_filter_fn(struct dma_chan *chan, void *param)
{
	int req = *(int *)param;

	if (chan->chan_id == req)
		return true;
	else
		return false;
}

static int mtk_dma_init(void)
{
	return platform_driver_register(&mtk_dma_driver);
}
subsys_initcall(mtk_dma_init);

static void __exit mtk_dma_exit(void)
{
	platform_driver_unregister(&mtk_dma_driver);
}
module_exit(mtk_dma_exit);

MODULE_DESCRIPTION("MediaTek MTK CA7DMA Controller Driver");
MODULE_LICENSE("GPL v2");
