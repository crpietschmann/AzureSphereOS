// SPDX-License-Identifier: GPL-2.0
/*
 * ADC driver for IIO for the mt3620.  This code was based originally
 * on the mt6577 auxadc driver (mt3620_auxadc.c) and modified for the mt3620.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 * Place - Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <stdbool.h>

#define DMA_BUFFER_MAX_SIZE	SZ_2K
#define DMA_SLAVE_ID_ADC	5
#define DMA_PERIOD_MIN_SIZE	32
#define MT3620_ADC_SAMPLE_SIZE	4
#define MT3620_ADC_BITS_PER_SAMPLE 12

/* CTL Register definitions */
#define MT3620_ADC_CTL0 0x0
#define MT3620_ADC_CTL1 0x4
#define MT3620_ADC_CTL2 0x8
#define MT3620_ADC_CTL3 0xc
#define MT3620_ADC_CTL4 0x10

/* CTL FIELD Definitions */
#define MT3620_ADC_CTL0_REG_CH_MAP		GENMASK(31, 16)
#define MT3620_ADC_CTL0_REG_T_INIT		GENMASK(15, 9)
#define MT3620_ADC_CTL0_REG_T_CH		GENMASK(7, 4)
#define MT3620_ADC_CTL0_PMODE_EN		BIT(8)
#define MT3620_ADC_CTL0_REG_AVG_MODE		GENMASK(3, 1)
#define MT3620_ADC_CTL0_ADC_FSM_EN		BIT(0)
#define MT3620_ADC_CTL2_REG_ADC_TIMESTAMP_EN	BIT(21)
#define MT3620_ADC_CTL3_VREF_MODE		BIT(31)
#define MT3620_ADC_CTL3_CLK_SRC			BIT(17)
#define MT3620_ADC_CTL3_CLK_INV_PMU		BIT(16)
#define MT3620_ADC_CTL3_CLK_PHASE		BIT(15)
#define MT3620_ADC_CTL3_VCM			BIT(13)
#define MT3620_ADC_CTL3_INPUT_MUX		BIT(11)
#define MT3620_ADC_CTL3_DITHER_STEP		GENMASK(9, 8)
#define MT3620_ADC_CTL3_DITHER			BIT(6)
#define MT3620_ADC_CTL3_COMP_PREAMP		BIT(4)
#define MT3620_ADC_CTL3_COMP_PREAMP_CURR	GENMASK(3, 2)
#define MT3620_ADC_CTL3_COMP_DELAY		GENMASK(1, 0)

/* CTL Field Values/Enums */
#define MT3620_ADC_CTL0_REG_AVG_MODE_1		0
#define MT3620_ADC_CTL0_REG_AVG_MODE_2		1
#define MT3620_ADC_CTL0_REG_AVG_MODE_4		2
#define MT3620_ADC_CTL0_REG_AVG_MODE_8		3
#define MT3620_ADC_CTL0_REG_AVG_MODE_16		4
#define MT3620_ADC_CTL0_REG_AVG_MODE_32		5
#define MT3620_ADC_CTL0_REG_AVG_MODE_64		6
#define MT3620_ADC_CTL0_REG_AVG_MODE_64_ALT	7

#define MT3620_ADC_CTL2_REG_ADC_TIMESTAMP_EN_ENABLE	1

#define MT3620_ADC_CTL3_VREF_MODE_2V5		0
#define MT3620_ADC_CTL3_VREF_MODE_1V8		1
#define MT3620_ADC_CTL3_CLK_SRC_ADC		0
#define MT3620_ADC_CTL3_CLK_SRC_PMU		1
#define MT3620_ADC_CTL3_CLK_INV_PMU_ORIGINAL	0
#define MT3620_ADC_CTL3_CLK_INV_PMU_INVERT	1
#define MT3620_ADC_CTL3_CLK_PHASE_ENABLE	1
#define MT3620_ADC_CTL3_CLK_PHASE_DISABLE	0
#define MT3620_ADC_CTL3_VCM_ENABLE		1
#define MT3620_ADC_CTL3_VCM_DISABLE		0
#define MT3620_ADC_CTL3_INPUT_MUX_ENABLE	1
#define MT3620_ADC_CTL3_INPUT_MUX_DISABLE	0
#define MT3620_ADC_CTL3_DITHER_STEP_2		0
#define MT3620_ADC_CTL3_DITHER_STEP_4		1
#define MT3620_ADC_CTL3_DITHER_STEP_8		2
#define MT3620_ADC_CTL3_DITHER_STEP_16		3
#define MT3620_ADC_CTL3_DITHER_ENABLE		1
#define MT3620_ADC_CTL3_DITHER_DISABLE		0
#define MT3620_ADC_CTL3_COMP_PREAMP_ENABLE	1
#define MT3620_ADC_CTL3_COMP_PREAMP_DISABLE	0
#define MT3620_ADC_CTL3_COMP_PREAMP_CURR_40UA	0
#define MT3620_ADC_CTL3_COMP_PREAMP_CURR_80UA	1
#define MT3620_ADC_CTL3_COMP_PREAMP_CURR_160UA	2
#define MT3620_ADC_CTL3_COMP_DELAY_3NS		0
#define MT3620_ADC_CTL3_COMP_DELAY_6NS		1
#define MT3620_ADC_CTL3_COMP_DELAY_9NS		2
#define MT3620_ADC_CTL3_COMP_DELAY_12NS		3

/* FIFO Register Deffinitions */
#define MT3620_ADC_FIFO_RBR		0x100
#define MT3620_ADC_FIFO_IER		0x104
#define MT3620_ADC_FIFO_FIFOCTRL	0x108
#define MT3620_ADC_FIFO_ADC_FIFO_LSR	0x114
#define MT3620_ADC_FIFO_DMA_EN		0x14C
#define MT3620_ADC_FIFO_TRI_LVL		0x160
#define MT3620_ADC_FIFO_DEBUG16		0x1D4

/* FIFO Field Definitions */
#define MT3620_ADC_FIFO_RBR_ADC_RBR_CHANNEL	GENMASK(3, 0)
#define MT3620_ADC_FIFO_RBR_ADC_RBR_SAMPLE	GENMASK(15, 4)
#define MT3620_ADC_FIFO_RBR_ADC_RBR_TIMESTAMP	GENMASK(31, 16)
#define MT3620_ADC_FIFO_IER_RXFEN		BIT(0)
#define MT3620_ADC_FIFO_IER_RXTEN		BIT(3)
#define MT3620_ADC_FIFO_FIFOCTRL_ADC_IIR	GENMASK(3, 0)
#define MT3620_ADC_FIFO_LSR_DR			BIT(0)
#define MT3620_ADC_FIFO_DMA_EN_RX_DMA_EN	BIT(0)
#define MT3620_ADC_FIFO_TRI_LVL_RX_TRI_LVL	GENMASK(6, 2)
#define MT3620_ADC_FIFO_DEBUG16_RX_PTR_READ	GENMASK(4, 0)
#define MT3620_ADC_FIFO_DEBUG16_RX_PTR_WRITE	GENMASK(9, 5)

#define MT3620_ADC_SLEEP_US	10
#define MT3620_ADC_FIFO_ADDRESS_SIZE 32
#define MT3620_ADC_FIFO_MAX_LENGTH 16

#define MT3620_ADC_DEFAULT_FREQUENCY 250
#define MT3620_ADC_INTERRUPT_REASON_RX_DATA 0x04
#define MT3620_ADC_MAX_CHANNELS 8
#define MT3620_ADC_REG_T_INIT_DEFAULT 20
#define MT3620_ADC_REG_T_CH_DEFAULT 8

struct mt3620_auxadc_dma {
	struct dma_slave_config	conf;
	struct dma_chan		*chan;
	dma_addr_t		addr;
	dma_cookie_t		cookie;
	u8			*buf;
	u32                     buf_size;
	dma_addr_t		phy_fifo_data_addr; 
	int			current_period;
	int			period_size;
};

/**
 * @reg_base: the base address of the mt3620_auxadc the driver is controlling
 * @lock: mutex to protect the device state
 * @clock: the clock from the device tree.  Used to get the clock_rate
 * @clock_rate: the ticks per second for the ADC clock (from device tree)
 * @frequency: the desired set frequency.  This is the scan frequency
 * @active_channels: the count of active channels used to read in from the fifo
 * @data: An array containing the raw ADC data for the current scan
 */

struct mt3620_auxadc_device {
	void __iomem *reg_base;
	struct mt3620_auxadc_dma dma; 
	struct mutex lock;
	struct clk *clock;
	unsigned long clock_rate;
	unsigned int frequency;
	unsigned int active_channels;
};

#ifdef CONFIG_MT3620_AUXADC_DEBUG
static u32 mt3620_auxadc_inter_scan_period_get(void __iomem *base)
{
	u32 period = readl(base + MT3620_ADC_CTL1);
	return period;
}
#endif

static void mt3620_auxadc_inter_scan_period_set(void __iomem *base, u32 period)
{
	writel(period, base + MT3620_ADC_CTL1);
}

static u8 mt3620_auxadc_scan_init_period_get(void __iomem *base)
{
	u32 reg;
	u8 period;

	reg = readl(base + MT3620_ADC_CTL0);
	period = FIELD_GET(MT3620_ADC_CTL0_REG_T_INIT, reg);
	return period;
}

static void mt3620_auxadc_scan_init_period_set(void __iomem *base, u8 period)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL0);
	reg &= ~MT3620_ADC_CTL0_REG_T_INIT;
	reg |= FIELD_PREP(MT3620_ADC_CTL0_REG_T_INIT, period);
	writel(reg, base + MT3620_ADC_CTL0);
}

static u8 mt3620_auxadc_channel_stable_period_get(void __iomem *base)
{
	u32 reg;
	u8 period;

	reg = readl(base + MT3620_ADC_CTL0);
	period = FIELD_GET(MT3620_ADC_CTL0_REG_T_CH, reg);
	return period;
}

static void mt3620_auxadc_channel_stable_period_set(void __iomem *base,
		u8 period)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL0);
	reg &= ~MT3620_ADC_CTL0_REG_T_CH;
	reg |= FIELD_PREP(MT3620_ADC_CTL0_REG_T_CH, period);
	writel(reg, base + MT3620_ADC_CTL0);

}

static void mt3620_auxadc_fsm_enable(void __iomem *base, bool enabled)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL0);
	reg &= ~MT3620_ADC_CTL0_ADC_FSM_EN;

	if (enabled == true)
		reg |= FIELD_PREP(MT3620_ADC_CTL0_ADC_FSM_EN, 1);

	writel(reg, base + MT3620_ADC_CTL0);

	/* The FSM must remain de-asserted for a period of not
	 * less than one ADC clock period.  If the ADC clk is 2Mhz, the
	 * de-assert period should be 750ns (1T + uncertainty).  From page 12 of
	 * the datasheet.  udelay 1us is close enough.
	 */

	if (enabled == false)
		udelay(1);
}

static void mt3620_auxadc_timestamp_enable(void __iomem *base, bool enable)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL2);
	reg &= ~MT3620_ADC_CTL2_REG_ADC_TIMESTAMP_EN;

	if (enable)
		reg |= FIELD_PREP(MT3620_ADC_CTL2_REG_ADC_TIMESTAMP_EN,
				MT3620_ADC_CTL2_REG_ADC_TIMESTAMP_EN_ENABLE);

	writel(reg, base + MT3620_ADC_CTL2);
}

static void mt3620_auxadc_vref_mode_set(void __iomem *base, u8 mode)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_VREF_MODE;
	reg |= FIELD_PREP(MT3620_ADC_CTL3_VREF_MODE, mode);

	writel(reg, base + MT3620_ADC_CTL3);
}

static u8 mt3620_auxadc_vref_mode_get(void __iomem *base)
{
	u32 reg;
	u8 mode;

	reg = readl(base + MT3620_ADC_CTL3);
	mode = FIELD_GET(MT3620_ADC_CTL3_VREF_MODE, reg);
	return mode;
}

static void mt3620_auxadc_clk_src_set(void __iomem *base, u32 source)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_CLK_SRC;
	reg |= FIELD_PREP(MT3620_ADC_CTL3_CLK_SRC, source);

	writel(reg, base + MT3620_ADC_CTL3);
}

static void mt3620_auxadc_clk_inv_pmu_set(void __iomem *base, u32 mode)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_CLK_INV_PMU;
	reg |= FIELD_PREP(MT3620_ADC_CTL3_CLK_INV_PMU, mode);

	writel(reg, base + MT3620_ADC_CTL3);
}

static void mt3620_auxadc_clk_phase_enable(void __iomem *base, bool enable)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_CLK_PHASE;

	if (enable)
		reg |= FIELD_PREP(MT3620_ADC_CTL3_CLK_PHASE,
				MT3620_ADC_CTL3_CLK_PHASE_ENABLE);

	writel(reg, base + MT3620_ADC_CTL3);
}

static void mt3620_auxadc_vcm_enable(void __iomem *base, bool enable)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_VCM;

	if (enable)
		reg |= FIELD_PREP(MT3620_ADC_CTL3_VCM,
				MT3620_ADC_CTL3_VCM_ENABLE);

	writel(reg, base + MT3620_ADC_CTL3);
}

static void mt3620_auxadc_input_mux_enable(void __iomem *base, bool enable)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_INPUT_MUX;
	if (enable)
		reg |= FIELD_PREP(MT3620_ADC_CTL3_INPUT_MUX,
				MT3620_ADC_CTL3_INPUT_MUX_ENABLE);

	writel(reg, base + MT3620_ADC_CTL3);
}

static void mt3620_auxadc_dither_step_set(void __iomem *base, u8 step)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_DITHER_STEP;
	reg |= FIELD_PREP(MT3620_ADC_CTL3_DITHER_STEP, step);

	writel(reg, base + MT3620_ADC_CTL3);
}

static u8 mt3620_auxadc_dither_step_get(void __iomem *base)
{
	u32 reg;
	u8 step;

	reg = readl(base + MT3620_ADC_CTL3);
	step = FIELD_GET(MT3620_ADC_CTL3_DITHER_STEP, reg);
	return step;
}

static void mt3620_auxadc_dither_enable(void __iomem *base, bool enable)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_DITHER;
	if (enable)
		reg |= FIELD_PREP(MT3620_ADC_CTL3_DITHER,
				MT3620_ADC_CTL3_DITHER_ENABLE);

	writel(reg, base + MT3620_ADC_CTL3);
}

static bool mt3620_auxadc_dither_query(void __iomem *base)
{
	u32 reg;
	u8 enabled;

	reg = readl(base + MT3620_ADC_CTL3);
	enabled = FIELD_GET(MT3620_ADC_CTL3_DITHER, reg);
	return (enabled == 1);
}

static void mt3620_auxadc_comparator_preamp_enable(void __iomem *base,
		bool enable)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_COMP_PREAMP;

	if (enable)
		reg |= FIELD_PREP(MT3620_ADC_CTL3_COMP_PREAMP,
				MT3620_ADC_CTL3_COMP_PREAMP_ENABLE);

	writel(reg, base + MT3620_ADC_CTL3);
}

static bool mt3620_auxadc_comparator_preamp_query(void __iomem *base)
{
	u32 reg;
	u8 enabled;

	reg = readl(base + MT3620_ADC_CTL3);
	enabled = FIELD_GET(MT3620_ADC_CTL3_COMP_PREAMP, reg);
	return (enabled == 1);
}

static void mt3620_auxadc_comparator_preamp_current_set(void __iomem *base,
		u8 curent)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_COMP_PREAMP_CURR;
	reg |= FIELD_PREP(MT3620_ADC_CTL3_COMP_PREAMP_CURR, curent);

	writel(reg, base + MT3620_ADC_CTL3);
}

static u8 mt3620_auxadc_comparator_preamp_current_get(void __iomem *base)
{
	u32 reg;
	u8 value;

	reg = readl(base + MT3620_ADC_CTL3);
	value = FIELD_GET(MT3620_ADC_CTL3_COMP_PREAMP_CURR, reg);
	return value;
}

static void mt3620_auxadc_comparator_delay_set(void __iomem *base,
		u32 delay)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL3);
	reg &= ~MT3620_ADC_CTL3_COMP_DELAY;
	reg |= FIELD_PREP(MT3620_ADC_CTL3_COMP_DELAY, delay);

	writel(reg, base + MT3620_ADC_CTL3);
}

static u8 mt3620_auxadc_comparator_delay_get(void __iomem *base)
{
	u32 reg;
	u8 delay;

	reg = readl(base + MT3620_ADC_CTL3);
	delay = FIELD_GET(MT3620_ADC_CTL3_COMP_DELAY, reg);
	return delay;
}

static unsigned int mt3620_auxadc_fifo_length_get(void __iomem *base)
{
	u32 reg;
	u32 read;
	u32 write;
	u32 length = 0;

	reg = readl(base + MT3620_ADC_FIFO_DEBUG16);
	read = FIELD_GET(MT3620_ADC_FIFO_DEBUG16_RX_PTR_READ, reg);
	write = FIELD_GET(MT3620_ADC_FIFO_DEBUG16_RX_PTR_WRITE, reg);

	if (write == read)
		length = 0;
	else if (write > read)
		length = write - read;
	else
		length = write + MT3620_ADC_FIFO_ADDRESS_SIZE - read;

	/* clamp at the actual max depth of the FIFO */
	if (length > MT3620_ADC_FIFO_MAX_LENGTH)
		length = MT3620_ADC_FIFO_MAX_LENGTH;

	return length;
}

static u32 mt3620_auxadc_fifo_read(void __iomem *base)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_FIFO_RBR);
	return reg;
}

static void mt3620_auxadc_fifo_clear(void __iomem *base)
{
	u32 length;
	u32 i;

	length = mt3620_auxadc_fifo_length_get(base);
	for (i = 0; i < length; i++)
		mt3620_auxadc_fifo_read(base);
}

static void mt3620_auxadc_dma_enable(void __iomem *base, bool enable)
{
	u32 reg = readl(base + MT3620_ADC_FIFO_DMA_EN);
	reg &= ~MT3620_ADC_FIFO_DMA_EN_RX_DMA_EN;
	if (enable)
		reg |= FIELD_PREP(MT3620_ADC_FIFO_DMA_EN_RX_DMA_EN, 1);

	writel(reg, base + MT3620_ADC_FIFO_DMA_EN);
}

static void mt3620_auxadc_fifo_watermark_set(void __iomem *base,
		unsigned int watermark)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_FIFO_TRI_LVL);
	reg &= ~MT3620_ADC_FIFO_TRI_LVL_RX_TRI_LVL;
	reg |= FIELD_PREP(MT3620_ADC_FIFO_TRI_LVL_RX_TRI_LVL, watermark);
	writel(reg, base + MT3620_ADC_FIFO_TRI_LVL);
}

static void mt3620_auxadc_channel_map_set(void __iomem *base, u8 map)
{
	u32 reg = 0;

	reg = readl(base + MT3620_ADC_CTL0);
	reg &= ~MT3620_ADC_CTL0_REG_CH_MAP;
	reg |= FIELD_PREP(MT3620_ADC_CTL0_REG_CH_MAP, map);
	writel(reg, base + MT3620_ADC_CTL0);
}

static void mt3620_auxadc_periodic_mode_enable(void __iomem *base, bool enable)
{
	u32 reg;

	reg = readl(base + MT3620_ADC_CTL0);
	reg &= ~MT3620_ADC_CTL0_PMODE_EN;
	if (enable)
		reg |= FIELD_PREP(MT3620_ADC_CTL0_PMODE_EN, 1);

	writel(reg, base + MT3620_ADC_CTL0);
}

static void mt3620_auxadc_average_mode_set(void __iomem *base, u8 mode)
{
	u32 reg = 0;

	reg = readl(base + MT3620_ADC_CTL0);
	reg &= ~MT3620_ADC_CTL0_REG_AVG_MODE;
	reg |= FIELD_PREP(MT3620_ADC_CTL0_REG_AVG_MODE, mode);
	writel(reg, base + MT3620_ADC_CTL0);
}

static u8 mt3620_auxadc_average_mode_get(void __iomem *base)
{
	u32 reg;
	u8 mode;

	reg = readl(base + MT3620_ADC_CTL0);
	mode = FIELD_GET(MT3620_ADC_CTL0_REG_AVG_MODE, reg);
	return mode;
}

static u8 mt3620_auxadc_average_mode_in_samples(u8 average_mode)
{
	u8 samples;

	switch (average_mode) {
	case MT3620_ADC_CTL0_REG_AVG_MODE_1:
		samples = 1;
		break;
	case MT3620_ADC_CTL0_REG_AVG_MODE_2:
		samples = 2;
		break;
	case MT3620_ADC_CTL0_REG_AVG_MODE_4:
		samples = 4;
		break;
	case MT3620_ADC_CTL0_REG_AVG_MODE_8:
		samples = 8;
		break;
	case MT3620_ADC_CTL0_REG_AVG_MODE_16:
		samples = 16;
		break;
	case MT3620_ADC_CTL0_REG_AVG_MODE_32:
		samples = 32;
		break;
	case MT3620_ADC_CTL0_REG_AVG_MODE_64:
	case MT3620_ADC_CTL0_REG_AVG_MODE_64_ALT:
		samples = 64;
		break;
	default:
		samples = 0;
		break;
	}

	return samples;
}



static u32 mt3620_auxadc_calculate_period(struct iio_dev *indio_dev)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	u32 scan_init = mt3620_auxadc_scan_init_period_get(adc_dev->reg_base);
	u32 chan_init = mt3620_auxadc_channel_stable_period_get(
			adc_dev->reg_base);
	u8 avg_mode = mt3620_auxadc_average_mode_get(adc_dev->reg_base);
	u32 avg_count = mt3620_auxadc_average_mode_in_samples(avg_mode);
	u32 channel_count = adc_dev->active_channels;

	/* The periodic capture state machine for the MT3620 is documented
	 * in the datasheet. The formula below is derived from that state
	 * machine.  The inputs for that formula are as follows.  First,  There
	 * are three periods that can be set:
	 *
	 * REG_T_INIT: number of clock cycles to wait at the beginning of each
	 *    scan, called scan_init in this code.
	 * REG_T_CH: number of clock cycles to wait at the beginning of each
	 *    channel read, called chan_init in this code.
	 * REG_PERIOD: The number of clock cycles to wait between each scan
	 *    called period in this code.
	 *
	 * Additionally, there are two other values needed.  The number of
	 * active channels (called channel_count), and the number of samples
	 * used in hardware averaging (avg_count).
	 *
	 * Finally, in addition to these values, there are additional clock
	 * cycles that are just part of how the hardware works.  Some of these
	 * values and constants are taken once per scan, and some are taken for
	 * each channel.  The base_period is the number of clock cycles not
	 * indluding REG_PERIOD (period) and is calculated as follows:
	 */

	u32 base_period = scan_init + 1 + channel_count *
			(chan_init + avg_count + 3);
	u32 target_period;
	u32 period;

	target_period = adc_dev->clock_rate / adc_dev->frequency;

	if (target_period < base_period)
		period = 0;
	else
		period = target_period - base_period - 1;

	return period;
}

static int mt3620_auxadc_sample_frequency_get(struct iio_dev *indio_dev)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);

	return adc_dev->frequency;
}

static int mt3620_auxadc_sample_frequency_set(struct iio_dev *indio_dev,
		int frequency)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);

	/* clamp the frequency */
	if (frequency <= 0)
		frequency = 1;

	if (frequency > adc_dev->clock_rate)
		frequency = adc_dev->clock_rate;

	adc_dev->frequency = frequency;

	return 0;
}


static unsigned int mt3620_auxadc_calculate_read_timeout(void __iomem *base,
		unsigned int clock_rate)
{
	u8 init;
	u8 channel;
	u8 average_mode;
	u8 average;
	unsigned int cycles = 0;
	unsigned int uSeconds = 0;

	init = mt3620_auxadc_scan_init_period_get(base);
	channel = mt3620_auxadc_channel_stable_period_get(base);
	average_mode = mt3620_auxadc_average_mode_get(base);
	average = mt3620_auxadc_average_mode_in_samples(average_mode);

	/* calculate our timeout.  It is:
	 * The init period + The channel period +
	 * The numer of averages we take + 3 clock cycles.
	 *
	 * That gives our clock cycles.  Then we scale for rate and
	 * convert to uSeconds.
	 */

	/* include cycles in our calculation to cause upgrading the math to
	 * unsigned int. And make our timeout 2x the expected delay.
	 */
	cycles = (cycles + init + channel + average + 3) * 2;
	uSeconds = (cycles * 1000000) / clock_rate;

	return uSeconds;
}

static int mt3620_auxadc_read(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan)
{
	u32 val = 0;
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	void __iomem *base = adc_dev->reg_base;
	int ret;
	u32 reg;
	int length;
	unsigned int timeout_value;

	mutex_lock(&adc_dev->lock);

	/* Do not do single-shot read if we are doing periodic capture */
	if (iio_buffer_enabled(indio_dev)) {
		dev_err(indio_dev->dev.parent, "Can't read a single value while doing periodic capture\n");
		ret = -EBUSY;
		goto err_unlock;
	}

	mt3620_auxadc_channel_map_set(base, BIT(chan->scan_index));
	adc_dev->active_channels = 1;
	mt3620_auxadc_periodic_mode_enable(base, false);
	mt3620_auxadc_fifo_clear(base);
	mt3620_auxadc_fifo_watermark_set(base, 1);
	mt3620_auxadc_fsm_enable(base, true);


	timeout_value = mt3620_auxadc_calculate_read_timeout(adc_dev->reg_base,
		adc_dev->clock_rate);


	ret = readx_poll_timeout(mt3620_auxadc_fifo_length_get, base,
			length, length >= 1,
			MT3620_ADC_SLEEP_US, timeout_value);

	if (ret < 0) {
		dev_err(indio_dev->dev.parent,
			"wait for data time out on ch %d\n",
			chan->channel);
		ret = -ETIMEDOUT;
		goto err_timeout;
	}

	mt3620_auxadc_fsm_enable(base, false);

	/* Grab the raw ADC count and rock on! */
	reg = mt3620_auxadc_fifo_read(base);
	val = FIELD_GET(MT3620_ADC_FIFO_RBR_ADC_RBR_SAMPLE, reg);

	mutex_unlock(&adc_dev->lock);

	return val;

err_timeout:

	/* Disable the ADC FSM */
	mt3620_auxadc_fsm_enable(base, false);
	mt3620_auxadc_fifo_clear(base);

	/* Note: this is a race condition here.  We have asked the hardware for
	 * a sample and it didn't deliver it before we timed out. But it may
	 * deliver it after we gave up and cleared the FIFO.  So the FIFO may
	 * not be clearthe next time we go to use it.  Code defensively.
	 */

err_unlock:
	mutex_unlock(&adc_dev->lock);

	return ret;
}

/* implementation of the IIO read_raw function. See: iio.h  struct iio_info
 * the meaning of val and val2 are dependent on the value returned.  For
 * example if IIO_VAL_INT is returned, only val has meaning.  If the value
 * return is IIO_VAL_INT_PLUS_NANO, then the integer portion of the number
 * is in val, and the fractional part is in val2. So the value would be the
 * equivalent to the floating point: val + 1e-9 * val2.
 */
static int mt3620_auxadc_read_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int *val,
				  int *val2,
				  long info)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (NULL == val || NULL == val2) {
			dev_err(indio_dev->dev.parent,
				"can not pass a null pointer for val or val2\n");
			return -EINVAL;
		}
		*val2 = 0;
		*val = mt3620_auxadc_read(indio_dev, chan);
		if (*val < 0) {
			dev_err(indio_dev->dev.parent,
				"failed to sample data on channel[%d]\n",
				chan->channel);
			return *val;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val == NULL) {
			dev_err(indio_dev->dev.parent,
				"can not pass a null pointer for val\n");
			return -EINVAL;
		}
		mutex_lock(&adc_dev->lock);
		*val = mt3620_auxadc_sample_frequency_get(indio_dev);
		mutex_unlock(&adc_dev->lock);
		if (*val < 0)
			return *val;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int mt3620_auxadc_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int val,
		int val2,
		long mask)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/* don't change the frequency if we are doing periodic capture
		 */
		mutex_lock(&adc_dev->lock);
		if (iio_buffer_enabled(indio_dev))
			ret =  -EBUSY;
		else
			ret = mt3620_auxadc_sample_frequency_set(indio_dev,
					val);
		mutex_unlock(&adc_dev->lock);

		return ret;
	}

	return -EINVAL;
}

static const struct iio_info mt3620_auxadc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &mt3620_auxadc_read_raw,
	.write_raw =  &mt3620_auxadc_write_raw,
};

static void mt3620_auxadc_dma_rx_complete(void *param)
{
	struct iio_dev *indio_dev = param;
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	struct mt3620_auxadc_dma *dma = &adc_dev->dma; 
	u8 *data; 
	int i;

	data = dma->buf + dma->current_period * dma->period_size;
	dma->current_period = 1 - dma->current_period; /* swap the buffer ID */

	for (i = 0; i < dma->period_size; i += indio_dev->scan_bytes) {
		iio_push_to_buffers(indio_dev, data);
		data += indio_dev->scan_bytes;
	}
}

static int mt3620_compute_dma_period(struct iio_dev *indio_dev)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	int period_size;
	int watermark;
	int scan_line_size;
	int kfifo_buffer_size; 
	int scan_line_count;
	int max_size; 
	int min_scan_line_count; 
	int max_scan_line_count; 
	int parity;

	/* Some considerations:
	 *
	 * #1 period_size must be a multiple of the length of a scan
	 * #2 period_size must not be bigger than the size of the kfifo buffer
	 * #3 period_size must be 8 byte aligned (multiple of 8 bytes)
	 * #4 period_size must be longer than 32 bytes
	 * #5 We should take a "hint" from watermark. 
	 * #6 period size must be less than half our max dma buffer size 
	 * 
	 * Our size will be <= watermark except when watermark is too small. 
	 */

	watermark = iio_buffer_get_watermark(indio_dev);
	scan_line_size = MT3620_ADC_SAMPLE_SIZE * adc_dev->active_channels;
	kfifo_buffer_size = iio_buffer_get_length(indio_dev) * scan_line_size; 
	
	/* Our goal is to define the number of scan-lines in our period. We are
	 * going to start by finding the parity of channels.  If the number of
	 * channels is even (0 parity) then any number of scan-lines works for 
	 * #3 above.  If it is odd (1 parity) then only odd numbers of scan
	 * lines are valid. 
	 */

	parity = 0x1 & adc_dev->active_channels; 

	/* compute the minmum valid scan line counts */
	if(scan_line_size < DMA_PERIOD_MIN_SIZE) {
		min_scan_line_count = DMA_PERIOD_MIN_SIZE / scan_line_size + 1; 

		/* check that we meet our alignment needs */
		min_scan_line_count = roundup(min_scan_line_count, parity + 1);
	}
	
	/* compute the maximum valid scan line counts */
	if (kfifo_buffer_size > DMA_BUFFER_MAX_SIZE / 2){
		max_size = DMA_BUFFER_MAX_SIZE / 2;
	}
	else {
		max_size = kfifo_buffer_size; 
	}
	max_size = rounddown(max_size, (parity + 1) * scan_line_size);
	max_scan_line_count = max_size / scan_line_size; 

	/* fail if we are overconstrained */
	if (max_scan_line_count < min_scan_line_count){
		return -EINVAL; 
	}

	/* So, we now know we CAN use DMA. there are solutions for period size 
	 * that work. Let's find the one that works that is closest to watermark
	 */
	scan_line_count = rounddown(watermark, parity + 1);
		
	if (scan_line_count < min_scan_line_count)
			scan_line_count = min_scan_line_count;

	if (scan_line_count > max_scan_line_count)
		scan_line_count = max_scan_line_count;
	
	period_size = scan_line_count * scan_line_size;

	return period_size; 
}

static int mt3620_auxadc_start_dma(struct iio_dev *indio_dev)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	struct mt3620_auxadc_dma *dma = &adc_dev->dma; 
	
	struct dma_async_tx_descriptor *desc;
	int ret; 

	dma->current_period = 0; /* We start to fill period 0 */

	ret = dmaengine_slave_config(dma->chan, &dma->conf);
	if (ret < 0){
		dev_err(indio_dev->dev.parent, "dmaengine_slave_config failed. Error %d\n", ret);
		return ret;	
	}

	dma->period_size = mt3620_compute_dma_period(indio_dev);
	dma->buf_size = dma->period_size * 2; 

	desc = dmaengine_prep_dma_cyclic(dma->chan, dma->addr, dma->buf_size, 
		dma->period_size, DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);

	if (!desc){
		dev_err(indio_dev->dev.parent, "dmaengine_prep_dma_cyclic failed.\n");
		return -ENOMEM;
	}

	desc->callback = mt3620_auxadc_dma_rx_complete;
	desc->callback_param = indio_dev;
	dma->cookie = dmaengine_submit(desc);
	if (dma_submit_error(dma->cookie)) {
		dev_err(indio_dev->dev.parent, "submitting dma failed.\n");
		return -ENOMEM;
	}

	dma_async_issue_pending(dma->chan);
	return 0;
}

int mt3620_auxadc_stop_dma(struct iio_dev *indio_dev)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	struct mt3620_auxadc_dma *dma = &adc_dev->dma;
	int ret; 

	ret = dmaengine_terminate_sync(dma->chan);

	return ret; 
}


static int mt3620_auxadc_buffer_postenable(struct iio_dev *indio_dev)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	u32 period;
	int ret; 

	mutex_lock(&adc_dev->lock);
	 
	adc_dev->active_channels = hweight_long(*indio_dev->active_scan_mask);
	mt3620_auxadc_fifo_clear(adc_dev->reg_base);
	mt3620_auxadc_fifo_watermark_set(adc_dev->reg_base,
			adc_dev->active_channels);
	mt3620_auxadc_channel_map_set(adc_dev->reg_base,
			*indio_dev->active_scan_mask);
	period = mt3620_auxadc_calculate_period(indio_dev);
	mt3620_auxadc_inter_scan_period_set(adc_dev->reg_base, period);
	mt3620_auxadc_periodic_mode_enable(adc_dev->reg_base, true);

	ret = mt3620_auxadc_start_dma(indio_dev); 
	if (ret < 0)
	{
		dev_err(indio_dev->dev.parent, "Failed to start DMA, err: %d\n", ret);
		goto err_unlock;
	}

	mt3620_auxadc_dma_enable(adc_dev->reg_base, true);
	mt3620_auxadc_fsm_enable(adc_dev->reg_base, true);

err_unlock:
	mutex_unlock(&adc_dev->lock);

	return ret;
}

static int mt3620_auxadc_buffer_predisable(struct iio_dev *indio_dev)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	int ret; 

	mutex_lock(&adc_dev->lock);

	mt3620_auxadc_fsm_enable(adc_dev->reg_base, false);
	mt3620_auxadc_dma_enable(adc_dev->reg_base, false);
	ret = mt3620_auxadc_stop_dma(indio_dev);
	mt3620_auxadc_periodic_mode_enable(adc_dev->reg_base, false);
	mt3620_auxadc_channel_map_set(adc_dev->reg_base, 0);

	mutex_unlock(&adc_dev->lock);

	return ret;
}

static const struct iio_buffer_setup_ops mt3620_buffer_setup_ops = {
	.preenable = NULL,
	.postenable = &mt3620_auxadc_buffer_postenable,
	.predisable = &mt3620_auxadc_buffer_predisable,
	.postdisable = NULL,
};

/*
 *  Extended property read and write functions
 *
 */

static const char * const mt3620_auxadc_avg_mode[] = {
	"avg_1_sample",
	"avg_2_samples",
	"avg_4_samples",
	"avg_8_samples",
	"avg_16_samples",
	"avg_32_samples",
	"avg_64_samples",
	"avg_64_samples_2",
};

static int mt3620_auxadc_iio_avg_mode_get(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	u8 avg_mode;

	mutex_lock(&adc_dev->lock);
	avg_mode = mt3620_auxadc_average_mode_get(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return avg_mode;
}

static int mt3620_auxadc_iio_avg_mode_set(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	int ret = 0;
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);

	mutex_lock(&adc_dev->lock);
	/* Do not do change the average mode if we are doing periodic capture */
	if (iio_buffer_enabled(indio_dev)) {
		dev_err(indio_dev->dev.parent, "Can't change average mode while doing periodic capture\n");
		ret = -EBUSY;
		goto err_unlock;
	}

	mt3620_auxadc_average_mode_set(adc_dev->reg_base, mode);

err_unlock:
	mutex_unlock(&adc_dev->lock);
	return ret;
}

static const struct iio_enum mt3620_auxadc_avg_mode_enum = {
	.items = mt3620_auxadc_avg_mode,
	.num_items = ARRAY_SIZE(mt3620_auxadc_avg_mode),
	.get = mt3620_auxadc_iio_avg_mode_get,
	.set = mt3620_auxadc_iio_avg_mode_set,
};


static const char * const mt3620_auxadc_vref_mode[] = {
	"vref_2v5",
	"vref_1v8"
};

static int mt3620_auxadc_iio_vref_mode_get(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	u8 vref_mode;

	mutex_lock(&adc_dev->lock);
	vref_mode = mt3620_auxadc_vref_mode_get(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return vref_mode;
}

static int mt3620_auxadc_iio_vref_mode_set(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int mode)
{
	int ret = 0;
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);

	mutex_lock(&adc_dev->lock);
	/* Do not do change the vref mode if we are doing periodic capture */
	if (iio_buffer_enabled(indio_dev)) {
		dev_err(indio_dev->dev.parent, "Can't change vref_mode while doing periodic capture\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	mt3620_auxadc_vref_mode_set(adc_dev->reg_base, mode);
err_unlock:
	mutex_unlock(&adc_dev->lock);
	return ret;
}

static const struct iio_enum mt3620_auxadc_vref_mode_enum = {
	.items = mt3620_auxadc_vref_mode,
	.num_items = ARRAY_SIZE(mt3620_auxadc_vref_mode),
	.get = mt3620_auxadc_iio_vref_mode_get,
	.set = mt3620_auxadc_iio_vref_mode_set,
};

static const char * const mt3620_auxadc_dithering_step_size[] = {
	"2_steps",
	"4_steps",
	"8_steps",
	"16_steps"
};

static int mt3620_auxadc_iio_dithering_step_size_get(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	int val;

	mutex_lock(&adc_dev->lock);
	val = mt3620_auxadc_dither_step_get(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return val;
}

static int mt3620_auxadc_iio_dithering_step_size_set(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	int ret = 0;
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);

	mutex_lock(&adc_dev->lock);
	/* Do not do change if we are doing periodic capture */
	if (iio_buffer_enabled(indio_dev)) {
		dev_err(indio_dev->dev.parent, "Can't change dithering step size while doing periodic capture\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	mt3620_auxadc_dither_step_set(adc_dev->reg_base, val);
err_unlock:
	mutex_unlock(&adc_dev->lock);
	return ret;
}

static const struct iio_enum mt3620_auxadc_dithering_step_enum = {
	.items = mt3620_auxadc_dithering_step_size,
	.num_items = ARRAY_SIZE(mt3620_auxadc_dithering_step_size),
	.get = mt3620_auxadc_iio_dithering_step_size_get,
	.set = mt3620_auxadc_iio_dithering_step_size_set,
};

static ssize_t mt3620_auxadc_iio_dither_query(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	bool enabled;

	mutex_lock(&adc_dev->lock);
	enabled = mt3620_auxadc_dither_query(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t mt3620_auxadc_iio_dither_enable(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan,
	 const char *buf, size_t len)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	int ret = len;
	bool enabled;

	ret = strtobool(buf, &enabled);
	if (ret)
		return ret;

	mutex_lock(&adc_dev->lock);
	/* Do not do change if we are doing periodic capture */
	if (iio_buffer_enabled(indio_dev)) {
		dev_err(indio_dev->dev.parent, "Can't enable or disable dither while doing periodic capture\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	mt3620_auxadc_dither_enable(adc_dev->reg_base, enabled);
err_unlock:
	mutex_unlock(&adc_dev->lock);

	return ret;
}

static ssize_t mt3620_auxadc_iio_comparator_preamp_query(
	struct iio_dev *indio_dev, uintptr_t private,
	const struct iio_chan_spec *chan, char *buf)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	bool enabled;

	mutex_lock(&adc_dev->lock);
	enabled = mt3620_auxadc_comparator_preamp_query(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", enabled);
}

static ssize_t mt3620_auxadc_iio_comparator_preamp_enable(
	struct iio_dev *indio_dev, uintptr_t private,
	const struct iio_chan_spec *chan, const char *buf, size_t len)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	int ret;
	bool enabled;

	ret = strtobool(buf, &enabled);
	if (ret)
		return ret;

	mutex_lock(&adc_dev->lock);
	/* Do not do change if we are doing periodic capture */
	if (iio_buffer_enabled(indio_dev)) {
		dev_err(indio_dev->dev.parent, "Can't enable or disable preamp while doing periodic capture\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	mt3620_auxadc_comparator_preamp_enable(adc_dev->reg_base, enabled);
	ret = len;
err_unlock:
	mutex_unlock(&adc_dev->lock);

	return ret;
}

static const char * const mt3620_auxadc_comparator_preamp_current[] = {
	"40ua_typical",
	"80ua_typical",
	"160ua_typical",
	"160ua_typical_2"
};

static int mt3620_auxadc_iio_comparator_preamp_current_get(
	struct iio_dev *indio_dev, const struct iio_chan_spec *chan)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	int val;

	mutex_lock(&adc_dev->lock);
	val = mt3620_auxadc_comparator_preamp_current_get(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return val;
}

static int mt3620_auxadc_iio_comparator_preamp_current_set(
	struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
	unsigned int val)
{
	int ret = 0;
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);

	mutex_lock(&adc_dev->lock);
	/* Do not do change if we are doing periodic capture */
	if (iio_buffer_enabled(indio_dev)) {
		dev_err(indio_dev->dev.parent, "Can't change preamp current while doing periodic capture\n");
		ret = -EBUSY;
		goto err_unlock;
	}
	mt3620_auxadc_comparator_preamp_current_set(adc_dev->reg_base, val);
err_unlock:
	mutex_unlock(&adc_dev->lock);
	return ret;
}

static const struct iio_enum mt3620_auxadc_comparator_preamp_current_enum = {
	.items = mt3620_auxadc_comparator_preamp_current,
	.num_items = ARRAY_SIZE(mt3620_auxadc_comparator_preamp_current),
	.get = mt3620_auxadc_iio_comparator_preamp_current_get,
	.set = mt3620_auxadc_iio_comparator_preamp_current_set,
};

static const char * const mt3620_auxadc_comparator_delay[] = {
	"3ns_typical",
	"6ns_typical",
	"9ns_typical",
	"12ns_typical"
};

static int mt3620_auxadc_iio_comparator_delay_get(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	int val;

	mutex_lock(&adc_dev->lock);
	val = mt3620_auxadc_comparator_delay_get(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return val;
}

static int mt3620_auxadc_iio_comparator_delay_set(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	int ret = 0;
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);

	mutex_lock(&adc_dev->lock);
	/* Do not do change if we are doing periodic capture */
	if (iio_buffer_enabled(indio_dev)) {
		dev_err(indio_dev->dev.parent, "Can't change comparator delay while doing periodic capture\n");
		ret = -EINVAL;
		goto err_unlock;
	}
	mt3620_auxadc_comparator_delay_set(adc_dev->reg_base, val);
err_unlock:
	mutex_unlock(&adc_dev->lock);
	return ret;
}

static const struct iio_enum mt3620_auxadc_comparator_delay_enum = {
	.items = mt3620_auxadc_comparator_delay,
	.num_items = ARRAY_SIZE(mt3620_auxadc_comparator_delay),
	.get = mt3620_auxadc_iio_comparator_delay_get,
	.set = mt3620_auxadc_iio_comparator_delay_set,
};


// read the number of bits in the ADC. 
static ssize_t mt3620_auxadc_iio_current_bits_query(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "%d\n", MT3620_ADC_BITS_PER_SAMPLE);
}

// write 
static ssize_t mt3620_auxadc_iio_reference_voltage_set(struct iio_dev *indio_dev,
	 uintptr_t private, const struct iio_chan_spec *chan,
	 const char *buf, size_t len)
{
	int integer;
	int fraction;
	int ret; 

	/* This is key, iio_str_to_fixpoint turns a floating-point string into 
	** a fixed point integer.  The 2nd argument is the denominator of
	** the fractional part.  So this call will have the whole part in 
	** integers, and the fractional part in 1/100000 units also stored as an 
	** integer 
	*/
	ret = iio_str_to_fixpoint(buf, 100000, &integer, &fraction);
	if (ret < 0)
		return ret; 
	
	/* Check for out of bounds using the fixed point representation. 
	** Greater than 2.5 or less than 1.8 is out of bounds
	*/ 
	if ((integer > 2) || (integer == 2 && fraction > 500000) || 
			(integer < 1) || (integer == 1 && fraction < 800000))
		return -EINVAL; 
	else { 
		/* Because of the previous if clause, the voltage range is 
		** in-bounds.  Now, if the fraction is 2.15 or greater, then we 
		** set the refernce voltage to 2v5, else 1v8
		*/
		if (integer == 2 && fraction >= 150000)
			ret = mt3620_auxadc_iio_vref_mode_set(indio_dev, chan, MT3620_ADC_CTL3_VREF_MODE_2V5); 
		else
			ret = mt3620_auxadc_iio_vref_mode_set(indio_dev, chan, MT3620_ADC_CTL3_VREF_MODE_1V8);
	}

	return ret; 
}

#ifdef CONFIG_MT3620_AUXADC_DEBUG

static ssize_t mt3620_auxadc_iio_inter_scan_period_read(
	struct iio_dev *indio_dev, uintptr_t private,
	const struct iio_chan_spec *chan, char *buf)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	u32 period;

	mutex_lock(&adc_dev->lock);
	period = mt3620_auxadc_inter_scan_period_get(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);

	return snprintf(buf, PAGE_SIZE, "%u\n", period);
}

static ssize_t mt3620_auxadc_iio_scan_init_period_read(struct iio_dev *indio_dev,
	uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	unsigned int period;

	mutex_lock(&adc_dev->lock);
	period = mt3620_auxadc_scan_init_period_get(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return snprintf(buf, PAGE_SIZE, "%u\n", period);
}


static ssize_t mt3620_auxadc_iio_channel_stable_period_read(
	struct iio_dev *indio_dev, uintptr_t private,
	const struct iio_chan_spec *chan, char *buf)
{
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	unsigned int period;

	mutex_lock(&adc_dev->lock);
	period = mt3620_auxadc_channel_stable_period_get(adc_dev->reg_base);
	mutex_unlock(&adc_dev->lock);
	return snprintf(buf, PAGE_SIZE, "%u\n", period);
}

#endif

static const struct iio_chan_spec_ext_info mt3620_ext_info[] = {
	IIO_ENUM("average_mode",
		IIO_SHARED_BY_ALL, &mt3620_auxadc_avg_mode_enum),
	IIO_ENUM_AVAILABLE("average_mode", &mt3620_auxadc_avg_mode_enum),
	IIO_ENUM("vref_mode",
		IIO_SHARED_BY_ALL, &mt3620_auxadc_vref_mode_enum),
	IIO_ENUM_AVAILABLE("vref_mode", &mt3620_auxadc_vref_mode_enum),
	IIO_ENUM("dithering_step",
		IIO_SHARED_BY_ALL, &mt3620_auxadc_dithering_step_enum),
	IIO_ENUM_AVAILABLE("dithering_step",
		&mt3620_auxadc_dithering_step_enum),
	{
		.name = "dithering_enable",
		.read = mt3620_auxadc_iio_dither_query,
		.write = mt3620_auxadc_iio_dither_enable,
		.shared = IIO_SHARED_BY_ALL
	},
	{
		.name = "comparator_preamplifier_enable",
		.read = mt3620_auxadc_iio_comparator_preamp_query,
		.write = mt3620_auxadc_iio_comparator_preamp_enable,
		.shared = IIO_SHARED_BY_ALL
	},
	IIO_ENUM("comparator_preamplifier_current",
		IIO_SHARED_BY_ALL, &mt3620_auxadc_comparator_preamp_current_enum),
	IIO_ENUM_AVAILABLE("comparator_preamplifier_current",
		&mt3620_auxadc_comparator_preamp_current_enum),
	IIO_ENUM("comparator_timing_loop_delay_time",
		IIO_SHARED_BY_ALL, &mt3620_auxadc_comparator_delay_enum),
	IIO_ENUM_AVAILABLE("comparator_timing_loop_delay_time",
		&mt3620_auxadc_comparator_delay_enum),
	{
		.name = "reference_voltage",
		.read = NULL,
		.write = mt3620_auxadc_iio_reference_voltage_set,
		.shared = IIO_SHARED_BY_ALL
	},
	{
		.name = "current_bits",
		.read = mt3620_auxadc_iio_current_bits_query,
		.write = NULL,
		.shared = IIO_SHARED_BY_ALL
	},
#ifdef CONFIG_MT3620_AUXADC_DEBUG
	{
		.name = "inter_scan_period",
		.read = mt3620_auxadc_iio_inter_scan_period_read,
		.shared = IIO_SHARED_BY_ALL
	},
	{
		.name = "scan_init_period",
		.read = mt3620_auxadc_iio_scan_init_period_read,
		.shared = IIO_SHARED_BY_ALL
	},
	{
		.name = "channel_stable_period",
		.read = mt3620_auxadc_iio_channel_stable_period_read,
		.shared = IIO_SHARED_BY_ALL
	},
#endif
	{}
};

#define MT3620_AUXADC_CHANNEL(idx) {				    \
		.type = IIO_VOLTAGE,				    \
		.indexed = 1,					    \
		.channel = (idx),				    \
		.scan_index = (idx),				    \
		.scan_type.sign = 'u',				    \
		.scan_type.realbits = MT3620_ADC_BITS_PER_SAMPLE,   \
		.scan_type.storagebits = 32,			    \
		.scan_type.shift = 4,				    \
		.scan_type.repeat = 0,				    \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	    \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.ext_info = mt3620_ext_info, 			    \
}

static const struct iio_chan_spec mt3620_auxadc_iio_channels[] = {
	MT3620_AUXADC_CHANNEL(0),
	MT3620_AUXADC_CHANNEL(1),
	MT3620_AUXADC_CHANNEL(2),
	MT3620_AUXADC_CHANNEL(3),
	MT3620_AUXADC_CHANNEL(4),
	MT3620_AUXADC_CHANNEL(5),
	MT3620_AUXADC_CHANNEL(6),
	MT3620_AUXADC_CHANNEL(7),
};

static int mt3620_auxadc_allocate_dma(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	struct mt3620_auxadc_dma *dma = &adc_dev->dma;
	
	/* Default slave configuration parameters */
	dma->conf.slave_id = DMA_SLAVE_ID_ADC;
	dma->conf.direction = DMA_DEV_TO_MEM;
	dma->conf.src_addr = dma->phy_fifo_data_addr;
	dma->conf.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	/* Get a channel for RX */
	dma->chan = dma_request_slave_channel(&pdev->dev, "adc_dma_rx");
	if (!dma->chan) {
		dev_err(&pdev->dev, "request DMA channel failed!\n");
		return -ENOMEM;
	}

	/* RX buffer */
	dma->buf = dma_alloc_coherent(dma->chan->device->dev, DMA_BUFFER_MAX_SIZE,
				      &dma->addr, GFP_KERNEL);
	if (!dma->buf) {
		dev_err(dma->chan->device->dev, "dma_alloc_coherent failed\n");
		goto err;
	}

	return 0;

	
err:
	dma_release_channel(dma->chan);
	return -ENOMEM;
}

void mt3620_auxadc_free_dma(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	struct mt3620_auxadc_dma *dma = &adc_dev->dma;

	if (dma->chan){
		dma_free_coherent(dma->chan->device->dev, DMA_BUFFER_MAX_SIZE, dma->buf,
			dma->addr);
		dma->buf = NULL;
		dma->addr = 0;
		dma_release_channel(dma->chan);
		dma->chan = NULL;
	}

	return; 
}

static int mt3620_auxadc_probe(struct platform_device *pdev)
{
	struct mt3620_auxadc_device *adc_dev;
	struct iio_buffer *buffer = NULL;
	struct resource *res;
	struct iio_dev *indio_dev;
	void __iomem *base;
	int ret;
	int id = -1;


	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*adc_dev));
	if (!indio_dev)
		return -ENOMEM;

	adc_dev = iio_priv(indio_dev);
	memset(adc_dev, 0, sizeof(*adc_dev));

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->info = &mt3620_auxadc_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = mt3620_auxadc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(mt3620_auxadc_iio_channels);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get the IORESOURCE_MEM platform resource\n");
		ret = -ENODEV;
		goto err_free_device;
	}

	adc_dev->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(adc_dev->reg_base)) {
		dev_err(&pdev->dev, "failed to get auxadc base address\n");
		ret = PTR_ERR(adc_dev->reg_base);
		goto err_free_device;
	}

	adc_dev->dma.phy_fifo_data_addr = (res->start + MT3620_ADC_FIFO_RBR);

	adc_dev->clock = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(adc_dev->clock)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(adc_dev->clock);
		goto err_free_device;
	}
	adc_dev->clock_rate = clk_get_rate(adc_dev->clock);

	adc_dev->frequency = MT3620_ADC_DEFAULT_FREQUENCY;

	id = of_alias_get_id(pdev->dev.of_node, "adc");
	if (id < 0) {
		dev_err(&pdev->dev, "failed to get device id\n");
		ret = id;
		goto err_free_device;
	}

	ret = dev_set_name(&indio_dev->dev, "adc%d", id);
	if (ret < 0)
		goto err_free_device;

	mutex_init(&adc_dev->lock);

	base = adc_dev->reg_base;

	/* set VREF mode to 2.5v  default */
	mt3620_auxadc_vref_mode_set(base, MT3620_ADC_CTL3_VREF_MODE_2V5);
	mt3620_auxadc_clk_src_set(base, MT3620_ADC_CTL3_CLK_SRC_ADC);
	mt3620_auxadc_clk_inv_pmu_set(base,
			MT3620_ADC_CTL3_CLK_INV_PMU_ORIGINAL);
	mt3620_auxadc_clk_phase_enable(base, true);
	mt3620_auxadc_vcm_enable(base, true);
	mt3620_auxadc_input_mux_enable(base, true);
	mt3620_auxadc_dither_step_set(base, MT3620_ADC_CTL3_DITHER_STEP_8);
	mt3620_auxadc_dither_enable(base, true);
	mt3620_auxadc_comparator_preamp_enable(base, true);
	mt3620_auxadc_comparator_preamp_current_set(base,
			MT3620_ADC_CTL3_COMP_PREAMP_CURR_80UA);
	mt3620_auxadc_comparator_delay_set(base,
		MT3620_ADC_CTL3_COMP_DELAY_6NS);

	/* delay 100u seconds before enabling ADC */
	udelay(100);

	/* SET One-shot and Averaging (to zeros!)*/
	mt3620_auxadc_periodic_mode_enable(base, false);
	mt3620_auxadc_average_mode_set(base, MT3620_ADC_CTL0_REG_AVG_MODE_1);

	/* set the default REG_T_INIT and REG_T_CH */
	mt3620_auxadc_scan_init_period_set(base, MT3620_ADC_REG_T_INIT_DEFAULT);
	mt3620_auxadc_channel_stable_period_set(base, MT3620_ADC_REG_T_CH_DEFAULT);

	/* enable hardware timestamping of each sample */
	mt3620_auxadc_timestamp_enable(base, true);

	/* Disable all 8 channels and set the TX level */
	mt3620_auxadc_channel_map_set(base, 0x00);
	mt3620_auxadc_fifo_watermark_set(base, 1);

	platform_set_drvdata(pdev, indio_dev);

	ret = mt3620_auxadc_allocate_dma(pdev);
	if (ret < 0)
		goto err_destroy_mutex; 

	/* IIO Buffers */

	buffer = devm_iio_kfifo_allocate(indio_dev->dev.parent);
	if (!buffer){
		ret = -ENOMEM;
		goto err_destroy_mutex; 
	}

	iio_device_attach_buffer(indio_dev, buffer); 
	indio_dev->setup_ops = &mt3620_buffer_setup_ops;

	/* register the IIO device. */
	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register iio device\n");
		goto err_destroy_mutex;
	}

	return 0;

err_destroy_mutex:
	mutex_destroy(&adc_dev->lock);

err_free_device:
	if (buffer != NULL)
		devm_iio_kfifo_free(indio_dev->dev.parent, buffer); 

	devm_iio_device_free(&pdev->dev, indio_dev);

	return ret;
}

static int mt3620_auxadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct mt3620_auxadc_device *adc_dev = iio_priv(indio_dev);
	void __iomem *base = adc_dev->reg_base;

	iio_device_unregister(indio_dev);
	mutex_lock(&adc_dev->lock);
	
	/* Disable the ADC FSM */
	mt3620_auxadc_dma_enable(base, false);
	mt3620_auxadc_fsm_enable(base, false);

	mt3620_auxadc_free_dma(pdev);

	devm_iio_kfifo_free(indio_dev->dev.parent, indio_dev->buffer);

	mutex_unlock(&adc_dev->lock);
	mutex_destroy(&adc_dev->lock);

	return 0;
}


static const struct of_device_id mt3620_auxadc_match[] = {
	{ .compatible = "mediatek,mt3620-auxadc" },
	{ /* Sentinel */ }
};

MODULE_DEVICE_TABLE(of, mt3620_auxadc_match);

static struct platform_driver mt3620_auxadc_driver = {
	.driver = {
		.name   = "mt3620-auxadc",
		.of_match_table = mt3620_auxadc_match,
	},
	.probe	= mt3620_auxadc_probe,
	.remove	= mt3620_auxadc_remove,
};

module_platform_driver(mt3620_auxadc_driver);

MODULE_AUTHOR("Zhiyong Tao <zhiyong.tao@mediatek.com>");
MODULE_AUTHOR("Alan Ludwig <alanlu@microsoft.com>");
MODULE_DESCRIPTION("MTK AUXADC Device Driver specialized for the MT3620");
MODULE_LICENSE("GPL v2");
