// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 I2C Bus Driver
 *
 * Copyright (c) 2018 Microsoft Corporation. All rights reserved.
 * Based on code Copyright (c) 2014 MediaTek Inc.
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

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>

#define GDMA_SLAVE_ID_I2C			(0)

#define MT3620_I2C_DRV_NAME			"i2c-mt3620"

/* DMA buffer size (for each RX, TX) */
#define MT3620_I2C_DMA_BUF_SIZE 		(PAGE_SIZE)

#define MT3620_I2C_DEFAULT_TIMEOUT_MS		(1000)
#define MT3620_I2C_DEFAULT_RETRIES		(1)
#define MT3620_I2C_MAX_MSG_LEN  		MT3620_I2C_DMA_BUF_SIZE

#define MT3620_I2C_FIFO_SIZE			(8)

#define I2C_CLK_100K				100000
#define I2C_CLK_400K				400000
#define I2C_CLK_1M				1000000

#define MT3620_I2C_CLK_DEFAULT			I2C_CLK_100K

#define MIN(x, y) 				((x) <= (y) ? (x) : (y))
#define BITSMASK(s,e)				(~(BIT(s)-1) & ((BIT(e) - 1) | BIT(e)))

/* 0x00 I2C_INT_CTRL */
#define I2C_MM_INT_STA				BIT(0)
#define I2C_MM_INT_EN				BIT(1)

/* 0x40 MM_PAD_CON0 */
#define I2C_DE_CNT_OFFSET			(0)
#define I2C_DE_CNT_MASK				BITSMASK(0, 4)
#define I2C_SCL_DRVH_EN				BIT(5)
#define I2C_SDA_DRVH_EN				BIT(6)
#define I2C_CLK_SYNC_EN				BIT(7)

/* 0x44 MM_CNT_VAL_PHL */
#define I2C_MM_CNT_PHASE_VAL1_OFFSET		(8)
#define I2C_MM_CNT_PHASE_VAL1_MASK		BITSMASK(8, 15)
#define I2C_MM_CNT_PHASE_VAL0_OFFSET		(0)
#define I2C_MM_CNT_PHASE_VAL0_MASK		BITSMASK(0, 7)

/* 0x48 MM_CNT_VAL_PHH */
#define I2C_MM_CNT_PHASE_VAL3_OFFSET		(8)
#define I2C_MM_CNT_PHASE_VAL3_MASK		BITSMASK(8, 15)
#define I2C_MM_CNT_PHASE_VAL2_OFFSET		(0)
#define I2C_MM_CNT_PHASE_VAL2_MASK		BITSMASK(0, 7)

/* 0x60 MM_ID_CON0 */
#define I2C_MM_SLAVE_ID_OFFSET			(0)
#define I2C_MM_SLAVE_ID_MASK			BITSMASK(0, 6)

/* 0x68 MM_PACK_CON0 */
#define I2C_MM_PACK_RW0_OFFSET			(0)
#define I2C_MM_PACK_RW1_OFFSET			(1)
#define I2C_MM_PACK_RW2_OFFSET			(2)
#define I2C_MM_PACK_RW_MASK			BITSMASK(0, 3)
#define I2C_MM_PACK_VAL_OFFSET			(4)
#define I2C_MM_PACK_VAL_MASK			BITSMASK(4, 5)

/* 0x6C MM_ACK_VAL */
#define I2C_ACK_PKT0_OFFSET			(8)
#define I2C_ACK_PKT1_OFFSET			(9)
#define I2C_ACK_PKT2_OFFSET			(10)
#define I2C_MM_ACK_ID_MASK			BITSMASK(8, 10)
#define I2C_MM_ACK_DATA_MASK			BITSMASK(0, 7)

/* 0x70 MM_CON0 */
#define I2C_MM_START_EN				BIT(0)
#define I2C_MM_GMODE				BIT(14)
#define I2C_MASTER_EN				BIT(15)

/* 0x74 MM_STATUS */
#define I2C_BUS_BUSY				BIT(0)
#define I2C_MM_ARB_HAD_LOSE			BIT(1)
#define I2C_MM_START_READY			BIT(2)

/* 0x78 MM_FIFO_CON0 */
#define I2C_MM_TX_FIFO_CLR_OFFSET		(1)
#define I2C_MM_RX_FIFO_CLR_OFFSET		(0)

/* 0x7C MM_FIFO_DATA */
#define I2C_MM_FIFO_DATA			BITSMASK(0, 7)

/* 0x80 MM_FIFO_STATUS */
#define MM_TX_FIFO_OVF				BIT(7)
#define MM_TX_FIFO_UDR				BIT(6)
#define MM_TX_FIFO_FUL				BIT(5)
#define MM_TX_FIFO_EMP				BIT(4)
#define MM_RX_FIFO_OVF				BIT(3)
#define MM_RX_FIFO_UDR				BIT(2)
#define MM_RX_FIFO_FUL				BIT(1)
#define MM_RX_FIFO_EMP				BIT(0)
#define MM_FIFO_STATUS_INIT			(MM_TX_FIFO_EMP | MM_RX_FIFO_EMP)

/* 0x84 MM_FIFO_PTR */
#define I2C_MM_RX_FIFO_RPTR_OFFSET		(0)
#define I2C_MM_RX_FIFO_RPTR_MASK		BITSMASK(0, 3)
#define I2C_MM_RX_FIFO_WPTR_OFFSET		(4)
#define I2C_MM_RX_FIFO_WPTR_MASK		BITSMASK(4, 7)
#define I2C_MM_TX_FIFO_RPTR_OFFSET		(8)
#define I2C_MM_TX_FIFO_RPTR_MASK		BITSMASK(8, 11)
#define I2C_MM_TX_FIFO_WPTR_OFFSET		(12)
#define I2C_MM_TX_FIFO_WPTR_MASK		BITSMASK(12, 15)

/* 0xC0 DMA_CON0 */
#define I2C_DMA_SLAVE_SELECT			BIT(5)
#define I2C_DMA_HANDSHAKE_EN			BIT(4)
#define I2C_DMA_HANDSHAKE_MASK			BITSMASK(4, 5)

/* 0xD8 S_FIFO_CON0 */
#define I2C_S_TX_FIFO_CLR_OFFSET		(1)
#define I2C_S_RX_FIFO_CLR_OFFSET		(0)

/* 0x00 ISU_GLOBAL_CTRL */
#define ISU_CR_SW_RST				BIT(0)
#define ISU_I2C_CLK_EN				BIT(8)

/* 0xA0 ISU_EA */
#define PAD_SPI_MOSI				BIT(1)
#define PAD_SPI_MISO				BIT(2)

enum ISU_REGS_OFFSET {
	OFFSET_ISU_GLOBAL_CTRL = 0x0,
	OFFSET_ISU_EA = 0xA0,
};

enum I2C_REGS_OFFSET {
	OFFSET_I2C_INT_CTRL = 0x0,
	OFFSET_MM_PAD_CON0 = 0x40,
	OFFSET_MM_CNT_VAL_PHL = 0x44,
	OFFSET_MM_CNT_VAL_PHH = 0x48,
	OFFSET_MM_CNT_BYTE_VAL_PK0 = 0x54,
	OFFSET_MM_CNT_BYTE_VAL_PK1 = 0x58,
	OFFSET_MM_CNT_BYTE_VAL_PK2 = 0x5C,
	OFFSET_MM_ID_CON0 = 0x60,
	OFFSET_MM_PACK_CON0 = 0x68,
	OFFSET_MM_ACK_VAL = 0x6C,
	OFFSET_MM_CON0 = 0x70,
	OFFSET_MM_STATUS = 0x74,
	OFFSET_MM_FIFO_CON0 = 0x78,
	OFFSET_MM_FIFO_STATUS = 0x80,
	OFFSET_MM_FIFO_PTR = 0x84,
	OFFSET_MM_FIFO_DATA = 0x90,
	OFFSET_DMA_CON0 = 0xC0,
	OFFSET_S_CON0 = 0xC4,
	OFFSET_S_ID_CON0 = 0xC8,
	OFFSET_S_ID_RECEIVED0 = 0xCC,
	OFFSET_S_ID_RECEIVED1 = 0xD0,
	OFFSET_S_FIFO_CON0 = 0xD8,
	OFFSET_S_FIFO_STATUS = 0xE0,
	OFFSET_S_FIFO_PTR = 0xE4,
	OFFSET_S_FIFO_DATA = 0xF0,
};

enum mt3620_trans_op {
	I2C_INVALID_OP = 0,
	I2C_MASTER_WR = 1,
	I2C_MASTER_RD,
	I2C_MASTER_WRRD,
};

struct mt3620_i2c_compatible {
	const struct i2c_adapter_quirks *quirks;
};

enum mt3620_i2c_irq_state {
	/* Wait flags */
	MT3620_I2C_WAITING_FOR_IRQ,
	MT3620_I2C_WAITING_FOR_TX_DMA,
	MT3620_I2C_WAITING_FOR_RX_DMA,

	/* Error flags */
	MT3620_I2C_ARB_LOST,
	MT3620_I2C_ADDR_NACKED,
	MT3620_I2C_DATA_NACKED	
};

struct mt3620_i2c {
	void __iomem *base;
	void __iomem *isubase;

	struct device *dev;
	struct i2c_adapter adap;

	/* Clock and bus speed configuration */
	struct clk *clk_main;
	unsigned int source_clk_hz;
	unsigned int requested_speed_hz;
	unsigned int configured_speed_hz;

	/* DMA state */
	struct dma_chan *dma_tx;
	struct dma_chan *dma_rx;
	struct scatterlist tx_sg;
	struct scatterlist rx_sg;
	unsigned char *tx_buf;
	unsigned char *rx_buf;

	/* Device interrupt state */
	volatile unsigned long irq_state;
	struct completion msg_complete;

	/* In-progress transfer state. */
	struct {
		struct i2c_msg *msgs;
		int num_msgs;

		bool use_dma;
		enum mt3620_trans_op op;
		u16 ack_len;
	} transfer;
};

/* Variant of set_mask_bits() that returns the original value */
static unsigned long mask_bits_and_return_old(volatile unsigned long *bits, unsigned long mask)
{
	unsigned long old, new;

	do {
		old = READ_ONCE(*bits);
		new = old & ~mask;
	} while (cmpxchg(bits, old, new) != old);

	return old;
}

static inline void mt3620_i2c_writew(struct mt3620_i2c *i2c, u8 offset, u16 value)
{
	writew(value, i2c->base + offset);
}

static inline u16 mt3620_i2c_readw(struct mt3620_i2c *i2c, u8 offset)
{
	return readw(i2c->base + offset);
}

static inline void mt3620_isu_writel(struct mt3620_i2c *i2c, u8 offset, u32 value)
{
	writel(value, i2c->isubase + offset);
}

static inline u32 mt3620_isu_readl(struct mt3620_i2c *i2c, u8 offset)
{
	return readl(i2c->isubase + offset);
}

static void mt3620_i2c_set_hw_timing(struct mt3620_i2c *i2c)
{
	u8 adjustment = 0;
	u8 remainder = 0;
	u16 phase = 0, phase0 = 0, phase1 = 0, phase2 = 0, phase3 = 0;
	u16 cnt_val_phl;
	u16 cnt_val_phh;
	u16 reg;

	/*
	 * This routine is responsible for computing the timings for each
	 * of 4 phases:
	 *   phase 0 - SDA to SCL rising edge timing (data setup time)
	 *   phase 1 - SCL rising edge to SDA timing (STOP condition period)
	 *   phase 2 - SDA to SCL falling edge timing (START condition period)
	 *   phase 3 - SCL falling edge to SDA timing (data hold time)
	 */

	/* If synchronization circuit is enabled, there is a delay of 2 cycles. */
	reg = mt3620_i2c_readw(i2c, OFFSET_MM_PAD_CON0);
	if (reg & I2C_CLK_SYNC_EN)
		adjustment += 2;

	/* Also adjust for deglitching. */
	if ((reg & I2C_DE_CNT_MASK) != 0)
		adjustment += ((reg & I2C_DE_CNT_MASK) + 4);

	/* Compute ratio of source clock cycles to desired I2C clock speed;
	 * split up cycles into 4 phases of equal length. */
	WARN_ON(i2c->requested_speed_hz != I2C_CLK_100K &&
		i2c->requested_speed_hz != I2C_CLK_400K &&
		i2c->requested_speed_hz != I2C_CLK_1M);

	phase = ((i2c->source_clk_hz / i2c->requested_speed_hz) / 4) - 1;
	remainder = (i2c->source_clk_hz / i2c->requested_speed_hz) % 4;

	/* Assign base value to each phase. */
	phase0 = phase1 = phase2 = phase3 = phase;

	/* Distribute remainder between phases. */
	switch (remainder) {
		case 1:
			phase0 += 1;
			break;

		case 2:
			phase0 += 1;
			phase2 += 1;
			break;

		case 3:
			phase0 += 1;
			phase2 += 1;
			phase3 += 1;
			break;

		default:
			break;
	}

	/* Move a few cycles between phases 0 and 2, depending on clock speed. */
	if (i2c->requested_speed_hz == I2C_CLK_400K) {
		phase0 += 5;
		phase2 -= 5;
	} else {
		phase0 += 3;
		phase2 -= 3;
	}

	/* Accommodate sync/deglitching delays. */
	if (adjustment > 0 && phase1 > 1) {
		u16 removed = MIN(adjustment, phase1 - 1);
		adjustment -= removed;
		phase1 -= removed;
	}
	if (adjustment > 0 && phase2 > 1) {
		u16 removed = MIN(adjustment, phase2 - 1);
		adjustment -= removed;
		phase2 -= removed;
	}
	if (adjustment > 0 && phase0 > 1) {
		u16 removed = MIN(adjustment, phase0 - 1);
		adjustment -= removed;
		phase0 -= removed;
	}

	/* Write counts back to registers. */
	cnt_val_phl = (phase1 << I2C_MM_CNT_PHASE_VAL1_OFFSET) |
		(phase0 << I2C_MM_CNT_PHASE_VAL0_OFFSET);

	cnt_val_phh = (phase3 << I2C_MM_CNT_PHASE_VAL3_OFFSET) |
		(phase2 << I2C_MM_CNT_PHASE_VAL2_OFFSET);

	mt3620_i2c_writew(i2c, OFFSET_MM_CNT_VAL_PHL, cnt_val_phl);
	mt3620_i2c_writew(i2c, OFFSET_MM_CNT_VAL_PHH, cnt_val_phh);

	/* Cache the last-configured speed. */
	i2c->configured_speed_hz = i2c->requested_speed_hz;
}

static void mt3620_i2c_reset_hw(struct mt3620_i2c *i2c)
{
	u32 reg;

	/*
	 * TODO: #68798
	 * 
	 * The EA enablement more appropriately belongs in the firmware code
	 * responsible for enabling the I2C function on this ISU. It's not
	 * clear if software reset of the whole ISU (across all functions)
	 * belongs here.  We will address this as part of the above listed
	 * work item.
	 */

	/* Clear cached state. */
	i2c->configured_speed_hz = 0;

	/* Enable EA for SDA/SCL. */
	reg = mt3620_isu_readl(i2c, OFFSET_ISU_EA);
	if (!(reg & PAD_SPI_MISO) || !(reg & PAD_SPI_MOSI))
		mt3620_isu_writel(i2c, OFFSET_ISU_EA, reg | PAD_SPI_MISO | PAD_SPI_MOSI);
}

static void mt3620_i2c_init_hw(struct mt3620_i2c *i2c)
{
	u16 reg;

	/* N.B. We assume that the ISU is already in the appropriate state
	 * and has been appropriately configured for I2C usage. */

	/*
	 * Enable I2C master mode, disable slave mode; this also cancels
	 * any in-progress PIO transfers.
	 */
	mt3620_i2c_writew(i2c, OFFSET_MM_CON0, I2C_MASTER_EN);
	mt3620_i2c_writew(i2c, OFFSET_S_CON0, 0);

	/* Clear master mode TX/RX FIFO */
	reg = (1 << I2C_MM_TX_FIFO_CLR_OFFSET) |
		(1 << I2C_MM_RX_FIFO_CLR_OFFSET);
	mt3620_i2c_writew(i2c, OFFSET_MM_FIFO_CON0, reg);

	/* Enable internal synchronization; set deglitch counting number to 1. */
	reg = I2C_CLK_SYNC_EN | (1 << I2C_DE_CNT_OFFSET);
	mt3620_i2c_writew(i2c, OFFSET_MM_PAD_CON0, reg);

	/* Clear interrupt status, disable interrupts. */
	mt3620_i2c_writew(i2c, OFFSET_I2C_INT_CTRL, I2C_MM_INT_STA);

	/* Configure timings based on desired speed. */
	if (i2c->requested_speed_hz != i2c->configured_speed_hz)
		mt3620_i2c_set_hw_timing(i2c);
}

static void mt3620_i2c_uninit_hw(struct mt3620_i2c *i2c)
{
	/* Disable interrupts. */
	mt3620_i2c_writew(i2c, OFFSET_I2C_INT_CTRL, 0);

	/* Disable I2C master mode, slave mode. */
	mt3620_i2c_writew(i2c, OFFSET_MM_CON0, 0);
	mt3620_i2c_writew(i2c, OFFSET_S_CON0, 0);
}

static void mt3620_i2c_dma_unmap_tx(struct mt3620_i2c *i2c)
{
	dma_unmap_single(i2c->dma_tx->device->dev,
				sg_dma_address(&i2c->tx_sg),
				sg_dma_len(&i2c->tx_sg), DMA_TO_DEVICE);
}

static void mt3620_i2c_dma_unmap_rx(struct mt3620_i2c *i2c)
{
	dma_unmap_single(i2c->dma_rx->device->dev,
				sg_dma_address(&i2c->rx_sg),
				sg_dma_len(&i2c->rx_sg), DMA_FROM_DEVICE);
}

static void mt3620_i2c_abort_and_unmap_dma(struct mt3620_i2c *i2c)
{
	if (test_and_clear_bit(MT3620_I2C_WAITING_FOR_TX_DMA, &i2c->irq_state)) {
		dmaengine_terminate_all(i2c->dma_tx);
		mt3620_i2c_dma_unmap_tx(i2c);
	}

	if (test_and_clear_bit(MT3620_I2C_WAITING_FOR_RX_DMA, &i2c->irq_state)) {
		dmaengine_terminate_all(i2c->dma_rx);
		mt3620_i2c_dma_unmap_rx(i2c);
	}
}

static void mt3620_i2c_dma_tx_callback(void *data)
{
	struct mt3620_i2c *i2c = data;

	/* Clear TX waiting flag; make sure we were expecting it. */
	u32 prev_state = mask_bits_and_return_old(&i2c->irq_state, BIT(MT3620_I2C_WAITING_FOR_TX_DMA));
	if (!(prev_state & BIT(MT3620_I2C_WAITING_FOR_TX_DMA))) {
		WARN_ON(!(prev_state & BIT(MT3620_I2C_WAITING_FOR_TX_DMA)));
		return;
	}

	mt3620_i2c_dma_unmap_tx(i2c);

	/* If this was the last wait, then complete the message. */
	if (!(prev_state & (BIT(MT3620_I2C_WAITING_FOR_IRQ) | BIT(MT3620_I2C_WAITING_FOR_RX_DMA)))) {
		complete(&i2c->msg_complete);
	}
}

static void mt3620_i2c_dma_rx_callback(void *data)
{
	struct mt3620_i2c *i2c = data;

	/* Clear RX waiting flag; make sure we were expecting it. */
	u32 prev_state = mask_bits_and_return_old(&i2c->irq_state, BIT(MT3620_I2C_WAITING_FOR_RX_DMA));
	if (!(prev_state & BIT(MT3620_I2C_WAITING_FOR_RX_DMA))) {
		WARN_ON(!(prev_state & BIT(MT3620_I2C_WAITING_FOR_RX_DMA)));
		return;
	}

	mt3620_i2c_dma_unmap_rx(i2c);

	/* If this was the last wait, then complete the message. */
	if (!(prev_state & (BIT(MT3620_I2C_WAITING_FOR_IRQ) | BIT(MT3620_I2C_WAITING_FOR_TX_DMA)))) {
		complete(&i2c->msg_complete);
	}
}

static int mt3620_i2c_dma_submit(struct mt3620_i2c *i2c, struct dma_chan *chan,
			      struct scatterlist *sg, dma_addr_t dma_addr,
			      u16 dma_len, enum dma_transfer_direction dir, bool tx)
{
	struct dma_async_tx_descriptor *txdesc;
	dma_cookie_t cookie;
	bool in_use;

	in_use = test_and_set_bit(
		tx ? MT3620_I2C_WAITING_FOR_TX_DMA : MT3620_I2C_WAITING_FOR_RX_DMA,
		&i2c->irq_state);

	if (in_use) {
		BUG();
		mt3620_i2c_abort_and_unmap_dma(i2c);
		return -EIO;
	}

	sg_dma_len(sg) = dma_len;
	sg_dma_address(sg) = dma_addr;
	txdesc = dmaengine_prep_slave_sg(chan, sg, /*sg_len=*/1, dir,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc) {
		dev_err(chan->device->dev, "dma prep slave sg failed.\n");
		mt3620_i2c_abort_and_unmap_dma(i2c);
		return -ENOMEM;
	}

	txdesc->callback = tx ? mt3620_i2c_dma_tx_callback : mt3620_i2c_dma_rx_callback;
	txdesc->callback_param = i2c;
	cookie = dmaengine_submit(txdesc);
	if (dma_submit_error(cookie)) {
		dev_err(chan->device->dev, "submitting dma failed.\n");
		mt3620_i2c_abort_and_unmap_dma(i2c);
		return -ENOMEM;
	}

	dma_async_issue_pending(chan);

	return 0;
}

static int mt3620_i2c_setup_tx_dma(struct mt3620_i2c *i2c, const struct i2c_msg *msg)
{
	dma_addr_t dma_tx_addr;
	int ret;

	if (msg->len > MT3620_I2C_DMA_BUF_SIZE)
		return -ENOMEM;

	/* Copy input data into DMA TX buffer. */
	if (msg->len > 0) {
		memcpy(i2c->tx_buf, msg->buf, msg->len);
	}

	dma_tx_addr = dma_map_single(i2c->dma_tx->device->dev,
						i2c->tx_buf, msg->len,
						DMA_TO_DEVICE);

	if (dma_mapping_error(i2c->dma_tx->device->dev, dma_tx_addr)) {
		dev_err(i2c->dma_tx->device->dev, "dma map tx failed\n");
		return -ENOMEM;
	}

	dev_dbg(i2c->dma_tx->device->dev, "submitting %u-byte TX I2C DMA\n", msg->len);

	ret = mt3620_i2c_dma_submit(i2c, i2c->dma_tx, &(i2c->tx_sg),
					dma_tx_addr, msg->len,
					DMA_MEM_TO_DEV, /*tx=*/true);

	if (ret) {
		dev_err(i2c->dma_tx->device->dev, "dma submit tx failed\n");
		return ret;
	}

	return 0;
}

static int mt3620_i2c_setup_rx_dma(struct mt3620_i2c *i2c, const struct i2c_msg *msg)
{
	dma_addr_t dma_rx_addr;
	int ret;

	if (msg->len > MT3620_I2C_DMA_BUF_SIZE)
		return -ENOMEM;

	dma_rx_addr = dma_map_single(i2c->dma_rx->device->dev,
					i2c->rx_buf, msg->len,
					DMA_FROM_DEVICE);
						
	if (dma_mapping_error(i2c->dma_rx->device->dev, dma_rx_addr)) {
		dev_err(i2c->dma_rx->device->dev, "dma map rx failed\n");
		return -ENOMEM;
	}

	dev_dbg(i2c->dma_rx->device->dev, "submitting %u-byte RX I2C DMA\n", msg->len);

	ret = mt3620_i2c_dma_submit(i2c, i2c->dma_rx, &(i2c->rx_sg),
					dma_rx_addr, msg->len,
					DMA_DEV_TO_MEM, /*tx=*/false);

	if (ret) {
		dev_err(i2c->dma_rx->device->dev, "dma submit rx failed\n");
		return ret;
	}

	return 0;
}

static int mt3620_i2c_setup_dma(struct mt3620_i2c *i2c)
{
	int ret;
	int i;

	ret = 0;
	for (i = 0; i < i2c->transfer.num_msgs; ++i) {
		if (i2c->transfer.msgs[i].flags & I2C_M_RD) {
			ret = mt3620_i2c_setup_rx_dma(i2c, &i2c->transfer.msgs[i]);
		} else {
			ret = mt3620_i2c_setup_tx_dma(i2c, &i2c->transfer.msgs[i]);
		}

		if (ret)
			break;
	}

	if (ret) {
		mt3620_i2c_abort_and_unmap_dma(i2c);
	}

	return ret;
}

static struct dma_chan *mt3620_i2c_request_dma_chan(struct device *dev,
						 char *chan_name,
						 enum dma_transfer_direction dir,
						 dma_addr_t port_addr)
{
	struct dma_chan *chan;
	struct dma_slave_config cfg;
	int ret;

	chan = dma_request_slave_channel(dev, chan_name);
	if (!chan) {
		dev_err(dev, "request DMA channel failed!\n");
		return chan;
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.direction = dir;
	cfg.slave_id = GDMA_SLAVE_ID_I2C;
	if (dir == DMA_MEM_TO_DEV) {
		cfg.dst_addr = port_addr;
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	} else {
		cfg.src_addr = port_addr;
		cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	}

	ret = dmaengine_slave_config(chan, &cfg);
	if (ret) {
		dev_err(dev, "slave_config failed for %s (%d)\n", chan_name, ret);
		dma_release_channel(chan);
		return NULL;
	}

	return chan;
}

static void mt3620_i2c_release_dma(struct mt3620_i2c *i2c)
{
	if (i2c->dma_tx) {
		dma_release_channel(i2c->dma_tx);
		i2c->dma_tx = NULL;
	}

	if (i2c->dma_rx) {
		dma_release_channel(i2c->dma_rx);
		i2c->dma_rx = NULL;
	}
}

static u8 mt3620_i2c_get_rx_fifo_size(struct mt3620_i2c *i2c)
{
	u16 fifo_ptr;
	u16 fifo_status;
	u8 rx_wptr;
	u8 rx_rptr;
	u8 size;

	fifo_ptr = mt3620_i2c_readw(i2c, OFFSET_MM_FIFO_PTR);
	fifo_status = mt3620_i2c_readw(i2c, OFFSET_MM_FIFO_STATUS);

	WARN_ON(fifo_status & (MM_RX_FIFO_OVF | MM_RX_FIFO_UDR));

	rx_wptr = (fifo_ptr & I2C_MM_RX_FIFO_WPTR_MASK) >> I2C_MM_RX_FIFO_WPTR_OFFSET;
	rx_rptr = (fifo_ptr & I2C_MM_RX_FIFO_RPTR_MASK) >> I2C_MM_RX_FIFO_RPTR_OFFSET;

	if (fifo_status & MM_RX_FIFO_FUL)
		size = MT3620_I2C_FIFO_SIZE;
	else if (rx_wptr >= rx_rptr)
		size = (rx_wptr - rx_rptr) & (MT3620_I2C_FIFO_SIZE - 1);
	else
		size = (MT3620_I2C_FIFO_SIZE - rx_rptr) + rx_wptr;

	WARN_ON(size > MT3620_I2C_FIFO_SIZE);

	return size;
}

static int mt3620_i2c_do_transfer(struct mt3620_i2c *i2c)
{
	int i;
	u8 *data_buf;
	u16 read_len;
	u16 write_len;
	u16 requested_len;
	u16 addr_reg;
	u16 packcon_reg;
	u16 status_reg;
	u16 fifo_status;
	u16 control_reg;
	int read_msg_index;
	int ret;

	/* Initialize the hardware to a known state (but not full reset).
	 * This will clear any underflow or overflow FIFO errors, in case
	 * we hit them during the preceding operation. */
	mt3620_i2c_init_hw(i2c);

	/* Check if master is ready; if it's not, then something's gone wrong. */
	status_reg = mt3620_i2c_readw(i2c, OFFSET_MM_STATUS);
	if (!(status_reg & I2C_MM_START_READY)) {
		dev_err_ratelimited(i2c->dev, "I2C master not ready to start; status=0x%x\n",
					(u32)status_reg);
		mt3620_i2c_reset_hw(i2c);
		return -EBUSY;
	}

	/* Write slave ID for target of messages. By this point, we're guaranteed
	 * that all messages are going to the same place. */
	addr_reg = i2c->transfer.msgs[0].addr & I2C_MM_SLAVE_ID_MASK;
	mt3620_i2c_writew(i2c, OFFSET_MM_ID_CON0, addr_reg);

	/*
	 * In write-then-read mode, we use the 'general' mode of the controller,
	 * which supports sending multiple messages between the first start
	 * condition and the stop condition.
	 */
	if (i2c->transfer.op == I2C_MASTER_WRRD) {
		mt3620_i2c_writew(i2c, OFFSET_MM_CON0,
			mt3620_i2c_readw(i2c, OFFSET_MM_CON0) | I2C_MM_GMODE);
	}

	/* Set transfer mode and length. */
	packcon_reg = (i2c->transfer.num_msgs - 1) << I2C_MM_PACK_VAL_OFFSET;
	for (i = 0; i < i2c->transfer.num_msgs; ++i) {
		bool is_read = (i2c->transfer.msgs[i].flags & I2C_M_RD);
		if (is_read) {
			packcon_reg |= 1 << (I2C_MM_PACK_RW0_OFFSET + i);
		}

		mt3620_i2c_writew(i2c,
			OFFSET_MM_CNT_BYTE_VAL_PK0 + (i * 4),
			i2c->transfer.msgs[i].len);
	}

	mt3620_i2c_writew(i2c, OFFSET_MM_PACK_CON0, packcon_reg);

	/* Prepare buffer data to start transfer */
	if (i2c->transfer.use_dma) {
		mt3620_i2c_writew(i2c, OFFSET_DMA_CON0, 0);

		ret = mt3620_i2c_setup_dma(i2c);
		if (ret < 0)
			return ret;

		/* Enable DMA handshaking with I2C. */
		mt3620_i2c_writew(i2c, OFFSET_DMA_CON0, I2C_DMA_HANDSHAKE_EN);
	} else {
		/* For write and write-then-read, we need to send data to FIFO. */
		if (i2c->transfer.op != I2C_MASTER_RD) {
			data_buf = i2c->transfer.msgs[0].buf;
			write_len = i2c->transfer.msgs[0].len;

			while (write_len--)
				writew_relaxed(*(data_buf++),
					       i2c->base + OFFSET_MM_FIFO_DATA);
		}
	}

	/* Enable master interrupt and update state to expect an interrupt. */
	mt3620_i2c_writew(i2c, OFFSET_I2C_INT_CTRL, I2C_MM_INT_EN);
	set_bit(MT3620_I2C_WAITING_FOR_IRQ, &i2c->irq_state);

	/* Trigger I2C operation. */
	control_reg = mt3620_i2c_readw(i2c, OFFSET_MM_CON0) | I2C_MM_START_EN;
	mt3620_i2c_writew(i2c, OFFSET_MM_CON0, control_reg);

	/* Wait for all I/O to complete, or for our timeout to occur. */
	ret = wait_for_completion_interruptible_timeout(&i2c->msg_complete, i2c->adap.timeout);

	/* Disable interrupts */
	mt3620_i2c_writew(i2c, OFFSET_I2C_INT_CTRL, 0);

	/* Handle timeout. */
	if (ret == 0) {
		dev_warn(i2c->dev, "transfer timeout after %d ms\n", jiffies_to_msecs(i2c->adap.timeout));

		/*
		 * We shouldn't get any interrupts now, but the operation may
		 * still yet complete. Update our state.
		 */

		clear_bit(MT3620_I2C_WAITING_FOR_IRQ, &i2c->irq_state);

		/* If we were using DMA, then we need to clean up any partial state. */
		if (i2c->transfer.use_dma) {
			mt3620_i2c_abort_and_unmap_dma(i2c);
		}


		/* Reset hardware. */
		mt3620_i2c_reset_hw(i2c);
		return -ETIMEDOUT;
	}

	/* Report on arbitration loss (for multi-master scenario). */
	if (test_bit(MT3620_I2C_ARB_LOST, &i2c->irq_state)) {
		mt3620_i2c_abort_and_unmap_dma(i2c);

		/*
		 * The I2C infrastructure expects to receive -EAGAIN on
		 * arbitration loss. It will trigger our caller to conditionally
		 * invoke retry logic.
		 */
		return -EAGAIN;
	}

	/* Report on a NACK for address ID. */
	if (test_bit(MT3620_I2C_ADDR_NACKED, &i2c->irq_state)) {
		dev_warn_ratelimited(i2c->dev, "slave id ACK error; addr=0x%x\n", (u32)i2c->transfer.msgs[0].addr);
		mt3620_i2c_abort_and_unmap_dma(i2c);

		return -ENXIO;
	}

	/* 
	 * Report partial read/write as a failure (for now).
	 * 
	 * TODO: 68799
	 * We need to find a Linux-accepted way to make a partial read/write
	 * succeed. This only seems to be supporteed today with SMBus operations
	 */
	if (test_bit(MT3620_I2C_DATA_NACKED, &i2c->irq_state)) {
		dev_err(i2c->dev, "data ACK error; addr=0x%x\n", (u32)i2c->transfer.msgs[0].addr);
		mt3620_i2c_abort_and_unmap_dma(i2c);

		return -ENXIO;
	}

	/* Perform copy-out from a read. */
	requested_len = 0;
	read_len = 0;
	if (i2c->transfer.op != I2C_MASTER_WR) {
		read_msg_index = (i2c->transfer.op == I2C_MASTER_RD) ? 0 : 1;

		data_buf = i2c->transfer.msgs[read_msg_index].buf;
		requested_len = i2c->transfer.msgs[read_msg_index].len;

		if (i2c->transfer.use_dma) {
			if (requested_len > 0) {
				memcpy(data_buf, i2c->rx_buf, requested_len);
			}
		} else {
			read_len = mt3620_i2c_get_rx_fifo_size(i2c);
			if (read_len == requested_len) {
				while (requested_len--)
					*(data_buf++) = readw_relaxed(i2c->base + OFFSET_MM_FIFO_DATA);
			} else {
				/* A partial read occurred. */
				mt3620_i2c_reset_hw(i2c);
				return -EIO;
			}
		}
	}

	/* Make sure RX FIFO is empty, with no underflow or overflow conditions. */
	fifo_status = mt3620_i2c_readw(i2c, OFFSET_MM_FIFO_STATUS);
	if (fifo_status != MM_FIFO_STATUS_INIT) {
		dev_warn(i2c->dev, "FIFO not in clean state; fifo_status=0x%x, "
			"used_dma=%u, op_type=%u, req_len=%u, read_len=%u\n",
			fifo_status, i2c->transfer.use_dma, i2c->transfer.op,
			requested_len, read_len);

		/* We should never hit this in a PIO scenario, but we are seeing
		 * RX FIFO underflow errors on DMA despite receiving back valid
		 * data. */
		if (!i2c->transfer.use_dma) {
			mt3620_i2c_reset_hw(i2c);
			return -EIO;
		}
	}

	return 0;
}

static int mt3620_i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int ret;
	int i;
	u32 max_rx_fifo_usage;
	u32 max_tx_fifo_usage;
	u32 old_irq_state;
	struct mt3620_i2c *i2c = i2c_get_adapdata(adap);

	i2c->irq_state = 0;

	/*
	 * We only support three I/O cases: single read, single write,
	 * combined write-then-read to same slave.  Each operation can transfer
	 * 0 or more bytes.
	 */

	i2c->transfer.msgs = msgs;
	i2c->transfer.num_msgs = num;

	/*
	 * We expect to receive 1 ACK bit for each operation, and on top
	 * of that, to also receive n more ACK bits for each byte written.
	 */
	i2c->transfer.ack_len = num;
	if ((num == 2) && (msgs[0].addr == msgs[1].addr) &&
	    (!(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD))) {
		i2c->transfer.op = I2C_MASTER_WRRD;
		i2c->transfer.ack_len += msgs[0].len;
	} else if (num == 1) {
		if (msgs[0].flags & I2C_M_RD) {
			i2c->transfer.op = I2C_MASTER_RD;
		} else {
			i2c->transfer.op = I2C_MASTER_WR;
			i2c->transfer.ack_len += msgs[0].len;
		}
	} else {
		dev_err(i2c->dev, "Unsupported msgs: num=%d\n", num);
		return -EOPNOTSUPP;
	}

	max_rx_fifo_usage = 0;
	max_tx_fifo_usage = 0;
	for (i = 0; i < num; ++i) {
		if (msgs[i].len > 0 && msgs[i].buf == NULL)
			return -EINVAL;

		if (msgs[i].flags & I2C_M_RD) {
			max_rx_fifo_usage += msgs[i].len;
			
			/* N.B. For now, we block 0-byte reads, because they
			 * cause the MT3620 I2C master controller to go into
			 * a stuck state; we will revisit this in the future. */
			if (msgs[i].len == 0)
				return -EOPNOTSUPP;
		} else {
			max_tx_fifo_usage += msgs[i].len;
		}
	}

	/* N.B. For now, we choose to use DMA if we will otherwise overflow
	 * our RX or TX buffer. In the future, we could increase complexity
	 * and separately choose whether to enable RX DMA vs. TX DMA. */
	i2c->transfer.use_dma =
		(max_rx_fifo_usage > MT3620_I2C_FIFO_SIZE) ||
		(max_tx_fifo_usage > MT3620_I2C_FIFO_SIZE);

	reinit_completion(&i2c->msg_complete);

	ret = clk_prepare_enable(i2c->clk_main);
	if (ret) {
		dev_err(i2c->dev, "clock enable failed!\n");
		return ret;
	}

	ret = mt3620_i2c_do_transfer(i2c);
	if (ret < 0)
		goto err_exit;

	/* The return value is number of executed messages. */
	ret = num;

err_exit:
	/* Clear state. */
	old_irq_state = mask_bits_and_return_old(&i2c->irq_state, /*mask=*/0xFFFFFFFF);

	/* There shouldn't be anything left in flight. The WARN_ON() invocations
	 * will log good details but move on, letting the BUG_ON() do its thing. */
	WARN_ON(old_irq_state & MT3620_I2C_WAITING_FOR_IRQ);
	WARN_ON(old_irq_state & MT3620_I2C_WAITING_FOR_RX_DMA);
	WARN_ON(old_irq_state & MT3620_I2C_WAITING_FOR_TX_DMA);
	BUG_ON(old_irq_state &
		(MT3620_I2C_WAITING_FOR_IRQ | MT3620_I2C_WAITING_FOR_RX_DMA | MT3620_I2C_WAITING_FOR_TX_DMA));

	clk_disable_unprepare(i2c->clk_main);
	return ret;
}

static bool mt3620_i2c_check_for_address_nack(struct mt3620_i2c *i2c, int expected_msg_count)
{
	u8 i;
	u16 ack_val;

	/* Retrieve the ACK bits from the transmission of slave addresses. */
	ack_val = (mt3620_i2c_readw(i2c, OFFSET_MM_ACK_VAL) & I2C_MM_ACK_ID_MASK) >>
		  I2C_ACK_PKT0_OFFSET;

	/*
	 * A successful ACK is a 0 bit (slave pulling low); if we see a 1 bit for
	 * any of the messages we expected to have sent, then report that there was
	 * an error.
	 */
	for (i = 0; i < expected_msg_count; i++) {
		if (ack_val & (0x1 << i))
			return true;
	}

	return false;
}

static bool mt3620_i2c_check_for_data_nack(struct mt3620_i2c *i2c, u16 len)
{
	u8 i;
	u16 ack_val;
	u16 ack_len;

	if (len > 8)
		ack_len = 8;
	else
		ack_len = len;

	/* Retrieve the ACK bits from the transmission of data bits. */
	ack_val = mt3620_i2c_readw(i2c, OFFSET_MM_ACK_VAL) & I2C_MM_ACK_DATA_MASK;

	/*
	 * A successful ACK is a 0 bit (slave pulling low); if we see a 1 bit
	 * anywhere, then report error.
	 */
	for (i = 0; i < ack_len; i++) {
		if (ack_val & (0x1 << i))
			return true;
	}

	return false;
}

static irqreturn_t mt3620_i2c_irq(int irqno, void *dev_id)
{
	struct mt3620_i2c *i2c = dev_id;
	u16 intr_stat;
	u32 prev_state;
	u16 status_reg;

	/* See if this was for us. */
	intr_stat = mt3620_i2c_readw(i2c, OFFSET_I2C_INT_CTRL);
	if (!(intr_stat & I2C_MM_INT_STA)) {
		/* It wasn't for us. */
		return IRQ_NONE;
	}

	/* Write 1 to clear. */
	mt3620_i2c_writew(i2c, OFFSET_I2C_INT_CTRL, intr_stat);

	/* Clear waiting flag; make sure we were expecting it. */
	prev_state = mask_bits_and_return_old(&i2c->irq_state, BIT(MT3620_I2C_WAITING_FOR_IRQ));
	if (!(prev_state & BIT(MT3620_I2C_WAITING_FOR_IRQ))) {
		dev_err(i2c->dev, "received unexpected I2C IRQ\n");
		return IRQ_HANDLED;
	}

	/* Check if the master had an arbitration loss. */
	status_reg = mt3620_i2c_readw(i2c, OFFSET_MM_STATUS);
	if (status_reg & I2C_MM_ARB_HAD_LOSE) {
		dev_warn_ratelimited(i2c->dev, "I2C arbitration loss; status=0x%x\n",
					(u32)status_reg);
		set_bit(MT3620_I2C_ARB_LOST, &i2c->irq_state);
		/* We also need to write-1-to-clear the bit in hardware. */
		mt3620_i2c_writew(i2c, OFFSET_MM_STATUS, status_reg);
	}

	/* Check for a NACK on address send from any of the messages. */
	if (mt3620_i2c_check_for_address_nack(i2c, i2c->transfer.num_msgs))
		set_bit(MT3620_I2C_ADDR_NACKED, &i2c->irq_state);

	/* Check for a NACK on data send from any of the messages. */
	if (mt3620_i2c_check_for_data_nack(i2c, i2c->transfer.ack_len))
		set_bit(MT3620_I2C_DATA_NACKED, &i2c->irq_state);

	/* If this was the last wait, then complete the message. */
	if (!(prev_state & (BIT(MT3620_I2C_WAITING_FOR_TX_DMA) | BIT(MT3620_I2C_WAITING_FOR_RX_DMA))))
		complete(&i2c->msg_complete);

	return IRQ_HANDLED;
}

static u32 mt3620_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static int mt3620_i2c_configure_speed(struct mt3620_i2c *i2c, u32 speed_hz)
{
	switch (speed_hz) {
		case I2C_CLK_100K:
		case I2C_CLK_400K:
		case I2C_CLK_1M:
			break;
		default:
			dev_err(i2c->dev, "Unsupported speed (%u Hz)\n", speed_hz);
			return -EOPNOTSUPP;
	}

	i2c->requested_speed_hz = speed_hz;
	return 0;
}

static int mt3620_i2c_set_speed(struct i2c_adapter *adap, u32 speed_hz)
{
	struct mt3620_i2c *i2c = i2c_get_adapdata(adap);
	return mt3620_i2c_configure_speed(i2c, speed_hz);
}

static const struct i2c_algorithm mt3620_i2c_algorithm = {
	.master_xfer = mt3620_i2c_transfer,
	.functionality = mt3620_i2c_functionality,
	.set_speed = mt3620_i2c_set_speed
};

static int mt3620_i2c_parse_dt(struct device_node *np, struct mt3620_i2c *i2c)
{
	int ret;
	u32 desired_speed_hz;

	ret = of_property_read_u32(np, "clock-frequency", &desired_speed_hz);
	if (ret < 0)
		desired_speed_hz = MT3620_I2C_CLK_DEFAULT;

	return mt3620_i2c_configure_speed(i2c, desired_speed_hz);
}

/* We only support 1 message, or the write-then-read special case of 2 messages */
static const struct i2c_adapter_quirks mt3620_i2c_quirks = {
	.flags = I2C_AQ_COMB_WRITE_THEN_READ,
	.max_num_msgs = 2,
	.max_read_len = MT3620_I2C_MAX_MSG_LEN,
	.max_write_len = MT3620_I2C_MAX_MSG_LEN,
	.max_comb_1st_msg_len = MT3620_I2C_MAX_MSG_LEN,
	.max_comb_2nd_msg_len = MT3620_I2C_MAX_MSG_LEN,
};

static const struct mt3620_i2c_compatible mt3620_compat = {
	.quirks = &mt3620_i2c_quirks,
};

static const struct of_device_id mt3620_i2c_of_match[] = {
	{ .compatible = "mediatek,mt3620-i2c", .data = &mt3620_compat },
	{}
};
MODULE_DEVICE_TABLE(of, mt3620_i2c_of_match);

static int mt3620_i2c_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	int ret = 0;
	struct mt3620_i2c *i2c;
	struct resource *res;
	dma_addr_t phy_fifo_data_addr;
	int irq;

	dev_info(&pdev->dev, "probe\n");

	i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2c);
	i2c->dev = &pdev->dev;

	of_id = of_match_node(mt3620_i2c_of_match, pdev->dev.of_node);
	if (!of_id) {
		dev_err(&pdev->dev, "failed to match OF node for i2c\n");
		return -EINVAL;
	}

	ret = mt3620_i2c_parse_dt(pdev->dev.of_node, i2c);
	if (ret) {
		dev_err(&pdev->dev, "failed to parse dtc!\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to determine i2c base address\n");
		return -ENODEV;
	}

	i2c->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2c->base)) {
		dev_err(&pdev->dev, "failed to map resource 0\n");
		return PTR_ERR(i2c->base);
	}

	phy_fifo_data_addr = (dma_addr_t)(res->start + OFFSET_MM_FIFO_DATA);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "failed to determine ISU base address\n");
		return -ENODEV;
	}

	i2c->isubase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2c->isubase)) {
		dev_err(&pdev->dev, "failed to map resource 1\n");
		return PTR_ERR(i2c->isubase);
	}

	i2c->clk_main = devm_clk_get(&pdev->dev, "main");
	if (IS_ERR(i2c->clk_main)) {
		dev_err(&pdev->dev, "failed to get clock (%d)\n", ret);
		return PTR_ERR(i2c->clk_main);
	}

	i2c->source_clk_hz = clk_get_rate(i2c->clk_main);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get irq (%d)\n", irq);
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, mt3620_i2c_irq,
			       IRQF_TRIGGER_NONE, dev_name(&pdev->dev), i2c);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq %i\n", ret);
		return ret;
	}

	i2c->dma_tx = mt3620_i2c_request_dma_chan(
		&pdev->dev, "i2c-gdma-tx", DMA_MEM_TO_DEV, phy_fifo_data_addr);
	i2c->dma_rx = mt3620_i2c_request_dma_chan(
		&pdev->dev, "i2c-gdma-rx", DMA_DEV_TO_MEM, phy_fifo_data_addr);

	if (!i2c->dma_tx || !i2c->dma_rx) {
		dev_err(&pdev->dev, "request dma channel fail.\n");
		return -EBADR;
	}

	sg_init_table(&i2c->tx_sg, 1);
	sg_init_table(&i2c->rx_sg, 1);

	i2c->tx_buf = devm_kzalloc(&pdev->dev, MT3620_I2C_DMA_BUF_SIZE, GFP_KERNEL);
	i2c->rx_buf = devm_kzalloc(&pdev->dev, MT3620_I2C_DMA_BUF_SIZE, GFP_KERNEL);

	if (!i2c->tx_buf || !i2c->rx_buf) {
		dev_err(&pdev->dev, "failed to allocate DMA buffer\n");
		return -ENOMEM;
	}

	init_completion(&i2c->msg_complete);

	i2c->adap.dev.of_node = pdev->dev.of_node;
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.class = I2C_CLASS_DEPRECATED;
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &mt3620_i2c_algorithm;
	i2c->adap.timeout = msecs_to_jiffies(MT3620_I2C_DEFAULT_TIMEOUT_MS);
	i2c->adap.retries = MT3620_I2C_DEFAULT_RETRIES;
	i2c->adap.nr = pdev->id;
	strlcpy(i2c->adap.name, dev_name(&pdev->dev),
		sizeof(i2c->adap.name));

	if (of_id->data != NULL)
		i2c->adap.quirks = ((struct mt3620_i2c_compatible *)(of_id->data))->quirks;

	i2c_set_adapdata(&i2c->adap, i2c);

	ret = i2c_add_numbered_adapter(&i2c->adap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add i2c bus to i2c core\n");
		return ret;
	}

	mt3620_i2c_reset_hw(i2c);

	return 0;
}

static int mt3620_i2c_remove(struct platform_device *pdev)
{
	struct mt3620_i2c *i2c = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "remove\n");

	/* There shouldn't be anything left in flight. The WARN_ON() invocations
	 * will log good details but move on, letting the BUG_ON() do its thing. */
	WARN_ON(test_bit(MT3620_I2C_WAITING_FOR_IRQ, &i2c->irq_state));
	WARN_ON(test_bit(MT3620_I2C_WAITING_FOR_RX_DMA, &i2c->irq_state));
	WARN_ON(test_bit(MT3620_I2C_WAITING_FOR_TX_DMA, &i2c->irq_state));
	BUG_ON(test_bit(MT3620_I2C_WAITING_FOR_IRQ, &i2c->irq_state) ||
		test_bit(MT3620_I2C_WAITING_FOR_RX_DMA, &i2c->irq_state) ||
		test_bit(MT3620_I2C_WAITING_FOR_TX_DMA, &i2c->irq_state));

	/* Remove adapter */
	i2c_del_adapter(&i2c->adap);

	/* Free allocated resources */
	mt3620_i2c_release_dma(i2c);

	/* Uninitialize hardware */
	mt3620_i2c_uninit_hw(i2c);

	return 0;
}

static struct platform_driver mt3620_i2c_driver = {
	.probe = mt3620_i2c_probe,
	.remove = mt3620_i2c_remove,
	.driver = {
		.name = MT3620_I2C_DRV_NAME,
		.of_match_table = of_match_ptr(mt3620_i2c_of_match),
	},
};

module_platform_driver(mt3620_i2c_driver);

MODULE_DESCRIPTION("MT3620 I2C Bus Driver");
MODULE_AUTHOR("Jun Gao <jun.gao@mediatek.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mt3620-i2c");
