// SPDX-License-Identifier: GPL-2.0
/*
 * Azure Sphere MT3620 SPI Driver
 *
 * Copyright (c) 2018 Microsoft Corporation. All rights reserved.
 * Based on code Copyright (c) 2015 MediaTek Inc.
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
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/spi/spi.h>

#define DRV_NAME "spi-mt3620"
#define DRV_VERSION "1.0"

// Only use DMA for transfers > 32 bytes.
#define TRANSFER_SIZE_THRESHOLD_DMA 32

// Busy-wait for tiny transfers (<=4 bytes)
#define TRANSFER_SIZE_THRESHOLD_IRQ 4

// DMA buffer size (for each RX, TX)
#define DMA_BUF_SIZE PAGE_SIZE

// SPIM needs 13 words written via DMA for one
// transfer.
#define SPI_DMA_SIZE (13 * 4)

//
// This is a bare-bones driver for the MT3620 SPI peripheral.
// No support yet for:
//  - LSB-first bit ordering
//  - different word widths than 8-bit
//  - Speed selection
//  - DMA
//

// SPIM registers
#define STCSR 0x0000
#define STCSR_SPI_MASTER_BUSY 0x00001000
#define STCSR_SPI_MASTER_START 0x00000100
#define SOAR 0x0004
#define SDOR0 0x0008
#define SDOR1 0x000C
#define SDOR2 0x0010
#define SDOR3 0x0014
#define SDOR4 0x0018
#define SDOR5 0x001C
#define SDOR6 0x0020
#define SDOR7 0x0024
#define SMMR 0x0028
#define SMMR_RS_SLAVE_SEL_BIT 29
#define SMMR_CLK_MODE 0x01000000
#define SMMR_RS_CLK_SEL_BIT 16
#define SMMR_CS_DSEL_CNT_BIT 11
#define SMMR_BOTH_DIRECTIONAL_DATA_MODE 0x00000400
#define SMMR_INT_EN 0x00000200
#define SMMR_SPI_START_SEL 0x00000100
#define SMMR_PFETCH_EN 0x00000080
#define SMMR_CPHA 0x00000020
#define SMMR_CPOL 0x00000010
#define SMMR_LSB_FIRST 0x00000008
#define SMMR_MORE_BUF_MODE 0x00000004
#define SMBCR 0x002C
#define SMBCR_CMD_BIT_CNT_BIT 24
#define SMBCR_MISO_BIT_CNT_BIT 12
#define SMBCR_MOSI_BIT_CNT_BIT 0
#define RSV 0x0030
#define SCSR 0x0034
#define SCSR_SPI_READ_OK 0x00000004
#define SCSR_SPI_WRITE_OK 0x00000002
#define SCSR_SPI_OK 0x00000001
#define CSPOL 0x0038
#define CSPOL_HALF_DMA_MODE_EN 0x00000100
#define DATAPORT 0x0040
#define SDIR0 0x0048
#define SDIR1 0x004C
#define SDIR2 0x0050
#define SDIR3 0x0054
#define SDIR4 0x0058
#define SDIR5 0x005C
#define SDIR6 0x0060
#define SDIR7 0x0064

enum mt3620_spi_state { IRQ_RX, IRQ_TX, IRQ_SPI };

struct mt3620_spi {
	void __iomem *base;
	struct spi_transfer *cur_transfer;
	struct spi_device *cur_device;
	int transfer_ptr;
	struct dma_chan *dma_tx;
	struct dma_chan *dma_rx;
	struct scatterlist tx_sg;
	struct scatterlist rx_sg;
	int tx_buf_wptr;
	int rx_buf_wptr;
	unsigned long dma_state;
	atomic_t irq_state;
	int use_dma;
	void *rx_buf;
	void *tx_buf;

	struct clk *spi_clk;
	int current_speed_hz;
	int current_rs_clk_sel;
	struct spi_message *current_msg;
};

static void mt3620_spi_cleanup_dma(struct mt3620_spi *spi_mt3620);
static int mt3620_spi_dma_continue(struct spi_master *master,
				   struct mt3620_spi *spi_mt3620,
				   struct spi_device *spi,
				   struct spi_transfer *transfer);

static struct dma_chan *
	mt3620_spi_request_dma_chan(struct device *dev, char *chan_name,
				    enum dma_transfer_direction dir,
				    dma_addr_t port_addr)
{
	struct dma_chan *chan;
	struct dma_slave_config cfg;
	int ret;

	chan = dma_request_slave_channel(dev, chan_name);
	if (!chan) {
		dev_dbg(dev, "request DMA channel failed!\n");
		return chan;
	}

	if (IS_ERR(chan)) {
		ret = PTR_ERR(chan);
		dev_dbg(dev, "request_channel failed for %s (%d)\n", chan_name,
			ret);
		return chan;
	}

	memset(&cfg, 0, sizeof(cfg));
	cfg.direction = dir;
	cfg.slave_id = 1 /* MTK_DMA_SLAVE_ID_SPI */;
	if (dir == DMA_MEM_TO_DEV) {
		cfg.dst_addr = port_addr;
		cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	} else {
		cfg.src_addr = port_addr;
		cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	}

	ret = dmaengine_slave_config(chan, &cfg);
	if (ret) {
		dev_dbg(dev, "slave_config failed for %s (%d)\n", chan_name,
			ret);
		dma_release_channel(chan);
		chan = NULL;
		return chan;
	}

	dev_dbg(dev, "got DMA channel for %s\n", chan_name);
	return chan;
}

static void mt3620_spi_write(struct mt3620_spi *spi_mt3620, u32 offset,
			     u32 data)
{
	writel(data, ((u8 *)spi_mt3620->base) + offset);
}

static u32 mt3620_spi_read(struct mt3620_spi *spi_mt3620, u32 offset)
{
	return readl(((u8 *)spi_mt3620->base) + offset);
}

static int mt3620_spi_prepare_message(struct spi_master *master,
				      struct spi_message *msg)
{
	struct mt3620_spi *spi_mt3620 = spi_master_get_devdata(master);

	// Set CS#0 to high-active; this will assert CS#0
	// for the message.
	// (as the transfer will use dummy CS#7)
	mt3620_spi_write(spi_mt3620, CSPOL, 1);
	return 0;
}

static int mt3620_spi_unprepare_message(struct spi_master *master,
					struct spi_message *msg)
{
	struct mt3620_spi *spi_mt3620 = spi_master_get_devdata(master);

	// Set CS#0 to low-active to de-assert CS#0.
	mt3620_spi_write(spi_mt3620, CSPOL, 0);

	// If the transfer is still in process, then abort.
	int old_irq_state = atomic_xchg(&spi_mt3620->irq_state, 0);
	if (old_irq_state) {
		BUG_ON(!spi_mt3620->cur_transfer);
		mt3620_spi_cleanup_dma(spi_mt3620);
		msg->status = -EIO;
		spi_mt3620->cur_transfer = NULL;
	}

	return 0;
}

u32 *mt3620_transfer_continue(struct mt3620_spi *spi_mt3620,
			      struct spi_device *spi, bool use_dma,
			      bool enable_irq)
{
	struct spi_transfer *transfer = spi_mt3620->cur_transfer;
	int is_read, is_write, nbits;
	u32 smbcr, stcsr, smmr, soar;
	u32 *tx_buf;
	int remaining;

	// calculate remaining bytes of this transfer
	remaining = transfer->len - spi_mt3620->transfer_ptr;

	// the hardware is capped at 32.
	if (remaining > 32)
		remaining = 32;

	nbits = remaining * 8;

	// we do not support empty transfers
	BUG_ON(nbits < 8);

	is_read = transfer->rx_buf != NULL;
	is_write = transfer->tx_buf != NULL;

	if (is_read && is_write) {
		dev_err(&spi->dev, "bidirectional transfers don't work.\n");
		is_read = 0;
	}

	if (!is_write) {
		smbcr = (0 << SMBCR_CMD_BIT_CNT_BIT) |
			(nbits << SMBCR_MISO_BIT_CNT_BIT) |
			(0 << SMBCR_MOSI_BIT_CNT_BIT);
		soar = 0;
	} else {
		smbcr = (8 << SMBCR_CMD_BIT_CNT_BIT) |
			(0 << SMBCR_MISO_BIT_CNT_BIT) |
			((nbits - 8) << SMBCR_MOSI_BIT_CNT_BIT);
		// To work around the issue that the first byte of outgoing transfers
		// gets corrupted, use the "opcode" register as the first byte.
		// the remaining data is sent via the data register.
		soar = *(u8 *)(transfer->tx_buf + spi_mt3620->transfer_ptr);
	}

	// Set SMMR:
	// - set clock divider
	// - enable interrupts (if requested)
	// - set CPOL/CPHA
	// - enable "more buf mode"
	// - assert unused chip select (we need to control chip select manually)
	smmr = SMMR_MORE_BUF_MODE | (7 << SMMR_RS_SLAVE_SEL_BIT) |
	       SMMR_PFETCH_EN;

	smmr |= spi_mt3620->current_rs_clk_sel << SMMR_RS_CLK_SEL_BIT;

	if (enable_irq)
		smmr |= SMMR_INT_EN;

	if (spi->mode & SPI_CPOL)
		smmr |= SMMR_CPOL;

	if (spi->mode & SPI_CPHA)
		smmr |= SMMR_CPHA;

	stcsr = STCSR_SPI_MASTER_START;

	if (!use_dma) {
		BUG_ON(atomic_read(&spi_mt3620->irq_state));
		if (enable_irq) {
			atomic_inc(&spi_mt3620->irq_state);
		}

		// disable DMA
		mt3620_spi_write(spi_mt3620, CSPOL,
				 mt3620_spi_read(spi_mt3620, CSPOL) &
					 ~CSPOL_HALF_DMA_MODE_EN);

		// transmit data (with first-byte workaround)
		if (is_write) {
			mt3620_spi_write(spi_mt3620, SOAR, soar);
			memcpy(((u8 *)spi_mt3620->base) + SDOR0,
			       transfer->tx_buf + spi_mt3620->transfer_ptr + 1,
			       remaining - 1);
		}

		// write control registers; write to STCSR will start transfer.
		mt3620_spi_write(spi_mt3620, SMMR, smmr);
		mt3620_spi_write(spi_mt3620, SMBCR, smbcr);
		mt3620_spi_write(spi_mt3620, STCSR, stcsr);
		return NULL;
	} else {
		mt3620_spi_write(spi_mt3620, CSPOL,
				 mt3620_spi_read(spi_mt3620, CSPOL) |
					 CSPOL_HALF_DMA_MODE_EN);
		tx_buf = spi_mt3620->tx_buf + spi_mt3620->tx_buf_wptr;

		// construct the DMA data:
		// SOAR, {data0..7}, SMMR, SMBCR, STCSR and a dummy word,
		// (13*4 = 52 bytes total)

		tx_buf[0] = soar;

		// tx_buf[1..8]
		if (is_write) {
			memcpy(&tx_buf[1],
			       transfer->tx_buf + spi_mt3620->transfer_ptr + 1,
			       remaining - 1);
		}

		tx_buf[9] = smmr;
		tx_buf[10] = smbcr;
		tx_buf[11] = stcsr;
		tx_buf[12] = 0; // dummy

		// increment write pointer; caller already validated
		// that this does not overflow the DMA buffer.
		spi_mt3620->tx_buf_wptr += SPI_DMA_SIZE;

		// update transfer ptr right away
		spi_mt3620->transfer_ptr += remaining;

		// also update RX pointer so we know how many bytes
		// to receive.
		spi_mt3620->rx_buf_wptr += remaining;
		return tx_buf;
	}
}

static void mt3620_spi_dma_unmap(struct mt3620_spi *spi_mt3620)
{
	struct spi_transfer *trans = spi_mt3620->cur_transfer;
	BUG_ON(!trans);

	dma_unmap_single(spi_mt3620->dma_tx->device->dev,
			 sg_dma_address(&spi_mt3620->tx_sg),
			 sg_dma_len(&spi_mt3620->tx_sg), DMA_TO_DEVICE);

	if (trans->rx_buf)
		dma_unmap_single(spi_mt3620->dma_rx->device->dev,
				 sg_dma_address(&spi_mt3620->rx_sg),
				 sg_dma_len(&spi_mt3620->rx_sg),
				 DMA_FROM_DEVICE);
}

static void mt3620_spi_cleanup_dma(struct mt3620_spi *spi_mt3620)
{
	struct spi_transfer *trans = spi_mt3620->cur_transfer;

	dmaengine_terminate_all(spi_mt3620->dma_tx);
	if (trans->rx_buf)
		dmaengine_terminate_all(spi_mt3620->dma_rx);

	// we no longer expect this callback to happen.
	clear_bit(IRQ_TX, &spi_mt3620->dma_state);
	clear_bit(IRQ_RX, &spi_mt3620->dma_state);
	mt3620_spi_dma_unmap(spi_mt3620);
}

static void mt3620_spi_dma_may_continue(void *data, int type)
{
	struct spi_master *master = data;
	struct mt3620_spi *spi_mt3620 = spi_master_get_devdata(master);
	struct spi_transfer *transfer = spi_mt3620->cur_transfer;

	BUG_ON(!spi_mt3620->use_dma);
	BUG_ON(!transfer);

	// The SPI IRQ races the DMA callbacks, so all 3 combinations
	// (RX comes last, TX comes last, SPI comes last) must be supported.
	BUG_ON(!test_bit(type, &spi_mt3620->dma_state));
	clear_bit(type, &spi_mt3620->dma_state);

	// If this is the last IRQ to wait for, continue.
	if (atomic_dec_and_test(&spi_mt3620->irq_state)) {
		// unmap DMA buffers; this will invalidate the cache so we can read
		// the RX data.
		mt3620_spi_dma_unmap(spi_mt3620);

		if (transfer->rx_buf != NULL) {
			// copy back received data
			memcpy(transfer->rx_buf + spi_mt3620->transfer_ptr -
				       spi_mt3620->rx_buf_wptr,
			       spi_mt3620->rx_buf, spi_mt3620->rx_buf_wptr);
		}

		// check if end of message is reached, and terminate transfer if so.
		// otherwise, continue DMA transfer.
		if (spi_mt3620->transfer_ptr == transfer->len) {
			spi_finalize_current_transfer(master);
			spi_mt3620->cur_transfer = NULL;
		} else {
			int ret;
			ret = mt3620_spi_dma_continue(master, spi_mt3620,
						      spi_mt3620->cur_device,
						      transfer);
			if (ret != 0) {
				spi_finalize_current_transfer(master);
				spi_mt3620->cur_transfer = NULL;
			}
		}
	}
}

static void mt3620_spi_dma_tx_callback(void *data)
{
	mt3620_spi_dma_may_continue(data, IRQ_TX);
}

static void mt3620_spi_dma_rx_callback(void *data)
{
	struct spi_master *master = data;
	struct mt3620_spi *spi_mt3620 = spi_master_get_devdata(master);
	struct spi_transfer *transfer = spi_mt3620->cur_transfer;

	// We should reiceive an RX callback only for reads.
	BUG_ON(!transfer);
	BUG_ON(!transfer->rx_buf);
	mt3620_spi_dma_may_continue(data, IRQ_RX);
}

static int mt3620_spi_dma_submit(struct spi_master *master,
				 struct dma_chan *chan, struct scatterlist *sg,
				 dma_addr_t dma_addr, u16 dma_len,
				 enum dma_transfer_direction dir,
				 bool tx_callback)
{
	dma_cookie_t cookie;
	struct dma_async_tx_descriptor *txdesc;
	struct mt3620_spi *spi_mt3620 = spi_master_get_devdata(master);

	sg_dma_len(sg) = dma_len;
	sg_dma_address(sg) = dma_addr;
	txdesc = dmaengine_prep_slave_sg(chan, sg, 1, dir,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc) {
		dev_err(chan->device->dev, "dma prep slave sg failed.\n");
		mt3620_spi_cleanup_dma(spi_mt3620);
		return -ENOMEM;
	}

	if (tx_callback)
		txdesc->callback = mt3620_spi_dma_tx_callback;
	else
		txdesc->callback = mt3620_spi_dma_rx_callback;

	txdesc->callback_param = master;
	cookie = dmaengine_submit(txdesc);
	if (dma_submit_error(cookie)) {
		dev_err(chan->device->dev, "submitting dma failed.\n");
		mt3620_spi_cleanup_dma(spi_mt3620);
		return -ENOMEM;
	}

	dma_async_issue_pending(chan);

	return 0;
}

static int mt3620_spi_dma_continue(struct spi_master *master,
				   struct mt3620_spi *spi_mt3620,
				   struct spi_device *spi,
				   struct spi_transfer *transfer)
{
	int ret;
	dma_addr_t dma_tx_addr, dma_rx_addr;
	u32 *tx_buf = NULL;

	// Prepare DMA buffer; stop when transfer is complete
	// or buffer is full.
	// mt3620_transfer_continue also updates the expected read pointer
	// for the receive data.
	spi_mt3620->tx_buf_wptr = 0;
	spi_mt3620->rx_buf_wptr = 0;
	while (spi_mt3620->tx_buf_wptr <= (DMA_BUF_SIZE - SPI_DMA_SIZE) &&
	       transfer->len != spi_mt3620->transfer_ptr) {
		tx_buf = mt3620_transfer_continue(spi_mt3620, spi, true, false);
	}

	BUG_ON(!tx_buf);

	// Enable SPI IRQ for last chunk.
	tx_buf[9] |= SMMR_INT_EN;

	dma_tx_addr = dma_map_single(spi_mt3620->dma_tx->device->dev,
				     spi_mt3620->tx_buf,
				     spi_mt3620->tx_buf_wptr, DMA_TO_DEVICE);
	if (dma_mapping_error(spi_mt3620->dma_tx->device->dev, dma_tx_addr)) {
		dev_dbg(spi_mt3620->dma_tx->device->dev, "dma map failed.\n");
		return -ENOMEM;
	}

	BUG_ON(atomic_read(&spi_mt3620->irq_state) != 0);
	atomic_inc(&spi_mt3620->irq_state);
	set_bit(IRQ_TX, &spi_mt3620->dma_state);

	if (transfer->rx_buf) {
		int rx_size;
		BUG_ON(!spi_mt3620->rx_buf_wptr);

		// For receiving, align upwards to a multiple of 32-bit.
		rx_size = spi_mt3620->rx_buf_wptr;
		rx_size += (-rx_size) & 3;

		set_bit(IRQ_RX, &spi_mt3620->dma_state);
		atomic_inc(&spi_mt3620->irq_state);
		dma_rx_addr = dma_map_single(spi_mt3620->dma_rx->device->dev,
					     spi_mt3620->rx_buf, rx_size,
					     DMA_FROM_DEVICE);
		if (dma_mapping_error(spi_mt3620->dma_rx->device->dev,
				      dma_rx_addr)) {
			dev_dbg(spi_mt3620->dma_rx->device->dev, "dma map "
								 "failed.\n");
			return -ENOMEM;
		}

		ret = mt3620_spi_dma_submit(master, spi_mt3620->dma_rx,
					    &spi_mt3620->rx_sg, dma_rx_addr,
					    rx_size, DMA_MEM_TO_DEV, false);
		if (ret != 0)
			return ret;
	}

	set_bit(IRQ_SPI, &spi_mt3620->dma_state);
	atomic_inc(&spi_mt3620->irq_state);

	ret = mt3620_spi_dma_submit(master, spi_mt3620->dma_tx,
				    &spi_mt3620->tx_sg, dma_tx_addr,
				    spi_mt3620->tx_buf_wptr, DMA_MEM_TO_DEV,
				    true);

	return ret;
}

static int mt3620_spi_transfer_one(struct spi_master *master,
				   struct spi_device *spi,
				   struct spi_transfer *transfer)
{
	struct mt3620_spi *spi_mt3620 = spi_master_get_devdata(master);

	bool use_dma = transfer->len > TRANSFER_SIZE_THRESHOLD_DMA;
	bool use_irq = transfer->len > TRANSFER_SIZE_THRESHOLD_IRQ;

	// all previous transfers must have been finished by now.
	BUG_ON(mt3620_spi_read(spi_mt3620, SCSR) != 0);

	if (transfer->speed_hz != spi_mt3620->current_speed_hz) {
		int div;

		// The clock is hclk/(2+rs_clk_sel), with hclk typically
		// being 80MHz (PLL) or XTAL.

		if (transfer->speed_hz > 0)
			div = DIV_ROUND_UP(clk_get_rate(spi_mt3620->spi_clk),
					   transfer->speed_hz);
		else
			div = 4097;

		// Clip high rates to max speed.

		if (div >= 2)
			div -= 2;

		// Clip low rates at min speed (hclk/4097)

		if (div > 4095)
			div = 4095;

		spi_mt3620->current_rs_clk_sel = div;
		spi_mt3620->current_speed_hz = transfer->speed_hz;
	}

	if (!transfer->len)
		return 0;

	spi_mt3620->cur_transfer = transfer;
	spi_mt3620->cur_device = spi;
	spi_mt3620->transfer_ptr = 0;

	spi_mt3620->use_dma = use_dma;

	if (!use_dma) {
		// transmit a single burst up to 32-bytes.
		mt3620_transfer_continue(spi_mt3620, spi, false, use_irq);

		// for tiny transfers, busy wait; otherwise,
		// wait for interrupt.
		if (!use_irq) {
			while (!(mt3620_spi_read(spi_mt3620, SCSR) &
				 SCSR_SPI_OK)) {
				cpu_relax_lowlatency();
			}

			if (transfer->rx_buf) {
				memcpy(transfer->rx_buf +
					       spi_mt3620->transfer_ptr,
				       ((u8 *)spi_mt3620->base) + SDIR0,
				       transfer->len);
			}

			spi_finalize_current_transfer(master);
			spi_mt3620->cur_transfer = NULL;
			return 0;
		}
	} else {
		int ret;
		ret = mt3620_spi_dma_continue(master, spi_mt3620, spi,
					      transfer);
		if (ret)
			return ret;
	}

	return 1;
}

static irqreturn_t mt3620_spi_interrupt(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct mt3620_spi *spi_mt3620 = spi_master_get_devdata(master);
	u32 scsr;
	int remaining;

	scsr = mt3620_spi_read(spi_mt3620, SCSR);

	if (scsr & SCSR_SPI_OK) {
		struct spi_transfer *transfer = spi_mt3620->cur_transfer;

		// This could happen if the transfer previously timed out, but
		// eventually finished.
		if (!atomic_read(&spi_mt3620->irq_state)) {
			printk(KERN_INFO DRV_NAME ": received unexpected SPI "
						  "IRQ\n");
			return IRQ_HANDLED;
		}

		BUG_ON(!transfer);

		if (spi_mt3620->use_dma) {
			mt3620_spi_dma_may_continue(master, IRQ_SPI);
		} else {
			// PIO; double check assumptions.
			BUG_ON(!transfer);
			BUG_ON(test_bit(IRQ_TX, &spi_mt3620->dma_state));

			atomic_dec(&spi_mt3620->irq_state);

			remaining = transfer->len - spi_mt3620->transfer_ptr;

			if (remaining > 32)
				remaining = 32;

			// if this is a read, capture the received data.
			if (transfer->rx_buf) {
				memcpy(transfer->rx_buf +
					       spi_mt3620->transfer_ptr,
				       ((u8 *)spi_mt3620->base) + SDIR0,
				       remaining);
			}

			// update transfer ptr
			spi_mt3620->transfer_ptr += remaining;

			// check if end of message is reached.
			if (spi_mt3620->transfer_ptr == transfer->len) {
				spi_finalize_current_transfer(master);
				spi_mt3620->cur_transfer = NULL;
			} else {
				mt3620_transfer_continue(spi_mt3620,
							 spi_mt3620->cur_device,
							 false, true);
			}
		}
	} else {
		printk(KERN_INFO DRV_NAME ": spurious IRQ\n");
	}

	return IRQ_HANDLED;
}

static const struct of_device_id mt3620_spi_of_match[] = {
	{.compatible = "mediatek,mt3620-spi" },
	{}
};

MODULE_DEVICE_TABLE(of, mt3620_spi_of_match);

static int mt3620_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct mt3620_spi *spi_mt3620;
	struct resource *res;
	int irq, ret;
	dma_addr_t phy_data_port_addr;

	master = spi_alloc_master(&pdev->dev, sizeof(*spi_mt3620));

	if (!master) {
		dev_err(&pdev->dev, "failed to alloc spi master\n");
		return -ENOMEM;
	}

	master->dev.of_node = pdev->dev.of_node;
	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->num_chipselect = 2;
	master->max_speed_hz = 30000000;
	master->flags = 0;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->transfer_one = mt3620_spi_transfer_one;
	master->prepare_message = mt3620_spi_prepare_message;
	master->unprepare_message = mt3620_spi_unprepare_message;

	spi_mt3620 = spi_master_get_devdata(master);

	spi_mt3620->rx_buf = NULL;
	spi_mt3620->tx_buf = NULL;

	platform_set_drvdata(pdev, master);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "failed to determine base address\n");
		goto err_put_master;
	}

	spi_mt3620->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(spi_mt3620->base)) {
		ret = PTR_ERR(spi_mt3620->base);
		goto err_put_master;
	}

	phy_data_port_addr = (dma_addr_t)(res->start + DATAPORT);

	spi_mt3620->dma_state = 0;
	atomic_set(&spi_mt3620->irq_state, 0);
	spi_mt3620->dma_tx = mt3620_spi_request_dma_chan(
		&pdev->dev, "spi-dma-tx", DMA_MEM_TO_DEV, phy_data_port_addr);
	spi_mt3620->dma_rx = mt3620_spi_request_dma_chan(
		&pdev->dev, "spi-dma-rx", DMA_DEV_TO_MEM, phy_data_port_addr);

	if (!(spi_mt3620->dma_tx) || !(spi_mt3620->dma_rx)) {
		dev_err(&pdev->dev, "request dma channel fail.\n");
		goto err_put_master;
	}

	sg_init_table(&spi_mt3620->tx_sg, 1);
	sg_init_table(&spi_mt3620->rx_sg, 1);

	spi_mt3620->rx_buf = kzalloc(DMA_BUF_SIZE, GFP_KERNEL);
	spi_mt3620->tx_buf = kzalloc(DMA_BUF_SIZE, GFP_KERNEL);

	if (!spi_mt3620->rx_buf || !spi_mt3620->rx_buf) {
		dev_err(&pdev->dev, "failed to allocate DMA buffer\n");
		goto err_put_master;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get irq (%d)\n", irq);
		ret = irq;
		goto err_put_master;
	}

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	ret = devm_request_irq(&pdev->dev, irq, mt3620_spi_interrupt,
			       IRQF_TRIGGER_NONE, dev_name(&pdev->dev), master);
	if (ret) {
		dev_err(&pdev->dev, "failed to register irq (%d)\n", ret);
		goto err_put_master;
	}

	spi_mt3620->spi_clk = devm_clk_get(&pdev->dev, NULL);

	if (!spi_mt3620->spi_clk) {
		dev_err(&pdev->dev, "failed to get clock (%d)\n", ret);
		goto err_put_master;
	}

	// Maximum SPI speed is hclk/2.
	master->max_speed_hz = clk_get_rate(spi_mt3620->spi_clk) / 2;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "failed to register master (%d)\n", ret);
		goto err_disable_runtime_pm;
	}

	return 0;

err_disable_runtime_pm:
	pm_runtime_disable(&pdev->dev);
err_put_master:
	if (spi_mt3620->rx_buf)
		kfree(spi_mt3620->rx_buf);
	if (spi_mt3620->tx_buf)
		kfree(spi_mt3620->tx_buf);
	spi_master_put(master);

	return ret;
}

static int mt3620_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct mt3620_spi *spi_mt3620 = spi_master_get_devdata(master);

	pm_runtime_disable(&pdev->dev);
	if (spi_mt3620->rx_buf)
		kfree(spi_mt3620->rx_buf);
	if (spi_mt3620->tx_buf)
		kfree(spi_mt3620->tx_buf);
	spi_master_put(master);

	return 0;
}

static struct platform_driver mt3620_spi_driver = {
	.probe = mt3620_spi_probe,
	.remove = mt3620_spi_remove,
	.driver =
		{
			.name = DRV_NAME,
			.of_match_table = mt3620_spi_of_match,
		},
};

static s32 __init mt3620_spi_init(void)
{
	return platform_driver_register(&mt3620_spi_driver);
}

static void __exit mt3620_spi_exit(void)
{
	platform_driver_unregister(&mt3620_spi_driver);
}

module_init(mt3620_spi_init);
module_exit(mt3620_spi_exit);

MODULE_DESCRIPTION("MT3620 SPI Controller driver");
MODULE_AUTHOR("Microsoft");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mtk-spi");
