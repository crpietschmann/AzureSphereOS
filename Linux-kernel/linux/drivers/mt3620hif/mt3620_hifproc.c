// SPDX-License-Identifier: GPL-2.0
/*
 * MT3620 hif Proc  driver
 *
 * Copyright (c) 2018 MediaTek. All rights reserved.
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


#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/spinlock_types.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/mmc/sdio.h>
#include <linux/delay.h>

#include <mt3620/mt3620_hifapi.h>


#include "mt3620_hifproc.h"

#define DRIVER_NAME "mt3620-hif-proc"

struct mt3620_hif_proc *g_pmt3620_hif_proc;


enhance_mode_data_struct_t g_mt3620_last_enhance_mode_data_struct;
wifi_hif_tx_flow_control_t g_mt3620_hif_tx_flow_control_stat;
uint8_t g_mt3620_hif_tx_flow_ctrl_en = 0;
hif_stat_t g_mt3620_hif_stat;
struct mt3620_hif_func g_mt3620_hif_func;

uint32_t mt3620_use_dma = 0;
extern void __iomem *DMA_CHANNEL_BASE;
void DMA_Clock_Enable(uint8_t channel)
{
	uint32_t val = 0;
	val |= ((channel <= 31)?(1 << channel):(1 << (channel-32)));
	writel_relaxed(val, DMA_CHANNEL_BASE +DMA_CH_EN_SET(channel));
}


/**
 *	mt3620_hif_fn0_read_byte - read a single byte from SDIO function 0
 *	@func: an SDIO function of the card
 *	@addr: address to read
 *	@err_ret: optional status value from transfer
 *
 *	Reads a single byte from the address space of SDIO function 0.
 *	If there is a problem reading the address, 0xff is returned
 *	and @err_ret will contain the error code.
 */
uint8_t mt3620_hif_fn0_read_byte(struct mt3620_hif_func *func, uint32_t addr,
	int32_t *err_ret)
{
	//int32_t ret;
	uint8_t val;
	mt3620_hif_gen3_cmd52_info info;
	info.word = 0;


	/* CMD52 read 1-byte of func0 */

	if (err_ret)
		*err_ret = 0;

	/* 1. Setup command information */
	info.field.rw_flag = SDIO_GEN3_READ;
	info.field.func_num = 0; //func->num;
	info.field.addr = addr;


	writel_relaxed(info.word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);
	/* 2. CMD52 read  data  */
	val = readl_relaxed(g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD52_DATA);

	return val;
}

/**
 *	mt3620_hif_fn0_write_byte - write a single byte to SDIO function 0
 *	@func: an SDIO function of the card
 *	@b: byte to write
 *	@addr: address to write to
 *	@err_ret: optional status value from transfer
 *
 *	Writes a single byte to the address space of SDIO function 0.
 *	@err_ret will contain the status of the actual transfer.
 *
 *	Only writes to the vendor specific CCCR registers (0xF0 -
 *	0xFF) are permiited; @err_ret will be set to -EINVAL for *
 *	writes outside this range.
 */
void mt3620_hif_fn0_write_byte(struct mt3620_hif_func *func, uint8_t b, uint32_t addr,
	int32_t *err_ret)
{
	//int32_t ret;
	mt3620_hif_gen3_cmd52_info info;
	info.word = 0;

	/* CMD52 write 1-byte of func0 */

	if (err_ret)
		*err_ret = 0;

	/* 1. Setup command information */
	info.field.rw_flag = SDIO_GEN3_WRITE;
	info.field.func_num = 0; //func->num;
	info.field.addr = addr;
	info.field.data = b;
	dev_dbg(g_pmt3620_hif_proc->dev, "=== write f0, setup=0x%x\n", (unsigned int)info.word);


	writel_relaxed(info.word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);
	/* 2. CMD52 write dummy 0 to trigger write  data  */
	writel_relaxed(b,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD52_DATA);

}

/**
 *	mt3620_hif_readl - read a 32 bit integer from a SDIO function
 *	@func: SDIO function to access
 *	@addr: address to read
 *	@err_ret: optional status value from transfer
 *
 *	Reads a 32 bit integer from the address space of a given SDIO
 *	function. If there is a problem reading the address,
 *	0xffffffff is returned and @err_ret will contain the error
 *	code.
 */
uint32_t mt3620_hif_readl(struct mt3620_hif_func *func, uint32_t addr, int32_t *err_ret)
{
	uint32_t value;
	unsigned long flags;
	mt3620_hif_gen3_cmd53_info info;
	spin_lock_irqsave(&g_pmt3620_hif_proc->hif_lock, flags);

	if (err_ret)
		*err_ret = 0;

	/* CMD53 incremental mode to read 4-byte */
	/* 1. Setup command information */
	info.word = 0;
	info.field.rw_flag = SDIO_GEN3_READ;
	info.field.func_num =  SDIO_GEN3_FUNCTION_WIFI ;//func->num; //SDIO_GEN3_FUNCTION_WIFI;
	info.field.block_mode = SDIO_GEN3_BYTE_MODE; /* byte  mode */
	info.field.op_mode = SDIO_GEN3_FIXED_PORT_MODE; // SDIO-GEN3 only apply to fix port, forget  SDIO_GEN3_INCREMENT_MODE; /* increment mode */
	info.field.addr = addr;
	info.field.count = 4;

	writel_relaxed(info.word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);
	/* 2. CMD53 read out */
	value = readl_relaxed(g_pmt3620_hif_proc->phif_sysram_base + SDIO_GEN3_CMD53_DATA);

	spin_unlock_irqrestore(&g_pmt3620_hif_proc->hif_lock, flags);

	return value;
}

/**
 *	mt3620_hif_writel - write a 32 bit integer to a SDIO function
 *	@func: SDIO function to access
 *	@b: integer to write
 *	@addr: address to write to
 *	@err_ret: optional status value from transfer
 *
 *	Writes a 32 bit integer to the address space of a given SDIO
 *	function. @err_ret will contain the status of the actual
 *	transfer.
 */
void mt3620_hif_writel(struct mt3620_hif_func *func, uint32_t b, uint32_t addr, int32_t *err_ret)
{
	mt3620_hif_gen3_cmd53_info info;
	unsigned long flags;
	spin_lock_irqsave(&g_pmt3620_hif_proc->hif_lock, flags);
	if (err_ret)
	*err_ret = 0;

	/* CMD53 incremental mode to read 4-byte */
	/* 1. Setup command information */
	info.word = 0;
	info.field.rw_flag = SDIO_GEN3_WRITE;
	info.field.func_num = SDIO_GEN3_FUNCTION_WIFI;  //func->num; //SDIO_GEN3_FUNCTION_WIFI;
	info.field.block_mode = SDIO_GEN3_BYTE_MODE; /* byte  mode */
	info.field.op_mode = SDIO_GEN3_FIXED_PORT_MODE; // SDIO-GEN3 only apply to fix port, forget  SDIO_GEN3_INCREMENT_MODE; /* increment mode */
	info.field.addr = addr;
	info.field.count = 4;

	writel_relaxed(info.word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);
	/* 2. CMD53 write data  */
	writel_relaxed(b,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD53_DATA);
	spin_unlock_irqrestore(&g_pmt3620_hif_proc->hif_lock, flags);
}



int32_t mt3620_hif_cr_read(uint32_t addr, uint32_t *value)
{

	int32_t ret = HIF_STATUS_SUCCESS;
	struct mt3620_hif_func *dev_func = &g_mt3620_hif_func;


	*value = mt3620_hif_readl(dev_func, addr, &ret);
	if (ret)
	{
		dev_err(g_pmt3620_hif_proc->dev, "<<%s>> Read register 0x%08x failed. Error = %d\n",
			__FUNCTION__,
			(unsigned int)addr,
			(int)ret);
	}
	return ret;
}

int32_t mt3620_hif_cr_write(uint32_t addr, uint32_t value)
{
	int32_t ret = HIF_STATUS_SUCCESS;
	struct mt3620_hif_func *dev_func = &g_mt3620_hif_func;

	mt3620_hif_writel(dev_func, value, addr, &ret);

	if (ret)
	{
		dev_err(g_pmt3620_hif_proc->dev, "<<%s>> Write register 0x%08x failed. Error = %d\n",
			__FUNCTION__,
			(unsigned int)addr,
			(int)ret);
	}

	return ret;
}


void mt3620_hif_read_port_pio_garbage(mt3620_hif_gen3_cmd53_info *info, int32_t count)
{
	int32_t i;

	writel_relaxed(info->word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);
	/* 2. CMD53 read out */
	for (i = 0; i < count; i+=4)
	{
		readl_relaxed(g_pmt3620_hif_proc->phif_sysram_base + SDIO_GEN3_CMD53_DATA);
	}

}

void mt3620_hif_read_port_dma(mt3620_hif_gen3_cmd53_info *info, void *dst, int32_t count)
{
	unsigned long flags;
    uint32_t reg = 0;
    uint32_t fixed_port_address = 0;
    uint32_t con = 0, connect = 0;
    uint8_t ch = MT3620_DMA_WIFI_CHANNEL;
	uint8_t read_count = 0;
    fixed_port_address = 0x60001000;

    spin_lock_irqsave(&g_pmt3620_hif_proc->hif_lock, flags);
    writel_relaxed(info->word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);
    
	/* DMA setting */
    DMA_Clock_Enable(ch);


    writel_relaxed(0x0,DMA_CHANNEL_BASE + DMA_START(ch));
    writel_relaxed(0x1,DMA_CHANNEL_BASE + DMA_RESET(ch));
    writel_relaxed((uint32_t)fixed_port_address,DMA_CHANNEL_BASE +DMA_SRC(ch));
    writel_relaxed((uint32_t)dst,DMA_CHANNEL_BASE+DMA_DST(ch));
    writel_relaxed(count,DMA_CHANNEL_BASE+DMA_COUNT(ch));

    con |= (0x1 << 30); // WIF_HP = 1, (4*DW) ; WIF_HP=0, (DW)
    con |= (0x2 << 28); // Read size is 4-byte (DW)
    con |= (0x0 << 24); // Write size
    con |= (0x1 << 4); // SFIX (No SINC)
    con |= (0x0 << 3); // No DFIX (DINC)
    writel_relaxed(con,DMA_CHANNEL_BASE+DMA_CON(ch));

    connect |= (0x1 << 2); //DIR = 1, HIF to SRAM => Write
    connect |= (0x0 << 0); //CONNECT = 1 => handshaking signal
    writel_relaxed(connect,DMA_CHANNEL_BASE+DMA_CONNECT(ch));
    writel_relaxed(0x1,DMA_CHANNEL_BASE + DMA_START(ch));

	do {
		read_count++;
		reg = readl_relaxed(DMA_CHANNEL_BASE + DMA_INT_FLAG(ch));
	} while ((reg != 0x01) && (read_count <= READ_DMA_STATUS_MAX_COUNT));

	writel_relaxed(0x1,DMA_CHANNEL_BASE + DMA_INT_FLAG(ch));

	if(read_count >= READ_DMA_STATUS_MAX_COUNT)
		dev_err(g_pmt3620_hif_proc->dev,"ERROR! %s, READ DMA transfer fail", __FUNCTION__);	

	spin_unlock_irqrestore(&g_pmt3620_hif_proc->hif_lock, flags);  
}


void mt3620_hif_read_port_pio(mt3620_hif_gen3_cmd53_info *info, void *dst, int32_t count)
{
	int32_t i;
	int32_t drop_count = 0;
	unsigned long flags;

	spin_lock_irqsave(&g_pmt3620_hif_proc->hif_lock, flags);

	writel_relaxed(info->word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);
	/* 2. CMD53 read out */
	for (i = 0; i < count/4; i++)
	{
		uint32_t value = readl_relaxed(g_pmt3620_hif_proc->phif_sysram_base + SDIO_GEN3_CMD53_DATA);
		if (((i*4) < HIF_MAX_RX_PKT_SIZE) && dst != NULL)
		{
			((uint32_t*)dst)[i] = value;
		}
		else
			drop_count += 4;
	}

	spin_unlock_irqrestore(&g_pmt3620_hif_proc->hif_lock, flags);
}

static void mt3620_hif_write_port_dma(mt3620_hif_gen3_cmd53_info *info, void *src, int32_t count)
{
    uint32_t reg;
    uint32_t fixed_port_address = 0x60001000;
    uint32_t con = 0, connect = 0;
    uint8_t ch = MT3620_DMA_WIFI_CHANNEL;
	uint32_t src_address = (uint32_t)src;
    unsigned long flags;
	uint8_t read_count =0;


	spin_lock_irqsave(&g_pmt3620_hif_proc->hif_lock, flags);

	writel_relaxed(info->word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);
    
	
	/* DMA setting */
    DMA_Clock_Enable(ch);

    writel_relaxed(0x0,DMA_CHANNEL_BASE + DMA_START(ch));
    writel_relaxed(0x1,DMA_CHANNEL_BASE + DMA_RESET(ch));
    writel_relaxed((uint32_t)src_address,DMA_CHANNEL_BASE+DMA_SRC(ch));
    writel_relaxed(fixed_port_address,DMA_CHANNEL_BASE+DMA_DST(ch));
    writel_relaxed(count,DMA_CHANNEL_BASE+DMA_COUNT(ch));

    con |= (0x1 << 30); // WIF_HP = 1, (4*DW) ; WIF_HP=0, (DW)
    con |= (0x0 << 28); // Read size
    con |= (0x2 << 24); // Write size is 4-byte(DW)
    con |= (0x0 << 4); // No SFIX (SINC)
    con |= (0x1 << 3); // DFIX (No DINC)
    writel_relaxed(con,DMA_CHANNEL_BASE+DMA_CON(ch));

    connect |= (0x0 << 2); //DIR = 0, SRAM to HIF => Read
    connect |= (0x0 << 0); //CONNECT  = 1 (handshaking signal)
    writel_relaxed(connect,DMA_CHANNEL_BASE+DMA_CONNECT(ch));
    writel_relaxed(0x1,DMA_CHANNEL_BASE + DMA_START(ch));
	
	
	do {
		read_count++;
		reg = readl_relaxed(DMA_CHANNEL_BASE + DMA_INT_FLAG(ch));
	} while ((reg != 0x01) && (read_count <= READ_DMA_STATUS_MAX_COUNT));

	writel_relaxed(0x1,DMA_CHANNEL_BASE + DMA_INT_FLAG(ch));

	if(read_count >= READ_DMA_STATUS_MAX_COUNT)
		dev_err(g_pmt3620_hif_proc->dev,"ERROR! %s, Write DMA transfer fail", __FUNCTION__);	
	
	spin_unlock_irqrestore(&g_pmt3620_hif_proc->hif_lock, flags);  
}


static void mt3620_hif_write_port_pio(mt3620_hif_gen3_cmd53_info *info, void *src, int32_t count)
{
	int32_t i;
	unsigned long flags;
	uint32_t value = 0;

	spin_lock_irqsave(&g_pmt3620_hif_proc->hif_lock, flags);

	writel_relaxed(info->word,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD_SETUP);

	for (i = 0; i < count/4; i++)
	{

		value = ((uint32_t*)src)[i];
		writel_relaxed(value,g_pmt3620_hif_proc->phif_sysram_base+SDIO_GEN3_CMD53_DATA);
	}

	spin_unlock_irqrestore(&g_pmt3620_hif_proc->hif_lock, flags);
}

int32_t mt3620_hif_read_port_garbage(uint32_t addr, int32_t size)
{
	mt3620_hif_gen3_cmd53_info info;
	int32_t count;
	struct mt3620_hif_func *func = &g_mt3620_hif_func;


	/* CMD53 port mode to write n-byte, if count >= block size => block mode, otherwise =>  byte mode  */
	count = ALIGN_4BYTE(size);

	/* 1. Setup command information */
	info.word = 0;
	info.field.rw_flag = SDIO_GEN3_READ;
	info.field.func_num = SDIO_GEN3_FUNCTION_WIFI;//func->num; //SDIO_GEN3_FUNCTION_WIFI;
	if (count >= func->blksize)
	{
		info.field.block_mode = SDIO_GEN3_BLOCK_MODE; /* block  mode */
		info.field.count = count/func->blksize;
		if (count % func->blksize > 0)
			info.field.count++;
		count = info.field.count * func->blksize;
	}
	else
	{
		info.field.block_mode = SDIO_GEN3_BYTE_MODE; /* byte  mode */
		info.field.count = count;
	}
	info.field.op_mode = SDIO_GEN3_FIXED_PORT_MODE; /* fix mode */
	info.field.addr = addr;

	mt3620_hif_read_port_pio_garbage(&info, count);

	return 0;
}



/**
 *	mt3620_hif_read_port - read from a FIFO on a SDIO function
 *	@func: SDIO function to access
 *	@dst: buffer to store the data
 *	@addr: address of (single byte) FIFO
 *	@count: number of bytes to read
 *
 *	Reads from the specified FIFO of a given SDIO function. Return
 *	value indicates if the transfer succeeded or not.
 */
int32_t mt3620_hif_read_port(struct mt3620_hif_func *func, void *dst, uint32_t addr,
	int32_t size)
{
	mt3620_hif_gen3_cmd53_info info;
	int32_t count;
	dma_addr_t buffer_dma_addr = DMA_ERROR_CODE;

	if ((unsigned int)dst & 0x3)
	{
		dev_err(g_pmt3620_hif_proc->dev,"ERROR! %s, align error, dst = 0x%x\n", __FUNCTION__,(unsigned int)dst);
		mt3620_hif_read_port_garbage(addr, size);
		return HIF_STATUS_FAIL;
	}

	if(mt3620_use_dma)
	{
		buffer_dma_addr =
	    dma_map_single(g_pmt3620_hif_proc->dev, (uint32_t *)dst, size, DMA_FROM_DEVICE);
		if (dma_mapping_error(g_pmt3620_hif_proc->dev, buffer_dma_addr)) {
			return HIF_STATUS_FAIL;
		}

		if(!(IS_ALIGN_4((uint32_t)buffer_dma_addr)))
		{
			dev_err(g_pmt3620_hif_proc->dev,"DMA ALIGN ERROR! %s, align error, dma_addr = 0x%x\n", __FUNCTION__,buffer_dma_addr);
        	return HIF_STATUS_FAIL;
		}

	}
	/* CMD53 port mode to write n-byte, if count >= block size => block mode, otherwise =>  byte mode  */
	count = ALIGN_4BYTE(size);

	/* 1. Setup command information */
	info.word = 0;
	info.field.rw_flag = SDIO_GEN3_READ;
	info.field.func_num = SDIO_GEN3_FUNCTION_WIFI ;//func->num; //SDIO_GEN3_FUNCTION_WIFI;
	if (count >= func->blksize)
	{
		info.field.block_mode = SDIO_GEN3_BLOCK_MODE; /* block  mode */
		info.field.count = count/func->blksize;
		if (count % func->blksize > 0)
			info.field.count++;
		count = info.field.count * func->blksize;
	}
	else
	{
		info.field.block_mode = SDIO_GEN3_BYTE_MODE; /* byte  mode */
		info.field.count = count;
	}
	info.field.op_mode = SDIO_GEN3_FIXED_PORT_MODE; /* fix mode */
	info.field.addr = addr;

    if ((mt3620_use_dma)&&
        (info.field.block_mode == SDIO_GEN3_BLOCK_MODE)&&
        (count <= HIF_MAX_RX_PKT_SIZE))
    {
		if (count >  HIF_MAX_RX_PKT_SIZE)
		{
			count = HIF_MAX_RX_PKT_SIZE;
		}
		mt3620_hif_read_port_dma(&info, (void *)buffer_dma_addr, count); //dma function to be implemented

		if (buffer_dma_addr != DMA_ERROR_CODE) {
				dma_unmap_single(g_pmt3620_hif_proc->dev, buffer_dma_addr,
				 size, DMA_FROM_DEVICE);
		}

    }
	else
	{
		mt3620_hif_read_port_pio(&info, dst, count);
	}

	return HIF_STATUS_SUCCESS;
}


int32_t mt3620_hif_write_port(struct mt3620_hif_func *func, uint32_t addr, void *src,
	int32_t size)
{
	mt3620_hif_gen3_cmd53_info info;
	int32_t count;
	dma_addr_t buffer_dma_addr = DMA_ERROR_CODE;

	if (!(IS_ALIGN_4((uint32_t)src))/* && (func->use_dma)*/)
	{
		dev_err(g_pmt3620_hif_proc->dev,"ERROR! %s, align error, src = 0x%x\n", __FUNCTION__,(unsigned int)src);
		return HIF_STATUS_FAIL;
	}

	if(mt3620_use_dma)
	{
		buffer_dma_addr =
	    dma_map_single(g_pmt3620_hif_proc->dev, (uint32_t *)src, size, DMA_TO_DEVICE);
		if (dma_mapping_error(g_pmt3620_hif_proc->dev, buffer_dma_addr)) {
			return HIF_STATUS_FAIL;
		}

		if(!(IS_ALIGN_4((uint32_t)buffer_dma_addr)))
		{
			dev_err(g_pmt3620_hif_proc->dev,"DMA ALIGN ERROR! %s, align error, dma_addr = 0x%x\n", __FUNCTION__,buffer_dma_addr);
        	return HIF_STATUS_FAIL;
		}

	}

	/* CMD53 port mode to write n-byte, if count >= block size => block mode, otherwise =>  byte mode  */
	count = ALIGN_4BYTE(size);

	/* 1. Setup command information */
	info.word = 0;
	info.field.rw_flag = SDIO_GEN3_WRITE;
	info.field.func_num = SDIO_GEN3_FUNCTION_WIFI; //func->num; //SDIO_GEN3_FUNCTION_WIFI;

	if (count >= func->blksize)
	{
		info.field.block_mode = SDIO_GEN3_BLOCK_MODE; /* block  mode */
		info.field.count = count/func->blksize;
		if (count % func->blksize > 0)
			info.field.count++;
		count = info.field.count * func->blksize;
	}
	else
	{
		info.field.block_mode = SDIO_GEN3_BYTE_MODE; /* byte  mode */
			info.field.count = count;
	}

	info.field.op_mode = SDIO_GEN3_FIXED_PORT_MODE; /* fix mode */
	info.field.addr = addr;

    if ((mt3620_use_dma)&&
       (info.field.block_mode == SDIO_GEN3_BLOCK_MODE)&&
        (count < HIF_MAX_RX_PKT_SIZE))
    {
		mt3620_hif_write_port_dma(&info, (void *)buffer_dma_addr, count);

		if (buffer_dma_addr != DMA_ERROR_CODE) {
			dma_unmap_single(g_pmt3620_hif_proc->dev, buffer_dma_addr,
			size, DMA_TO_DEVICE);
		}
    }
	else
	{
		mt3620_hif_write_port_pio(&info, src, count);

	}

	return HIF_STATUS_SUCCESS;
 }




/**
 *	mt3620_hif_set_block_size - set the block size of an SDIO function
 *	@func: SDIO function to change
 *	@blksz: new block size or 0 to use the default.
 *
 *	The default block size is the largest supported by both the function
 *	and the host, with a maximum of 512 to ensure that arbitrarily sized
 *	data transfer use the optimal (least) number of commands.
 *
 *	A driver may call this to override the default block size set by the
 *	core. This can be used to set a block size greater than the maximum
 *	that reported by the card; it is the driver's responsibility to ensure
 *	it uses a value that the card supports.
 *
 *	Returns 0 on success, -EINVAL if the host does not support the
 *	requested block size, or -EIO (etc.) if one of the resultant FBR block
 *	size register writes failed.
 *
 */
int32_t mt3620_hif_set_block_size(struct mt3620_hif_func *func, uint32_t blksz)
{
	int32_t ret;

	mt3620_hif_fn0_write_byte(func, (blksz & 0xff),
		SDIO_FBR_BASE(func->num) + SDIO_FBR_BLKSIZE, &ret);

	if (ret)
		return ret;

	mt3620_hif_fn0_write_byte(func, ((blksz >> 8) & 0xff),
		SDIO_FBR_BASE(func->num) + SDIO_FBR_BLKSIZE + 1, &ret);

	if (ret)
		return ret;
	func->blksize = blksz;
	return 0;
}


int32_t mt3620_hif_get_irq(struct mt3620_hif_func *func)
{
	int32_t ret;
	uint8_t reg=0;

	dev_dbg(g_pmt3620_hif_proc->dev,"SDIO: Enabling IRQ for func%d...\n",(int)func->num);

	reg = mt3620_hif_fn0_read_byte(func,SDIO_CCCR_IENx, &ret);
	if (ret)
		return ret;

	reg |= 1 << func->num;

	reg |= 1; /* Master interrupt enable */

	dev_dbg(g_pmt3620_hif_proc->dev,"Write IENx=0x%x\n", reg);
	mt3620_hif_fn0_write_byte(func, reg, SDIO_CCCR_IENx, &ret);
	if (ret)
		return ret;

	reg = mt3620_hif_fn0_read_byte(func,SDIO_CCCR_IENx, &ret);
	if (ret)
		return ret;
	dev_dbg(g_pmt3620_hif_proc->dev,"===> IENx=0x%x\n", reg);
	return ret;
}


/**
 *	mt3620_hif_enable_func - enables a SDIO function for usage
 *	@func: SDIO function to enable
 *
 *	Powers up and activates a SDIO function so that register
 *	access is possible.
 */
int32_t mt3620_hif_enable_func(struct mt3620_hif_func *func)
{
	int32_t ret;
	uint8_t reg;

	reg = mt3620_hif_fn0_read_byte(func, SDIO_CCCR_IOEx, &ret);
	if (ret)
		goto err;

	reg |= 1 << func->num;

	mt3620_hif_fn0_write_byte(func, reg, SDIO_CCCR_IOEx, &ret);
	if (ret) 
		goto err;

	reg = mt3620_hif_fn0_read_byte(func, SDIO_CCCR_IORx, &ret);
	if (ret)
		goto err;


	if (!(reg & (1 << func->num)))
	{
		ret = -ETIME;
		goto err;
	}

	return 0;

err:
	dev_err(g_pmt3620_hif_proc->dev, "SDIO: Failed to enable Function %d\n", (int)func->num);
	return ret;
}

int32_t mt3620_hif_enable_enhance_mode(void)
{
	uint32_t reg_value = 0;
	/* enable RX enhance mode */


	if (mt3620_hif_cr_read(WHCR, &reg_value))
	{
		dev_err(g_pmt3620_hif_proc->dev, "FAIL. read WHCR.\n");
		return HIF_STATUS_FAIL;
	}

	reg_value &= ~MAX_HIF_RX_LEN_NUM_MASK;
	reg_value |= MAX_HIF_RX_LEN_NUM(CFG_MAX_HIF_RX_LEN_NUM);
//	reg_value |= RX_ENHANCE_MODE;




	if (mt3620_hif_cr_write(WHCR, reg_value))
	{
		dev_err(g_pmt3620_hif_proc->dev, "FAIL. write WHCR.\n");
		return HIF_STATUS_FAIL;
	}
	reg_value = 0;
	if (mt3620_hif_cr_read(WHCR, &reg_value))
	{
		dev_err(g_pmt3620_hif_proc->dev, "FAIL. read WHCR.\n");
		return HIF_STATUS_FAIL;
	}

	dev_dbg(g_pmt3620_hif_proc->dev, "Enable enhance mode, WHCR=0x%x\n", (unsigned int)reg_value);

	return 0;
}




void mt3620_hif_tx_flow_control_init(void)
{
	g_mt3620_hif_tx_flow_control_stat.reserve_quota_page_cnt = DEFAULT_N9_PSE_PAGE_QUOTA;
	g_mt3620_hif_tx_flow_control_stat.page_sz = DEFAULT_N9_PSE_PAGE_SIZE;
	g_mt3620_hif_tx_flow_control_stat.available_page_cnt = DEFAULT_N9_PSE_PAGE_QUOTA;
	g_mt3620_hif_tx_flow_control_stat.current_page_cnt = 0;
	g_mt3620_hif_tx_flow_ctrl_en = 1;
}



uint8_t mt3620_hif_get_ownership(void)
{
	uint32_t value, counter = 0, addr;
	int32_t ret;
	uint8_t status = true;
	bool fw_own;
	unsigned long flags;

	spin_lock_irqsave(&g_pmt3620_hif_proc->own_lock, flags);
	fw_own = g_pmt3620_hif_proc->is_fw_own;

	if(fw_own == false) // means driver has the ownership
	{
		spin_unlock_irqrestore(&g_pmt3620_hif_proc->own_lock, flags);  
		goto err;
	}

	spin_unlock_irqrestore(&g_pmt3620_hif_proc->own_lock, flags);

	addr = WHLPCR;

	// 1. check if it is already driver own
	ret = mt3620_hif_cr_read(addr, &value);

	dev_dbg(g_pmt3620_hif_proc->dev, "rd a:0x%x ,d:0x%x\n",addr, value);

	if (ret)
	{
		dev_err(g_pmt3620_hif_proc->dev, "Ownership is already driver\n");
		goto err;
	}

	if (!GET_W_FW_OWN_REQ_SET(value))
	{
		// request owner ship
		value |= W_FW_OWN_REQ_CLR;

		dev_dbg(g_pmt3620_hif_proc->dev, "wr a:0x%x ,d:0x%x\n",addr, value);
		ret = mt3620_hif_cr_write(addr, value);
		if (ret)
		{
			dev_err(g_pmt3620_hif_proc->dev, "request owner ship write fail\n");
			goto err;
		}

		// check if the ownership back
		counter = 0;
		while (!GET_W_FW_OWN_REQ_SET(value))
		{
			if (counter > 20000) // wait for at least 1 second
			{
				status = false;
				break;
			}

			if(counter > 0 && counter%1000 == 0)
			{
				// request ownership
				value |= W_FW_OWN_REQ_CLR;
				ret = mt3620_hif_cr_write(addr, value);
				if (ret)
				{
					dev_err(g_pmt3620_hif_proc->dev, "request owner ship write fail\n");
					goto err;
				}

				dev_dbg(g_pmt3620_hif_proc->dev, "request ownership again\n");
			}
				udelay(50); //delay 50us

			ret = mt3620_hif_cr_read(addr, &value);
			dev_dbg(g_pmt3620_hif_proc->dev, "rd a:0x%x ,d:0x%x, c:%d\n",addr, value, counter);
			if (ret)
			{
				goto err;
			}
			counter++;
		}
		spin_lock_irqsave(&g_pmt3620_hif_proc->own_lock, flags);
		g_pmt3620_hif_proc->is_fw_own = false;
		spin_unlock_irqrestore(&g_pmt3620_hif_proc->own_lock, flags);
	}
	else
    {
	//it seems that the ownership already tranfered to driver
        spin_lock_irqsave(&g_pmt3620_hif_proc->own_lock, flags);
        g_pmt3620_hif_proc->is_fw_own = false;
        spin_unlock_irqrestore(&g_pmt3620_hif_proc->own_lock, flags);
    }

err:
	/*io read/write fail*/
	if (ret)
	{
		status = false;
		dev_err(g_pmt3620_hif_proc->dev, "err, c:%d\n",counter);
	}

	return status;
}


uint8_t mt3620_hif_set_ownership(void)
{
    uint8_t status;
    unsigned long flags;


    status = mt3620_hif_cr_write(WHLPCR, W_FW_OWN_REQ_SET);  
    
    spin_lock_irqsave(&g_pmt3620_hif_proc->own_lock, flags);
    g_pmt3620_hif_proc->is_fw_own = true;
    spin_unlock_irqrestore(&g_pmt3620_hif_proc->own_lock, flags);


    return status;
}

int32_t mt3620_hif_open(void)
{
	struct mt3620_hif_func *func = &g_mt3620_hif_func;
	unsigned long flags;
	int32_t  ret = HIF_STATUS_SUCCESS;

	g_mt3620_hif_func.blksize = MY_HIF_BLOCK_SIZE;
	g_mt3620_hif_func.num = SDIO_GEN3_FUNCTION_WIFI;
#if (CFG_WIFI_HIF_GDMA_EN == 1)
	g_mt3620_hif_func.use_dma = 1;
#else
	g_mt3620_hif_func.use_dma = 0;
#endif


	// Enable GDMA
	if (g_mt3620_hif_func.use_dma)
	{
		//Need to Implement
	}




	// function enable

	ret = mt3620_hif_enable_func(func);

	if (ret)
	{
		dev_err(g_pmt3620_hif_proc->dev, "<<%s>> Enable function failed. Error = %d.\n", __FUNCTION__, (int)ret);
		goto err;
	}

	spin_lock_init(&g_pmt3620_hif_proc->own_lock);
	// Initialize the ownership flag
	spin_lock_irqsave(&g_pmt3620_hif_proc->own_lock, flags);
	g_pmt3620_hif_proc->is_fw_own = true;
	spin_unlock_irqrestore(&g_pmt3620_hif_proc->own_lock, flags);
	// set block size

	ret = mt3620_hif_set_block_size(func, func->blksize);

	mt3620_hif_get_irq(func);

	mt3620_hif_get_ownership();

	mt3620_hif_enable_enhance_mode();

	mt3620_hif_tx_flow_control_init();


	if (mt3620_hif_cr_write(WHIER, (RX0_DONE_INT_EN | RX1_DONE_INT_EN | FW_OWN_BACK_INT_EN)))
	{
		dev_err(g_pmt3620_hif_proc->dev, "FAIL. write WHIER failed (1).\n");
		ret = HIF_STATUS_FAIL;
		goto err;
	}
	else
	{
		uint32_t reg_value = 0;
		mt3620_hif_cr_read(WHISR, &reg_value);
		if ((reg_value & (RX0_DONE_INT_EN | RX1_DONE_INT_EN | FW_OWN_BACK_INT_EN)))
		{
			dev_err(g_pmt3620_hif_proc->dev, "FAIL. WHISR.RX0/1_DONE interrupt should be cleared first. (2). WHISR = 0x%08x.\n", (unsigned int)reg_value);
			ret = HIF_STATUS_FAIL;
			goto err;
		}
		reg_value = 0;
		mt3620_hif_cr_read(WHIER, &reg_value);
		if (!(reg_value & (RX0_DONE_INT_EN | RX1_DONE_INT_EN | FW_OWN_BACK_INT_EN)))
		{
			dev_err(g_pmt3620_hif_proc->dev, "FAIL. write WHIER failed (2). WHIER = 0x%08x.\n", (unsigned int)reg_value);
			ret = HIF_STATUS_FAIL;
			goto err;
		}
	}
err:
	return ret;
}

EXPORT_SYMBOL(mt3620_hif_open);


void mt3620_hif_disable_interrupt(void)
{
	int32_t ret;
	ret = mt3620_hif_cr_write(WHLPCR, W_INT_EN_CLR);

}


void mt3620_hif_enable_interrupt(void)
{

	mt3620_hif_cr_write(WHLPCR, W_INT_EN_SET);

}

EXPORT_SYMBOL(mt3620_hif_enable_interrupt);


void mt3620_hif_disable_whier_trx_int(void)
{

	uint32_t val;
	mt3620_hif_cr_read(WHIER, &val);
	val &= ~(TX_DONE_INT_EN | RX0_DONE_INT_EN | RX1_DONE_INT_EN);
	mt3620_hif_cr_write(WHIER, val);
}



void mt3620_hif_enable_whier_rx_int(void)
{

   uint32_t val;
   mt3620_hif_cr_read(WHIER, &val);
   val |= (RX0_DONE_INT_EN | RX1_DONE_INT_EN);
   mt3620_hif_cr_write(WHIER, val);
}

void mt3620_hif_enable_whier_tx_int(void)
{
	uint32_t val;
	mt3620_hif_cr_read(WHIER, &val);
	val |= (TX_DONE_INT_EN);
	mt3620_hif_cr_write(WHIER, val);
}


void mt3620_hif_tx_flow_control_update_free_page_cnt(enhance_mode_data_struct_t *p_enhance_mode_data)
{

#if (HIF_DEBUG_MODE_EN == 1)
	uint32_t i;
#endif

	wifi_hif_tx_flow_control_t* ctrl = &g_mt3620_hif_tx_flow_control_stat;

	#if (HIF_DEBUG_MODE_EN == 1)
	for (i = 0; i < NUM_OF_WIFI_TXQ; i ++)
	{
		ctrl->free_page_cnt_by_wifi_txq[i] += p_enhance_mode_data->tx_info.u.free_page_num[i];
	}
	ctrl->total_free_page_cnt += p_enhance_mode_data->tx_info.u.free_page_num[WIFI_TXQ_CNT_IDX_14_TXCFFA];
#endif


	ctrl->current_page_cnt -=  p_enhance_mode_data->tx_info.u.free_page_num[WIFI_TXQ_CNT_IDX_14_TXCFFA];

	ctrl->available_page_cnt =
		ctrl->reserve_quota_page_cnt - ctrl->current_page_cnt;

	/* let network driver know how much free TX space is remaining */
	mt3620_hif_api_handle_tx_update(ctrl->available_page_cnt * ctrl->page_sz);
}


int32_t mt3620_hif_tx_flow_control_check_and_update_tx(int32_t port, uint32_t pkt_len)
{

	uint32_t send_page = 0;
	int32_t ret = HIF_STATUS_SUCCESS;
	unsigned long flags;
	wifi_hif_tx_flow_control_t* ctrl = &g_mt3620_hif_tx_flow_control_stat;
	uint32_t val = 0;

	spin_lock_irqsave(&g_mt3620_hif_tx_flow_control_stat.hif_tx_flow_lock, flags);

	if (0 == ctrl->page_sz) {
		dev_err(g_pmt3620_hif_proc->dev, "ctrl->page_sz=%u \n",ctrl->page_sz);
		spin_unlock_irqrestore(&g_mt3620_hif_tx_flow_control_stat.hif_tx_flow_lock, flags);
		return HIF_STATUS_FAIL;
	}

	/* Reading Free Page Count */
	mt3620_hif_cr_read(WTQCR7, &val);
	ctrl->current_page_cnt -= (val & 0xFFFF);
	ctrl->available_page_cnt = ctrl->reserve_quota_page_cnt - ctrl->current_page_cnt;
	send_page = pkt_len / ctrl->page_sz;

	if ((pkt_len % ctrl->page_sz) > 0)
		send_page ++;

#if (HIF_DEBUG_MODE_EN == 1)
	if (g_mt3620_hif_tx_flow_ctrl_en)
	{
		if (send_page <= ctrl->available_page_cnt)
		{
			ctrl->send_page_cnt_by_tx_port[port] += send_page;
			ctrl->total_send_page_cnt += send_page;
			ctrl->send_pkt_cnt_by_tx_port[port] ++;
			ctrl->total_send_pkt_cnt ++;

			ctrl->current_page_cnt += send_page;

			if (ctrl->max_page_cnt < ctrl->current_page_cnt)
				ctrl->max_page_cnt = ctrl->current_page_cnt;

			ctrl->available_page_cnt =
				ctrl->reserve_quota_page_cnt - ctrl->current_page_cnt;

			ret = HIF_STATUS_SUCCESS;
		}
		else
		{
			ctrl->total_drop_pkt_cnt ++;
			ret = HIF_STATUS_FAIL;
		}
	}
	else
	{
		ctrl->send_page_cnt_by_tx_port[port] += send_page;
		ctrl->total_send_page_cnt += send_page;
		ctrl->send_pkt_cnt_by_tx_port[port] ++;
		ctrl->total_send_pkt_cnt ++;

		ctrl->current_page_cnt += send_page;

		if (ctrl->max_page_cnt < ctrl->current_page_cnt)
			ctrl->max_page_cnt = ctrl->current_page_cnt;

		ctrl->available_page_cnt =
			ctrl->reserve_quota_page_cnt - ctrl->current_page_cnt;
		ret = HIF_STATUS_SUCCESS;
	}
#else
		if (send_page <= ctrl->available_page_cnt)
		{
			ctrl->current_page_cnt += send_page;
			ctrl->available_page_cnt =
				ctrl->reserve_quota_page_cnt - ctrl->current_page_cnt;
			ret = HIF_STATUS_SUCCESS;
		}
		else
		{
			ctrl->total_drop_pkt_cnt ++;
			ret = HIF_STATUS_FAIL;
		}
#endif /* (HIF_DEBUG_MODE_EN == 1) */
	if(ret == HIF_STATUS_FAIL)
	{
		dev_dbg(g_pmt3620_hif_proc->dev, "sp=%u apc=%d rpc=%u ccp=%d tdpc=%u\n",send_page,ctrl->available_page_cnt,ctrl->reserve_quota_page_cnt,ctrl->current_page_cnt,ctrl->total_drop_pkt_cnt);

	}
	spin_unlock_irqrestore(&g_mt3620_hif_tx_flow_control_stat.hif_tx_flow_lock, flags);

	/* let network driver know how much free TX space is remaining */
	mt3620_hif_api_handle_tx_update(ctrl->available_page_cnt * ctrl->page_sz);

	return ret;
}




int32_t mt3620_hif_fifo_read(uint32_t addr, uint8_t *buf, size_t size)
{
	int32_t ret = HIF_STATUS_SUCCESS;
	struct mt3620_hif_func *dev_func = &g_mt3620_hif_func;

	ret = mt3620_hif_read_port(dev_func, buf, addr, size);
	if (ret)
	{
		ret = HIF_STATUS_FAIL;
	}
	return ret;
}



/*use to write data*/
int32_t mt3620_hif_fifo_write(uint8_t *buf, size_t size)
{
	int32_t ret = HIF_STATUS_SUCCESS;
	struct mt3620_hif_func *dev_func = &g_mt3620_hif_func;
	ret = mt3620_hif_write_port(dev_func, WTDR1, buf, size);
	return ret;
}





static void mt3620_hif_received_data(int32_t port, int16_t rx_len)
{
	struct n9_event_header *pmessage_ptr = NULL;
	uint32_t read_len = 0, addr = WRDR1;
	struct sk_buff *pskb=NULL;
#if (HIF_DEBUG_MODE_EN == 1)
	uint32_t continuous_allocate_fail_cnt = 0;
#endif

	if (port == WIFI_HIF_RX_PORT0_IDX){
		addr = WRDR0;
	}
	read_len = rx_len + WIFI_HIF_RX_CS_OFFLOAD_STATUS_LEN;

#if (HIF_DEBUG_MODE_EN == 1)
	if (rx_len > HIF_MAX_RX_PKT_SIZE){
		dev_warn(g_pmt3620_hif_proc->dev,"hif: receive too large pkt, rx_len = %u\n", rx_len);
		/* Driver must read the RX0/1 data completely reported by
		   interrupt enhance mode. No interrupt is trigger again for
		   the remaining packets.
		 */
		g_mt3620_hif_stat.rx_port[port].rx_invalid_sz_packet_cnt++;
		if (rx_len > g_mt3620_hif_stat.rx_port[port].rx_max_invalid_sz)
			g_mt3620_hif_stat.rx_port[port].rx_max_invalid_sz = rx_len;
		mt3620_hif_read_port_garbage(addr, read_len);
		return;
	}
#endif

	if(port == WIFI_HIF_RX_PORT0_IDX)
	{
		pmessage_ptr =  kmalloc(HIF_MAX_RX_PKT_SIZE, GFP_KERNEL);
		if (!pmessage_ptr) {
			dev_err(g_pmt3620_hif_proc->dev, "Cannot Allocate memeory for RX events\n");
			return;
		}

		mt3620_hif_fifo_read(addr, (uint8_t *)pmessage_ptr, read_len);

	}
	else
	{

		pskb = alloc_skb(HIF_MAX_RX_PKT_SIZE, GFP_ATOMIC);
		if (pskb != NULL)
		{
				mt3620_hif_fifo_read(addr, pskb->data, read_len);
			#if (HIF_DEBUG_MODE_EN == 1)
				continuous_allocate_fail_cnt = 0;
			#endif
		}

		else
		{
#if (HIF_DEBUG_MODE_EN == 1)
			dev_err(g_pmt3620_hif_proc->dev,"WARN! connsys: can't allocate buffer\n");
			continuous_allocate_fail_cnt ++;
			if ((continuous_allocate_fail_cnt > CFG_HIF_3628_CONTINOUS_ALLOCATE_FAIL_PRINT_CNT_VAL)&&
				(continuous_allocate_fail_cnt%CFG_HIF_3628_CONTINOUS_ALLOCATE_FAIL_PRINT_CNT_VAL == 1))
					dev_warn(NULL,"WARN! connsys: can't allocate buffer for %u times\n",
						(unsigned int)continuous_allocate_fail_cnt);

#endif
			mt3620_hif_read_port_garbage(addr, read_len);

#if (HIF_DEBUG_MODE_EN == 1)

			g_mt3620_hif_stat.rx_port[port].rx_allocate_fail_cnt ++;

#endif
			dev_err(g_pmt3620_hif_proc->dev, "Cannot Allocate memeory for RX data packets\n");
			return;
		}
	}
	if (rx_len >= WIFI_HIF_HEADER_LEN)
	{
		/* cmd packet */
		if (port == WIFI_HIF_RX_PORT0_IDX)
		{
			mt3620_hif_api_handle_n9_message(pmessage_ptr);
			kfree(pmessage_ptr);
			pmessage_ptr = NULL;
		}
		else  /* data packet */
		{
			mt3620_hif_api_handle_n9_data(pskb,rx_len);
			//Dont free the skb buffer, upper layer take case of skb memory

		}
	}
	else
	{
		pskb->len = rx_len;
		dev_err(g_pmt3620_hif_proc->dev, "[%s] WARN! len is less than WIFI HIF header length..\n",__FUNCTION__);
	}
	//packet handle complete now free the buffer


#if (HIF_DEBUG_MODE_EN == 1)

	g_mt3620_hif_stat.rx_port[port].rx_packet_cnt ++;

#endif
}




///
/// Bottom half IRQ interrupt handler for when we're reading data
///
/// @data - Interupt data - mbox_chan pointer
///
/// Note - this is a work queue and not a tasklet so rx callbacks can alloc
/// memory
static void mt3620_hif_rx_interrupt_bottom_half(struct work_struct *work)
{
	uint32_t rx_len = 0;//rx length of packet
	int32_t port=0;
	uint32_t ilen=0;
	uint32_t value=0;//cmd53 value
	enhance_mode_data_struct_t *p_int_enhance=
	(enhance_mode_data_struct_t *)&g_mt3620_last_enhance_mode_data_struct;

	// get ownership back
	if (mt3620_hif_get_ownership() == false)
	{
		dev_err(g_pmt3620_hif_proc->dev, "%s :get ownership failed",__FUNCTION__);
	}

	for (port = 0; port < NUM_OF_WIFI_HIF_RX_PORT; port++)
	{
		if (p_int_enhance->rx_info.u.valid_len_num[port] == 0)
			continue;

		for (ilen = 0; ilen < p_int_enhance->rx_info.u.valid_len_num[port]; ilen++)
		{
#if (HIF_DEBUG_MODE_EN == 1)
			int32_t rerror = 0;
			rerror = mt3620_hif_cr_read( WRPLR, &value);
			if (rerror)
			{
				dev_err(g_pmt3620_hif_proc->dev, "[%s] Read WRPLR failed. Error = 0x%x\n", __FUNCTION__, rerror);
				g_mt3620_hif_stat.rx_port[port].rx_error_cnt++;
				mt3620_hif_enable_whier_tx_int();
				mt3620_hif_enable_interrupt();
				return ;
			}
			else
			{
				if (port == WIFI_HIF_RX_PORT0_IDX)
				{
					rx_len = GET_RX0_PACKET_LENGTH(value);
				}
				else
				{
					rx_len = GET_RX1_PACKET_LENGTH(value);
				}
			}
#else
			mt3620_hif_cr_read(WRPLR, &value);
			if (port == WIFI_HIF_RX_PORT0_IDX)
				rx_len = GET_RX0_PACKET_LENGTH(value);
			else
				rx_len = GET_RX1_PACKET_LENGTH(value);
#endif
			if (rx_len == 0)
				break;
			mt3620_hif_received_data(port, rx_len);
		}
	}

	mt3620_hif_enable_whier_rx_int();
	mt3620_hif_enable_whier_tx_int();
	mt3620_hif_enable_interrupt();

}




///
/// IRQ interrupt handler for when we're reading data
///
/// @irq - IRQ number
/// @p - Pointer to mbox_chan object
/// @returns - IRQ result status code
static irqreturn_t mt3620_hif_interrupt(int irq, void *p)
{
	struct mt3620_hif_proc *prproc = (struct mt3620_hif_proc *)p;

	enhance_mode_data_struct_t *p_int_enhance;

#if (HIF_DEBUG_MODE_EN == 1)
	g_mt3620_hif_stat.number_of_int ++;
#endif


	mt3620_hif_disable_interrupt();

	mt3620_hif_disable_whier_trx_int();

	mt3620_hif_fifo_read(WHISR, (uint8_t *)(&g_mt3620_last_enhance_mode_data_struct), LEN_INT_ENHANCE_MODE);

	p_int_enhance = &g_mt3620_last_enhance_mode_data_struct;

	if (p_int_enhance->WHISR_reg_val == 0)
	{
		mt3620_hif_cr_write(WHLPCR, W_INT_EN_SET);
		mt3620_hif_enable_interrupt();
		return IRQ_NONE;
	}




#if (HIF_DEBUG_MODE_EN == 1)
	if ((p_int_enhance->WHISR_reg_val) & ABNORMAL_INT)
	{

		g_mt3620_hif_stat.number_of_abnormal_int ++;

	}

	if ((p_int_enhance->WHISR_reg_val) & FW_OWN_BACK_INT)
	{
		g_mt3620_hif_stat.number_of_fw_own_back ++;

	}
#endif



	if ((p_int_enhance->WHISR_reg_val) & TX_DONE_INT)
	{
#if (HIF_DEBUG_MODE_EN == 1)
			g_mt3620_hif_stat.num_of_tx_int ++;
#endif

			mt3620_hif_tx_flow_control_update_free_page_cnt(p_int_enhance);
	}

	if ((p_int_enhance->WHISR_reg_val) & (RX0_DONE_INT | RX1_DONE_INT))
	{
		if (((p_int_enhance->WHISR_reg_val) & TX_DONE_INT) == 0)
		{
			mt3620_hif_tx_flow_control_update_free_page_cnt(p_int_enhance);
		}

#if (HIF_DEBUG_MODE_EN == 1)
		g_mt3620_hif_stat.num_of_rx_int ++;
#endif
		schedule_work(&prproc->read_work);
	}
	else
	{
		mt3620_hif_enable_whier_rx_int();
		mt3620_hif_enable_whier_tx_int();
		mt3620_hif_enable_interrupt();
	}

	return IRQ_HANDLED;
}


///
/// Initialize the hif rproc driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int mt3620_hif_probe(struct platform_device *pdev)
{
	struct mt3620_hif_proc *pmt3620hif = NULL;
	struct resource *regs = NULL;
	int ret = SUCCESS;
	struct device_node *top_np;
	const __be32 *value;
	void __iomem *top_base;
	uint wfsoc_reg;

	// obtain top pointer

	top_np = of_find_node_by_name(NULL, "top");
	if (!top_np) {
		dev_err(pmt3620hif->dev,
			"'top' not found");
		return -ENODEV;
	}

	value = of_get_property(top_np, "reg", NULL);

	top_base = ioremap(be32_to_cpup(value), be32_to_cpup(value + 1));

	wfsoc_reg = readl_relaxed(top_base + TOP_WFSOC) | TOP_WFSOC_SET_RESET;

		writel_relaxed(wfsoc_reg,top_base + TOP_WFSOC);

	// Allocate memory for our driver state
	pmt3620hif = devm_kzalloc(&pdev->dev, sizeof(*pmt3620hif), GFP_KERNEL);
	if (!pmt3620hif) {
		return -ENOMEM;
	}

	pmt3620hif->dev = &pdev->dev;

	g_pmt3620_hif_proc = pmt3620hif;


	// Set up our spin lock and list of relay items
	spin_lock_init(&pmt3620hif->hif_lock);
	spin_lock_init(&g_mt3620_hif_tx_flow_control_stat.hif_tx_flow_lock);
	INIT_LIST_HEAD(&pmt3620hif->hif_management_list);

	dev_dbg(pmt3620hif->dev, "Starting mt3620 remote hif_proc driver\n");


	// Load the HIF sysram offset
	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pmt3620hif->phif_sysram_base= devm_ioremap_resource(pmt3620hif->dev, regs);

	if (IS_ERR(pmt3620hif->phif_sysram_base)) {
		return PTR_ERR(pmt3620hif->phif_sysram_base);
	}



	// Setup wlan HIF interrupts
	pmt3620hif->read_irq = platform_get_irq(pdev, 0);

	ret = devm_request_irq(pmt3620hif->dev, pmt3620hif->read_irq,
			mt3620_hif_interrupt, 0, DRIVER_NAME, pmt3620hif);
	if (unlikely(ret)) {
		dev_err(pmt3620hif->dev,
			"Failed to register RX HIF interrupt: %d", ret);
		return ret;
	}


	dev_dbg(pmt3620hif->dev, "devm_request_irq hif_proc driver\n");

	INIT_WORK(&pmt3620hif->read_work,
			mt3620_hif_rx_interrupt_bottom_half);

	//Initialization WLAN HIF
	mt3620_hif_open();


	platform_set_drvdata(pdev, pmt3620hif);

	// Set up our global pointer, this will be used by the remote API

	mt3620_hif_api_init();

	return ret;
}

///
/// Teardown the hifrproc driver
///
/// @pdev - Platform device for this module
/// @returns -  0 for success
static int mt3620_hif_remove(struct platform_device *pdev)
{
	struct mt3620_hif_proc *hifproc = platform_get_drvdata(pdev);
	unsigned long flags;

	if (!hifproc) {
		return -EINVAL;
	}

	mt3620_hif_api_shutdown();
	spin_lock_irqsave(&g_pmt3620_hif_proc->own_lock, flags);
	g_pmt3620_hif_proc->is_fw_own = true;
	spin_unlock_irqrestore(&g_pmt3620_hif_proc->own_lock, flags);

	return SUCCESS;
}

static const struct of_device_id mt3620_hif_match[] = {
	{.compatible = "mediatek,mt3620-hif-proc"}, {/* Sentinel */},
};

MODULE_DEVICE_TABLE(of, mt3620_hif_match);

static struct platform_driver mt3620_hif_driver = {
	.probe = mt3620_hif_probe,
	.remove = mt3620_hif_remove,
	.driver =
	{
		.name = DRIVER_NAME, .of_match_table = mt3620_hif_match,
	},
};

module_platform_driver(mt3620_hif_driver);

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("MT3620 hif driver specific functions");
MODULE_ALIAS("platform:mt3620-hif-proc");
