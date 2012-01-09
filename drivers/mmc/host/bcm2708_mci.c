/*
 *  linux/drivers/mmc/host/bcm2708_mci.c - Broadcom BCM2708 MCI driver
 *
 *  Copyright (C) 2010 Broadcom, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <linux/io.h>
#include <asm/sizes.h>
#include <mach/power.h>
#include <mach/dma.h>

#include "bcm2708_mci.h"

#define DRIVER_NAME "bcm2708_mci"

#define DBG(host,fmt,args...)	\
	pr_debug("%s: %s: " fmt, mmc_hostname(host->mmc), __func__ , args)

#define USE_DMA
#define USE_DMA_IRQ

#define VALIDATE_CLOCK_DIV 0

#define POWER_OFF 0
#define POWER_LAZY_OFF 1
#define POWER_ON  2

#define BCM2708_MCI_SLEEP_TIMEOUT 1000	 /* msecs */

static void do_command(void __iomem *base, u32 c, u32 a)
{
	writel(a, base + BCM2708_MCI_ARGUMENT);
	writel(c | BCM2708_MCI_ENABLE, base + BCM2708_MCI_COMMAND);

	while (readl(base + BCM2708_MCI_COMMAND) & BCM2708_MCI_ENABLE);
}

static void discard_words(void __iomem *base, int words)
{
	int i;
	for (i = 0; i < words; i++) {
		while (!(readl(base + BCM2708_MCI_STATUS) &
			 BCM2708_MCI_DATAFLAG));
		readl(base + BCM2708_MCI_DATA);
	}
}

/**
 * Show the DMA-using status
 */
static ssize_t attr_dma_show(struct device *_dev,
			     struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(_dev);

	if (mmc) {
		struct bcm2708_mci_host *host = mmc_priv(mmc);
		return sprintf(buf, "%d\n", host->use_dma? 1: 0);
	} else
		return -EINVAL;
}

/**
 * Set the DMA-using status
 */
static ssize_t attr_dma_store(struct device *_dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(_dev);

	if (mmc) {
#ifdef USE_DMA
		struct bcm2708_mci_host *host = mmc_priv(mmc);
		int on = simple_strtol(buf, NULL, 0);
		host->use_dma = on;
#endif
		return count;
	} else
		return -EINVAL;
}

static DEVICE_ATTR(use_dma, S_IRUGO | S_IWUGO, attr_dma_show, attr_dma_store);

/**
 * Show the readonly status
 */
static ssize_t attr_readonly_show(struct device *_dev,
				  struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(_dev);

	if (mmc) {
		struct bcm2708_mci_host *host = mmc_priv(mmc);
		return sprintf(buf, "%d\n", host->readonly? 1: 0);
	} else
		return -EINVAL;
}

/**
 * Set the readonly status
 */
static ssize_t attr_readonly_store(struct device *_dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(_dev);

	if (mmc) {
		struct bcm2708_mci_host *host = mmc_priv(mmc);
		int on = simple_strtol(buf, NULL, 0);
		host->readonly = on;
		return count;
	} else
		return -EINVAL;
}

static DEVICE_ATTR(readonly, S_IRUGO | S_IWUGO,
		   attr_readonly_show, attr_readonly_store);

/**
 * Show the status of the interface
 */
static ssize_t attr_status_show(struct device *_dev,
				struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc = (struct mmc_host *)dev_get_drvdata(_dev);

	if (mmc) {
		struct bcm2708_mci_host *host = mmc_priv(mmc);
		int rc;

		rc = sprintf(buf, "present: yes\npower: %s\n"
			     "bus: %d bit%s\ntiming: %s\n"
			     "clock: %u Hz (div %u)\n"
			     "dma: %s\nread-only %s\n",
			     host->power_state==POWER_ON? "on":
			     host->power_state==POWER_OFF? "off":
			     host->power_state==POWER_LAZY_OFF? "lazy-off":
			     "<unknown>",
			     host->current_bus_width==MMC_BUS_WIDTH_1? 1:
			     host->current_bus_width==MMC_BUS_WIDTH_4? 4:
			     0,
			     host->current_bus_width==MMC_BUS_WIDTH_1? "": "s",
			     host->current_timing==MMC_TIMING_LEGACY? "legacy":
			     host->current_timing==MMC_TIMING_SD_HS? "hs(SD)":
			     host->current_timing==MMC_TIMING_MMC_HS? "hs(MMC)":
			     "<unknown>",
			     host->current_clock_hz, host->current_clkdiv,
			     host->use_dma? "on": "off",
			     host->readonly? "on": "off"
			    );
		return rc;
	} else
		return sprintf(buf, "present: no\n");
}

static DEVICE_ATTR(status, S_IRUGO, attr_status_show, NULL);

static void wait_for_complete(struct bcm2708_mci_host *host,
			      void __iomem *mmc_base)
{
#ifdef USE_SDHOST_IRQ
#error not implemented yet
#else
	while ((readl(mmc_base + BCM2708_MCI_STATUS) &
		(BCM2708_MCI_HSTS_BUSY | BCM2708_MCI_HSTS_BLOCK)) == 0)
		continue;

	writel(BCM2708_MCI_HSTS_BUSY | BCM2708_MCI_HSTS_BLOCK,
	       mmc_base + BCM2708_MCI_STATUS);
#endif
}

static void
bcm2708_mci_start_command(struct bcm2708_mci_host *host,
			  struct mmc_command *cmd, struct mmc_data *data)
{
	void __iomem *mmc_base = host->mmc_base;
#ifdef USE_DMA
	void __iomem *dma_base = host->dma_base;
#endif
	u32 status;
	u32 c;
	int /*bool*/ redo = 0; /*FALSE*/
	/* retrying is not always safe and is handled by the caller */

	DBG(host, "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	if (host->readonly &&
	    (cmd->opcode == 19 /*bustest/retune*/  ||
	     cmd->opcode == 24 /*write block*/	   ||
	     cmd->opcode == 25 /*write multiple*/  ||
	     cmd->opcode == 32 /*sd erase start*/  ||
	     cmd->opcode == 33 /*sd erase end*/	   ||
	     cmd->opcode == 35 /*mmc erase start*/ ||
	     cmd->opcode == 36 /*mmc erase end*/)) {
		printk(KERN_ERR "%s: "DRIVER_NAME" attempted opcode %d"
		       "in read-only mode\n",
		       mmc_hostname(host->mmc), cmd->opcode);
		return;
	}

back:

	/*
	 * clear the controller status register
	 */

	writel(-1, mmc_base + BCM2708_MCI_STATUS);

	/*
	 * build the command register write, incorporating no
	 * response, long response, busy, read and write flags
	 */

	c = cmd->opcode;
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			c |= BCM2708_MCI_LONGRESP;
	} else
		c |= BCM2708_MCI_NORESP;
	if (cmd->flags & MMC_RSP_BUSY)
		c |= BCM2708_MCI_BUSY;
	if (data) {
		if (data->flags & MMC_DATA_READ)
			c |= BCM2708_MCI_READ;
		else
			c |= BCM2708_MCI_WRITE;
	}

	/*
	 * run the command and wait for it to complete
	 */

	do_command(mmc_base, c, cmd->arg);

	if (c & BCM2708_MCI_BUSY)
		wait_for_complete(host, mmc_base);

	/*
	 * retrieve the response and error (if any)
	 */

	status = readl(mmc_base + BCM2708_MCI_STATUS);

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[3] = readl(mmc_base + BCM2708_MCI_RESPONSE0);
		cmd->resp[2] = readl(mmc_base + BCM2708_MCI_RESPONSE1);
		cmd->resp[1] = readl(mmc_base + BCM2708_MCI_RESPONSE2);
		cmd->resp[0] = readl(mmc_base + BCM2708_MCI_RESPONSE3);
/*		printk("%08x:%08x:%08x:%08x %08x\n", cmd->resp[3],
		       cmd->resp[2], cmd->resp[1], cmd->resp[0], status); */
	} else {
		cmd->resp[0] = readl(mmc_base + BCM2708_MCI_RESPONSE0);
/*		printk("%08x %08x\n", cmd->resp[0], status); */
	}

	if (status & BCM2708_MCI_CMDTIMEOUT) {
		printk(KERN_ERR "%s: "DRIVER_NAME" timeout with opcode = %d, "
		       "data = %p, timeout = %d",
		       mmc_hostname(host->mmc), cmd->opcode, data,
		       readl(mmc_base + BCM2708_MCI_TIMEOUT));
		if (data)
			printk(" data->sg_len = %d\n", data->sg_len);
		else
			printk("\n");
		if (redo) {
		   printk(KERN_ERR "%s: retry opcode %d\n",
			  mmc_hostname(host->mmc), cmd->opcode);
			redo = 0/*FALSE*/;
			goto back;
		} else
			cmd->error = -ETIMEDOUT;
	}

	/*
	 * pump data if necessary
	 */

	if (data) {
		unsigned int sg_len = data->sg_len;
		struct scatterlist *sg_ptr = data->sg;

		data->bytes_xfered = 0;

#ifdef USE_DMA
		if (host->use_dma && bcm_sg_suitable_for_dma(sg_ptr, sg_len)) {
			int i, count = dma_map_sg(&host->dev->dev, sg_ptr, sg_len, data->flags & MMC_DATA_READ ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

			/* for reads, we can do the transfers without pausing */
			if (data->flags & MMC_DATA_READ) {
				for (i = 0; i < count; i++) {
					struct bcm2708_dma_cb *cb = &host->cb_base[i];

					cb->info   = BCM2708_DMA_PER_MAP(BCM2708_DMA_DREQ_SDHOST)|BCM2708_DMA_S_DREQ|BCM2708_DMA_D_WIDTH|BCM2708_DMA_D_INC;
					cb->src	   = 0x7e202040;
					cb->dst	   = sg_dma_address(&sg_ptr[i]);
					cb->length = sg_dma_len(&sg_ptr[i]);
					cb->stride = 0;

					if (i == count - 1) {
						cb->info |= BCM2708_DMA_INT_EN;
						cb->next = 0;
					} else
						cb->next = host->cb_handle +
						(i + 1) * sizeof(struct bcm2708_dma_cb);

					cb->pad[0] = 0;
					cb->pad[1] = 0;

					data->bytes_xfered += sg_ptr[i].length;
				}

				dsb();	/* ARM data synchronization (push) operation */

				writel(host->cb_handle, dma_base + BCM2708_DMA_ADDR);
				writel(BCM2708_DMA_ACTIVE, dma_base + BCM2708_DMA_CS);

#ifdef USE_DMA_IRQ
				down(&host->sem); /* wait for DMA IRQ */
#else
				while ((readl(dma_base + BCM2708_DMA_CS) & BCM2708_DMA_ACTIVE))
					continue;
#endif
			}

			/* ... for writing, we have to wait after every 512 bytes */

			else {
				for (i = 0; i < count; i++) {
					struct bcm2708_dma_cb *cb = &host->cb_base[i];
					unsigned int len = sg_dma_len(&sg_ptr[i]);
					unsigned int offset = 0;
					BUG_ON(len & 511);

					while (len != 0) {

						cb->info   = BCM2708_DMA_PER_MAP(BCM2708_DMA_DREQ_SDHOST)|
							     BCM2708_DMA_S_WIDTH|BCM2708_DMA_S_INC|BCM2708_DMA_D_DREQ;
						cb->src	   = sg_dma_address(&sg_ptr[i]) + offset;
						cb->dst	   = 0x7e202040;
						cb->stride = 0;
						cb->length = 512;
						len -= 512;
						offset += 512;

						/* we always do 512 bytes only and then wait for the write to complete */
						cb->info |= BCM2708_DMA_INT_EN;
						cb->next = 0;

						cb->pad[0] = 0;
						cb->pad[1] = 0;

						data->bytes_xfered += 512;

						dsb();	/* ARM data synchronization (push) operation */

						writel(host->cb_handle, dma_base + BCM2708_DMA_ADDR);
						writel(BCM2708_DMA_ACTIVE, dma_base + BCM2708_DMA_CS);

#ifdef USE_DMA_IRQ
						down(&host->sem); /* wait for DMA IRQ */
#else

						while ((readl(dma_base + BCM2708_DMA_CS) & BCM2708_DMA_ACTIVE))
							continue;
#endif

						wait_for_complete(host, mmc_base);
					}
				}
			}


			dma_unmap_sg(&host->dev->dev, sg_ptr, sg_len,
				     data->flags & MMC_DATA_READ ?
					DMA_FROM_DEVICE : DMA_TO_DEVICE);
		} else
#endif /* USE_DMA */
		while (sg_len) {
			unsigned long flags;

			/*
			 * map the current scatter buffer
			 */

			char *buffer = bcm2708_mci_kmap_atomic(sg_ptr, &flags);

			/*
			 * pump the data
			 */

			u32 *ptr = (u32 *)(buffer);
			u32 *lim = (u32 *)(buffer + sg_ptr->length);

			while (ptr < lim) {
				while (!(readl(mmc_base + BCM2708_MCI_STATUS) &
					 BCM2708_MCI_DATAFLAG));

				if (data->flags & MMC_DATA_READ)
					*ptr++ = readl(mmc_base +
						       BCM2708_MCI_DATA);
				else
					writel(*ptr++, mmc_base +
						       BCM2708_MCI_DATA);
			}

			/*
			 * unmap the buffer
			 */

			bcm2708_mci_kunmap_atomic(buffer, &flags);

			/*
			 * if we were reading, and we have completed this
			 * page, ensure that the data cache is coherent
			 */

			if (data->flags & MMC_DATA_READ)
				flush_dcache_page(sg_page(sg_ptr));

			data->bytes_xfered += sg_ptr->length;

			sg_ptr++;
			sg_len--;
		}

		if (host->is_acmd && cmd->opcode == SD_APP_SEND_SCR)
			discard_words(mmc_base, 126);
		if (host->is_acmd && cmd->opcode == SD_APP_SEND_NUM_WR_BLKS)
			discard_words(mmc_base, 127);
		if (!host->is_acmd && cmd->opcode == SD_SWITCH)
			discard_words(mmc_base, 112);

		if (data->stop)
			bcm2708_mci_start_command(host, data->stop, 0);
	}

	/*
	 * remember if we're an application command
	 */

	host->is_acmd = cmd->opcode == MMC_APP_CMD;
}

static int/*bool*/ bcm2708_mci_set_bus_width(struct bcm2708_mci_host *host,
					     unsigned char width)
{
	unsigned hconfig = readl(host->mmc_base + BCM2708_MCI_HOSTCONFIG);

	switch (width) {
		case MMC_BUS_WIDTH_1:
			writel(hconfig & ~(1<<2)/*WIDEDATABUS*/,
			       host->mmc_base + BCM2708_MCI_HOSTCONFIG);
			return 1;
		break;

		case MMC_BUS_WIDTH_4:
			writel(hconfig | (1<<2)/*WIDEDATABUS*/,
			       host->mmc_base + BCM2708_MCI_HOSTCONFIG);
			return 1;
		break;

		default:
			return 0;
	}
}

static int/*bool*/ bcm2708_mci_set_clkdiv(struct bcm2708_mci_host *host,
					  unsigned int divisor)
{
	divisor -= 2; /* we store this value in SDCDIV */

	/* Only bottom bits 11 bits of SDCDIV honoured */
	if (divisor != host->current_clkdiv && divisor < (1<<12)) {
		writel(divisor, host->mmc_base + BCM2708_MCI_CLKDIV);
		host->current_clkdiv = divisor;
		return 1;
	} else
		return 0;
}

static unsigned int bcm2708_mci_get_clkdiv(struct bcm2708_mci_host *host)
{
	return readl(host->mmc_base + BCM2708_MCI_CLKDIV) + 2;
}

static void bcm2708_mci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct bcm2708_mci_host *host = mmc_priv(mmc);

	if (mrq->data && !is_power_of_2(mrq->data->blksz)) {
		printk(KERN_ERR "%s: Unsupported block size (%d bytes)\n",
			mmc_hostname(mmc), mrq->data->blksz);
		mrq->cmd->error = -EINVAL;
		mmc_request_done(mmc, mrq);
		return;
	}

	bcm2708_mci_start_command(host, mrq->cmd, mrq->data);

	mmc_request_done(host->mmc, mrq);
}


static void bcm2708_mci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct bcm2708_mci_host *host = mmc_priv(mmc);

	mmc_host_enable(host->mmc);

	if (ios->power_mode != host->power_mode)
	{
		switch (ios->power_mode)
		{
		case MMC_POWER_OFF:
			bcm_power_request(host->power_handle, BCM_POWER_NONE);
			break;

		case MMC_POWER_UP:
			bcm_power_request(host->power_handle, BCM_POWER_SDCARD);
			/*
			 * We need an extra 10ms delay of 10ms before we
			 * can apply clock after applying power
			 */
			mdelay(10);
			break;

		case MMC_POWER_ON:
			mdelay(10);
			/* do_send_init_stream = 1; */
			break;
		}

		host->power_mode = ios->power_mode;
	}

	if (host->current_bus_width != ios->bus_width) {
		if (bcm2708_mci_set_bus_width(host, ios->bus_width))
			host->current_bus_width = ios->bus_width;
		else
			printk(KERN_ERR "%s: unsupported bus width requested "
			       "(id %d)\n",
			       mmc_hostname(mmc), ios->bus_width);
	}
	if (host->current_timing != ios->timing) {
		switch (ios->timing) {
			case MMC_TIMING_LEGACY:
				break;
			case MMC_TIMING_SD_HS:
				break;
			case MMC_TIMING_MMC_HS:
				break;
		}
		printk(KERN_ERR "%s: unsupported timing requested (id %d)\n",
			mmc_hostname(mmc), ios->timing);
	}
	if (host->current_clock_hz != ios->clock) {
		unsigned int hz = ios->clock;

		if (host->base_clock != 0) {
			if (hz == 0) {
				/* Does this may mean "turn the clock off"? */
				/* We'll set a minimum speed anyway */
				hz = 100000; /*100kHz */
			}
#if VALIDATE_CLOCK_DIV
			{
				unsigned int clkdiv =
					     bcm2708_mci_get_clkdiv(host);
				unsigned int current_clk =
					     host->base_clock/clkdiv;

				if (current_clk != host->current_clock_hz)
					printk(KERN_ERR "%s: current clock "
					       "rate estimate inacurate - "
					       "was %uHz, measured %uHz\n",
					       mmc_hostname(mmc),
					       host->current_clock_hz,
					       current_clk);
			}
#endif
			if (bcm2708_mci_set_clkdiv(host, host->base_clock/hz))
				host->current_clock_hz = hz;
				/* (information only) */
			else
				hz = 0;
		}

		if (hz == 0)
			printk(KERN_ERR "%s: unsupported clock rate "
			       "requested (%uHz)\n",
				mmc_hostname(mmc), ios->clock);
	}

	if (host->power_mode == MMC_POWER_OFF)
		mmc_host_disable(host->mmc);
	else
		mmc_host_lazy_disable(host->mmc);
}

static int bcm2708_mci_enable(struct mmc_host *mmc)
{
	struct bcm2708_mci_host *host = mmc_priv(mmc);
	int rc;

	if (host->power_state == POWER_OFF)
	{
		rc = bcm_power_request(host->power_handle, BCM_POWER_SDCARD);

		if (rc == 0)
		{
			mmc_power_restore_host(mmc);
			host->power_state = POWER_ON;
		}
		else
		{
			printk(KERN_ERR "mmc power up request failed\n");
		}
	}
	else
	{
		host->power_state = POWER_ON;
		rc = 0;
	}

	return rc;
}

static int bcm2708_mci_disable(struct mmc_host *mmc, int lazy)
{
	struct bcm2708_mci_host *host = mmc_priv(mmc);
	int rc;

	if ((host->power_state == POWER_ON) && lazy)
	{
		host->power_state = POWER_LAZY_OFF;

		return BCM2708_MCI_SLEEP_TIMEOUT;
	}

	rc = bcm_power_request(host->power_handle, BCM_POWER_NONE);

	if (rc == 0)
		host->power_state = POWER_OFF;
	else
		printk(KERN_ERR "mmc power down request failed\n");

	return rc;
}

/*
 * Handle completion of command and data transfers.
 */

#if 0
static irqreturn_t bcm2708_mci_command_irq(int irq, void *dev_id)
{
	struct bcm2708_mci_host *host = dev_id;

	writel(BCM2708_DMA_INT, host->dma_base + BCM2708_DMA_CS);

	printk(KERN_ERR "irq\n");

	return IRQ_RETVAL(IRQ_HANDLED);
}
#endif

#ifdef USE_DMA_IRQ
static irqreturn_t bcm2708_mci_data_irq(int irq, void *dev_id)
{
	struct bcm2708_mci_host *host = (struct bcm2708_mci_host *)dev_id;
	irqreturn_t handled = IRQ_NONE;

	if (0 != (BCM2708_DMA_INT & readl(host->dma_base + BCM2708_DMA_CS)))
	{
		/* acknowledge interrupt */
		writel(BCM2708_DMA_INT, host->dma_base + BCM2708_DMA_CS);

		dsb(); /* ARM data synchronization (push) operation */

		handled = IRQ_HANDLED;
		up(&host->sem); /* signal DMA handler that DMA is complete */
	}

	return IRQ_RETVAL(handled);
}
#endif

static const struct mmc_host_ops bcm2708_mci_ops = {
	.request = bcm2708_mci_request,
	.set_ios = bcm2708_mci_set_ios,
	.enable	 = bcm2708_mci_enable,
	.disable = bcm2708_mci_disable,
};

static int __devinit bcm2708_mci_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct bcm2708_mci_host *host;
	struct resource *mmc_res;
#if 0
	struct resource *cmd_res;
#endif
	struct clk *mmc_clk;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct bcm2708_mci_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		dev_dbg(&pdev->dev, "couldn't allocate mmc host\n");
		goto fail0;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->readonly = 0;
#ifdef USE_DMA
	host->use_dma = 1;
#else
	host->use_dma = 0;
#endif
	host->dev = pdev;

	ret = bcm_power_open(&host->power_handle);
	if (ret != 0)
		goto fail0a;
	host->power_mode = MMC_POWER_OFF;
	host->power_state = POWER_OFF;
	host->current_bus_width = MMC_BUS_WIDTH_1;
	host->current_timing = MMC_TIMING_LEGACY;

	sema_init(&host->sem, 0);

	mmc_clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(mmc_clk)) {
		printk(KERN_INFO "%s: clock '%s' unknown - "
		      "clock changes disallowed - err %ld\n",
		       mmc_hostname(mmc), dev_name(&pdev->dev),
		       PTR_ERR(mmc_clk));
		host->base_clock = 0;
	} else {
		/* mmc->caps |= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED */
		host->base_clock = clk_get_rate(mmc_clk);
	}

	host->cb_base = dma_alloc_writecombine(&pdev->dev, SZ_4K,
					       &host->cb_handle, GFP_KERNEL);
	if (!host->cb_base) {
		ret = -ENOMEM;
		goto fail1;
	}


	mmc_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mmc_res) {
		ret = -ENXIO;
		goto fail2;
	}

	if (!request_mem_region(mmc_res->start,
				mmc_res->end - mmc_res->start + 1,
				DRIVER_NAME)) {
		ret = -EBUSY;
		goto fail2;
	}

	/*
	 * Map I/O regions
	 */

	host->mmc_base = ioremap(mmc_res->start, resource_size(mmc_res));
	if (!host->mmc_base) {
		ret = -ENOMEM;
		goto fail4;
	}

	ret = bcm_dma_chan_alloc(BCM_DMA_FEATURE_FAST, &host->dma_base,
				 &host->dma_irq);
	if (ret < 0) {
		printk(KERN_ERR "%s: couldn't allocate a DMA channel\n",
		       mmc_hostname(mmc));
		goto fail5;
	}
	host->dma_chan = ret;

	/*
	 * Grab interrupts.
	 */

#ifdef USE_DMA_IRQ
	ret = request_irq(host->dma_irq, bcm2708_mci_data_irq,
			  IRQF_SHARED, DRIVER_NAME " (dat)", host);
	if (ret) {
		goto fail6;
	}
#endif
#if 0
	cmd_res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!cmd_res) {
		ret = -ENXIO;
		goto fail6;
	}

	ret = request_irq(cmd_res->start, bcm2708_mci_command_irq,
			  IRQF_SHARED, DRIVER_NAME " (cmd)", host);
	if (ret) {
		goto fail6;
	}
#endif
	host->current_clock_hz = 0;
	host->current_clkdiv = bcm2708_mci_get_clkdiv(host);
	host->is_acmd = 0;

	mmc->ops = &bcm2708_mci_ops;
	mmc->f_min = 375000;
	mmc->f_max = 25000000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	/*
	 * We can do SGIO
	 */
	mmc->max_hw_segs = 16;
	mmc->max_phys_segs = NR_SG;

	/*
	 * Since we only have a 16-bit data length register, we must
	 * ensure that we don't exceed 2^16-1 bytes in a single request.
	 */
	mmc->max_req_size = 65535;

	/*
	 * Set the maximum segment size.  Since we aren't doing DMA
	 * (yet) we are only limited by the data length register.
	 */
	mmc->max_seg_size = mmc->max_req_size;

	/*
	 * Block size can be up to 2048 bytes, but must be a power of two.
	 */
	mmc->max_blk_size = 2048;

	/*
	 * No limit on the number of blocks transferred.
	 */
	mmc->max_blk_count = mmc->max_req_size;

	/*
	 * We support 4-bit data (at least on the DB) and power management
	 */

	mmc->caps |= MMC_CAP_4_BIT_DATA | MMC_CAP_DISABLE;

	mmc_add_host(mmc);

	platform_set_drvdata(pdev, mmc);
	ret = device_create_file(&pdev->dev, &dev_attr_readonly);
	ret = device_create_file(&pdev->dev, &dev_attr_use_dma);
	ret = device_create_file(&pdev->dev, &dev_attr_status);

	printk(KERN_INFO "%s: BCM2708 SD host at 0x%08llx DMA %d IRQ %d\n",
	       mmc_hostname(mmc),
	       (unsigned long long)mmc_res->start,
	       host->dma_chan, host->dma_irq);

	return 0;

#ifdef USE_DMA_IRQ
	free_irq(host->dma_irq, host);
#endif
fail6:
	bcm_dma_chan_free(host->dma_chan);
fail5:
	iounmap(host->mmc_base);
fail4:
	release_mem_region(mmc_res->start, mmc_res->end - mmc_res->start + 1);
fail2:
	dma_free_writecombine(&pdev->dev, SZ_4K, host->cb_base,
			      host->cb_handle);
fail1:
	bcm_power_close(host->power_handle);
fail0a:
	mmc_free_host(mmc);
fail0:
	dev_err(&pdev->dev, "probe failed, err %d\n", ret);
	return ret;
}

static int __devexit bcm2708_mci_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_status);
	device_remove_file(&pdev->dev, &dev_attr_use_dma);
	device_remove_file(&pdev->dev, &dev_attr_readonly);

	if (mmc) {
		struct bcm2708_mci_host *host = mmc_priv(mmc);
		struct resource *res;

		mmc_host_enable(mmc);
		mmc_remove_host(mmc);
#ifdef USE_DMA_IRQ
		free_irq(host->dma_irq, host);
#endif
		mmc_host_disable(mmc);

		iounmap(host->mmc_base);
		bcm_dma_chan_free(host->dma_chan);

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		release_mem_region(res->start, resource_size(res));
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		release_mem_region(res->start, resource_size(res));

		dma_free_writecombine(&pdev->dev, SZ_4K, host->cb_base,
				      host->cb_handle);

		bcm_power_close(host->power_handle);
		mmc_free_host(mmc);
		platform_set_drvdata(pdev, NULL);

		return 0;
	} else
		return -1;
}

#ifdef CONFIG_PM
static int bcm2708_mci_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_suspend_host(mmc);

	return ret;
}

static int bcm2708_mci_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define bcm2708_mci_suspend	NULL
#define bcm2708_mci_resume	NULL
#endif

static struct platform_driver bcm2708_mci_driver = {
	.probe		= bcm2708_mci_probe,
	.remove		= bcm2708_mci_remove,
	.suspend	= bcm2708_mci_suspend,
	.resume		= bcm2708_mci_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init bcm2708_mci_init(void)
{
	return platform_driver_register(&bcm2708_mci_driver);
}

static void __exit bcm2708_mci_exit(void)
{
	platform_driver_unregister(&bcm2708_mci_driver);
}


module_init(bcm2708_mci_init);
module_exit(bcm2708_mci_exit);

MODULE_DESCRIPTION("BCM2708 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm2708_mci");
