/*
 *  linux/arch/arm/mach-pxa/lx.c
 *
 *  Author:     Ben Dooks
 *  Copyright   (c) 2004 Simtec Electronics
 *
 *  Based on:
 *    linux/arch/arm/mach-pxa/lubbock.c
 *
 *    Author:	Nicolas Pitre
 *    Created:	Jun 15, 2001
 *    Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/usb/sl811.h>
#include <linux/gpio.h>
#include <linux/memblock.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c/pxa-i2c.h>
#include <linux/i2c-pxa.h>
#include <linux/mtd/nand-gpio.h>
#include <video/s1d13xxxfb.h>

#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/pxa2xx-regs.h>
#include <mach/pxa25x.h>
#include <mach/udc.h>
#include <mach/audio.h>
#include <mach/pxafb.h>
#include <mach/lx-pcon.h>
#include <mach/lx.h>
#include <mach/irda.h>
#include <mach/mmc.h>
#include <mach/mfp-pxa25x.h>

#include "generic.h"

/*
 * Note: we don't add low power mode information to this table; the
 * PXA is completely powered down in these modes, so it's pointless.
 */
static unsigned long lx_pin_config[] __initdata = {
	/* PM */
	GPIO0_GPIO,
	GPIO1_GPIO,

	GPIO2_GPIO,				/* SL811 host IRQ */
	GPIO3_GPIO,				/* nHeadphones */
	GPIO4_GPIO,				/* PENDET */
	GPIO5_GPIO,				/* Modem IRQ */

	/* MMC */
	GPIO6_MMC_CLK,
	GPIO7_GPIO,				/* MMC int */
	GPIO8_MMC_CS0,
	GPIO9_GPIO,				/* MMC detect */
	GPIO10_GPIO,				/* MMC WP */

	GPIO11_GPIO,
	GPIO12_GPIO,
	GPIO13_GPIO,

	GPIO14_GPIO | MFP_LPM_DRIVE_LOW,	/* SL811 reset */
	GPIO15_nCS_1,				/* NAND */

	GPIO16_PWM0_OUT,			/* Beep signal */
	GPIO17_PWM1_OUT,			/* Beep volume */

	GPIO18_RDY,

	/* SPI DMA */
	GPIO19_DREQ_1,

	/* NAND */
	GPIO20_GPIO,				/* Ready */
	GPIO21_GPIO | MFP_LPM_DRIVE_LOW,	/* CLE */
	GPIO22_GPIO | MFP_LPM_DRIVE_LOW,	/* ALE */

	/* SPI */
	GPIO23_SSP1_SCLK,
	GPIO24_SSP1_SFRM,
	GPIO25_SSP1_TXD,
	GPIO26_SSP1_RXD,
	GPIO27_SSP1_EXTCLK,

	/* AC97 */
	GPIO28_AC97_BITCLK,
	GPIO29_AC97_SDATA_IN_0,
	GPIO30_AC97_SDATA_OUT,
	GPIO31_AC97_SYNC,

	/* S1D13806 */
	GPIO33_nCS_5,

	/* FFUART */
	GPIO34_FFUART_RXD,
	GPIO35_FFUART_CTS,
	GPIO36_FFUART_DCD,
	GPIO37_FFUART_DSR,
	GPIO38_FFUART_RI,
	GPIO39_FFUART_TXD,
	GPIO40_FFUART_DTR,
	GPIO41_FFUART_RTS,

	/* BT Uart */
	GPIO42_BTUART_RXD,
	GPIO43_BTUART_TXD,
	GPIO44_BTUART_CTS,
	GPIO45_BTUART_RTS,

	/* IrDA */
	GPIO46_STUART_RXD,
	GPIO47_STUART_TXD,

	/* PCMCIA */
	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO50_nPIOR,
	GPIO51_nPIOW,
	GPIO52_nPCE_1,
	GPIO53_nPCE_2,
	GPIO54_nPSKTSEL,
	GPIO55_nPREG,
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,

	/* IrDA transceiver control */
	GPIO58_GPIO | MFP_LPM_DRIVE_LOW,
	GPIO59_GPIO | MFP_LPM_DRIVE_LOW,

	GPIO60_GPIO | MFP_LPM_DRIVE_LOW,	/* USB slave enable */

	/* More NAND */
	GPIO61_GPIO | MFP_LPM_DRIVE_LOW,	/* NWP */
	GPIO62_GPIO | MFP_LPM_DRIVE_HIGH,	/* NCE */

	GPIO63_GPIO | MFP_LPM_DRIVE_LOW,	/* AC97 - reset */
	GPIO65_GPIO | MFP_LPM_DRIVE_HIGH,	/* AC97 - amp */

	/* CF */
	GPIO70_GPIO | MFP_LPM_DRIVE_LOW,	/* Reset */
	GPIO71_GPIO,				/* Ready/IRQ */
	GPIO72_GPIO,				/* StatusChange */
	GPIO73_GPIO,				/* CardDetect */

	/* PCMCIA */
	GPIO74_GPIO | MFP_LPM_DRIVE_LOW,	/* Reset */
	GPIO75_GPIO,				/* CardDetect */
	GPIO76_GPIO,				/* StatusChange */
	GPIO77_GPIO,				/* Ready/IRQ */

	GPIO78_nCS_2,
	GPIO79_nCS_3,				/* Modem CS */
	GPIO80_nCS_4,				/* SL811 CS */
};

static void lx_power_off(void)
{
	pcon_pwr_setstate(PCON_PWRSTATE_Off);
}

static void lx_restart(char str, const char *cmd)
{
	pcon_pwr_setstate(PCON_PWRSTATE_HWReset);
}

static struct pxa2xx_spi_master lx_ssp_master_info = {
	.num_chipselect	= 1,
	.enable_dma	= 1
};

static struct pxa2xx_spi_chip lx_battmon_chip = {
	.tx_threshold	= 8,
	.rx_threshold	= 8,
	.dma_burst_size	= 8,
};

static struct spi_board_info lx_spi_board_info[] __initdata = { {
	.modalias		= "lx-battery",
	.max_speed_hz		= 921600,
	.chip_select		= 0,
	.mode			= SPI_MODE_0,
	.bus_num		= 1,
	.controller_data	= &lx_battmon_chip,
},
};

static struct s1d13xxxfb_regval lx_s1d13xxx_regs[] = {
	{ S1DREG_MISC,			0x00 },
	{ S1DREG_COM_DISP_MODE,		0x00 },
	{ S1DREG_GPIO_CNF0,		0xFF },
	{ S1DREG_GPIO_CNF1,		0x1F },
	{ S1DREG_GPIO_CTL0,		0x7F },
	{ S1DREG_GPIO_CTL1,		0x00 },
	{ S1DREG_CLK_CNF,		0x01 },
	{ S1DREG_LCD_CLK_CNF,		0x00 },
	{ S1DREG_CRT_CLK_CNF,		0x02 },
	{ S1DREG_MPLUG_CLK_CNF,		0x02 },
	{ S1DREG_CPU2MEM_WST_SEL,	0x01 },
	{ S1DREG_SDRAM_REF_RATE,	0x03 },
	{ S1DREG_SDRAM_TC0,		0x00 },
	{ S1DREG_SDRAM_TC1,		0x01 },
	{ S1DREG_MEM_CNF,		0x80 },
	{ S1DREG_PANEL_TYPE,		0x25 },
	{ S1DREG_MOD_RATE,		0x00 },
	{ S1DREG_LCD_DISP_HWIDTH,	0x63 },
	{ S1DREG_LCD_NDISP_HPER,	0x1F },
	{ S1DREG_TFT_FPLINE_START,	0x01 },
	{ S1DREG_TFT_FPLINE_PWIDTH,	0x0B },
	{ S1DREG_LCD_DISP_VHEIGHT0,	0x57 },
	{ S1DREG_LCD_DISP_VHEIGHT1,	0x02 },
	{ S1DREG_LCD_NDISP_VPER,	0x1B },
	{ S1DREG_TFT_FPFRAME_START,	0x0A },
	{ S1DREG_TFT_FPFRAME_PWIDTH,	0x01 },
	{ S1DREG_LCD_DISP_MODE,		0x05 },
	{ S1DREG_LCD_MISC,		0x00 },
	{ S1DREG_LCD_DISP_START0,	0x00 },
	{ S1DREG_LCD_DISP_START1,	0x00 },
	{ S1DREG_LCD_DISP_START2,	0x00 },
	{ S1DREG_LCD_MEM_OFF0,		0x20 },
	{ S1DREG_LCD_MEM_OFF1,		0x03 },
	{ S1DREG_LCD_PIX_PAN,		0x00 },
	{ S1DREG_LCD_DISP_FIFO_HTC,	0x00 },
	{ S1DREG_LCD_DISP_FIFO_LTC,	0x00 },
	{ S1DREG_CRT_DISP_HWIDTH,	0x00 },
	{ S1DREG_CRT_NDISP_HPER,	0x00 },
	{ S1DREG_CRT_HRTC_START,	0x00 },
	{ S1DREG_CRT_HRTC_PWIDTH,	0x00 },
	{ S1DREG_CRT_DISP_VHEIGHT0,	0x00 },
	{ S1DREG_CRT_DISP_VHEIGHT1,	0x00 },
	{ S1DREG_CRT_NDISP_VPER,	0x00 },
	{ S1DREG_CRT_VRTC_START,	0x00 },
	{ S1DREG_CRT_VRTC_PWIDTH,	0x00 },
	{ S1DREG_TV_OUT_CTL,		0x00 },
	{ S1DREG_CRT_DISP_MODE,		0x00 },
	{ S1DREG_CRT_DISP_START0,	0x00 },
	{ S1DREG_CRT_DISP_START1,	0x00 },
	{ S1DREG_CRT_DISP_START2,	0x00 },
	{ S1DREG_CRT_MEM_OFF0,		0x00 },
	{ S1DREG_CRT_MEM_OFF1,		0x00 },
	{ S1DREG_CRT_PIX_PAN,		0x00 },
	{ S1DREG_CRT_DISP_FIFO_HTC,	0x00 },
	{ S1DREG_CRT_DISP_FIFO_LTC,	0x00 },
	{ S1DREG_LCD_CUR_CTL,		0x00 },
	{ S1DREG_LCD_CUR_START,		0x00 },
	{ S1DREG_LCD_CUR_XPOS0,		0x00 },
	{ S1DREG_LCD_CUR_XPOS1,		0x00 },
	{ S1DREG_LCD_CUR_YPOS0,		0x00 },
	{ S1DREG_LCD_CUR_YPOS1,		0x00 },
	{ S1DREG_LCD_CUR_BCTL0,		0x00 },
	{ S1DREG_LCD_CUR_GCTL0,		0x00 },
	{ S1DREG_LCD_CUR_RCTL0,		0x00 },
	{ S1DREG_LCD_CUR_BCTL1,		0x00 },
	{ S1DREG_LCD_CUR_GCTL1,		0x00 },
	{ S1DREG_LCD_CUR_RCTL1,		0x00 },
	{ S1DREG_LCD_CUR_FIFO_HTC,	0x00 },
	{ S1DREG_CRT_CUR_CTL,		0x00 },
	{ S1DREG_CRT_CUR_START, 	0x00 },
	{ S1DREG_CRT_CUR_XPOS0, 	0x00 },
	{ S1DREG_CRT_CUR_XPOS1, 	0x00 },
	{ S1DREG_CRT_CUR_YPOS0, 	0x00 },
	{ S1DREG_CRT_CUR_YPOS1, 	0x00 },
	{ S1DREG_CRT_CUR_BCTL0, 	0x00 },
	{ S1DREG_CRT_CUR_GCTL0, 	0x00 },
	{ S1DREG_CRT_CUR_RCTL0, 	0x00 },
	{ S1DREG_CRT_CUR_BCTL1, 	0x00 },
	{ S1DREG_CRT_CUR_GCTL1, 	0x00 },
	{ S1DREG_CRT_CUR_RCTL1, 	0x00 },
	{ S1DREG_CRT_CUR_FIFO_HTC,	0x00 },
	{ S1DREG_BBLT_CTL0,		0x00 },
	{ S1DREG_BBLT_CTL1,		0x01 },
	{ S1DREG_BBLT_CC_EXP,		0x00 },
	{ S1DREG_BBLT_OP,		0x0C },
	{ S1DREG_BBLT_SRC_START0,	0x00 },
	{ S1DREG_BBLT_SRC_START1,	0x00 },
	{ S1DREG_BBLT_SRC_START2,	0x00 },
	{ S1DREG_BBLT_DST_START0,	0x28 },
	{ S1DREG_BBLT_DST_START1,	0xBD },
	{ S1DREG_BBLT_DST_START2,	0x06 },
	{ S1DREG_BBLT_MEM_OFF0,		0x20 },
	{ S1DREG_BBLT_MEM_OFF1,		0x03 },
	{ S1DREG_BBLT_WIDTH0,		0x09 },
	{ S1DREG_BBLT_WIDTH1,		0x00 },
	{ S1DREG_BBLT_HEIGHT0,		0x0b },
	{ S1DREG_BBLT_HEIGHT1,		0x00 },
	{ S1DREG_BBLT_BGC0,		0x00 },
	{ S1DREG_BBLT_BGC1,		0x00 },
	{ S1DREG_BBLT_FGC0,		0x00 },
	{ S1DREG_BBLT_FGC1,		0x00 },
	{ S1DREG_LKUP_MODE,		0x00 },
	{ S1DREG_LKUP_ADDR,		0x00 },
	{ S1DREG_PS_CNF,		0x10 },
	{ S1DREG_PS_STATUS,		0x00 },
	{ S1DREG_CPU2MEM_WDOGT,		0x00 },
	{ S1DREG_COM_DISP_MODE,		0x01 },
};

static struct s1d13xxxfb_pdata lx_s1d13xxx_data = {
	.initregs = lx_s1d13xxx_regs,
	.initregssize = ARRAY_SIZE(lx_s1d13xxx_regs),
};

static struct resource lx_s1d13xxx_resources[] = {
	[0] = {
		.start	= PXA_CS5_PHYS + (1<<21),
		.end	= PXA_CS5_PHYS + (1<<21) + (1280*1024)-1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= PXA_CS5_PHYS,
		.end	= PXA_CS5_PHYS + 0xfff,
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device lx_device_fb = {
	.name        = S1D_DEVICENAME,
	.id          = 0,
	.dev         = {
		.platform_data     = &lx_s1d13xxx_data,
	},
	.resource      = lx_s1d13xxx_resources,
	.num_resources = ARRAY_SIZE(lx_s1d13xxx_resources)
};


static struct resource sl811_resources[] = {
	[0] = {
		.start	= PXA_CS4_PHYS + 0x000,
		.end	= PXA_CS4_PHYS + 0x100,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= PXA_CS4_PHYS + (1<<16) + 0x000,
		.end	= PXA_CS4_PHYS + (1<<16) + 0x100,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start  = IRQ_GPIO(GPIO_LX_USBH_IRQ),
		.end    = IRQ_GPIO(GPIO_LX_USBH_IRQ),
		.flags  = IORESOURCE_IRQ,
	}
};

static void lx_hc_port_power(struct device *dev, int is_on)
{
	pcon_set_usbhost_power(is_on);
}

static void lx_hc_reset(struct device *dev)
{
	gpio_set_value(GPIO_LX_USBH_RST, 0);
	ndelay(400);	/* nRST for 16+ clocks @ 48MHz */
	gpio_set_value(GPIO_LX_USBH_RST, 1);
}

static struct sl811_platform_data lx_sl811 = {
	.potpg		= 100,
	.port_power	= lx_hc_port_power,
	.reset		= lx_hc_reset,
};

static struct platform_device lx_device_usbc = {
	.name        = "sl811-hcd",
	.id          = -1,
	.dev         = {
		.platform_data     = &lx_sl811,
	},
	.resource      = sl811_resources,
	.num_resources = ARRAY_SIZE(sl811_resources)
};


static struct mtd_partition nand_partition_info[] = {
	{
		.name	= "LX nand 1",
		.offset	= 0x00400000,
		.size	= 0x03800000,
	}, {
		.name	= "LX nand 2",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	}
};

static void nand_adjust(struct gpio_nand_platdata *p, size_t size)
{
	if (size <= 32*1024*1024)
		p->parts[0].size -= 0x2000000;
}

static struct resource nand_resources[] = {
	{
		.start	= 0x04000000,
		.end	= 0x07ffffff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 0x08000000,
		.end	= 0x0fffffff,
		.flags	= IORESOURCE_MEM,
	}
};

static struct gpio_nand_platdata nand_platdata = {
	.gpio_cle = GPIO_LX_NAND_CLE,
	.gpio_ale = GPIO_LX_NAND_ALE,
	.gpio_nce = GPIO_LX_NAND_NCE,
	.gpio_nwp = GPIO_LX_NAND_NWP,
	.gpio_rdy = GPIO_LX_NAND_RDY,
	.adjust_parts = nand_adjust,
	.parts = nand_partition_info,
	.num_parts = ARRAY_SIZE(nand_partition_info),
	.chip_delay = 15, /* 15 us command delay time */
	.options = NAND_NO_AUTOINCR | NAND_BUSWIDTH_16,
};

static struct platform_device lx_device_nand = {
	.name		= "gpio-nand",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(nand_resources),
	.resource	= nand_resources,
	.dev = {
		.platform_data = &nand_platdata,
	},
};

static struct platform_device lx_device_pm = {
	.name		= "lx-pm",
	.id		= -1,
};

static struct platform_device *lx_devices[] __initdata = {
	&lx_device_pm,
	&lx_device_fb,
	&lx_device_usbc,
	&lx_device_nand,
};


static int lx_mci_init(struct device *dev, irqreturn_t (*fn)(int, void *), void *data)
{
	return request_irq(IRQ_GPIO(GPIO_LX_MMC_DET), fn,
			   IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
			   "pxamci", data);
}

static void lx_mci_setpower(struct device *dev, unsigned int vdd_bits)
{
	pcon_set_mmc_power(vdd_bits ? 1 : 0);
}

static void lx_mci_exit(struct device *dev, void *data)
{
	free_irq(IRQ_GPIO(9), data);
}

static struct pxamci_platform_data lx_mci_info = {
	.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34,
	.detect_delay_ms	= 50 * HZ / 1000,
	.init			= lx_mci_init,
	.setpower		= lx_mci_setpower,
	.exit			= lx_mci_exit,
	.gpio_card_detect	= -1,
	.gpio_card_ro		= -1,
	.gpio_power		= -1
};


static struct lx_eeprom_emu lx_eeprom = {
	.size	= I2C_EEPROM_EMU_SIZE,
	.watch	= LIST_HEAD_INIT(lx_eeprom.watch),
};

int lx_i2cslave_addwatcher(struct lx_i2cslave_watcher *watcher, void *data)
{
	struct lx_i2cslave_watch *watch;
	unsigned long flags;

	if (watcher->start + watcher->size > lx_eeprom.size)
		return -EINVAL;

	watch = kmalloc(sizeof(struct lx_i2cslave_watch), GFP_KERNEL);
	if (watch) {
		watch->start = watcher->start;
		watch->end = watcher->start + watcher->size - 1;
		watch->ops = watcher;
		watch->data = data;

		local_irq_save(flags);
		list_add(&watch->node, &lx_eeprom.watch);
		local_irq_restore(flags);
	}

	return watch ? 0 : -ENOMEM;
}
EXPORT_SYMBOL(lx_i2cslave_addwatcher);

void lx_i2cslave_delwatcher(struct lx_i2cslave_watcher *watcher, void *data)
{
	struct lx_i2cslave_watch *watch, *n;
	unsigned long flags;

	list_for_each_entry_safe(watch, n, &lx_eeprom.watch, node) {
		if (watch->ops == watcher && watch->data == data) {
			local_irq_save(flags);
			list_del(&watch->node);
			local_irq_restore(flags);
			kfree(watch);
		}
	}
}
EXPORT_SYMBOL(lx_i2cslave_delwatcher);

static void lx_eeprom_emu_event(void *ptr, i2c_slave_event_t event)
{
	struct lx_eeprom_emu *emu = ptr;
	emu->seen_start = event == I2C_SLAVE_EVENT_START_WRITE;
}

static int lx_eeprom_emu_read(void *ptr)
{
	struct lx_eeprom_emu *emu = ptr;
	int ret = emu->bytes[emu->ptr];
	emu->ptr = (emu->ptr + 1) % emu->size;
	return ret;
}

static void lx_eeprom_emu_write(void *ptr, unsigned int val)
{
	struct lx_eeprom_emu *emu = ptr;
	struct lx_i2cslave_watch *w;

	if (emu->seen_start != 0) {
		emu->ptr = val;
		emu->seen_start = 0;
		return;
	}

	emu->bytes[emu->ptr] = val;

	list_for_each_entry(w, &emu->watch, node) {
		if (!w->ops || !w->ops->write)
			continue;
		if (w->start <= emu->ptr && w->end >= emu->ptr)
			w->ops->write(w->data, emu->ptr, val);
	}

	emu->ptr = (emu->ptr + 1) % emu->size;
}

static struct i2c_slave_client lx_slave_client = {
	.data	= &lx_eeprom,
	.event	= lx_eeprom_emu_event,
	.read	= lx_eeprom_emu_read,
	.write	= lx_eeprom_emu_write,
};

static struct i2c_pxa_platform_data lx_i2c_info = {
	.slave_addr	= 0xe0 >> 1,
	.slave		= &lx_slave_client,
};

static struct i2c_board_info lx_i2c_devices[] __initdata = {
	{ I2C_BOARD_INFO("lxpcon", 0x13) },
	{ I2C_BOARD_INFO("rv5c386", 0x32) },
	{ I2C_BOARD_INFO("lxnvram", 0x53) },
};


static void lx_icp_mode(struct device *dev, int mode)
{
	gpio_set_value(GPIO_LX_IRDA_MODE, !!(mode & IR_FIRMODE));
	pxa2xx_transceiver_mode(dev, mode);
	gpio_set_value(GPIO_LX_IRDA_EN, !!(mode & IR_OFF));
}

static struct pxaficp_platform_data lx_icp_info = {
	.transceiver_cap	= IR_SIRMODE | IR_FIRMODE | IR_OFF,
	.transceiver_mode	= lx_icp_mode,
};

static int lx_audio_startup(struct snd_pcm_substream *substream, void *priv)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		gpio_direction_output(GPIO_LX_SPKR_OFF, 0);
	return 0;
}

static void lx_audio_shutdown(struct snd_pcm_substream *substream, void *priv)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		gpio_direction_output(GPIO_LX_SPKR_OFF, 1);
}

static pxa2xx_audio_ops_t lx_ac97_info = {
	.startup	= lx_audio_startup,
	.shutdown	= lx_audio_shutdown,
};

static struct gpio_default_state {
	int gpio;
	int value;
	const char *name;
} gpio_state[] = {
	{ GPIO_LX_USBH_RST,  0, "SL811 reset" },/* Hold SL811 in reset */
	{ GPIO_LX_IRDA_EN,   0, "IrDA off" },	/* IrDA transceiver off */
	{ GPIO_LX_IRDA_MODE, 0, "IrDA mode" },	/* SIR */
	{ GPIO_LX_NAND_NWP,  0, NULL },		/* NAND write protected */
	{ GPIO_LX_NAND_NCE,  1, NULL },		/* NAND not selected */
	{ GPIO_LX_AC97_RST,  1, NULL },		/* Release AC97 reset */
	{ GPIO_LX_SPKR_OFF,  1, NULL }		/* Power down speaker */
};

static void __init lx_reserve(void)
{
	/*
	 * Seems to be occupied by BooSt bootloader. We need to preserve
	 * this region for suspend/resume to work correctly.
	 *
	 * XXX: What is the correct size to reserve? Original Psion sources
	 * tell about 4MB, but in reality reserve only 256kB.
	 */
	memblock_reserve(0xa0000000, 0x40000);
}

static void __init lx_init(void)
{
	int i;

	PCFR = PCFR_OPDE | PCFR_FS | PCFR_FP;

	pxa2xx_mfp_config(ARRAY_AND_SIZE(lx_pin_config));

	pxa_set_ffuart_info(NULL);
	pxa_set_stuart_info(NULL);

	/*
	 * Initialise the default GPIO values
	 */
	for (i = 0; i < ARRAY_SIZE(gpio_state); i++) {
		const char *name = gpio_state[i].name;
		if (!name)
			name = "[init]";
		gpio_request(gpio_state[i].gpio, name);
		gpio_direction_output(gpio_state[i].gpio, gpio_state[i].value);
		if (!gpio_state[i].name)
			gpio_free(gpio_state[i].gpio);
	}

	pcon_set_serial_power(1);

	platform_add_devices(ARRAY_AND_SIZE(lx_devices));

	pxa_set_ac97_info(&lx_ac97_info);
	pxa_set_mci_info(&lx_mci_info);
	pxa_set_ficp_info(&lx_icp_info);
	pxa_set_i2c_info(&lx_i2c_info);
	i2c_register_board_info(0, ARRAY_AND_SIZE(lx_i2c_devices));

	pxa2xx_set_spi_info(1, &lx_ssp_master_info);
	spi_register_board_info(lx_spi_board_info, ARRAY_SIZE(lx_spi_board_info));

	pm_power_off = lx_power_off;
	arm_pm_restart = lx_restart;
}

MACHINE_START(NETBOOKPRO, "Psion Teklogix NetBook Pro")
	/* Maintainer: Ben Dooks, Vincent Sanders. Simtec Electronics <linux@simtec.co.uk> */
	.boot_params	= 0xa0200100,
	.map_io		= pxa25x_map_io,
	.reserve	= lx_reserve,
	.init_irq	= pxa25x_init_irq,
	.timer		= &pxa_timer,
	.init_machine	= lx_init,
MACHINE_END
