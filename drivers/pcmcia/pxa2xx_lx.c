/*
 * linux/drivers/pcmica/pxa2xx_lx.c
 *
 * Based on: linux/drivers/pcmcia/pxa2xx_mainstone.c
 *
 * NetBookPro PCMCIA specific routines.
 *
 * Modifications for LX by:
 * Author:      Ben Dooks
 * Modified:    Jun 08, 2004
 * Copyright:   Simtec Electronics <linux@xxxxxxxxxxxx>
 *
 * Original:
 * Created:	May 12, 2004
 * Author:	Nicolas Pitre
 * Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <pcmcia/ss.h>

#include <asm/mach-types.h>
#include <mach/lx.h>

#include "soc_common.h"

/* GPIO Lines
 * Socket:         0              1
 *   CardDetect    75             73
 *   StatusChange  76             72
 *   Ready/IRQ     77             71
 *   Reset         74             70
 */
struct our_socket {
	unsigned int detect;	/* gpio for detect */
	unsigned int status;	/* gpio for status change */
	unsigned int ready;	/* gpio for ready/irq */
	unsigned int reset;	/* gpio for card reset */
};

static struct our_socket sockets[2] = {
	[0] = {
		.detect	= GPIO_LX_CF_DETECT,
		.status	= GPIO_LX_CF_STATUS,
		.ready	= GPIO_LX_CF_RDY,
		.reset	= GPIO_LX_CF_RST,
	},
	[1] = {
		.detect	= GPIO_LX_PCMCIA_DETECT,
		.status	= GPIO_LX_PCMCIA_STATUS,
		.ready	= GPIO_LX_PCMCIA_RDY,
		.reset	= GPIO_LX_PCMCIA_RST,
	},
};

static struct pcmcia_irqs irqs[] = {
	{ 0, IRQ_GPIO(GPIO_LX_CF_DETECT), "CF Change Detect" },
	{ 0, IRQ_GPIO(GPIO_LX_CF_STATUS), "CF Status Change" },
	{ 1, IRQ_GPIO(GPIO_LX_PCMCIA_DETECT), "PCMCIA Change Detect" },
	{ 1, IRQ_GPIO(GPIO_LX_PCMCIA_STATUS), "PCMCIA Status Change" },
};

static int lx_req_gpio(int gpio, const char *name, int dir, int val)
{
	int err = gpio_request(gpio, name);
	if (err == 0) {
		if (dir)
			err = gpio_direction_output(gpio, val);
		else
			err = gpio_direction_input(gpio);
		if (err)
			gpio_free(gpio);
	}
	return err;
}

static int lx_pcmcia_hw_init(struct soc_pcmcia_socket *skt)
{
	struct our_socket *oursock = &sockets[skt->nr];
	int err;

	err = lx_req_gpio(oursock->detect, "PCMCIA detect", 0, 0);
	if (err)
		goto err_det;
	err = lx_req_gpio(oursock->ready, "PCMCIA IRQ", 0, 0);
	if (err)
		goto err_rdy;
	err = lx_req_gpio(oursock->reset, "PCMCIA reset", 1, 1);
	if (err)
		goto err_rst;
	err = lx_req_gpio(oursock->status, "PCMCIA status", 0, 0);
	if (err)
		goto err_stat;

	skt->socket.pci_irq = gpio_to_irq(oursock->ready);
	return soc_pcmcia_request_irqs(skt, irqs, ARRAY_SIZE(irqs));

 err_stat:
	gpio_free(oursock->reset);
 err_rst:
	gpio_free(oursock->ready);
 err_rdy:
	gpio_free(oursock->detect);
 err_det:
	return err;
}

static void lx_pcmcia_hw_shutdown(struct soc_pcmcia_socket *skt)
{
	struct our_socket *oursock = &sockets[skt->nr];
	soc_pcmcia_free_irqs(skt, irqs, ARRAY_SIZE(irqs));
	gpio_free(oursock->status);
	gpio_free(oursock->detect);
	gpio_free(oursock->ready);
	gpio_free(oursock->reset);
}

static void lx_pcmcia_socket_state(struct soc_pcmcia_socket *skt,
				    struct pcmcia_state *state)
{
	struct our_socket *oursock = &sockets[skt->nr];

	state->detect = gpio_get_value(oursock->detect) ? 0 : 1;
	state->ready  = gpio_get_value(oursock->ready) ? 1 : 0;

	/* none of these are available on this implementation */
	state->bvd1   = 1;
	state->bvd2   = 1;
	state->vs_3v  = 1;
	state->vs_Xv  = 0;
	state->wrprot = 0;

	pr_debug("skt[%d]: detect=%d, ready=%d\n",
	    skt->nr, state->detect, state->ready);
}

static int lx_pcmcia_configure_socket(struct soc_pcmcia_socket *skt,
				       const socket_state_t *state)
{
	struct our_socket *oursock = &sockets[skt->nr];
	int ret = 0;

	pr_debug("skt[%d]: configure\n", skt->nr);

	/* we currently haven't got the code sorted out to
	 * send the PCon chip any configuration info for the
	 * socket
	 */
	if (state->flags & SS_RESET) {
		/* assert reset for socket */
		pr_debug("skt[%d]: reset on\n", skt->nr);
		gpio_set_value(oursock->reset, 0);
	} else {
		/* de-assert reset */

		pr_debug("skt[%d]: reset off\n", skt->nr);
		gpio_set_value(oursock->reset, 1);
	}

	return ret;
}

static void lx_pcmcia_socket_init(struct soc_pcmcia_socket *skt)
{
	soc_pcmcia_enable_irqs(skt, irqs, ARRAY_SIZE(irqs));
}

static void lx_pcmcia_socket_suspend(struct soc_pcmcia_socket *skt)
{
	soc_pcmcia_disable_irqs(skt, irqs, ARRAY_SIZE(irqs));
}

static struct pcmcia_low_level lx_pcmcia_ops = {
	.owner			= THIS_MODULE,
	.hw_init		= lx_pcmcia_hw_init,
	.hw_shutdown		= lx_pcmcia_hw_shutdown,
	.socket_state		= lx_pcmcia_socket_state,
	.configure_socket	= lx_pcmcia_configure_socket,
	.socket_init		= lx_pcmcia_socket_init,
	.socket_suspend		= lx_pcmcia_socket_suspend,
	.nr			= 2,
};

static struct platform_device *lx_pcmcia_device;

static int __init lx_pcmcia_init(void)
{
	int ret;

	if (!machine_is_netbookpro())
		return -ENODEV;

	printk(KERN_INFO "LX: installing PCMCIA driver device\n");

	lx_pcmcia_device = platform_device_alloc("pxa2xx-pcmcia", -1);
	if (!lx_pcmcia_device)
		return -ENOMEM;

	ret = platform_device_add_data(lx_pcmcia_device, &lx_pcmcia_ops,
				       sizeof(lx_pcmcia_ops));
	if (ret == 0)
		ret = platform_device_add(lx_pcmcia_device);

	if (ret)
		platform_device_put(lx_pcmcia_device);

	return ret;
}

static void __exit lx_pcmcia_exit(void)
{
	platform_device_unregister(lx_pcmcia_device);
}

module_init(lx_pcmcia_init);
module_exit(lx_pcmcia_exit);

MODULE_LICENSE("GPL");
