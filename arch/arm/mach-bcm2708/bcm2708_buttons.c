/*
 *	 linux/arch/arm/mach-bcm2708/dummy_buttons.c
 *
 *	 Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device provides a mechanism for controlling the buttons of VideoCore
 * subsystems.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/vcio.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

static char phys[32];
static void *buttonsbits;	/* written by VC */

/*
 * Timer function which is run every poll_ms ms when the device is opened.
 * The dev input variable is set to the the input_dev pointer.
 */
static void bcm_buttons_poll(struct input_polled_dev *dev)
{
	struct input_dev *input = dev->input;
	unsigned b = readl(buttonsbits);
	input_report_key(input, BTN_0, b & (1 << 0));	/*a */
	input_report_key(input, BTN_1, b & (1 << 1));	/*b */
	input_report_key(input, BTN_2, b & (1 << 2));	/*c */
	input_report_key(input, BTN_3, b & (1 << 3));	/*d */
	input_report_key(input, BTN_4, b & (1 << 4));	/*action */

	input_report_key(input, BTN_5, b & (1 << 5));	/*left */
	input_report_key(input, BTN_6, b & (1 << 6));	/*right */
	input_report_key(input, BTN_7, b & (1 << 7));	/*down */
	input_report_key(input, BTN_8, b & (1 << 8));	/*up */
	input_report_key(input, BTN_9, b & (1 << 9));	/*spare */
	input_sync(input);
}

static int __devinit bcm_buttons_probe(struct platform_device *pdev)
{
	struct input_polled_dev *input_poll;
	struct input_dev *input;
	dma_addr_t dma = 0;
	void *mem;
	int error = 0;
	int poll_ms = 10;
	input_poll = input_allocate_polled_device();
	if (!input_poll) {
		printk(KERN_ERR ": not enough memory for input device\n");
		error = -ENOMEM;
		goto out_free_polldev;
	}
	mem = dma_alloc_coherent(&pdev->dev,
		PAGE_ALIGN(sizeof(unsigned)), &dma, GFP_KERNEL);

	if (NULL == mem) {
		printk(KERN_ERR ": unable to allocate buttonsbits buffers\n");
		error = -ENOMEM;
		goto out_free_polldev;
	} else {
		buttonsbits = mem;
		/* send the DMA address for the circular buffers to the coms
		 * processor through the VideoCore I/O mailbox
		 */
		bcm_mailbox_write(MBOX_CHAN_BUTTONS, dma);
		printk(KERN_ERR
			": registered virtual buttonsbits @%08x via MBOX %p\n",
			dma, __io_address(ARM_0_MAIL1_WRT));
	}
	platform_set_drvdata(pdev, input_poll);

	/* set input-polldev handlers */
	/*input_poll->private = pdata; */
	input_poll->poll = bcm_buttons_poll;
	input_poll->poll_interval = poll_ms;

	input = input_poll->input;
	input->name = pdev->name;
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

	input_set_capability(input, EV_KEY, BTN_0);
	input_set_capability(input, EV_KEY, BTN_1);
	input_set_capability(input, EV_KEY, BTN_2);
	input_set_capability(input, EV_KEY, BTN_3);
	input_set_capability(input, EV_KEY, BTN_4);
	input_set_capability(input, EV_KEY, BTN_5);
	input_set_capability(input, EV_KEY, BTN_6);
	input_set_capability(input, EV_KEY, BTN_7);
	input_set_capability(input, EV_KEY, BTN_8);
	input_set_capability(input, EV_KEY, BTN_9);
	snprintf(phys, sizeof(phys), "%s/input0", dev_name(&pdev->dev));
	input->phys = phys;

	error = input_register_polled_device(input_poll);
	if (error) {
		printk(KERN_ERR ": could not register input device\n");
		error = -ENOMEM;
		goto out_free_polldev;
	}
	return 0;

out_free_polldev:
	input_free_polled_device(input_poll);
	platform_set_drvdata(pdev, NULL);
	return error;
}

static int __devexit bcm_buttons_remove(struct platform_device *pdev)
{
	struct input_polled_dev *input = platform_get_drvdata(pdev);
	/*struct bcm_buttons_platform_data *pdata = input->private; */

	input_unregister_polled_device(input);
	input_free_polled_device(input);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver bcm_buttons_device_driver = {
	.probe = bcm_buttons_probe,
	.remove = __devexit_p(bcm_buttons_remove),
	.driver = {
			.name = "bcm2708_vcbuttons",
			.owner = THIS_MODULE,
		  }
};

static int __init bcm_buttons_init(void)
{
	return platform_driver_register(&bcm_buttons_device_driver);
}

module_init(bcm_buttons_init);

static void __exit bcm_buttons_exit(void)
{
	platform_driver_unregister(&bcm_buttons_device_driver);
}

module_exit(bcm_buttons_exit);

MODULE_AUTHOR("Dom Cobley");
MODULE_DESCRIPTION("Broadcom BCM2708 buttons driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:INPUT");	/* work with hotplug and coldplug */
