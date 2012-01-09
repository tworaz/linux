/*
 *	 linux/arch/arm/mach-bcm2708/dummy_touchscreen.c
 *
 *	 Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device provides a mechanism for controlling the touchscreen of VideoCore
 * subsystems.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/io.h>
#include <mach/vcio.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#define INPUT_ABS_X_MIN	0
#define INPUT_ABS_X_MAX	1023
#define INPUT_ABS_Y_MIN	0
#define INPUT_ABS_Y_MAX	1023

static char phys[32];
static void *buttonsbits;	/* written by VC */

/*
 * Timer function which is run every poll_ms ms when the device is opened.
 * The dev input variable is set to the the input_dev pointer.
 */
static void bcm_touchscreen_poll(struct input_polled_dev *dev)
{
	struct input_dev *input = dev->input;
	unsigned touched, absx, absy;
	unsigned b = readl(buttonsbits);
	touched = (b >> 22) & 1;
	absx = (b >> 0) & INPUT_ABS_X_MAX;
	absy = (b >> 10) & INPUT_ABS_Y_MAX;

	input_report_key(input, BTN_LEFT, b & (1 << 20)); /* a */
	input_report_key(input, BTN_RIGHT, b & (1 << 21)); /* b */

	if (touched) {
		input_report_key(input, BTN_TOUCH, 1);
		input_report_abs(input, ABS_X, absx);
		input_report_abs(input, ABS_Y, absy);
	} else {
		input_report_key(input, BTN_TOUCH, 0);
	}
	input_sync(input);
}

static int __devinit gpio_mouse_probe(struct platform_device *pdev)
{
	struct input_polled_dev *input_poll;
	struct input_dev *input;
	dma_addr_t dma = 0;
	int error = 0;
	int poll_ms = 10;
	void *mem;
	input_poll = input_allocate_polled_device();
	if (!input_poll) {
		printk(KERN_ERR  ": not enough memory for input device\n");
		error = -ENOMEM;
		goto out_free_polldev;
	}

	mem = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(sizeof(unsigned)), &dma,
				    GFP_KERNEL);
	if (NULL == mem) {
		printk(KERN_ERR ": unable to allocate buttonsbits buffers\n");
		error = -ENOMEM;
		goto out_free_polldev;
	} else {
		buttonsbits = mem;
		bcm_mailbox_write(MBOX_CHAN_TOUCH, dma);
		printk(KERN_ERR
		       ": registered buttonsbits @%08x through MBOX %p\n", dma,
		       __io_address(ARM_0_MAIL1_WRT));
	}

	platform_set_drvdata(pdev, input_poll);

	/* set input-polldev handlers */
	input_poll->poll = bcm_touchscreen_poll;
	input_poll->poll_interval = poll_ms;

	input = input_poll->input;
	input->name = pdev->name;
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

	input_set_capability(input, EV_ABS, ABS_X);
	input_set_capability(input, EV_ABS, ABS_Y);
	input_set_abs_params(input,
				ABS_X, INPUT_ABS_X_MIN, INPUT_ABS_X_MAX, 0, 0);
	input_set_abs_params(input,
				ABS_Y, INPUT_ABS_Y_MIN, INPUT_ABS_Y_MAX, 0, 0);

	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_capability(input, EV_KEY, BTN_LEFT);
	input_set_capability(input, EV_KEY, BTN_RIGHT);
	snprintf(phys, sizeof(phys), "%s/input1", dev_name(&pdev->dev));
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

static int __devexit gpio_mouse_remove(struct platform_device *pdev)
{
	struct input_polled_dev *input = platform_get_drvdata(pdev);

	input_unregister_polled_device(input);
	input_free_polled_device(input);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver gpio_mouse_device_driver = {
	.probe = gpio_mouse_probe,
	.remove = __devexit_p(gpio_mouse_remove),
	.driver = {
		   .name = "bcm2708_vctouch",
		   .owner = THIS_MODULE,
		   }
};

static int __init gpio_mouse_init(void)
{
	return platform_driver_register(&gpio_mouse_device_driver);
}

module_init(gpio_mouse_init);

static void __exit gpio_mouse_exit(void)
{
	platform_driver_unregister(&gpio_mouse_device_driver);
}

module_exit(gpio_mouse_exit);

MODULE_AUTHOR("Dom Cobley");
MODULE_DESCRIPTION("Broadcom dummy touchscreen driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:INPUT");	/* work with hotplug and coldplug */
