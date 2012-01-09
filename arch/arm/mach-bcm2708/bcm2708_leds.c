/*
 *	 linux/arch/arm/mach-bcm2708/power.c
 *
 *	 Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device provides a mechanism for controlling the LEDS of VideoCore
 * subsystems.
 */

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <linux/input.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <mach/vcio.h>
#include <linux/dma-mapping.h>

static void *ledbits;	/* read by VC */

static void dummy_led_set(unsigned bit, int on)
{
	if (on)
		writel(readl(ledbits) | (1 << bit) , ledbits);
	else
		writel(readl(ledbits) & ~(1 << bit) , ledbits);
}

static unsigned dummy_led_get(int bit)
{
	return readl(ledbits) & (1 << bit);
}

static void dummy_led_set0(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(0, brightness);
}

static void dummy_led_set1(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(1, brightness);
}

static void dummy_led_set2(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(2, brightness);
}

static void dummy_led_set3(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(3, brightness);
}

static void dummy_led_set4(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(4, brightness);
}

static void dummy_led_set5(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(5, brightness);
}

static void dummy_led_set6(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(6, brightness);
}

static void dummy_led_set7(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(7, brightness);
}

static void dummy_led_set8(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(8, brightness);
}

static void dummy_led_set9(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	dummy_led_set(9, brightness);
}

static enum led_brightness dummy_led_get0(struct led_classdev *cdev)
{
	return dummy_led_get(0);
}

static enum led_brightness dummy_led_get1(struct led_classdev *cdev)
{
	return dummy_led_get(1);
}

static enum led_brightness dummy_led_get2(struct led_classdev *cdev)
{
	return dummy_led_get(2);
}

static enum led_brightness dummy_led_get3(struct led_classdev *cdev)
{
	return dummy_led_get(3);
}

static enum led_brightness dummy_led_get4(struct led_classdev *cdev)
{
	return dummy_led_get(4);
}

static enum led_brightness dummy_led_get5(struct led_classdev *cdev)
{
	return dummy_led_get(5);
}

static enum led_brightness dummy_led_get6(struct led_classdev *cdev)
{
	return dummy_led_get(6);
}

static enum led_brightness dummy_led_get7(struct led_classdev *cdev)
{
	return dummy_led_get(7);
}

static enum led_brightness dummy_led_get8(struct led_classdev *cdev)
{
	return dummy_led_get(8);
}

static enum led_brightness dummy_led_get9(struct led_classdev *cdev)
{
	return dummy_led_get(9);
}

static struct led_classdev dummy_uled[] = {
	{
	 .name = "led0",
	 .brightness_set = dummy_led_set0,
	 .brightness_get = dummy_led_get0,
	 .default_trigger = "mmc0",
	 },
	{
	 .name = "led1",
	 .brightness_set = dummy_led_set4,
	 .brightness_get = dummy_led_get4,
	 },
	{
	 .name = "led2",
	 .brightness_set = dummy_led_set1,
	 .brightness_get = dummy_led_get1,
	 .default_trigger = "cpu",
	 },
	{
	 .name = "led3",
	 .brightness_set = dummy_led_set3,
	 .brightness_get = dummy_led_get3,
	 },
	{
	 .name = "led4",
	 .brightness_set = dummy_led_set2,
	 .brightness_get = dummy_led_get2,
	 },
	{
	 .name = "led5",
	 .brightness_set = dummy_led_set9,
	 .brightness_get = dummy_led_get9,
	 },
	{
	 .name = "led6",
	 .brightness_set = dummy_led_set8,
	 .brightness_get = dummy_led_get8,
	 },
	{
	 .name = "led7",
	 .brightness_set = dummy_led_set7,
	 .brightness_get = dummy_led_get7,
	 },
	{
	 .name = "led8",
	 .brightness_set = dummy_led_set6,
	 .brightness_get = dummy_led_get6,
	 },
	{
	 .name = "led9",
	 .brightness_set = dummy_led_set5,
	 .brightness_get = dummy_led_get5,
	 .default_trigger = "heartbeat",
	 },
};

static int __init dummy_led_init(void)
{
	int ret = 0;
	dma_addr_t dma = 0;
	void *mem;

	mem =
	    dma_alloc_coherent(NULL, PAGE_ALIGN(sizeof(unsigned)), &dma,
			       GFP_KERNEL);
	if (NULL == mem) {
		printk(KERN_ERR ": unable to allocate ledbits buffers\n");
		ret = -ENOMEM;
	} else {
		ledbits = mem;
		bcm_mailbox_write(MBOX_CHAN_LEDS, dma);
		printk(KERN_ERR
		       ": registered ledbits @%08x through MBOX %p\n", dma,
		       __io_address(ARM_0_MAIL1_WRT));
	}

	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 0);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 1);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 2);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 3);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 4);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 5);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 6);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 7);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 8);
	if (!ret)
		ret = led_classdev_register(NULL, dummy_uled + 9);

	return ret;
}

static void __exit dummy_led_exit(void)
{
	led_classdev_unregister(dummy_uled + 0);
	led_classdev_unregister(dummy_uled + 1);
	led_classdev_unregister(dummy_uled + 2);
	led_classdev_unregister(dummy_uled + 3);
	led_classdev_unregister(dummy_uled + 4);
	led_classdev_unregister(dummy_uled + 5);
	led_classdev_unregister(dummy_uled + 6);
	led_classdev_unregister(dummy_uled + 7);
	led_classdev_unregister(dummy_uled + 8);
	led_classdev_unregister(dummy_uled + 9);
}

module_init(dummy_led_init);
module_exit(dummy_led_exit);

MODULE_ALIAS("platform:dummy-led");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("User LED support BCM2708");
MODULE_AUTHOR("Dom Cobley");
