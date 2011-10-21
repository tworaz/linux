/*
 *  linux/arch/arm/mach-pxa/lx_pm.c
 *
 *  Power Management code for Psion Teklogix NetBook Pro.
 *
 *  Copyright (c) 2011 Peter Tworek
 *
 *  Based on original Psion 2.6.9 kernel code:
 *    Author:     Ben Dooks
 *    Copyright   (c) 2004 Simtec Electronics
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/pxa2xx-regs.h>
#include <mach/pm.h>
#include <mach/lx.h>

#ifdef CONFIG_PM
static int lx_pxa_pm_enter(suspend_state_t state)
{
	void *sleep_save = phys_to_virt(0xa0000000);

	writel(virt_to_phys(cpu_resume), sleep_save);
	writel(3, sleep_save + 0xfc);

	return pxa_pm_enter(state);
}

static const struct platform_suspend_ops lx_pm_ops = {
	.prepare	= pxa_pm_prepare,
	.finish		= pxa_pm_finish,
	.enter		= lx_pxa_pm_enter,
	.valid		= suspend_valid_only_mem,
};
#endif

static irqreturn_t lx_pm_irq(int irqno, void *data)
{
	pr_err("LX: Suspend Request IRQ\n");

	return IRQ_HANDLED;
}

static int __devinit lx_pm_probe(struct platform_device *pdev)
{
	int ret;
#if 0
	ret = request_irq(IRQ_GPIO(GPIO_LX_BATT_LOW_IRQ), lx_pm_irq,
	                  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	                  "Battery Low", NULL);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request battery low IRQ: %d\n", ret);
	}
#endif
	ret = request_irq(IRQ_GPIO(GPIO_LX_SUSP_REQ_IRQ), lx_pm_irq,
	                  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	                  "Suspend Request", NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to register suspend request IRQ: %d\n", ret);
	}

#ifdef CONFIG_PM
	suspend_set_ops(&lx_pm_ops);
#endif

	return 0;
}

static int lx_pm_remove(struct platform_device *pdev)
{
#if 0
	free_irq(IRQ_GPIO(GPIO_LX_BATT_LOW_IRQ), NULL);
#endif
	free_irq(IRQ_GPIO(GPIO_LX_SUSP_REQ_IRQ), NULL);

	return 0;
}

static struct platform_driver lx_pm_driver = {
	.driver		= {
		.name		= "lx-pm",
	},
	.probe		= lx_pm_probe,
	.remove		= lx_pm_remove,

};

static int __devinit lx_pm_init(void)
{
	return platform_driver_register(&lx_pm_driver);
}

static void lx_pm_exit(void)
{
	platform_driver_unregister(&lx_pm_driver);
}

late_initcall(lx_pm_init);
module_exit(lx_pm_exit);
