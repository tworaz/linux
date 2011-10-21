/*
 * drivers/power/lx_power.c
 *
 * Copytight (c) 2011 Peter Tworek
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <mach/lx-pcon.h>

struct lx_pwr_dev {
	struct device		*dev;

	struct mutex		lock;
	wait_queue_head_t	wait_q;
	pcon_pwrstate_t		state;
	int			state_changed:1;
};

struct lx_pwr_dev *cur_lx_pwr;

static const char *lx_pwr_decodestate(pcon_pwrstate_t state)
{
	switch (state) {
	case PCON_PWRSTATE_HWReset:
		return "Hardware-Reset";

	case PCON_PWRSTATE_SWReset:
		return "Software-Reset";

	case PCON_PWRSTATE_Off:
		return "Off";

	case PCON_PWRSTATE_Suspend:
		return "Suspend";

	case PCON_PWRSTATE_Sleep:
		return "Sleep";

	case PCON_PWRSTATE_Booting:
		return "Booting";

	case PCON_PWRSTATE_LoadingOS:
		return "Loading-OS";

	case PCON_PWRSTATE_Idle:
		return "Idle";

	case PCON_PWRSTATE_Run:
		return "Run";

	case PCON_PWRSTATE_Comatose:
		return "Comatose";

	case PCON_PWRSTATE_PowerFail:
		return "Power-Fail";

	default:
		return "Unknown-Power-State";
	}
}

int lx_pwr_setstate(pcon_pwrstate_t state)
{
	int ret;

	if (cur_lx_pwr == NULL) {
		return -ENOSYS;
	}

	mutex_lock(&cur_lx_pwr->lock);
	cur_lx_pwr->state_changed = 0;
	mutex_unlock(&cur_lx_pwr->lock);

	ret = pcon_pwr_setstate(state);
	if (ret) {
		dev_err(cur_lx_pwr->dev, "failed to initiate pcon power state change!");
		return ret;
	}

	ret = wait_event_interruptible_timeout(cur_lx_pwr->wait_q,
	                                       cur_lx_pwr->state_changed, 2*HZ);
	if (ret <= 0) {
		dev_err(cur_lx_pwr->dev, "failed to change PCON power state to %s!\n",
		        lx_pwr_decodestate(state));
		return 1;
	}

	return 0;
}
EXPORT_SYMBOL(lx_pwr_setstate);

pcon_pwrstate_t lx_pwr_getstate(void)
{
	pcon_pwrstate_t ret;

	if (cur_lx_pwr == NULL) {
		return PCON_PWRSTATE_Booting;
	}

	mutex_lock(&cur_lx_pwr->lock);
	ret = cur_lx_pwr->state;
	mutex_unlock(&cur_lx_pwr->lock);

	return ret;
}
EXPORT_SYMBOL(lx_pwr_getstate);

static void lx_pwr_write(void *parm, unsigned int addr, unsigned char val)
{
	struct lx_pwr_dev *data = parm;

	pr_info("PCon power state changed to: %s\n", lx_pwr_decodestate(val));

	mutex_lock(&data->lock);
	data->state = val;
	data->state_changed = 1;
	mutex_unlock(&data->lock);

	wake_up_interruptible(&data->wait_q);
}

static struct lx_i2cslave_watcher lx_pwr_watcher = {
	.start	= PHST_POWER_START,
	.size	= PHST_POWER_SIZE,
	.write	= lx_pwr_write,
};

static int lx_pwr_probe(struct platform_device *pdev)
{
	struct lx_pwr_dev *data;

	data = kzalloc(sizeof(struct lx_pwr_dev), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&pdev->dev, "no memory for battery data\n");
		return -ENOMEM;
	}

	data->dev = &pdev->dev;
	data->state = PCON_PWRSTATE_Run; /* Set in lx-pcon initialization. */
	init_waitqueue_head(&data->wait_q);
	mutex_init(&data->lock);
	lx_i2cslave_addwatcher(&lx_pwr_watcher, data);

	platform_set_drvdata(pdev, data);

	cur_lx_pwr = data;

	pr_info("LX: Power driver\n");

	return 0;
}

static int __devexit lx_pwr_remove(struct platform_device *pdev)
{
	struct lx_pwr_dev *data = platform_get_drvdata(pdev);

	lx_i2cslave_delwatcher(&lx_pwr_watcher, data);
	platform_set_drvdata(pdev, NULL);
	mutex_destroy(&data->lock);

	cur_lx_pwr = NULL;
	kfree(data);

	return 0;
}

static int lx_pwr_suspend(struct platform_device *pdev, pm_message_t state)
{
	return lx_pwr_setstate(PCON_PWRSTATE_Suspend);
}

static int lx_pwr_resume(struct platform_device *pdev)
{
	return lx_pwr_setstate(PCON_PWRSTATE_Run);
}

static struct platform_driver __refdata lx_pwr = {
	.driver = {
		.name	= "lx-pwr",
		.owner	= THIS_MODULE,
	},
	.probe		= lx_pwr_probe,
	.remove		= __devexit_p(lx_pwr_remove),
	.suspend	= lx_pwr_suspend,
	.resume		= lx_pwr_resume,
};

static int __init lx_pwr_init(void)
{
	return platform_driver_register(&lx_pwr);
}

static void __exit lx_pwr_exit(void)
{
	platform_driver_unregister(&lx_pwr);
}

module_init(lx_pwr_init);
module_exit(lx_pwr_exit);

MODULE_AUTHOR("Peter Tworek <tworaz666@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LX Power driver");
MODULE_ALIAS("platform:lx-pwr");
