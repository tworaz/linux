/*
 *  linux/drivers/power/lx_battery.c
 *
 *  Battery driver for Psion Teklogix NetBook Pro
 *  Copyright (C) 2011 Peter Tworek
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/power_supply.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>

#include <linux/spi/spi.h>

#include <mach/lx-pcon.h>

#define LX_PCON_BATTINFO_MAGIC		(0xADB0)
#define LX_PCON_BATTINFO_MAGIC_SWAPPED	(0xB0AD)

/* This structure should be 32bytes long. */
struct power_state {
	u16	flags;
	u16	unused[4];
	u16	max_capacity;
	u16	temperature;
	u16	charge_state;
	u16	main_batt_voltage;
	u16	backup_batt_voltage;
	u8	main_batt_capacity_percent;
	u8	backup_batt_capacity_percent;
	u16	charge_current;
	u16	discharge_current;
	u16	pad[2];
	u16	magic;
} __attribute__((__packed__));

enum {
	POWER_FLAG_BatteryPresent	= 0x0001,
	POWER_FLAG_ACPresent		= 0x0002,
	POWER_FLAG_ChargeEnabled	= 0x0004,
	POWER_FLAG_BackupBatteryPresent	= 0x0008,
	POWER_FLAG_Calibrated		= 0x0010,
	POWER_FLAG_Calibrating		= 0x0020
};

enum {
	CHARGE_STATE_NotCharging	= 0,
	CHARGE_STATE_Charging		= 1,
	CHARGE_STATE_QuickChargeComplete= 2,
	CHARGE_STATE_ChargeComplete	= 3
};


struct driver_data {
	struct spi_device 	*spi;

	struct spi_transfer	spi_xfer;
	struct spi_message 	spi_rx_msg;
	struct power_state	spi_rxbuf;

	struct task_struct 	*thread;
	struct power_state	pwr_state;
	struct mutex		pwr_state_lock;

	struct power_supply 	wall;
	struct power_supply 	main_battery;
	struct power_supply	backup_battery;
	char			wall_name[20];
	char			main_battery_name[20];
	char			backup_battery_name[20];
	int			ps_registered:1;
};

static void lx_battery_get_status(struct power_state *state, int *status)
{
	if (state->charge_state == CHARGE_STATE_NotCharging &&
	    state->flags & POWER_FLAG_ACPresent) {
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else if (state->charge_state == CHARGE_STATE_NotCharging) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (state->charge_state == CHARGE_STATE_Charging) {
		*status = POWER_SUPPLY_STATUS_CHARGING;
	} else if (state->charge_state == CHARGE_STATE_ChargeComplete ||
	           state->charge_state == CHARGE_STATE_QuickChargeComplete) {
		*status = POWER_SUPPLY_STATUS_FULL;
	} else {
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

static void lx_battery_get_current(const struct power_state *state, int *uA)
{
	if (state->charge_state == CHARGE_STATE_Charging) {
		*uA = state->charge_current * 1000;
	} else {
		*uA = -state->discharge_current * 1000;
	}
}

static void lx_battery_time_to_empty(const struct power_state *state, int *sec)
{
	if (state->charge_state == CHARGE_STATE_Charging ||
	    state->discharge_current == 0) {
		*sec = 0;
	} else {
		*sec = state->max_capacity *
			state->main_batt_capacity_percent * 36 /
			state->discharge_current;
	}
}

static void lx_battery_time_to_full(const struct power_state *state, int *sec)
{
	if (state->charge_state == CHARGE_STATE_Charging && state->charge_current) {
		*sec = state->max_capacity *
			(100 - state->main_batt_capacity_percent) * 36 /
			state->charge_current;
	} else {
		*sec = 0;
	}

}

static int lx_wall_get_prop(struct power_supply *psy,
                            enum power_supply_property psp,
                            union power_supply_propval *val)
{
	struct driver_data *bdata = dev_get_drvdata(psy->dev->parent);
	int ret = 0;

	mutex_lock(&bdata->pwr_state_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (bdata->pwr_state.flags & POWER_FLAG_ACPresent);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&bdata->pwr_state_lock);

	return ret;
}

static enum power_supply_property lx_wall_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int lx_main_battery_get_prop(struct power_supply *psy,
			            enum power_supply_property psp,
			            union power_supply_propval *val)
{
	struct driver_data *bdata = dev_get_drvdata(psy->dev->parent);
	struct power_state *state = &bdata->pwr_state;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = (state->flags & POWER_FLAG_BatteryPresent);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		lx_battery_get_status(state, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = state->main_batt_capacity_percent;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = (state->temperature - 273) * 10;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		lx_battery_get_current(state, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		lx_battery_time_to_empty(state, &val->intval);
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		lx_battery_time_to_full(state, &val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = (state->main_batt_voltage * 1000);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property lx_main_battery_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int lx_backup_battery_get_prop(struct power_supply *psy,
			              enum power_supply_property psp,
			              union power_supply_propval *val)
{
	struct driver_data *bdata = dev_get_drvdata(psy->dev->parent);
	struct power_state *state = &bdata->pwr_state;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(state->flags & POWER_FLAG_BackupBatteryPresent);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = state->backup_batt_capacity_percent;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = (state->backup_batt_voltage * 1000);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property lx_backup_battery_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int lx_battery_ps_register(struct driver_data *data)
{
	int ret = 0;

	ret = power_supply_register(&data->spi->dev, &data->wall);
	if (ret)
		return 1;

	ret = power_supply_register(&data->spi->dev, &data->main_battery);
	if (ret)
		goto err_wall;

	ret = power_supply_register(&data->spi->dev, &data->backup_battery);
	if (ret)
		goto err_main_battery;

	data->ps_registered = 1;
	return 0;

err_main_battery:
	power_supply_unregister(&data->main_battery);
err_wall:
	power_supply_unregister(&data->wall);

	return ret;
}

static int lx_battery_thread(void *param)
{
	struct driver_data *data = param;
	struct device *dev = &data->spi->dev;
	int ret, count, timeout, failcount = 0;
	u16 *src, *dst;
	u16 tmp, res;

	src = (u16 *)(&data->spi_rxbuf);
	dst = (u16 *)(&data->pwr_state);

	ret = pcon_monitorbattery(1, PCON_FLG_ASYNC);
	if (ret) {
		dev_err(dev, "Failed to turn on PCON battery monitor: %d\n", ret);
		return ret;
	}

	while (1) {
		timeout = 11 * HZ;

batt_thread_wait:
		if (failcount && (failcount % 10) == 0) {
			dev_err(dev, "Failed to refresh battery state %d times in a row!",
			       failcount);
		}

		ret = schedule_timeout_interruptible(timeout);

		if (kthread_should_stop())
			break;

		if (lx_pwr_getstate() != PCON_PWRSTATE_Run)
			continue;

		if (ret > 0) {
			timeout = ret;
			goto batt_thread_wait;
		}

		ret = spi_sync(data->spi, &data->spi_rx_msg);
		if (ret) {
			dev_err(dev, "Power state read failed!\n");
			failcount++;
			continue;
		}

		if (data->spi_rxbuf.magic != LX_PCON_BATTINFO_MAGIC_SWAPPED) {
			failcount++;
			timeout = 2*HZ;
			goto batt_thread_wait;
		}

		/* need to 16bit byteswap the data */
		mutex_lock(&data->pwr_state_lock);
		for (count = 0; count < sizeof(struct power_state)/sizeof(u16); count++) {
			tmp = src[count];
			res = tmp >> 8;
			res |= (tmp << 8);
			dst[count] = res;
		}
		mutex_unlock(&data->pwr_state_lock);

		failcount = 0;

		if (!data->ps_registered) {
			if (lx_battery_ps_register(data)) {
				dev_err(dev, "Failed to register power supply"
				        " sub-drivers!\n");
				break;
			}
		}
	}

	pcon_monitorbattery(0, PCON_FLG_ASYNC);
	return 0;
}

static int __devinit lx_battery_probe(struct spi_device *spi)
{
	struct driver_data *data;
	struct power_supply *backup_battery;
	struct power_supply *main_battery;
	struct power_supply *wall;
	int ret;

	data = kzalloc(sizeof(struct driver_data), GFP_KERNEL);
	if (data == NULL) {
		pr_err("lx-battery: no memory for battery data\n");
		return -ENOMEM;
	}

	wall = &data->wall;
	main_battery = &data->main_battery;
	backup_battery = &data->backup_battery;

	snprintf(data->wall_name, sizeof(data->wall_name), "lx-wall");
	snprintf(data->main_battery_name, sizeof(data->main_battery_name),
	         "lx-main-battery");
	snprintf(data->backup_battery_name, sizeof(data->backup_battery_name),
	         "lx-backup-battery");

	wall->name = data->wall_name;
	wall->type = POWER_SUPPLY_TYPE_MAINS;
	wall->properties = lx_wall_props;
	wall->num_properties = ARRAY_SIZE(lx_wall_props);
	wall->get_property = lx_wall_get_prop;

	main_battery->name = data->main_battery_name;
	main_battery->type = POWER_SUPPLY_TYPE_BATTERY;
	main_battery->properties = lx_main_battery_props;
	main_battery->num_properties = ARRAY_SIZE(lx_main_battery_props);
	main_battery->get_property = lx_main_battery_get_prop;
	main_battery->use_for_apm = 1;

	backup_battery->name = data->backup_battery_name;
	backup_battery->type = POWER_SUPPLY_TYPE_BATTERY;
	backup_battery->properties = lx_backup_battery_props;
	backup_battery->num_properties = ARRAY_SIZE(lx_backup_battery_props);
	backup_battery->get_property = lx_backup_battery_get_prop;
	backup_battery->use_for_apm = 1;

	data->thread = kthread_run(lx_battery_thread, data, "lx-battery");
	if (IS_ERR(data->thread)) {
		ret = PTR_ERR(data->thread);
		pr_err("lx-battery: cannot start thread: %d\n", ret);
		goto err_kzalloc;
	}

	mutex_init(&data->pwr_state_lock);

	data->spi_xfer.tx_buf = NULL;
	data->spi_xfer.rx_buf = &data->spi_rxbuf;
	data->spi_xfer.len = sizeof(struct power_state);
	data->spi_xfer.bits_per_word = 16;

	spi_message_init(&data->spi_rx_msg);
	spi_message_add_tail(&data->spi_xfer, &data->spi_rx_msg);

	data->spi = spi;
	spi_set_drvdata(spi, data);

	pr_info("LX: Battery driver\n");

	return 0;

err_kzalloc:
	kfree(data);
	return ret;
}

static int __devexit lx_battery_remove(struct spi_device *spi)
{
	struct driver_data *data = spi_get_drvdata(spi);

	kthread_stop(data->thread);

	if (data->ps_registered) {
		power_supply_unregister(&data->wall);
		power_supply_unregister(&data->main_battery);
		power_supply_unregister(&data->backup_battery);
	}

	mutex_destroy(&data->pwr_state_lock);
	kfree(data);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static int lx_battery_resume(struct spi_device *pdev)
{
	int ret;

	ret = pcon_monitorbattery(1, PCON_FLG_ASYNC);
	if (ret) {
		pr_err("Failed to resume PCON battery monitor: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct spi_driver lx_battery_driver = {
	.driver = {
		.name	= "lx-battery",
		.owner	= THIS_MODULE,
	},
	.probe	= lx_battery_probe,
	.remove	= __devexit_p(lx_battery_remove),
	.resume = lx_battery_resume,
};

static int __init lx_battery_init(void)
{
	return spi_register_driver(&lx_battery_driver);
}
module_init(lx_battery_init);

static void __exit lx_battery_exit(void)
{
	spi_unregister_driver(&lx_battery_driver);
}
module_exit(lx_battery_exit);

MODULE_AUTHOR("Peter Tworek <tworaz666@gmail.com>");
MODULE_DESCRIPTION("LX Battery driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lx-battery");
