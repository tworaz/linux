/*
 * LED CPU Activity Trigger, revision 3.5
 *
 * Copyright 2006 Thomas Tuttle
 *
 * Author: Thomas Tuttle <thinkinginbinary@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/leds.h>
#include <linux/kernel_stat.h>
#include <asm/cputime.h>

#define UPDATE_INTERVAL (5) /* delay between updates, in ms */

/* This *would* be an enum, but I use them as the private data in the
 * include_* module parameters, so they know which kind of time to
 * include or exclude. */

#define NUM_TIME_TYPES 4

static int USER   = 0;
static int NICE   = 1;
static int SYSTEM = 2;
static int IOWAIT = 3;

static int include_time[NUM_TIME_TYPES] = { 1, 0, 1, 0 };

static void ledtrig_cpu_timerfunc(unsigned long data);

DEFINE_LED_TRIGGER(ledtrig_cpu);
static DEFINE_TIMER(ledtrig_cpu_timer, ledtrig_cpu_timerfunc, 0, 0);

static cputime64_t cpu_usage(void)
{
	int i;
	cputime64_t time = cputime64_zero;

	for_each_possible_cpu(i) {
		if (include_time[USER])
			time = cputime64_add(time, kstat_cpu(i).cpustat.user);
		if (include_time[NICE])
			time = cputime64_add(time, kstat_cpu(i).cpustat.nice);
		if (include_time[SYSTEM])
			time = cputime64_add(time, kstat_cpu(i).cpustat.system);
		if (include_time[IOWAIT])
			time = cputime64_add(time, kstat_cpu(i).cpustat.iowait);
	}

	return time;
}

static enum led_brightness last_led_state;
static cputime64_t last_cputime;

static void ledtrig_cpu_timerfunc(unsigned long data)
{
	cputime64_t this_cputime = cpu_usage();
	/* XXX: This assumes that cputime64_t can be subtracted.
	 * Nobody has defined cputime64_sub, so I had to do this instead. */
	cputime64_t used_cputime = this_cputime - last_cputime;
	enum led_brightness led_state = (used_cputime > 0) ? LED_FULL : LED_OFF;
	if (led_state != last_led_state)
		led_trigger_event(ledtrig_cpu, led_state);
	last_led_state = led_state;
	last_cputime = cpu_usage();

	mod_timer(&ledtrig_cpu_timer, jiffies + msecs_to_jiffies(UPDATE_INTERVAL));
}

static int __init ledtrig_cpu_init(void)
{
	led_trigger_register_simple("cpu", &ledtrig_cpu);
	led_trigger_event(ledtrig_cpu, LED_OFF);
	last_led_state = LED_OFF;
	last_cputime = cpu_usage();
	mod_timer(&ledtrig_cpu_timer, jiffies + msecs_to_jiffies(UPDATE_INTERVAL));
	return 0;
}

static void __exit ledtrig_cpu_exit(void)
{
	del_timer_sync(&ledtrig_cpu_timer);
	led_trigger_event(ledtrig_cpu, LED_OFF);
	led_trigger_unregister_simple(ledtrig_cpu);
}

static int get_include_time(char *buffer, struct kernel_param *kp)
{
	int which_time = *((int*)kp->arg);
	if ((which_time < 0) || (which_time >= NUM_TIME_TYPES))
		return -EINVAL;
	sprintf(buffer, "%d", include_time[which_time]);
	return strlen(buffer);
}

static int set_include_time(const char *val, struct kernel_param *kp)
{
	int which_time = *((int*)kp->arg);
	int new_value;
	if ((which_time < 0) || (which_time >= NUM_TIME_TYPES))
		return -EINVAL;
	if (sscanf(val, "%d", &new_value) != 1)
		return -EINVAL;
	include_time[which_time] = new_value;
	return 0;
}

module_param_call(include_user, set_include_time, get_include_time, &USER, S_IWUSR | S_IRUGO);
module_param_call(include_nice, set_include_time, get_include_time, &NICE, S_IWUSR | S_IRUGO);
module_param_call(include_system, set_include_time, get_include_time, &SYSTEM, S_IWUSR | S_IRUGO);
module_param_call(include_iowait, set_include_time, get_include_time, &IOWAIT, S_IWUSR | S_IRUGO);

MODULE_AUTHOR("Thomas Tuttle <thinkinginbinary@gmail.com>");
MODULE_DESCRIPTION("LED CPU Activity Trigger");
MODULE_PARM_DESC(include_user, "Include user time when measuring CPU activity");
MODULE_PARM_DESC(include_nice, "Include nice time when measuring CPU activity");
MODULE_PARM_DESC(include_system, "Include system time when measuring CPU activity");
MODULE_PARM_DESC(include_iowait, "Include IO wait time when measuring CPU activity");
MODULE_LICENSE("GPL");

module_init(ledtrig_cpu_init);
module_exit(ledtrig_cpu_exit);
