/*
 *  linux/arch/arm/mach-bcm2708/vcos_linuxkernel.c
 *
 *  Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define  VCOS_INLINE_BODIES
#include "interface/vcos/vcos.h"

#ifndef VCOS_DEFAULT_STACK_SIZE
#define VCOS_DEFAULT_STACK_SIZE 4096
#endif

static VCOS_THREAD_ATTR_T default_attrs = {
	0,
	VCOS_DEFAULT_STACK_SIZE,
};

DECLARE_MUTEX(lock);

VCOS_STATUS_T vcos_thread_create(VCOS_THREAD_T * thread,
				 const char *name,
				 VCOS_THREAD_ATTR_T * attrs,
				 VCOS_THREAD_ENTRY_FN_T entry, void *arg)
{
	*thread = kthread_run((int (*)(void *))entry, arg, name);
	vcos_assert(*thread != NULL);
	return VCOS_SUCCESS;
}

void vcos_thread_join(VCOS_THREAD_T * thread, void **pData)
{
	void *ret = (void *)kthread_stop(*thread);
	if (pData)
		*pData = ret;
}

uint32_t vcos_getmicrosecs(void)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	return (tv.tv_sec * 1000000) + tv.tv_usec;
}

void vcos_timer_init(void)
{
}

void vcos_log_impl(const VCOS_LOG_CAT_T *cat, const char *fmt, ...)
{
	va_list ap;
	if (cat->flags.want_prefix)
		printk("%s: ", cat->name);

	va_start(ap, fmt);
	vprintk(fmt, ap);
	va_end(ap);
	printk("\n");
}

void vcos_vlog_impl(const VCOS_LOG_CAT_T *cat, const char *fmt, va_list args)
{
	if (cat->flags.want_prefix)
		printk("%s: ", cat->name);
	vprintk(fmt, args);
	printk("\n");
}

void vcos_log_register(const char *name, VCOS_LOG_CAT_T *category)
{
	memset(category, 0, sizeof(*category));
	category->name = name;
	category->level = VCOS_LOG_ERROR;
	category->flags.want_prefix = 1;
}

void vcos_log_unregister(VCOS_LOG_CAT_T *category)
{
	category->level = VCOS_LOG_NEVER;
}

VCOS_STATUS_T vcos_init(void)
{
	return VCOS_SUCCESS;
}

void vcos_deinit(void)
{
}

void vcos_global_lock(void)
{
	down(&lock);
}

void vcos_global_unlock(void)
{
	up(&lock);
}

void vcos_thread_exit(void *arg)
{
	/* This isn't possible */
	vcos_assert(0);
}

void vcos_thread_attr_init(VCOS_THREAD_ATTR_T * attrs)
{
	*attrs = default_attrs;
}

void _vcos_task_timer_set(void (*pfn) (void *), void *cxt, VCOS_UNSIGNED ms)
{
	vcos_assert(0);	/* no timers on event groups yet */
}

void _vcos_task_timer_cancel(void)
{
}

int vcos_snprintf(char *buf, size_t buflen, const char *fmt, ...)
{
	int ret;
	va_list ap;
	va_start(ap, fmt);
	ret = vsnprintf(buf, buflen, fmt, ap);
	va_end(ap);
	return ret;
}

int vcos_llthread_running(VCOS_THREAD_T * t)
{
	vcos_assert(0);	/* this function only exists as a nasty hack for the video codecs! */
	return 1;
}
