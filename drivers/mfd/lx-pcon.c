/*
 *  linux/drivers/mfd/lx-pcon.c
 *
 *  Driver for LX PCON chip
 *  Copyright (C) 2004 Simtec Electronics
 *
 *  based on linux/drivers/acorn/char/pcf8583.c
 *  Copyright (C) 2000 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
/* This driver provides the low-level i2c sending for the LX's
 * power-management and general support processor. This allows other parts
 * of the system to send messages to the chip for processing (including
 * the userland)
 *
 * Note, this code does not provide the i2c client facilities needed by
 * the PCon to send data to the systems that need information feeding in.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>

#include <mach/lx-pcon.h>

struct pcon_client {
	struct i2c_client	*client;
	struct task_struct	*thread;
	wait_queue_head_t	thr_wait;
	struct platform_device	*kbd;
	struct platform_device	*pwr;
	void			*pcon_dev;
};

/*
 * command entry for the command queue
 */
struct pcon_cmd_ent {
	struct pcon_cmd_ent	*next;
	struct completion	done;

	unsigned int		flags;

#define PCON_FLG__ASYNC		(1 << 0) /* defined elsewhere */
#define PCON_FLG_CHKRDY		(1 << 1)
#define PCON_FLG_RECVMSG	(1 << 2)

	int                     sendlen;
	int                     result;
	unsigned char           sendmsg[16];
	unsigned char           recvmsg[1];
};

static struct pcon_cmd_ent *queue;
static struct pcon_client *cur_client;

#define PCON_I2C_ADDR (0x26 >> 1)

static int
pcon_do_read(struct pcon_client *pc, int loc, unsigned char *result)
{
	struct i2c_client *client = pc->client;
	unsigned char loc_data[1];
	struct i2c_msg msgs[2] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 1,
			.buf    = loc_data,
		}, {
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 1,
			.buf    = result,
		}
	};

	loc_data[0] = loc;

	return i2c_transfer(client->adapter, msgs, 2);
}

static int pcon_send(struct pcon_client *pc, unsigned char *data, int length)
{
	struct i2c_client *client = pc->client;
	struct i2c_msg msgs[1] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = length,
			.buf    = data
		}
	};

	return i2c_transfer(client->adapter, msgs, 1) == 1 ? 1 : 0;
}

static int pcon_check_ready(struct pcon_client *client)
{
	unsigned char inbuff;
	int ret;

	while (1) {
		/* the sfi code places this in the way to make sure that the
		 * friendly pcon isn't going to ack us... dunno if this is
		 * going to help us or not with our problems with reboot
		 * after suspend...
		 *
		 * yep, this delay fixes the problem.
		 */
		msleep(100);

		ret = pcon_do_read(client, PCON_CMD_ADDR, &inbuff);

		if (ret == 2 && inbuff == PCON_CMD_Ready)
			return 0;

		/* ok, we didn't get an ok response, let us
		 * wait for a little while and try again
		 */
		msleep(100);
	}

	return 0;
}

static int pcon_thread(void *param)
{
	struct pcon_client *pc = param;
	unsigned long flags;
	int ret;

	current->flags |= PF_NOFREEZE;

	while (1) {
		struct pcon_cmd_ent *cmd;

		wait_event_interruptible(pc->thr_wait, queue ||
				kthread_should_stop());

		if (kthread_should_stop())
			break;

		/*
		 * remove one item from the queue, and execute it
		 */
		local_irq_save(flags);
		cmd = queue;
		if (cmd != NULL)
			queue = cmd->next;
		local_irq_restore(flags);

		if (!cmd)
			continue;

		if (cmd->flags & PCON_FLG_CHKRDY)
			pcon_check_ready(pc);

		if (cmd->flags & PCON_FLG_RECVMSG)
			ret = pcon_do_read(pc, cmd->sendmsg[0], cmd->recvmsg);
		else
			ret = pcon_send(pc, cmd->sendmsg, cmd->sendlen);

		cmd->result = ret;

		if ((cmd->flags & PCON_FLG_ASYNC) == 0) {
			complete(&cmd->done);
		} else {
			kfree(cmd);
		}
	}

	return 0;
}

static int pcon_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pcon_client *pc;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	dev_info(&client->dev, "LX: pcon found at 0x%02x\n", client->addr);

	pc = kzalloc(sizeof(*pc), GFP_KERNEL);
	if (pc == NULL) {
		pr_err("pcon: no memory for pcon client data\n");
		ret = -ENOMEM;
		goto done;
	}

	pc->client = client;
	i2c_set_clientdata(client, pc);

	pc->kbd = platform_device_alloc("lx-kbd", -1);
	pc->pwr = platform_device_alloc("lx-pwr", -1);
	if (!pc->kbd || !pc->pwr) {
		ret = -ENOMEM;
		goto done;
	}

	pc->kbd->dev.parent = &client->dev;
	pc->pwr->dev.parent = &client->dev;

	init_waitqueue_head(&pc->thr_wait);

	cur_client = pc;

	pc->thread = kthread_run(pcon_thread, pc, "pcon");
	if (IS_ERR(pc->thread)) {
		ret = PTR_ERR(pc->thread);
		pr_err("pcon: cannot start thread: %d\n", ret);
		goto done;
	}

	ret = platform_device_add(pc->kbd);
	if (ret < 0)
		goto done;

	ret = platform_device_add(pc->pwr);
	if (ret < 0) {
		platform_device_del(pc->kbd);
		goto done;
	}

	if (pcon_pwr_setstate(PCON_PWRSTATE_Run)) {
		platform_device_del(pc->kbd);
		platform_device_del(pc->pwr);
		pr_err("pcon: failed to change power state!");
		goto done;
	}
	pr_info("Moving pcon power state to Run\n");

	ret = 0;

done:
	if (ret < 0) {
		if (!IS_ERR(pc->thread) && pc->thread)
			kthread_stop(pc->thread);
		platform_device_put(pc->kbd);
		platform_device_put(pc->pwr);
		kfree(pc);
	}

	return ret;
}

static int pcon_remove(struct i2c_client *client)
{
	struct pcon_client *pc = i2c_get_clientdata(client);

	platform_device_unregister(pc->pwr);
	platform_device_unregister(pc->kbd);
	kthread_stop(pc->thread);
	kfree(pc);
	return 0;
}

static int
__pcon_command(struct pcon_client *pc, unsigned int flags, unsigned char *msg,
		int len, int *result)
{
	struct pcon_cmd_ent *ent;
	unsigned long irq_flags;
	int ret = 0;

	ent = kmalloc(sizeof(*ent), (flags & PCON_FLG_ASYNC)
			? GFP_ATOMIC : GFP_KERNEL);
	if (ent == NULL) {
		pr_err("%s: no memory for command\n", __func__);
		return -ENOMEM;
	}

	init_completion(&ent->done);
	ent->next    = NULL;
	ent->flags   = flags;
	ent->sendlen = len;
	memcpy(ent->sendmsg, msg, len);

	local_irq_save(irq_flags);

	if (queue == NULL) {
		queue = ent;
	} else {
		struct pcon_cmd_ent *ptr = queue;

		for (; ptr->next != NULL; ptr = ptr->next)
			;

		ptr->next = ent;
	}

	local_irq_restore(irq_flags);

	if (pc != NULL)
		wake_up(&pc->thr_wait);

	if (flags & PCON_FLG_ASYNC) {
		/* nothing to do, leave it to it. */
	} else {
		WARN_ON(pc == NULL);
		wait_for_completion(&ent->done);

		if (flags & PCON_FLG_RECVMSG) {
			if (ent->result == 2) {
				if (result)
					*result = ent->recvmsg ? 1 : 0;
				ret = 0;
			} else
				ret = ent->result;
		}

		/* dump this, we've been left with it*/
		kfree(ent);
	}

	return ret;
}

int pcon_simple_cmd(unsigned int cmd, unsigned int val, unsigned int flags)
{
	unsigned char sendmsg[3];

	might_sleep_if(flags != PCON_FLG_ASYNC);

	sendmsg[0] = PCON_CMD_ADDR;
	sendmsg[1] = cmd;
	sendmsg[2] = val;

	return __pcon_command(cur_client, flags | PCON_FLG_CHKRDY,
	                      sendmsg, sizeof(sendmsg), NULL);
}
EXPORT_SYMBOL(pcon_simple_cmd);

int pcon_simple_write(unsigned int addr, unsigned int val, unsigned int flags)
{
	unsigned char sendmsg[2];

	might_sleep_if(flags != PCON_FLG_ASYNC);

	sendmsg[0] = addr;
	sendmsg[1] = val;

	return __pcon_command(cur_client, flags, sendmsg, sizeof(sendmsg),
	                      NULL);
}
EXPORT_SYMBOL(pcon_simple_write);

int pcon_rtc_wakeup(struct rtc_time *tm)
{
	unsigned char sendmsg[10];

	might_sleep();

	sendmsg[0] = PCON_CMD_ADDR;
	sendmsg[1] = PCON_CMD_SetWakeupAlarm;
	sendmsg[2] = tm->tm_sec;
	sendmsg[3] = tm->tm_min;
	sendmsg[4] = tm->tm_hour;
	sendmsg[5] = tm->tm_wday; /* 6 = Saturday */
	sendmsg[6] = tm->tm_mday; /* 1 = 1st */
	sendmsg[7] = tm->tm_mon;
	sendmsg[8] = tm->tm_year >> 8;
	sendmsg[9] = tm->tm_year & 255;

	return __pcon_command(cur_client, PCON_FLG_CHKRDY, sendmsg,
	                      sizeof(sendmsg), NULL);
}
EXPORT_SYMBOL(pcon_rtc_wakeup);

int pcon_pwr_setstate(pcon_pwrstate_t pwrstate)
{
	unsigned char sendmsg[2];

	if (cur_client == NULL) {
		pr_err("%s: no client for write\n", __func__);
		return -ENODEV;
	}

	sendmsg[0] = PCON_PWRSTATE_ADDR;
	sendmsg[1] = pwrstate;

	if (pcon_send(cur_client, sendmsg, sizeof(sendmsg)))
		return 0;
	else
		return -EIO;
}
EXPORT_SYMBOL(pcon_pwr_setstate);

static const struct i2c_device_id pcon_id[] = {
	{ "lxpcon", 0 },
	{ }
};

static struct i2c_driver pcon_driver = {
	.driver         = {
		.owner          = THIS_MODULE,
		.name           = "pcon",
	},
	.id_table       = pcon_id,
	.probe          = pcon_probe,
	.remove         = pcon_remove,
};

static __init int pcon_init(void)
{
	return i2c_add_driver(&pcon_driver);
}

static __exit void pcon_exit(void)
{
	i2c_del_driver(&pcon_driver);
}

MODULE_AUTHOR("Ben Dooks <ben@simtec.co.uk>");
MODULE_DESCRIPTION("Psion Teklogix NetBook Pro PCon driver");

module_init(pcon_init);
module_exit(pcon_exit);
