/*
 * linux/drivers/char/lx-nvram.c
 *
 * (c) 2004 Simtec Electronics
 *
 * Based (heavily) on CMOS/NV-RAM driver in drivers/char/nvram.c
 *
 *	1.3	Original Psion version of the driver
 *	1.4	Cleaned up and ported to kernel 2.6.31
 */

#define LX_NVRAM_VERSION	"1.4"

#include <linux/types.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/nvram.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <mach/lx.h>
#include <mach/lx-nvram.h>

#define DEBUG

#define NVRAM_BYTES (EEPROM_LX_SIZE_IN_BYTES)

// nvram flags
#define NVRAM_EXCL  (2)
#define NVRAM_WRITE (1)

static int nvram_open_cnt;
static int nvram_open_mode;
static struct i2c_client *nvram_client;

static DEFINE_MUTEX(nvram_mutex);
static DEFINE_SPINLOCK(nvram_state_lock);

static int touch_unlocked = 0;

#define NVRAM_ACCESS_READ	I2C_M_RD
#define NVRAM_ACCESS_WRITE	I2C_M_NOSTART

/*
 * __lxnvram_access - low level EEPROM IO accessor
 * @client: i2c client
 * @buf: data buffer to read/write
 * @addr: EEPROM address (0 <= addr <= NVRAM_BYTES
 * @len: size of data length
 * @mode: %NVRAM_ACCESS_READ or %NVRAM_ACCESS_WRITE
 */
int __lxnvram_access(struct i2c_client *client, unsigned char *buf,
                     unsigned char addr, size_t len, int mode)
{
	struct i2c_msg msgs[2];
	int ret;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &addr;

	msgs[1].addr = client->addr;
	msgs[1].flags = mode;
	msgs[1].len = len;
	msgs[1].buf = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);

	return ret == 2 ? 0 : ret < 0 ? ret : -EIO;
}

static void __lxnv_write_byte(unsigned char addr, unsigned char byte)
{
	if (addr >= EEPROM_LX_BOOST_SECURITY_ADDRESS &&
	    addr < EEPROM_LX_BOOST_SECURITY_ADDRESS + EEPROM_LX_BOOST_SECURITY_SIZE)
		return;

	if (addr >= EEPROM_LX_TOUCH_CALIBRATION_DATA_ADDRESS &&
	    addr < EEPROM_LX_TOUCH_CALIBRATION_DATA_ADDRESS +
	           EEPROM_LX_TOUCH_CALIBRATION_DATA_SIZE &&
	    !touch_unlocked)
		return;

	__lxnvram_access(nvram_client, &byte, addr, 1, NVRAM_ACCESS_WRITE);
}

/*
 * The are the file operation function for user access to /dev/nvram
 */

static long long lxnvram_llseek(struct file *file, loff_t offset, int origin)
{
	switch (origin) {
	case 0:
		/* nothing to do */
		break;
	case 1:
		offset += file->f_pos;
		break;
	case 2:
		offset += NVRAM_BYTES;
		break;
	}
	return (offset >= 0) ? (file->f_pos = offset) : -EINVAL;
}

static ssize_t lxnvram_read(struct file *file, char __user *buf,
                          size_t count, loff_t *ppos)
{
	unsigned char contents[NVRAM_BYTES];
        loff_t loc = *ppos;
	ssize_t ret;

	spin_lock(&nvram_state_lock);

	if (!nvram_client) {
		ret = -ENXIO;
		goto out;
	}

	if (loc >= NVRAM_BYTES) {
		ret = 0;
		goto out;
	}

	if ((loc + count) > NVRAM_BYTES)
		count = NVRAM_BYTES - (unsigned int)loc;

	memset(contents, 0xff, sizeof(contents));

	ret = __lxnvram_access(nvram_client, contents, (unsigned char)loc,
			   count, NVRAM_ACCESS_READ);
	if (ret)
		goto out;

	if (copy_to_user(buf, contents, count)) {
		ret = -EFAULT;
		goto out;
	}

	*ppos = loc + count;
	ret = count;

 out:
	spin_unlock(&nvram_state_lock);

	return ret;
}

static ssize_t lxnvram_write(struct file *file, const char __user *buf,
                           size_t count, loff_t *ppos)
{
	unsigned char contents[NVRAM_BYTES];
	unsigned i = *ppos;
	unsigned char *tmp;

	if (!nvram_client)
		return -ENXIO;

	if (i >= NVRAM_BYTES)
		return 0;	/* Past EOF */

	if (count > NVRAM_BYTES - i)
		count = NVRAM_BYTES - i;
	if (count > NVRAM_BYTES)
		return -EFAULT; /* Can't happend, but prove it to gcc */

	if (copy_from_user(contents, buf, count))
		return -EFAULT;

	spin_lock(&nvram_state_lock);

	for (tmp = contents; count--; ++i, ++tmp)
		__lxnv_write_byte(i, *tmp);
	touch_unlocked = 0;

	spin_unlock(&nvram_state_lock);

	*ppos = i;

	return tmp - contents;
}

static long lxnvram_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int addr;

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	if (!nvram_client)
		return -ENXIO;

	switch (cmd) {

	case NVRAM_INIT:
		mutex_lock(&nvram_mutex);

		for (addr = 0; addr < NVRAM_BYTES; addr++)
			__lxnv_write_byte(addr, 0);

		touch_unlocked = 0;

		mutex_unlock(&nvram_mutex);
		return 0;

	case NVRAM_SETCKS:
		mutex_lock(&nvram_mutex);

		touch_unlocked = 0;

		mutex_unlock(&nvram_mutex);
		return 0;

	case NVRAM_UNLOCK_TOUCH_CALIBRATION:
		mutex_lock(&nvram_mutex);

		touch_unlocked = 1;

		mutex_unlock(&nvram_mutex);
		return 0;

	default:
		return -ENOTTY;
	}
}

static int lxnvram_open(struct inode *inode, struct file *file)
{
	if (!nvram_client)
		return -ENXIO;

	spin_lock(&nvram_state_lock);

	if ((nvram_open_cnt && (file->f_flags & O_EXCL)) ||
	    (nvram_open_mode & NVRAM_EXCL) ||
	    ((file->f_mode & FMODE_WRITE) && (nvram_open_mode & NVRAM_WRITE))) {
		spin_unlock(&nvram_state_lock);
		return -EBUSY;
	}

	if (file->f_flags & O_EXCL)
		nvram_open_mode |= NVRAM_EXCL;
	if (file->f_mode & FMODE_WRITE)
		nvram_open_mode |= NVRAM_WRITE;
	nvram_open_cnt++;

	spin_unlock(&nvram_state_lock);

	return 0;
}

static int lxnvram_release(struct inode *inode, struct file *file)
{
	spin_lock(&nvram_state_lock);

	nvram_open_cnt--;

	/* if only one instance is open, clear the EXCL bit */
	if (nvram_open_mode & NVRAM_EXCL)
		nvram_open_mode &= ~NVRAM_EXCL;
	if (file->f_mode & 2)
		nvram_open_mode &= ~NVRAM_WRITE;

	spin_unlock(&nvram_state_lock);

	return 0;
}

static struct file_operations lxnvram_fops = {
	.owner		= THIS_MODULE,
	.llseek		= lxnvram_llseek,
	.read		= lxnvram_read,
	.write		= lxnvram_write,
	.unlocked_ioctl	= lxnvram_ioctl,
	.open		= lxnvram_open,
	.release	= lxnvram_release,
};

static struct miscdevice lxnvram_dev = {
	.minor	= NVRAM_MINOR,
	.name	= "nvram",
	.fops	= &lxnvram_fops
};

static int lxnvram_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ret = misc_register(&lxnvram_dev);
	if (ret) {
		printk(KERN_ERR "nvram: can't misc_register on minor=%d\n",
			NVRAM_MINOR);
		return ret;
	}

	nvram_client = client;

	printk(KERN_INFO "LX: non-volatile memory driver v" LX_NVRAM_VERSION "\n");

	return 0;
}

static int lxnvram_remove(struct i2c_client *client)
{
	misc_deregister(&lxnvram_dev);
	nvram_client = NULL;
	return 0;
}

static const struct i2c_device_id lxnvram_id[] = {
       { "lxnvram", 0 },
       { }
};

static struct i2c_driver lxnvram_driver = {
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "lxnvram",
	},
	.id_table	= lxnvram_id,
	.probe		= lxnvram_probe,
	.remove		= lxnvram_remove
};

static int __init nvram_init(void)
{
	return i2c_add_driver(&lxnvram_driver);
}

static void __exit nvram_exit(void)
{
	i2c_del_driver(&lxnvram_driver);
}

module_init(nvram_init);
module_exit(nvram_exit);

MODULE_LICENSE("GPL");
