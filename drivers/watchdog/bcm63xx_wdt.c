/*
 *  Broadcom BCM63xx SoC watchdog driver
 *
 *  Copyright (C) 2007, Miguel Gaio <miguel.gaio@efixo.com>
 *  Copyright (C) 2008, Florian Fainelli <florian@openwrt.org>
 *  Copyright 2015 Simon Arlott
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/resource.h>
#include <linux/platform_device.h>

#include <bcm63xx_cpu.h>
#include <bcm63xx_io.h>
#include <bcm63xx_regs.h>
#include <bcm63xx_timer.h>

#define PFX KBUILD_MODNAME

#define WDT_HZ			50000000		/* Fclk */
#define WDT_DEFAULT_TIME	30			/* seconds */
#define WDT_MAX_TIME		(0xffffffff / WDT_HZ)	/* seconds */

struct bcm63xx_wdt_hw {
	raw_spinlock_t lock;
	void __iomem *regs;
	unsigned long inuse;
	bool running;
};
static struct bcm63xx_wdt_hw bcm63xx_wdt_device;

static int expect_close;

static int wdt_time = WDT_DEFAULT_TIME;
static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
	__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/* HW functions */
static void bcm63xx_wdt_hw_start(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&bcm63xx_wdt_device.lock, flags);
	bcm_writel(wdt_time * WDT_HZ, bcm63xx_wdt_device.regs + WDT_DEFVAL_REG);
	bcm_writel(WDT_START_1, bcm63xx_wdt_device.regs + WDT_CTL_REG);
	bcm_writel(WDT_START_2, bcm63xx_wdt_device.regs + WDT_CTL_REG);
	bcm63xx_wdt_device.running = true;
	raw_spin_unlock_irqrestore(&bcm63xx_wdt_device.lock, flags);
}

static void bcm63xx_wdt_hw_stop(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&bcm63xx_wdt_device.lock, flags);
	bcm_writel(WDT_STOP_1, bcm63xx_wdt_device.regs + WDT_CTL_REG);
	bcm_writel(WDT_STOP_2, bcm63xx_wdt_device.regs + WDT_CTL_REG);
	bcm63xx_wdt_device.running = false;
	raw_spin_unlock_irqrestore(&bcm63xx_wdt_device.lock, flags);
}

/* The watchdog interrupt occurs when half the timeout is remaining */
static void bcm63xx_wdt_isr(void *data)
{
	struct bcm63xx_wdt_hw *hw = &bcm63xx_wdt_device;
	unsigned long flags;

	raw_spin_lock_irqsave(&hw->lock, flags);
	if (!hw->running) {
		/* Stop the watchdog as it shouldn't be running */
		bcm_writel(WDT_STOP_1, hw->regs + WDT_CTL_REG);
		bcm_writel(WDT_STOP_2, hw->regs + WDT_CTL_REG);
	} else {
		u32 timeleft = bcm_readl(hw->regs + WDT_CTL_REG);
		u32 ms;

		if (timeleft >= 2) {
			/* The only way to clear this level triggered interrupt
			 * without disrupting the normal running of the watchdog
			 * is to restart the watchdog with the current remaining
			 * time value (which will be half the previous timeout)
			 * so the interrupt occurs again at 1/4th, 1/8th, etc.
			 * of the original timeout value until we reboot.
			 *
			 * This is done with a lock held in case userspace is
			 * trying to restart the watchdog on another CPU.
			 */
			bcm_writel(timeleft, hw->regs + WDT_DEFVAL_REG);
			bcm_writel(WDT_START_1, hw->regs + WDT_CTL_REG);
			bcm_writel(WDT_START_2, hw->regs + WDT_CTL_REG);
		} else {
			/* The watchdog cannot be started with a time of less
			 * than 2 ticks (it won't fire).
			 */
			die(PFX ": watchdog timer expired\n", get_irq_regs());
		}

		ms = timeleft / (WDT_HZ / 1000);
		pr_alert("warning timer fired, reboot in %ums\n", ms);
	}
	raw_spin_unlock_irqrestore(&hw->lock, flags);
}

static int bcm63xx_wdt_settimeout(int new_time)
{
	if ((new_time <= 0) || (new_time > WDT_MAX_TIME))
		return -EINVAL;

	wdt_time = new_time;

	return 0;
}

static int bcm63xx_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &bcm63xx_wdt_device.inuse))
		return -EBUSY;

	bcm63xx_wdt_hw_start();
	return nonseekable_open(inode, file);
}

static int bcm63xx_wdt_release(struct inode *inode, struct file *file)
{
	if (expect_close == 42)
		bcm63xx_wdt_hw_stop();
	else {
		pr_crit("Unexpected close, not stopping watchdog!\n");
		bcm63xx_wdt_hw_start();
	}
	clear_bit(0, &bcm63xx_wdt_device.inuse);
	expect_close = 0;
	return 0;
}

static ssize_t bcm63xx_wdt_write(struct file *file, const char *data,
				size_t len, loff_t *ppos)
{
	if (len) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			expect_close = 0;

			for (i = 0; i != len; i++) {
				char c;
				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					expect_close = 42;
			}
		}
		bcm63xx_wdt_hw_start();
	}
	return len;
}

static struct watchdog_info bcm63xx_wdt_info = {
	.identity       = PFX,
	.options        = WDIOF_SETTIMEOUT |
				WDIOF_KEEPALIVEPING |
				WDIOF_MAGICCLOSE,
};


static long bcm63xx_wdt_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_value, retval = -EINVAL;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &bcm63xx_wdt_info,
			sizeof(bcm63xx_wdt_info)) ? -EFAULT : 0;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);

	case WDIOC_SETOPTIONS:
		if (get_user(new_value, p))
			return -EFAULT;

		if (new_value & WDIOS_DISABLECARD) {
			bcm63xx_wdt_hw_stop();
			retval = 0;
		}
		if (new_value & WDIOS_ENABLECARD) {
			bcm63xx_wdt_hw_start();
			retval = 0;
		}

		return retval;

	case WDIOC_KEEPALIVE:
		bcm63xx_wdt_hw_start();
		return 0;

	case WDIOC_SETTIMEOUT:
		if (get_user(new_value, p))
			return -EFAULT;

		if (bcm63xx_wdt_settimeout(new_value))
			return -EINVAL;

		bcm63xx_wdt_hw_start();

	case WDIOC_GETTIMEOUT:
		return put_user(wdt_time, p);

	default:
		return -ENOTTY;

	}
}

static const struct file_operations bcm63xx_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= bcm63xx_wdt_write,
	.unlocked_ioctl	= bcm63xx_wdt_ioctl,
	.open		= bcm63xx_wdt_open,
	.release	= bcm63xx_wdt_release,
};

static struct miscdevice bcm63xx_wdt_miscdev = {
	.minor	= WATCHDOG_MINOR,
	.name	= "watchdog",
	.fops	= &bcm63xx_wdt_fops,
};


static int bcm63xx_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *r;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resources\n");
		return -ENODEV;
	}

	bcm63xx_wdt_device.regs = devm_ioremap_nocache(&pdev->dev, r->start,
							resource_size(r));
	if (!bcm63xx_wdt_device.regs) {
		dev_err(&pdev->dev, "failed to remap I/O resources\n");
		return -ENXIO;
	}

	raw_spin_lock_init(&bcm63xx_wdt_device.lock);
	bcm63xx_wdt_device.running = false;

	ret = bcm63xx_timer_register(TIMER_WDT_ID, bcm63xx_wdt_isr, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register wdt timer isr\n");
		return ret;
	}

	if (bcm63xx_wdt_settimeout(wdt_time)) {
		bcm63xx_wdt_settimeout(WDT_DEFAULT_TIME);
		dev_info(&pdev->dev,
			": wdt_time value must be 1 <= wdt_time <= %d, using %d\n",
			WDT_MAX_TIME, wdt_time);
	}

	ret = misc_register(&bcm63xx_wdt_miscdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register watchdog device\n");
		goto unregister_timer;
	}

	dev_info(&pdev->dev, " started, timer margin: %d sec\n",
						WDT_DEFAULT_TIME);

	return 0;

unregister_timer:
	bcm63xx_timer_unregister(TIMER_WDT_ID);
	return ret;
}

static int bcm63xx_wdt_remove(struct platform_device *pdev)
{
	if (!nowayout)
		bcm63xx_wdt_hw_stop();

	misc_deregister(&bcm63xx_wdt_miscdev);
	bcm63xx_timer_unregister(TIMER_WDT_ID);
	return 0;
}

static void bcm63xx_wdt_shutdown(struct platform_device *pdev)
{
	bcm63xx_wdt_hw_stop();
}

static struct platform_driver bcm63xx_wdt_driver = {
	.probe	= bcm63xx_wdt_probe,
	.remove = bcm63xx_wdt_remove,
	.shutdown = bcm63xx_wdt_shutdown,
	.driver = {
		.name = "bcm63xx-wdt",
	}
};

module_platform_driver(bcm63xx_wdt_driver);

MODULE_AUTHOR("Miguel Gaio <miguel.gaio@efixo.com>");
MODULE_AUTHOR("Florian Fainelli <florian@openwrt.org>");
MODULE_DESCRIPTION("Driver for the Broadcom BCM63xx SoC watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm63xx-wdt");
