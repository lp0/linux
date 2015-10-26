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

#include <linux/bcm63xx_wdt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/watchdog.h>

#ifdef CONFIG_BCM63XX
# include <bcm63xx_regs.h>
# include <bcm63xx_timer.h>
#endif

#define PFX KBUILD_MODNAME

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
	__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

struct bcm63xx_wdt_hw {
	raw_spinlock_t lock;
	void __iomem *base;
	struct clk *clk;
	u32 clock_hz;
	int irq;
	bool running;
};

static int bcm63xx_wdt_start(struct watchdog_device *wdd)
{
	struct bcm63xx_wdt_hw *hw = watchdog_get_drvdata(wdd);
	unsigned long flags;

	raw_spin_lock_irqsave(&hw->lock, flags);
	__raw_writel(wdd->timeout * hw->clock_hz, hw->base + WDT_DEFVAL_REG);
	__raw_writel(WDT_START_1, hw->base + WDT_CTL_REG);
	__raw_writel(WDT_START_2, hw->base + WDT_CTL_REG);
	hw->running = true;
	raw_spin_unlock_irqrestore(&hw->lock, flags);
	return 0;
}

static int bcm63xx_wdt_stop(struct watchdog_device *wdd)
{
	struct bcm63xx_wdt_hw *hw = watchdog_get_drvdata(wdd);
	unsigned long flags;

	raw_spin_lock_irqsave(&hw->lock, flags);
	__raw_writel(WDT_STOP_1, hw->base + WDT_CTL_REG);
	__raw_writel(WDT_STOP_2, hw->base + WDT_CTL_REG);
	hw->running = false;
	raw_spin_unlock_irqrestore(&hw->lock, flags);
	return 0;
}

static unsigned int bcm63xx_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct bcm63xx_wdt_hw *hw = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&hw->lock, flags);
	val = __raw_readl(hw->base + WDT_CTL_REG);
	val /= hw->clock_hz;
	raw_spin_unlock_irqrestore(&hw->lock, flags);
	return val;
}

static int bcm63xx_wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	wdd->timeout = timeout;
	return bcm63xx_wdt_start(wdd);
}

/* The watchdog interrupt occurs when half the timeout is remaining */
#ifdef CONFIG_BCM63XX
static void bcm63xx_wdt_interrupt(void *data)
#else
static irqreturn_t bcm63xx_wdt_interrupt(int irq, void *data)
#endif
{
	struct watchdog_device *wdd = data;
	struct bcm63xx_wdt_hw *hw = watchdog_get_drvdata(wdd);
	unsigned long flags;

	raw_spin_lock_irqsave(&hw->lock, flags);
	if (!hw->running) {
		/* Oops */
		__raw_writel(WDT_STOP_1, hw->base + WDT_CTL_REG);
		__raw_writel(WDT_STOP_2, hw->base + WDT_CTL_REG);
	} else {
		u32 timeleft = __raw_readl(hw->base + WDT_CTL_REG);
		u32 ms;

		if (timeleft >= 2) {
			/* The only way to stop this interrupt without masking
			 * the whole timer interrupt or disrupting the intended
			 * behaviour of the watchdog is to restart the watchdog
			 * with the remaining time value so that the interrupt
			 * occurs again at 1/4th, 1/8th, etc. of the timeout
			 * until we reboot.
			 *
			 * This is done with a lock held in case userspace is
			 * restarting the watchdog on another CPU.
			 */
			__raw_writel(timeleft, hw->base + WDT_DEFVAL_REG);
			__raw_writel(WDT_START_1, hw->base + WDT_CTL_REG);
			__raw_writel(WDT_START_2, hw->base + WDT_CTL_REG);
		} else {
			/* The watchdog cannot be started with a time of less
			 * than 2 ticks (it won't fire).
			 */
			die(PFX ": watchdog timer expired\n", get_irq_regs());
		}

		ms = timeleft / (hw->clock_hz / 1000);
		dev_alert(wdd->dev, "warning timer fired, reboot in %ums", ms);
	}
	raw_spin_unlock_irqrestore(&hw->lock, flags);
#ifndef CONFIG_BCM63XX
	return IRQ_HANDLED;
#endif
}

static struct watchdog_ops bcm63xx_wdt_ops = {
	.owner = THIS_MODULE,
	.start = bcm63xx_wdt_start,
	.stop = bcm63xx_wdt_stop,
	.get_timeleft = bcm63xx_wdt_get_timeleft,
	.set_timeout = bcm63xx_wdt_set_timeout,
};

static const struct watchdog_info bcm63xx_wdt_info = {
	.options = WDIOC_GETTIMELEFT | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "BCM63xx Watchdog",
};

static int bcm63xx_wdt_probe(struct platform_device *pdev)
{
	struct bcm63xx_wdt_hw *hw;
	struct watchdog_device *wdd;
	struct resource *r;
	unsigned int timeleft;
	int ret;

	hw = devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	wdd = devm_kzalloc(&pdev->dev, sizeof(*wdd), GFP_KERNEL);
	if (!hw || !wdd)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resources\n");
		return -ENODEV;
	}

	hw->base = devm_ioremap_nocache(&pdev->dev, r->start, resource_size(r));
	if (!hw->base) {
		dev_err(&pdev->dev, "failed to remap I/O resources\n");
		return -ENXIO;
	}

#ifdef CONFIG_BCM63XX
	hw->clk = devm_clk_get(&pdev->dev, "periph");
#else
	hw->clk = devm_clk_get(&pdev->dev, NULL);
#endif
	if (IS_ERR(hw->clk)) {
		dev_err(&pdev->dev, "unable to request clock\n");
		return PTR_ERR(hw->clk);
	}

	hw->clock_hz = clk_get_rate(hw->clk);
	if (!hw->clock_hz) {
		dev_err(&pdev->dev, "unable to fetch clock rate\n");
		return -EINVAL;
	}

	ret = clk_prepare_enable(hw->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		return ret;
	}

	raw_spin_lock_init(&hw->lock);
	hw->running = false;

	wdd->parent = &pdev->dev;
	wdd->ops = &bcm63xx_wdt_ops;
	wdd->info = &bcm63xx_wdt_info;
	wdd->min_timeout = 1;
	wdd->max_timeout = 0xffffffff / hw->clock_hz;
	wdd->timeout = min(30U, wdd->max_timeout);

	watchdog_set_drvdata(wdd, hw);
	platform_set_drvdata(pdev, wdd);

	watchdog_init_timeout(wdd, 0, &pdev->dev);
	watchdog_set_nowayout(wdd, nowayout);

	timeleft = bcm63xx_wdt_get_timeleft(wdd);
	if (timeleft > 0)
		hw->running = true;

#ifdef CONFIG_BCM63XX
	ret = bcm63xx_timer_register(TIMER_WDT_ID, bcm63xx_wdt_interrupt, wdd);
	if (ret) {
		dev_err(&pdev->dev, "failed to register with bcm63xx_timer\n");
		goto disable_clk;
	}
	hw->irq = 0;
#endif

	ret = watchdog_register_device(wdd);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register watchdog device\n");
#ifdef CONFIG_BCM63XX
		goto unregister_timer;
#else
		goto disable_clk;
#endif
	}

#ifndef CONFIG_BCM63XX
	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq) {
		ret = devm_request_irq(&pdev->dev, hw->irq,
			bcm63xx_wdt_interrupt, IRQF_TIMER,
			dev_name(&pdev->dev), wdd);
		if (ret)
			hw->irq = 0;
	}
#endif

	if (hw->irq) {
		dev_info(&pdev->dev,
			"%s at MMIO 0x%p (irq = %d, timeout = %us, max_timeout = %us)",
			dev_name(wdd->dev), hw->base, hw->irq,
			wdd->timeout, wdd->max_timeout);
	} else {
		dev_info(&pdev->dev,
			"%s at MMIO 0x%p (timeout = %us, max_timeout = %us)",
			dev_name(wdd->dev), hw->base,
			wdd->timeout, wdd->max_timeout);
	}

	if (timeleft > 0)
		dev_alert(wdd->dev, "running, reboot in %us\n", timeleft);
	return 0;

#ifdef CONFIG_BCM63XX
unregister_timer:
	bcm63xx_timer_unregister(TIMER_WDT_ID);
#endif
disable_clk:
	clk_disable(hw->clk);
	return ret;
}

static int bcm63xx_wdt_remove(struct platform_device *pdev)
{
	struct watchdog_device *wdd = platform_get_drvdata(pdev);
	struct bcm63xx_wdt_hw *hw = watchdog_get_drvdata(wdd);

	if (hw->irq)
		devm_free_irq(&pdev->dev, hw->irq, wdd);

#ifdef CONFIG_BCM63XX
	bcm63xx_timer_unregister(TIMER_WDT_ID);
#endif
	watchdog_unregister_device(wdd);
	clk_disable(hw->clk);
	return 0;
}

static void bcm63xx_wdt_shutdown(struct platform_device *pdev)
{
	struct watchdog_device *wdd = platform_get_drvdata(pdev);

	bcm63xx_wdt_stop(wdd);
}

static const struct of_device_id bcm63xx_wdt_dt_ids[] = {
	{ .compatible = "brcm,bcm63xx-wdt" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm63xx_wdt_dt_ids);

static struct platform_driver bcm63xx_wdt_driver = {
	.probe	= bcm63xx_wdt_probe,
	.remove = bcm63xx_wdt_remove,
	.shutdown = bcm63xx_wdt_shutdown,
	.driver = {
		.name = "bcm63xx-wdt",
		.of_match_table = bcm63xx_wdt_dt_ids,
	}
};

module_platform_driver(bcm63xx_wdt_driver);

MODULE_AUTHOR("Miguel Gaio <miguel.gaio@efixo.com>");
MODULE_AUTHOR("Florian Fainelli <florian@openwrt.org>");
MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("Driver for the Broadcom BCM63xx SoC watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm63xx-wdt");
