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
#include <linux/mod_devicetable.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/interrupt.h>
#include <linux/resource.h>
#include <linux/platform_device.h>

#define PFX KBUILD_MODNAME

#ifdef CONFIG_BCM63XX
# define WDT_CLK_NAME		"periph"
#else
# define WDT_CLK_NAME		NULL
#endif

struct bcm63xx_wdt_hw {
	struct watchdog_device wdd;
	raw_spinlock_t lock;
	void __iomem *regs;
	struct clk *clk;
	unsigned long clock_hz;
	int irq;
	bool running;
};

#define to_wdt_hw(x) container_of(x, struct bcm63xx_wdt_hw, wdd)

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
	__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int bcm63xx_wdt_start(struct watchdog_device *wdd)
{
	struct bcm63xx_wdt_hw *hw = to_wdt_hw(wdd);
	unsigned long flags;

	raw_spin_lock_irqsave(&hw->lock, flags);
	__raw_writel(wdd->timeout * hw->clock_hz, hw->regs + WDT_DEFVAL_REG);
	__raw_writel(WDT_START_1, hw->regs + WDT_CTL_REG);
	__raw_writel(WDT_START_2, hw->regs + WDT_CTL_REG);
	hw->running = true;
	raw_spin_unlock_irqrestore(&hw->lock, flags);
	return 0;
}

static int bcm63xx_wdt_stop(struct watchdog_device *wdd)
{
	struct bcm63xx_wdt_hw *hw = to_wdt_hw(wdd);
	unsigned long flags;

	raw_spin_lock_irqsave(&hw->lock, flags);
	__raw_writel(WDT_STOP_1, hw->regs + WDT_CTL_REG);
	__raw_writel(WDT_STOP_2, hw->regs + WDT_CTL_REG);
	hw->running = false;
	raw_spin_unlock_irqrestore(&hw->lock, flags);
	return 0;
}

static unsigned int bcm63xx_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct bcm63xx_wdt_hw *hw = to_wdt_hw(wdd);
	unsigned long flags;
	u32 val;

	raw_spin_lock_irqsave(&hw->lock, flags);
	val = __raw_readl(hw->regs + WDT_CTL_REG);
	val /= hw->clock_hz;
	raw_spin_unlock_irqrestore(&hw->lock, flags);
	return val;
}

static int bcm63xx_wdt_set_timeout(struct watchdog_device *wdd,
	unsigned int timeout)
{
	wdd->timeout = timeout;
	return 0;
}

/* The watchdog interrupt occurs when half the timeout is remaining */
static irqreturn_t bcm63xx_wdt_interrupt(int irq, void *data)
{
	struct bcm63xx_wdt_hw *hw = data;
	unsigned long flags;

	raw_spin_lock_irqsave(&hw->lock, flags);
	if (!hw->running) {
		/* Stop the watchdog as it shouldn't be running */
		__raw_writel(WDT_STOP_1, hw->regs + WDT_CTL_REG);
		__raw_writel(WDT_STOP_2, hw->regs + WDT_CTL_REG);
	} else {
		u32 timeleft = __raw_readl(hw->regs + WDT_CTL_REG);
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
			__raw_writel(timeleft, hw->regs + WDT_DEFVAL_REG);
			__raw_writel(WDT_START_1, hw->regs + WDT_CTL_REG);
			__raw_writel(WDT_START_2, hw->regs + WDT_CTL_REG);
		} else {
			/* The watchdog cannot be started with a time of less
			 * than 2 ticks (it won't fire).
			 */
			die(PFX ": watchdog timer expired\n", get_irq_regs());
		}

		ms = timeleft / (hw->clock_hz / 1000);
		dev_alert(hw->wdd.dev,
			"warning timer fired, reboot in %ums\n", ms);
	}
	raw_spin_unlock_irqrestore(&hw->lock, flags);
	return IRQ_HANDLED;
}

static struct watchdog_ops bcm63xx_wdt_ops = {
	.owner = THIS_MODULE,
	.start = bcm63xx_wdt_start,
	.stop = bcm63xx_wdt_stop,
	.get_timeleft = bcm63xx_wdt_get_timeleft,
	.set_timeout = bcm63xx_wdt_set_timeout,
};

static const struct watchdog_info bcm63xx_wdt_info = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "BCM63xx Watchdog",
};

static int bcm63xx_wdt_probe(struct platform_device *pdev)
{
	struct bcm63xx_wdt_hw *hw;
	struct watchdog_device *wdd;
	struct resource *r;
	u32 timeleft1, timeleft2;
	unsigned int timeleft;
	int ret;

	hw = devm_kzalloc(&pdev->dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return -ENOMEM;

	wdd = &hw->wdd;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "failed to get resources\n");
		return -ENODEV;
	}

	hw->regs = devm_ioremap_nocache(&pdev->dev, r->start, resource_size(r));
	if (!hw->regs) {
		dev_err(&pdev->dev, "failed to remap I/O resources\n");
		return -ENXIO;
	}

	hw->clk = devm_clk_get(&pdev->dev, WDT_CLK_NAME);
	if (IS_ERR(hw->clk)) {
		if (PTR_ERR(hw->clk) != -EPROBE_DEFER)
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

	wdd->parent = &pdev->dev;
	wdd->ops = &bcm63xx_wdt_ops;
	wdd->info = &bcm63xx_wdt_info;
	wdd->min_timeout = 1;
	wdd->max_timeout = 0xffffffff / hw->clock_hz;
	wdd->timeout = min(30U, wdd->max_timeout);

	platform_set_drvdata(pdev, hw);

	watchdog_init_timeout(wdd, 0, &pdev->dev);
	watchdog_set_nowayout(wdd, nowayout);

	/* Compare two reads of the time left value, 2 clock ticks apart */
	rmb();
	timeleft1 = __raw_readl(hw->regs + WDT_CTL_REG);
	udelay(DIV_ROUND_UP(1000000, hw->clock_hz / 2));
	/* Ensure the register is read twice */
	rmb();
	timeleft2 = __raw_readl(hw->regs + WDT_CTL_REG);

	/* If the time left is changing, the watchdog is running */
	if (timeleft1 != timeleft2) {
		hw->running = true;
		timeleft = bcm63xx_wdt_get_timeleft(wdd);
	} else {
		hw->running = false;
		timeleft = 0;
	}

	ret = watchdog_register_device(wdd);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register watchdog device\n");
		goto disable_clk;
	}

	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq >= 0) {
		ret = devm_request_irq(&pdev->dev, hw->irq,
			bcm63xx_wdt_interrupt, IRQF_TIMER,
			dev_name(&pdev->dev), hw);
		if (ret)
			hw->irq = -1;
	}

	if (hw->irq >= 0) {
		dev_info(&pdev->dev,
			"%s at MMIO 0x%p (irq = %d, timeout = %us, max_timeout = %us)",
			dev_name(wdd->dev), hw->regs, hw->irq,
			wdd->timeout, wdd->max_timeout);
	} else {
		dev_info(&pdev->dev,
			"%s at MMIO 0x%p (timeout = %us, max_timeout = %us)",
			dev_name(wdd->dev), hw->regs,
			wdd->timeout, wdd->max_timeout);
	}

	if (hw->running)
		dev_alert(wdd->dev, "running, reboot in %us\n", timeleft);
	return 0;

disable_clk:
	clk_disable_unprepare(hw->clk);
	return ret;
}

static int bcm63xx_wdt_remove(struct platform_device *pdev)
{
	struct bcm63xx_wdt_hw *hw = platform_get_drvdata(pdev);

	if (hw->irq >= 0)
		devm_free_irq(&pdev->dev, hw->irq, hw);

	watchdog_unregister_device(&hw->wdd);
	clk_disable_unprepare(hw->clk);
	return 0;
}

static void bcm63xx_wdt_shutdown(struct platform_device *pdev)
{
	struct bcm63xx_wdt_hw *hw = platform_get_drvdata(pdev);

	bcm63xx_wdt_stop(&hw->wdd);
}

static const struct of_device_id bcm63xx_wdt_dt_ids[] = {
	{ .compatible = "brcm,bcm6345-wdt" },
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
