/*
 * Copyright 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "sdhci-pltfm.h"

#define REG_BOOT_TIMEOUT	0x70
#define REG_DBG_SEL		0x74
#define REG_EXRDFIFO_EN		0x80
#define REG_EXRDFIFO_CFG	0x84
#define REG_TUNE_STEP		0x88
#define REG_TUNE_STEPS_STD	0x8c
#define REG_TUNE_STEPS_DDR	0x90
#define REG_SPI_INT_SPT		0xf0

/* The Arasan has a bug whereby it may lose the content of
 * successive writes to registers that are within two SD-card clock
 * cycles of each other (a clock domain crossing problem).
 * It seems, however, that the data register does not have this problem.
 */
static void bcm2708_sdhci_clock_delay(struct sdhci_host *host, int reg)
{
	unsigned int ns_2clk;

	if (reg == SDHCI_BUFFER || host->clock == 0)
		return;

	/* host->clock is the clock freq in Hz */
	ns_2clk = 2000000000/host->clock;
	ndelay(ns_2clk);
}

static void bcm2708_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
	writel(val, host->ioaddr + reg);
	bcm2708_sdhci_clock_delay(host, reg);
}

static void bcm2708_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
	writew(val, host->ioaddr + reg);
	bcm2708_sdhci_clock_delay(host, reg);
}

static void bcm2708_sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
	writeb(val, host->ioaddr + reg);
	bcm2708_sdhci_clock_delay(host, reg);
}

static unsigned int bcm2708_sdhci_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *phost = sdhci_priv(host);
	return phost->clock;
}

static struct sdhci_ops bcm2708_sdhci_ops __devinitconst = {
	.write_l = bcm2708_sdhci_writel,
	.write_w = bcm2708_sdhci_writew,
	.write_b = bcm2708_sdhci_writeb,

	.get_max_clock = bcm2708_sdhci_get_max_clock,
};

static struct sdhci_pltfm_data bcm2708_sdhci_pdata __devinitconst = {
	.quirks = SDHCI_QUIRK_BROKEN_CARD_DETECTION
		| SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK
		| SDHCI_QUIRK_BROKEN_TIMEOUT_VAL
		| SDHCI_QUIRK_MISSING_CAPS
		| SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_OCR_FROM_REGULATOR
		| SDHCI_QUIRK2_START_PIO_FROM_INT,
	.caps = SDHCI_CAN_DO_HISPD,
	.ops = &bcm2708_sdhci_ops
};

static int __devinit bcm2708_sdhci_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOMEM;
	base = devm_request_and_ioremap(&pdev->dev, res);
	if (!base)
		return -EIO;

	/* Disable extension data FIFO unless DMA is enabled */
	writel(0, base + REG_EXRDFIFO_EN);
	writel(0, base + REG_EXRDFIFO_CFG);
	devm_ioremap_release(&pdev->dev, base);
	devm_release_mem_region(&pdev->dev, res->start, resource_size(res));

	return sdhci_pltfm_register(pdev, &bcm2708_sdhci_pdata);
}

static int __devexit bcm2708_sdhci_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static struct of_device_id bcm2708_sdhci_match[] __devinitconst = {
	{ .compatible = "broadcom,bcm2708-sdhci" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm2708_sdhci_match);

static struct platform_driver bcm2708_sdhci_driver = {
	.probe	= bcm2708_sdhci_probe,
	.remove	= __devexit_p(bcm2708_sdhci_remove),
	.driver	= {
		.name = "sdhci-of-bcm2708",
		.owner = THIS_MODULE,
		.of_match_table = bcm2708_sdhci_match,
		.pm = SDHCI_PLTFM_PMOPS
	}
};
module_platform_driver(bcm2708_sdhci_driver);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("Broadcom BCM2708 SDHCI driver");
MODULE_LICENSE("GPL");
