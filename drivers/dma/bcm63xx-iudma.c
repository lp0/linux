/*
 * BCM63xx IUDMA Controller
 *
 * Copyright 2015 Simon Arlott
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
 * Derived from drivers/net/ethernet/broadcom/bcm63xx_enet.c:
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 *
 * Derived from shared/opensource/include/bcm963xx/63268_map_part.h:
 * Copyright 2000-2010 Broadcom Corp.
 */

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

/* Channel configuration */
#define IUDMA_C_CHANNEL_BASE_REG(x)	(0x10 * (x))
#define IUDMA_C_CFG_REG			0x00
#define   IUDMA_C_CFG_ENABLE			BIT(0)
#define   IUDMA_C_CFG_PKT_HALT			BIT(1)	/* idle after an EOP flag is detected */
#define   IUDMA_C_CFG_BURST_HALT		BIT(2)	/* idle after finish current memory burst */
#define IUDMA_C_INT_STATUS_REG		0x04
#define IUDMA_C_INT_ENABLE_REG		0x08
#define   IUDMA_C_INT_PKT_DONE			BIT(0)	/* buffer done */
#define   IUDMA_C_INT_BUF_DONE			BIT(1)	/* transfer complete */
#define   IUDMA_C_INT_NO_DESC			BIT(2)	/* no valid descriptors */
#define   IUDMA_C_INT_RX_ERROR			BIT(3)	/* rxdma detect client protocol error */
#define IUDMA_C_MAX_BURST_REG		0x0c

/* Channel configuration (no state RAM) */
#define IUDMA_N_CHANNEL_BASE_REG(x)	(0x40 * (x))
#define IUDMA_N_CFG_REG			0x00
#define   IUDMA_N_CFG_ENABLE			BIT(0)
#define   IUDMA_N_CFG_PKT_HALT			BIT(1)	/* idle after an EOP flag is detected */
#define   IUDMA_N_CFG_BURST_HALT		BIT(2)	/* idle after finish current memory burst */
#define   IUDMA_N_CFG_CHAINING			BIT(3)
#define   IUDMA_N_CFG_WRAP_ENABLE		BIT(4)
#define   IUDMA_N_CFG_FLOWC_ENABLE		BIT(5)
#define IUDMA_N_MAX_BURST_REG		0x04
#define IUDMA_N_RSTART_REG		0x08
#define IUDMA_N_LEN_REG			0x0c
#define IUDMA_N_INT_STATUS_REG		0x14
#define IUDMA_N_INT_ENABLE_REG		0x18
#define   IUDMA_N_INT_PKT_DONE			BIT(0)	/* buffer done */
#define   IUDMA_N_INT_BUF_DONE			BIT(1)	/* transfer complete */
#define   IUDMA_N_INT_NO_DESC			BIT(2)	/* no valid descriptors */
#define   IUDMA_N_INT_RX_ERROR			BIT(3)	/* rxdma detect client protocol error */
#define IUDMA_N_FLOWC_REG		0x1c
#define IUDMA_N_BUFALLOC_REG		0x20

/* State RAM */
#define IUDMA_S_CHANNEL_BASE_REG(x)	(0x10 * (x))
#define IUDMA_S_RSTART_REG		0x00	/* descriptor ring start address */
#define IUDMA_S_STATE_DATA_REG		0x04	/* state/bytes done/ring offset */
#define IUDMA_S_DESC_LEN_STATUS_REG	0x08	/* buffer descriptor status and len */
#define IUDMA_S_DESC_BASE_BUFPTR_REG	0x0c	/* buffer descriptor current processing */

struct bcm63xx_iudma_ch {
	unsigned int id;
	void __iomem *base_c;
	void __iomem *base_s;
	int irq;
	spinlock_t lock;
};

struct bcm63xx_iudma {
	void __iomem *base;
	void __iomem *base_c;
	void __iomem *base_s;

	struct dma_device ddev;
	unsigned int n_channels;
	struct bcm63xx_iudma_ch chan[];
};

struct bcm63xx_iudma_of_data {
	unsigned int n_channels;
	unsigned int dmac_offset;
	unsigned int dmas_offset;
};

static inline bool bcm63xx_iudma_has_sram(struct bcm63xx_iudma *iudma)
{
	return iudma->base_s != NULL;
}

static inline bool bcm63xx_iudma_ch_has_sram(struct bcm63xx_iudma_ch *chan)
{
	return chan->base_s != NULL;
}

static inline void bcm63xx_iudma_int_disable(struct bcm63xx_iudma_ch *chan)
{
	if (bcm63xx_iudma_ch_has_sram(chan))
		__raw_writel(0, chan->base_c + IUDMA_C_INT_ENABLE_REG);
	else
		__raw_writel(0, chan->base_c + IUDMA_N_INT_ENABLE_REG);
}

static inline void bcm63xx_iudma_int_enable(struct bcm63xx_iudma_ch *chan)
{
	if (bcm63xx_iudma_ch_has_sram(chan))
		__raw_writel(IUDMA_C_INT_PKT_DONE,
			chan->base_c + IUDMA_C_INT_ENABLE_REG);
	else
		__raw_writel(IUDMA_N_INT_PKT_DONE | IUDMA_N_INT_BUF_DONE |
			IUDMA_C_INT_NO_DESC,
			chan->base_c + IUDMA_N_INT_ENABLE_REG);
}

static inline void bcm63xx_iudma_dma_disable(struct bcm63xx_iudma_ch *chan)
{
	if (bcm63xx_iudma_ch_has_sram(chan))
		__raw_writel(0, chan->base_c + IUDMA_C_CFG_REG);
	else
		__raw_writel(0, chan->base_c + IUDMA_N_CFG_REG);
}

static inline void bcm63xx_iudma_dma_enable(struct bcm63xx_iudma_ch *chan)
{
	if (bcm63xx_iudma_ch_has_sram(chan))
		__raw_writel(IUDMA_C_CFG_ENABLE,
			chan->base_c + IUDMA_C_CFG_REG);
	else
		__raw_writel(IUDMA_N_CFG_ENABLE | IUDMA_N_CFG_CHAINING |
			IUDMA_N_CFG_WRAP_ENABLE | IUDMA_N_CFG_FLOWC_ENABLE,
			chan->base_c + IUDMA_N_CFG_REG);
}

static int bcm63xx_iudma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm63xx_iudma *iudma;
	const struct bcm63xx_iudma_of_data *of_data;
	struct resource *res;
	bool has_sram;
	unsigned int i;

	of_data = of_device_get_match_data(dev);
	if (!of_data)
		return -EINVAL;

	iudma = devm_kzalloc(dev,
		sizeof(*iudma) + sizeof(*iudma->chan) * of_data->n_channels,
		GFP_KERNEL);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iudma->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(iudma->base))
		return PTR_ERR(iudma->base);

	iudma->base_c = iudma->base + of_data->dmac_offset;
	if (of_data->dmas_offset)
		iudma->base_s = iudma->base + of_data->dmas_offset;

	has_sram = bcm63xx_iudma_has_sram(iudma);

	for (i = 0; i < of_data->n_channels; i++) {
		struct bcm63xx_iudma_ch *ch = &iudma->chan[i];

		ch->irq = platform_get_irq(pdev, i);
		if (ch->irq >= 0) {
			spin_lock_init(&ch->lock);

			if (has_sram) {
				ch->base_c = iudma->base_c +
					IUDMA_C_CHANNEL_BASE_REG(i);
				ch->base_s = iudma->base_s +
					IUDMA_S_CHANNEL_BASE_REG(i);
			} else {
				ch->base_c = iudma->base_c +
					IUDMA_N_CHANNEL_BASE_REG(i);
				ch->base_s = 0;
			}

			bcm63xx_iudma_int_disable(ch);
			bcm63xx_iudma_dma_disable(ch);

			iudma->n_channels++;
		} else
			break;
	}

	if (!iudma->n_channels) {
		dev_err(dev, "device has no channel IRQs\n");
		return -EINVAL;
	}

	dev_info(dev, "registered BCM63xx IUDMA controller at MMIO 0x%p (%u channels)",
			iudma->base, iudma->n_channels);
	return 0;
}

static int bcm63xx_iudma_remove(struct platform_device *pdev)
{
	struct bcm63xx_iudma *iudma = platform_get_drvdata(pdev);

	dma_async_device_unregister(&iudma->ddev);

	return 0;
}

static const struct bcm63xx_iudma_of_data bcm63xx_iudma_8ch = {
	.n_channels = 8,
	.dmac_offset = 0x40,
	.dmas_offset = 0,
};

static const struct bcm63xx_iudma_of_data bcm63xx_iudma_16ch = {
	.n_channels = 16,
	.dmac_offset = 0x100,
	.dmas_offset = 0x200,
};

static const struct bcm63xx_iudma_of_data bcm63xx_iudma_32ch = {
	.n_channels = 32,
	.dmac_offset = 0x200,
	.dmas_offset = 0x400,
};

static const struct of_device_id bcm63xx_iudma_of_match[] = {
	{ .compatible = "brcm,bcm6345-iudma", .data = &bcm63xx_iudma_8ch },

	{ .compatible = "brcm,bcm3368-iudma", .data = &bcm63xx_iudma_16ch },
	{ .compatible = "brcm,bcm6338-iudma", .data = &bcm63xx_iudma_16ch },
	{ .compatible = "brcm,bcm6348-iudma", .data = &bcm63xx_iudma_16ch },
	{ .compatible = "brcm,bcm6358-iudma", .data = &bcm63xx_iudma_16ch },

	{ .compatible = "brcm,bcm6318-iudma", .data = &bcm63xx_iudma_32ch },
	{ .compatible = "brcm,bcm6328-iudma", .data = &bcm63xx_iudma_32ch },
	{ .compatible = "brcm,bcm6368-iudma", .data = &bcm63xx_iudma_32ch },
	{ .compatible = "brcm,bcm63168-iudma", .data = &bcm63xx_iudma_32ch },
	{ .compatible = "brcm,bcm63268-iudma", .data = &bcm63xx_iudma_32ch },
	{ .compatible = "brcm,bcm6816-iudma", .data = &bcm63xx_iudma_32ch },
	{ .compatible = "brcm,bcm68220-iudma", .data = &bcm63xx_iudma_32ch },

	{},
};
MODULE_DEVICE_TABLE(of, bcm2835_dma_of_match);

static struct platform_driver bcm63xx_iudma_driver = {
	.probe  = bcm63xx_iudma_probe,
	.remove = bcm63xx_iudma_remove,
	.driver = {
		.name = "bcm63xx-iudma",
		.of_match_table = of_match_ptr(bcm63xx_iudma_of_match),
	},
};

module_platform_driver(bcm63xx_iudma_driver);

MODULE_ALIAS("platform:bcm63xx-iudma");
MODULE_DESCRIPTION("BCM63xx IUDMA engine driver");
MODULE_AUTHOR("Simon Arlott");
MODULE_LICENSE("GPL");

