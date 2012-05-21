/*
 * Copyright (C) 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * TODO: describe Lite differences
 */

#include <linux/bitops.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/sizes.h>
#include "dmaengine.h"

#define MODULE_NAME "bcm2708_dma"

#define CACHE_LINE_MASK		31
#define MAX_CHANS		16

#define REG_CS			0x00	/* Control and Status */
#define REG_CONBLK_AD		0x04	/* Control Block Address */
#define REG_CB			0x08	/* Control Block */

#define REG_DEBUG		0x20
#define BCM_DEBUG_RLSN_ERR(n)	(n & BIT(0))		/* Read Last Not Set Error */
#define BCM_DEBUG_FIFO_ERR(n)	(n & BIT(1))
#define BCM_DEBUG_READ_ERR(n)	(n & BIT(2))
#define BCM_DEBUG_OUT_WR(n)	((n >> 4) & 0xF)	/* Outstanding Writes Count */
#define BCM_DEBUG_DMA_ID(n)	((n >> 8) & 0xFF)	/* DMA AXI ID */
#define BCM_DEBUG_DMA_STATE(n)	((n >> 16) & 0x1FF)	/* State Machine State */
#define BCM_DEBUG_VERSION(n)	((n >> 25) & 0x7)
#define BCM_DEBUG_LITE(n)	(n & BIT(28))

struct bcm2708_dmachan {
	void __iomem *base;
	int irq;
	bool in_use;

	int id;
	int axi_id;
	bool lite;
	int version;

	struct dma_chan dmachan;
};

struct bcm2708_dmadev {
	struct device *dev;
	struct spinlock lock;

	struct resource res[2];
	void __iomem *base[2];

	struct dma_device dmadev;
};

static inline struct bcm2708_dmadev *to_bcmdev(struct dma_device *dmadev)
{
	return container_of(dmadev, struct bcm2708_dmadev, dmadev);
}

static inline struct bcm2708_dmachan *to_bcmchan(struct dma_chan *dmachan)
{
	return container_of(dmachan, struct bcm2708_dmachan, dmachan);
}

static int bcm2708_dma_alloc_chan(struct dma_chan *dmachan)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	if (bcmchan->in_use)
		return -EBUSY;

	bcmchan->in_use = true;
	return 0;
}

static void bcm2708_dma_free_chan(struct dma_chan *dmachan)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	bcmchan->in_use = false;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_interrupt(
	struct dma_chan *dmachan, unsigned long flags)
{
	return ERR_PTR(-ENOSYS);
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_sg(
	struct dma_chan *dmachan,
	struct scatterlist *dst_sg, unsigned int dst_nents,
	struct scatterlist *src_sg, unsigned int src_nents,
	unsigned long flags)
{
	return ERR_PTR(-ENOSYS);
}

static int bcm2708_dma_control(struct dma_chan *dmachan,
	enum dma_ctrl_cmd cmd, unsigned long arg)
{
	return -ENOSYS;
}

static enum dma_status bcm2708_dma_tx_status(struct dma_chan *dmachan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	return DMA_ERROR;
}

static void bcm2708_dma_issue_pending(struct dma_chan *dmachan)
{
	WARN_ON(1);
}

static irqreturn_t bcm2708_dma_irq_handler(int irq, void *dev_id)
{
	struct dma_chan *dmachan = dev_id;
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	struct bcm2708_dmadev *bcmdev = to_bcmdev(dmachan->device);

	dev_dbg(bcmdev->dev, "irq for %d\n", bcmchan->id);
	return IRQ_NONE;
}

static int bcm2708_dma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct bcm2708_dmadev *bcmdev = devm_kzalloc(&pdev->dev,
		sizeof(*bcmdev), GFP_KERNEL);
	struct dma_device *dmadev = &bcmdev->dmadev;
	struct bcm2708_dmachan *bcmchan;
	struct dma_chan *dmachan;
	int ret, i;

	if (bcmdev == NULL)
		return -ENOMEM;

	bcmdev->dev = &pdev->dev;
	spin_lock_init(&bcmdev->lock);

	for (i = 0; i < 2; i++) {
		if (of_address_to_resource(np, i, &bcmdev->res[i])) {
			dev_err(bcmdev->dev, "missing resource %d\n", i);
			return -EINVAL;
		}

		if (resource_size(&bcmdev->res[i]) < SZ_4K) {
			dev_err(bcmdev->dev,
				"resource %d size too small (%#x)\n",
				i, resource_size(&bcmdev->res[i]));
			return -EINVAL;
		}

		bcmdev->base[i] = devm_request_and_ioremap(bcmdev->dev,
			&bcmdev->res[i]);
		if (!bcmdev->base[i]) {
			dev_err(bcmdev->dev, "error mapping io at %#lx\n",
				(unsigned long)bcmdev->res[i].start);
			return -EIO;
		}
	}

	dmadev->dev = bcmdev->dev;
	dmadev->chancnt = 0;
	INIT_LIST_HEAD(&dmadev->channels);
	for (i = 0; i < MAX_CHANS; i++) {
		u32 debug;
		char *tmp;
		int len;

		bcmchan = devm_kzalloc(bcmdev->dev,
			sizeof(*bcmchan), GFP_KERNEL);
		if (bcmchan == NULL)
			return -ENOMEM;
		dmachan = &bcmchan->dmachan;
		bcmchan->id = i;

		/* valid only for channels 0 - 14
		 * 15 has its own base address
		 */
		if (bcmchan->id == 15)
			bcmchan->base = bcmdev->base[1];
		else
			bcmchan->base = bcmdev->base[0] + i * SZ_256;

		debug = readl(bcmchan->base + REG_DEBUG);
		bcmchan->axi_id = BCM_DEBUG_DMA_ID(debug);
		bcmchan->version = BCM_DEBUG_VERSION(debug);
		bcmchan->lite = BCM_DEBUG_LITE(debug);

		bcmchan->irq = irq_of_parse_and_map(np, i);
		if (!bcmchan->irq) {
			devm_kfree(bcmdev->dev, bcmchan);
			continue;
		}

		len = strlen(dev_name(bcmdev->dev)) + 16;
		tmp = devm_kzalloc(bcmdev->dev, len, GFP_KERNEL);
		if (!tmp) {
			devm_kfree(bcmdev->dev, bcmchan);
			continue;
		}
		snprintf(tmp, len, "%s:chan%d", dev_name(dmadev->dev), i);

		ret = devm_request_irq(bcmdev->dev, bcmchan->irq,
			bcm2708_dma_irq_handler, 0, tmp, dmachan);
		if (ret) {
			devm_kfree(bcmdev->dev, tmp);
			devm_kfree(bcmdev->dev, bcmchan);
			continue;
		}

		dma_cookie_init(dmachan);
		dmachan->device = dmadev;

		list_add_tail(&dmachan->device_node, &dmadev->channels);
		dmadev->chancnt++;
	}

	if (dmadev->chancnt == 0) {
		dev_err(bcmdev->dev, "no usable channels\n");
		return -ENXIO;
	}

	dma_cap_set(DMA_INTERRUPT, dmadev->cap_mask);
	dma_cap_set(DMA_SG, dmadev->cap_mask);

	dmadev->max_xor = 0;
	dmadev->max_pq = 0;
	dmadev->copy_align = 32;
	dmadev->xor_align = 32;
	dmadev->pq_align = 32;
	dmadev->fill_align = 32;

	dmadev->device_alloc_chan_resources = bcm2708_dma_alloc_chan;
	dmadev->device_free_chan_resources = bcm2708_dma_free_chan;
	dmadev->device_prep_dma_interrupt = bcm2708_dma_prep_interrupt;
	dmadev->device_prep_dma_sg = bcm2708_dma_prep_sg;
	dmadev->device_control = bcm2708_dma_control;
	dmadev->device_tx_status = bcm2708_dma_tx_status;
	dmadev->device_issue_pending = bcm2708_dma_issue_pending;

	ret = dma_async_device_register(dmadev);
	if (ret) {
		dev_err(bcmdev->dev, "unable to register device (%d)\n", ret);
		return ret;
	}

	dev_info(bcmdev->dev, "%d channel%s at MMIO %#lx, %#lx\n",
		dmadev->chancnt, dmadev->chancnt != 1 ? "s" : "",
		(unsigned long)bcmdev->res[0].start,
		(unsigned long)bcmdev->res[1].start);

	list_for_each_entry(dmachan, &dmadev->channels, device_node) {
		bcmchan = to_bcmchan(dmachan);
		dev_dbg(bcmdev->dev, "channel %d, (irq = %d) 0x%02x v%d%s\n",
			bcmchan->id, bcmchan->irq, bcmchan->axi_id,
			bcmchan->version, bcmchan->lite ? " [lite]" : "");
	}

	platform_set_drvdata(pdev, bcmdev);
	return 0;
}

static int bcm2708_dma_remove(struct platform_device *pdev)
{
	struct bcm2708_dmadev *bcmdev = platform_get_drvdata(pdev);

	dma_async_device_unregister(&bcmdev->dmadev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct of_device_id bcm2708_dma_match[] __devinitconst = {
	{ .compatible = "broadcom,bcm2708-dma" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm2708_dma_match);

static struct platform_driver bcm2708_dma_driver = {
	.probe	= bcm2708_dma_probe,
	.remove	= bcm2708_dma_remove,
	.driver	= {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm2708_dma_match
	}
};
module_platform_driver(bcm2708_dma_driver);

MODULE_AUTHOR("Gray Girling, Simon Arlott");
MODULE_DESCRIPTION("Broadcom BCM2708 DMA channel manager driver");
MODULE_LICENSE("GPL");
