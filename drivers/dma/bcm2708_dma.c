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

enum dmadev_idx {
	D_FULL,
	D_LITE,
	D_MAX
};

struct bcm2708_dmachan {
	struct device *dev;
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

	/* Channels 0-14, 15 */
	struct resource res[2];
	void __iomem *base[2];

	/* Full/lite devices */
	struct dma_device dmadev[2];
};

static inline struct bcm2708_dmachan *to_bcmchan(struct dma_chan *dmachan)
{
	return container_of(dmachan, struct bcm2708_dmachan, dmachan);
}

static int bcm2708_dma_alloc_chan(struct dma_chan *dmachan)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d\n", __func__, bcmchan->id);

	if (bcmchan->in_use)
		return -EBUSY;

	bcmchan->in_use = true;
	return 0;
}

static void bcm2708_dma_free_chan(struct dma_chan *dmachan)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d\n", __func__, bcmchan->id);

	bcmchan->in_use = false;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_memcpy(
	struct dma_chan *dmachan, dma_addr_t dst, dma_addr_t src,
	size_t len, unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d: %08x(+%d)=>%08x [%lu]\n", __func__,
		bcmchan->id, src, len, dst, flags);

	return NULL;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_interrupt(
	struct dma_chan *dmachan, unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d: %lu\n", __func__,
		bcmchan->id, flags);

	return NULL;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_dma_sg(
	struct dma_chan *dmachan,
	struct scatterlist *dst_sg, unsigned int dst_nents,
	struct scatterlist *src_sg, unsigned int src_nents,
	unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d: %d=>%d [%lu]\n", __func__,
		bcmchan->id, src_nents, dst_nents, flags);

	return NULL;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_slave_sg(
	struct dma_chan *dmachan, struct scatterlist *sgl,
	unsigned int sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d: %p+%d [%d,%lu] %p\n", __func__,
		bcmchan->id, sgl, sg_len, direction, flags, context);

	return NULL;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_cyclic(
	struct dma_chan *dmachan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	void *context)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d: %08x+%d [%d,%d] %p\n", __func__,
		bcmchan->id, buf_addr, buf_len, period_len, direction, context);

	return NULL;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_interleaved(
	struct dma_chan *dmachan, struct dma_interleaved_template *xt,
	unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);	

	dev_dbg(bcmchan->dev, "%s: %d: %p [%lu]\n", __func__,
		bcmchan->id, xt, flags);

	return NULL;
}

static int bcm2708_dma_control(struct dma_chan *dmachan,
		enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d: %d %lu\n", __func__,
		bcmchan->id, cmd, arg);

	return -ENOSYS;
}

static enum dma_status bcm2708_dma_tx_status(struct dma_chan *dmachan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d\n", __func__, bcmchan->id);

	return DMA_ERROR;
}

static void bcm2708_dma_issue_pending(struct dma_chan *dmachan)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "%s: %d\n", __func__, bcmchan->id);

	WARN_ON(1);
}

static irqreturn_t bcm2708_dma_irq_handler(int irq, void *dev_id)
{
	struct dma_chan *dmachan = dev_id;
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_dbg(bcmchan->dev, "irq for %d\n", bcmchan->id);
	return IRQ_NONE;
}

static int bcm2708_dma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct bcm2708_dmadev *bcmdev = devm_kzalloc(&pdev->dev,
		sizeof(*bcmdev), GFP_KERNEL);
	struct dma_device *dmadev = bcmdev->dmadev;
	struct bcm2708_dmachan *bcmchan;
	struct dma_chan *dmachan;
	unsigned int nr_chans;
	int ret, i;

	if (bcmdev == NULL)
		return -ENOMEM;
	bcmdev->dev = &pdev->dev;

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

	for (i = 0; i < D_MAX; i++) {
		dmadev[i].dev = bcmdev->dev;
		dmadev[i].chancnt = 0;
		INIT_LIST_HEAD(&dmadev[i].channels);
	}

	for (i = 0; i < MAX_CHANS; i++) {
		u32 debug;
		char *tmp;
		int type, len;

		bcmchan = devm_kzalloc(bcmdev->dev,
			sizeof(*bcmchan), GFP_KERNEL);
		if (bcmchan == NULL)
			return -ENOMEM;
		dmachan = &bcmchan->dmachan;
		bcmchan->dev = bcmdev->dev;
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
		snprintf(tmp, len, "%s:chan%d", dev_name(bcmdev->dev), i);

		ret = devm_request_irq(bcmdev->dev, bcmchan->irq,
			bcm2708_dma_irq_handler, 0, tmp, dmachan);
		if (ret) {
			devm_kfree(bcmdev->dev, tmp);
			devm_kfree(bcmdev->dev, bcmchan);
			continue;
		}

		dma_cookie_init(dmachan);
		type = bcmchan->lite ? D_LITE : D_FULL;
		dmachan->device = &dmadev[type];

		list_add_tail(&dmachan->device_node, &dmadev[type].channels);
		dmadev[type].chancnt++;
	}

	nr_chans = dmadev[D_FULL].chancnt + dmadev[D_LITE].chancnt;
	if (nr_chans == 0) {
		dev_err(bcmdev->dev, "no usable channels\n");
		return -ENXIO;
	}

	dma_cap_set(DMA_MEMCPY, dmadev[D_FULL].cap_mask);
	dma_cap_set(DMA_INTERRUPT, dmadev[D_FULL].cap_mask);
	dma_cap_set(DMA_SLAVE, dmadev[D_FULL].cap_mask);
	dma_cap_set(DMA_SG, dmadev[D_FULL].cap_mask);
	dma_cap_set(DMA_CYCLIC, dmadev[D_FULL].cap_mask);
	dma_cap_set(DMA_INTERLEAVE, dmadev[D_FULL].cap_mask);

	dma_cap_set(DMA_MEMCPY, dmadev[D_LITE].cap_mask);
	dma_cap_set(DMA_INTERRUPT, dmadev[D_LITE].cap_mask);
	dma_cap_set(DMA_SLAVE, dmadev[D_LITE].cap_mask);
	dma_cap_set(DMA_SG, dmadev[D_LITE].cap_mask);
	dma_cap_set(DMA_CYCLIC, dmadev[D_LITE].cap_mask);

	for (i = 0; i < D_MAX; i++) {
		dmadev[i].copy_align = 0;

		dmadev[i].device_alloc_chan_resources = bcm2708_dma_alloc_chan;
		dmadev[i].device_free_chan_resources = bcm2708_dma_free_chan;

		dmadev[i].device_prep_dma_memcpy = bcm2708_dma_prep_memcpy;
		dmadev[i].device_prep_dma_interrupt = bcm2708_dma_prep_interrupt;
		dmadev[i].device_prep_dma_sg = bcm2708_dma_prep_dma_sg;
		dmadev[i].device_prep_slave_sg = bcm2708_dma_prep_slave_sg;
		dmadev[i].device_prep_dma_cyclic = bcm2708_dma_prep_cyclic;
		if (i != D_LITE)
			dmadev[i].device_prep_interleaved_dma = bcm2708_dma_prep_interleaved;
		dmadev[i].device_control = bcm2708_dma_control;

		dmadev[i].device_tx_status = bcm2708_dma_tx_status;
		dmadev[i].device_issue_pending = bcm2708_dma_issue_pending;

		ret = dma_async_device_register(&dmadev[i]);
		if (ret) {
			dev_err(bcmdev->dev,
				"unable to register device %d (%d)\n", i, ret);
			while (--i >= 0)
				dma_async_device_unregister(&bcmdev->dmadev[i]);
			return ret;
		}
	}

	dev_info(bcmdev->dev, "%u channel%s at MMIO %#lx, %#lx\n",
		nr_chans, nr_chans != 1 ? "s" : "",
		(unsigned long)bcmdev->res[0].start,
		(unsigned long)bcmdev->res[1].start);

	for (i = 0; i < D_MAX; i++) {
		list_for_each_entry(dmachan,
				&dmadev[i].channels, device_node) {
			bcmchan = to_bcmchan(dmachan);
			dev_dbg(bcmdev->dev,
				"channel %d, (irq = %d) 0x%02x v%d%s\n",
				bcmchan->id, bcmchan->irq,
				bcmchan->axi_id, bcmchan->version,
				bcmchan->lite ? " [lite]" : "");
		}
	}

	platform_set_drvdata(pdev, bcmdev);
	return 0;
}

static int bcm2708_dma_remove(struct platform_device *pdev)
{
	struct bcm2708_dmadev *bcmdev = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < D_MAX; i++)
		dma_async_device_unregister(&bcmdev->dmadev[i]);
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
