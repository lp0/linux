/*
 * Copyright 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * CB creation loops copied from fsldma.c (Freescale Semiconductor, Inc.)
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
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
#include "bcm2708_dma.h"

#define MODULE_NAME "bcm2708_dma"

static inline struct bcm2708_dmachan *to_bcmchan(struct dma_chan *dmachan)
{
	return container_of(dmachan, struct bcm2708_dmachan, dmachan);
}

static inline struct bcm2708_dmatx *to_bcmtx(
	struct dma_async_tx_descriptor *dmatx)
{
	return container_of(dmatx, struct bcm2708_dmatx, dmatx);
}

static int bcm2708_dma_alloc_chan(struct dma_chan *dmachan)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	unsigned long flags;

	dev_vdbg(bcmchan->dev, "%s: %d\n", __func__, bcmchan->id);

	spin_lock_irqsave(&bcmchan->lock, flags);
	bcmchan->cfg = 0;
	bcmchan->slave_cfg_from = 0;
	bcmchan->slave_cfg_to = 0;
	bcmchan->slave_addr_from = 0;
	bcmchan->slave_addr_to = 0;
	spin_unlock_irqrestore(&bcmchan->lock, flags);
	return 0;
}

static void bcm2708_dma_free_tx(struct bcm2708_dmatx *bcmtx)
{
	int i;

	for (i = 0; i < bcmtx->count; i++)
		dma_pool_free(bcmtx->chan->pool, bcmtx->desc[i].cb,
				bcmtx->desc[i].phys);
	if (bcmtx->memset_value)
		dma_pool_free(bcmtx->chan->pool, bcmtx->memset_value,
				bcmtx->memset_phys);
	kfree(bcmtx);
}

static void bcm2708_dma_delete(struct list_head *list)
{
	struct bcm2708_dmatx *bcmtx;
	struct bcm2708_dmatx *tmp;

	list_for_each_entry_safe(bcmtx, tmp, list, list) {
		list_del(&bcmtx->list);
		bcm2708_dma_free_tx(bcmtx);
	}
}

static void bcm2708_dma_free_chan(struct dma_chan *dmachan)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	unsigned long flags;

	dev_vdbg(bcmchan->dev, "%s: %d\n", __func__, bcmchan->id);

	mutex_lock(&bcmchan->prep_lock);
	bcm2708_dma_delete(&bcmchan->prep_list);
	mutex_unlock(&bcmchan->prep_lock);

	spin_lock_irqsave(&bcmchan->lock, flags);
	bcm2708_dma_delete(&bcmchan->pending);

	if (unlikely(bcmchan->active || bcmchan->paused)) {
		WARN_ON(1);

		writel(0, bcmchan->base + REG_CS);
		bcmchan->paused = true;
		spin_unlock_irqrestore(&bcmchan->lock, flags);

		synchronize_irq(bcmchan->irq);

		spin_lock_irqsave(&bcmchan->lock, flags);
		writel(0, bcmchan->base + REG_CONBLK_AD);
		writel(BCM_CS_ABORT, bcmchan->base + REG_CS);
		writel(0, bcmchan->base + REG_CONBLK_AD);
		bcmchan->active = false;
		bcmchan->paused = false;
	}

	bcm2708_dma_delete(&bcmchan->running);
	bcm2708_dma_delete(&bcmchan->completed);
	spin_unlock_irqrestore(&bcmchan->lock, flags);
}

static inline void bcm2708_dma_chain_cb(struct bcm2708_dmatx *prev,
	struct bcm2708_dmatx *next)
{
	BUG_ON(prev->desc[prev->count - 1].cb->next);
	prev->desc[prev->count - 1].cb->next = next->dmatx.phys;
}

static dma_cookie_t bcm2708_dma_submit(struct dma_async_tx_descriptor *dmatx)
{
	struct bcm2708_dmatx *bcmtx = to_bcmtx(dmatx);
	struct bcm2708_dmachan *bcmchan = bcmtx->chan;
	unsigned long flags;

	mutex_lock(&bcmchan->prep_lock);
	list_del(&bcmtx->list);
	mutex_unlock(&bcmchan->prep_lock);
	
	spin_lock_irqsave(&bcmchan->lock, flags);
	if (!list_empty(&bcmchan->pending))
		bcm2708_dma_chain_cb(list_last_entry(&bcmchan->pending,
				struct bcm2708_dmatx, list), bcmtx);
	list_add_tail(&bcmtx->list, &bcmchan->pending);
	dma_cookie_assign(&bcmtx->dmatx);
	spin_unlock_irqrestore(&bcmchan->lock, flags);

	dev_vdbg(bcmchan->dev, "%s: %d: %08x\n", __func__, bcmchan->id,
		bcmtx->dmatx.cookie);
	return bcmtx->dmatx.cookie;
}

static struct bcm2708_dmatx *bcm2708_dma_alloc_cb(struct bcm2708_dmatx *bcmtx,
	int count)
{
	int i;

	for (i = bcmtx->count; i < count; i++) {
		bcmtx->desc[i].cb = dma_pool_alloc(bcmtx->chan->pool,
				GFP_KERNEL, &bcmtx->desc[i].phys);
		if (unlikely(!bcmtx->desc[i].cb)) {
			bcm2708_dma_free_tx(bcmtx);
			return NULL;
		}
		if (i > 0)
			bcmtx->desc[i - 1].cb->next = bcmtx->desc[i].phys;
		bcmtx->desc[i].cb->pad = 0;
		bcmtx->count++;
	}
	bcmtx->desc[bcmtx->count - 1].cb->next = 0;
	return bcmtx;
}

static struct bcm2708_dmatx *bcm2708_dma_alloc_tx(
	struct bcm2708_dmachan *bcmchan, unsigned long flags, int count)
{
	struct bcm2708_dmatx *bcmtx;

	if (unlikely(count <= 0))
		return NULL;

	bcmtx = kzalloc(sizeof(*bcmtx) + count * sizeof(bcmtx->desc[0]),
			GFP_KERNEL);
	if (unlikely(!bcmtx))
		return NULL;

	bcmtx->chan = bcmchan;
	bcmtx->count = 0;
	bcmtx = bcm2708_dma_alloc_cb(bcmtx, count);
	if (unlikely(!bcmtx))
		return NULL;

	/**
	 * Ignored flags:
	 *   DMA_PREP_INTERRUPT	(this is always done)
	 *   DMA_CTRL_ACK	(descriptors aren't reused)
	 *   DMA_PREP_CONTINUE	(operations are executed in order)
	 *   DMA_PREP_FENCE	(operations are executed in order)
	 *
	 * Handled flags:
	 *   DMA_COMPL_SKIP_SRC_UNMAP
	 *   DMA_COMPL_SKIP_DEST_UNMAP
	 *   DMA_COMPL_SRC_UNMAP_SINGLE
	 *   DMA_COMPL_DEST_UNMAP_SINGLE
	 */

	bcmtx->dmatx.cookie = -EBUSY;
	bcmtx->dmatx.flags = flags;
	bcmtx->dmatx.chan = &bcmchan->dmachan;
	bcmtx->dmatx.phys = bcmtx->desc[0].phys;
	bcmtx->dmatx.tx_submit = bcm2708_dma_submit;
	return bcmtx;
}

static struct bcm2708_dmatx *bcm2708_dma_realloc_tx(struct bcm2708_dmatx *bcmtx,
	int count)
{
	struct bcm2708_dmatx *tmp;

	BUG_ON(unlikely(count <= 0));
	if (unlikely(count <= bcmtx->count)) {
		while (bcmtx->count > count) {
			bcmtx->count--;
			dma_pool_free(bcmtx->chan->pool,
					bcmtx->desc[bcmtx->count].cb,
					bcmtx->desc[bcmtx->count].phys);
		}
	}

	tmp = krealloc(bcmtx, sizeof(*bcmtx) + count * sizeof(bcmtx->desc[0]),
			GFP_KERNEL);
	if (unlikely(!tmp)) {
		bcm2708_dma_free_tx(bcmtx);
		return NULL;
	}
	return bcm2708_dma_alloc_cb(tmp, count);
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_memcpy(
	struct dma_chan *dmachan, dma_addr_t dst, dma_addr_t src,
	size_t len, unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	struct bcm2708_dmatx *bcmtx;
	size_t copy;
	int i = -1;

	dev_vdbg(bcmchan->dev, "%s: %d: %08x(+%d)=>%08x [%lu]\n", __func__,
		bcmchan->id, src, len, dst, flags);

	if (unlikely(len < 0))
		return NULL;

	bcmtx = bcm2708_dma_alloc_tx(bcmchan, flags, 1);
	if (unlikely(!bcmtx))
		return NULL;

	do {
		copy = min(len, MAX_LEN(bcmchan));

		bcmtx = bcm2708_dma_realloc_tx(bcmtx, ++i + 1);
		if (unlikely(!bcmtx))
			return NULL;

		bcmtx->desc[i].cb->ti = BCM_TI_DST_INC | BCM_TI_SRC_INC
			| BCM_TI_BURST_CHAN(bcmchan) | bcmchan->cfg;
		bcmtx->desc[i].cb->src = src;
		bcmtx->desc[i].cb->dst = dst;
		bcmtx->desc[i].cb->len = copy;
		bcmtx->desc[i].cb->stride = 0;

		len -= copy;
		src += copy;
		dst += copy;
	} while (len);

	bcmtx->desc[i].cb->ti |= BCM_TI_INTEN | BCM_TI_WAIT_RESP;

	mutex_lock(&bcmchan->prep_lock);
	list_add_tail(&bcmtx->list, &bcmchan->prep_list);
	mutex_unlock(&bcmchan->prep_lock);
	return &bcmtx->dmatx;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_memset(
	struct dma_chan *dmachan, dma_addr_t dst, int value, size_t len,
	unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	struct bcm2708_dmatx *bcmtx;
	size_t copy;
	int i = -1;

	dev_vdbg(bcmchan->dev, "%s: %d: %08x=>%08x(+%d) [%lu]\n", __func__,
		bcmchan->id, value, len, dst, flags);

	if (unlikely(len < 0))
		return NULL;

	bcmtx = bcm2708_dma_alloc_tx(bcmchan, flags, 1);
	if (unlikely(!bcmtx))
		return NULL;

	/* this wastes 4 bytes but avoids using a second pool */
	bcmtx->memset_value = dma_pool_alloc(bcmtx->chan->pool,
				GFP_KERNEL, &bcmtx->memset_phys);
	if (unlikely(!bcmtx->memset_value)) {
		bcm2708_dma_free_tx(bcmtx);
		return NULL;
	}

	do {
		copy = min(len, MAX_LEN(bcmchan));

		bcmtx = bcm2708_dma_realloc_tx(bcmtx, ++i + 1);
		if (unlikely(!bcmtx))
			return NULL;

		bcmtx->desc[i].cb->ti = BCM_TI_DST_INC
			| BCM_TI_BURST_CHAN(bcmchan) | bcmchan->cfg;
		bcmtx->desc[i].cb->src = bcmtx->memset_phys;
		bcmtx->desc[i].cb->dst = dst;
		bcmtx->desc[i].cb->len = copy;
		bcmtx->desc[i].cb->stride = 0;

		len -= copy;
		dst += copy;
	} while (len);

	bcmtx->desc[i].cb->ti |= BCM_TI_INTEN | BCM_TI_WAIT_RESP;
	*bcmtx->memset_value = value;

	mutex_lock(&bcmchan->prep_lock);
	list_add_tail(&bcmtx->list, &bcmchan->prep_list);
	mutex_unlock(&bcmchan->prep_lock);
	return &bcmtx->dmatx;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_interrupt(
	struct dma_chan *dmachan, unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	struct bcm2708_dmatx *bcmtx;

	dev_vdbg(bcmchan->dev, "%s: %d: %lu\n", __func__,
		bcmchan->id, flags);

	bcmtx = bcm2708_dma_alloc_tx(bcmchan, flags, 1);
	if (unlikely(!bcmtx))
		return NULL;

	bcmtx->desc[0].cb->ti = BCM_TI_INTEN | BCM_TI_SRC_IGNORE
		| BCM_TI_DST_IGNORE;
	bcmtx->desc[0].cb->src = 0;
	bcmtx->desc[0].cb->dst = 0;
	bcmtx->desc[0].cb->len = 0;
	bcmtx->desc[0].cb->stride = 0;

	mutex_lock(&bcmchan->prep_lock);
	list_add_tail(&bcmtx->list, &bcmchan->prep_list);
	mutex_unlock(&bcmchan->prep_lock);
	return &bcmtx->dmatx;
}

static struct dma_async_tx_descriptor *__bcm2708_dma_prep_dma_sg(
	struct dma_chan *dmachan,
	struct scatterlist *dst_sg, unsigned int dst_nents,
	struct scatterlist *src_sg, unsigned int src_nents,
	unsigned long flags, u32 cfg)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	struct bcm2708_dmatx *bcmtx;
	int max_count = max(src_nents, dst_nents);
	size_t src_avail, dst_avail;
	dma_addr_t src, dst;
	size_t len;
	int i = -1;

	dev_vdbg(bcmchan->dev, "%s: %d: %d=>%d [%lu]\n", __func__,
		bcmchan->id, src_nents, dst_nents, flags);

	if (unlikely(src_sg == NULL || dst_sg == NULL || max_count <= 0
			|| src_nents <= 0 || dst_nents <= 0))
		return NULL;

	bcmtx = bcm2708_dma_alloc_tx(bcmchan, flags, max_count);
	if (unlikely(!bcmtx))
		return NULL;

	src_avail = sg_dma_len(src_sg);
	dst_avail = sg_dma_len(dst_sg);

	while (true) {
		/* create the largest transaction possible */
		len = min_t(size_t, src_avail, dst_avail);
		len = min_t(size_t, len, MAX_LEN(bcmchan));
		if (len == 0)
			goto fetch;

		src = sg_dma_address(src_sg) + sg_dma_len(src_sg) - src_avail;
		dst = sg_dma_address(dst_sg) + sg_dma_len(dst_sg) - dst_avail;

		/* allocate and populate the descriptor */
		bcmtx = bcm2708_dma_realloc_tx(bcmtx, ++i + 1);
		if (unlikely(!bcmtx))
			return NULL;

		bcmtx->desc[i].cb->ti = BCM_TI_DST_INC | BCM_TI_SRC_INC
			| BCM_TI_BURST_CHAN(bcmchan) | cfg;
		bcmtx->desc[i].cb->src = src;
		bcmtx->desc[i].cb->dst = dst;
		bcmtx->desc[i].cb->len = len;
		bcmtx->desc[i].cb->stride = 0;

		/* update metadata */
		src_avail -= len;
		dst_avail -= len;

fetch:
		/* fetch the next dst scatterlist entry */
		if (dst_avail == 0) {
			/* no more entries: we're done */
			if (dst_nents == 0)
				break;

			/* fetch the next entry: if there are no more: done */
			dst_sg = sg_next(dst_sg);
			if (dst_sg == NULL)
				break;

			dst_nents--;
			dst_avail = sg_dma_len(dst_sg);
		}

		/* fetch the next src scatterlist entry */
		if (src_avail == 0) {
			/* no more entries: we're done */
			if (src_nents == 0)
				break;

			/* fetch the next entry: if there are no more: done */
			src_sg = sg_next(src_sg);
			if (src_sg == NULL)
				break;

			src_nents--;
			src_avail = sg_dma_len(src_sg);
		}
	}

	bcmtx->desc[i].cb->ti |= BCM_TI_INTEN | BCM_TI_WAIT_RESP;

	mutex_lock(&bcmchan->prep_lock);
	list_add_tail(&bcmtx->list, &bcmchan->prep_list);
	mutex_unlock(&bcmchan->prep_lock);
	return &bcmtx->dmatx;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_dma_sg(
	struct dma_chan *dmachan,
	struct scatterlist *dst_sg, unsigned int dst_nents,
	struct scatterlist *src_sg, unsigned int src_nents,
	unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	return __bcm2708_dma_prep_dma_sg(dmachan,
		dst_sg, dst_nents,
		src_sg, src_nents,
		flags, bcmchan->cfg);
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_slave_sg(
	struct dma_chan *dmachan, struct scatterlist *sgl,
	unsigned int sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	struct scatterlist dev_sg;
	size_t len = 0;
	int i;

	dev_vdbg(bcmchan->dev, "%s: %d: %p+%d [%d,%lu] %p\n", __func__,
		bcmchan->id, sgl, sg_len, direction, flags, context);

	for (i = 0; i < sg_len; i++)
		len += sg_dma_len(&sgl[i]);

	sg_init_table(&dev_sg, 1);
	sg_dma_len(&dev_sg) = len;

	if (direction == DMA_MEM_TO_DEV) {
		sg_dma_address(&dev_sg) = bcmchan->slave_addr_to;
		return __bcm2708_dma_prep_dma_sg(dmachan, &dev_sg, 1,
			sgl, sg_len, flags, bcmchan->slave_cfg_to);
	} else if (direction == DMA_DEV_TO_MEM) {
		sg_dma_address(&dev_sg) = bcmchan->slave_addr_from;
		return __bcm2708_dma_prep_dma_sg(dmachan, sgl, sg_len,
			&dev_sg, 1, flags, bcmchan->slave_cfg_from);
	}

	return NULL;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_cyclic(
	struct dma_chan *dmachan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	void *context)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	struct bcm2708_dmatx *bcmtx;

	dev_vdbg(bcmchan->dev, "%s: %d: %08x+%d [%d,%d] %p\n", __func__,
		bcmchan->id, buf_addr, buf_len, period_len, direction, context);

	/* could resort to a chain of (period_len / buf_len) CBs... */
	if (buf_len == 0 || buf_len > MAX_XLENGTH || buf_len > MAX_STRIDE
			|| period_len == 0 || (period_len % buf_len) != 0
			|| (period_len / buf_len) > MAX_YLENGTH)
		return NULL;

	if (direction != DMA_MEM_TO_DEV && direction != DMA_DEV_TO_MEM)
		return NULL;

	bcmtx = bcm2708_dma_alloc_tx(bcmchan,
		DMA_COMPL_SKIP_SRC_UNMAP | DMA_COMPL_SKIP_DEST_UNMAP, 1);
	if (unlikely(!bcmtx))
		return NULL;

	bcmtx->desc[0].cb->ti = BCM_TI_INTEN | BCM_TI_WAIT_RESP
		| BCM_TI_SRC_INC | BCM_TI_DST_INC | BCM_TI_TDMODE
		| BCM_TI_BURST_CHAN(bcmchan);
	if (direction == DMA_MEM_TO_DEV) {
		bcmtx->desc[0].cb->ti |= bcmchan->slave_cfg_to;
		bcmtx->desc[0].cb->src = buf_addr;
		bcmtx->desc[0].cb->dst = bcmchan->slave_addr_to;
	} else if (direction == DMA_DEV_TO_MEM) {
		bcmtx->desc[0].cb->ti |= bcmchan->slave_cfg_from;
		bcmtx->desc[0].cb->src = bcmchan->slave_addr_from;
		bcmtx->desc[0].cb->dst = buf_addr;
	}
	bcmtx->desc[0].cb->len = ((period_len / buf_len) << 16) | buf_len;
	/* stride defaults to 0 */

	mutex_lock(&bcmchan->prep_lock);
	list_add_tail(&bcmtx->list, &bcmchan->prep_list);
	mutex_unlock(&bcmchan->prep_lock);
	return &bcmtx->dmatx;
}

static struct dma_async_tx_descriptor *bcm2708_dma_prep_interleaved(
	struct dma_chan *dmachan, struct dma_interleaved_template *xt,
	unsigned long flags)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);	
	struct bcm2708_dmatx *bcmtx;
	u16 src_stride;
	u16 dst_stride;

	dev_vdbg(bcmchan->dev, "%s: %d: %p [%lu]\n", __func__,
		bcmchan->id, xt, flags);

	/* just use SG instead, it'll be easier */
	if (xt->frame_size != 1)
		return NULL;

	/* the engine also supports a negative icg, but this API doesn't */
	if (xt->sgl[0].size + xt->sgl[0].icg > MAX_STRIDE
				|| xt->sgl[0].size > MAX_XLENGTH
				|| xt->numf > MAX_YLENGTH
				|| xt->sgl[0].size < 0)
			return NULL;

	bcmtx = bcm2708_dma_alloc_tx(bcmchan, flags, 1);
	if (unlikely(!bcmtx))
		return NULL;

	bcmtx->desc[0].cb->ti = BCM_TI_INTEN | BCM_TI_WAIT_RESP
		| BCM_TI_SRC_INC | BCM_TI_DST_INC | BCM_TI_TDMODE
		| BCM_TI_BURST_CHAN(bcmchan) | bcmchan->cfg;
	bcmtx->desc[0].cb->src = xt->src_start;
	bcmtx->desc[0].cb->dst = xt->dst_start;
	bcmtx->desc[0].cb->len = (xt->numf << 16) | xt->sgl[0].size;
	src_stride = !xt->src_inc ? 0 : (xt->sgl[0].size
		+ xt->src_sgl ? xt->sgl[0].icg : 0);
	dst_stride = !xt->dst_inc ? 0 : (xt->sgl[0].size
		+ xt->dst_sgl ? xt->sgl[0].icg : 0);
	bcmtx->desc[0].cb->stride = src_stride | (dst_stride << 16);

	mutex_lock(&bcmchan->prep_lock);
	list_add_tail(&bcmtx->list, &bcmchan->prep_list);
	mutex_unlock(&bcmchan->prep_lock);
	return &bcmtx->dmatx;
}

static enum dma_status bcm2708_dma_tx_status(struct dma_chan *dmachan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);

	dev_vdbg(bcmchan->dev, "%s: %d\n", __func__, bcmchan->id);

	return dma_cookie_status(dmachan, cookie, txstate);
}

static void __bcm2708_dma_issue_pending(struct bcm2708_dmachan *bcmchan)
{
	struct bcm2708_dmatx *bcmtx;

	if (bcmchan->active || list_empty(&bcmchan->pending))
		return;

	dev_vdbg(bcmchan->dev, "%s: %d\n", __func__, bcmchan->id);

	list_splice_tail_init(&bcmchan->pending, &bcmchan->running);
	bcmtx = list_first_entry(&bcmchan->running, struct bcm2708_dmatx, list);

	dsb();
	writel(bcmtx->dmatx.phys, bcmchan->base + REG_CONBLK_AD);
	writel(BCM_CS_ACTIVE, bcmchan->base + REG_CS);

	bcmchan->active = true;
}

static void bcm2708_dma_issue_pending(struct dma_chan *dmachan)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	unsigned long flags;

	spin_lock_irqsave(&bcmchan->lock, flags);
	__bcm2708_dma_issue_pending(bcmchan);
	spin_unlock_irqrestore(&bcmchan->lock, flags);
}

static void bcm2708_dma_update_progress(struct bcm2708_dmachan *bcmchan,
	u32 block, bool issue)
{
	struct bcm2708_dmatx *bcmtx;
	struct bcm2708_dmatx *tmp;
	struct bcm2708_dmatx *last = NULL;
	LIST_HEAD(cyclic);
	int i;

	spin_lock(&bcmchan->lock);
	if (block == 0)
		bcmchan->active = false;

	list_for_each_entry_safe(bcmtx, tmp, &bcmchan->running, list) {
		if (block != 0) {
			for (i = 0; i < bcmtx->count; i++)
				if (block == bcmtx->desc[i].phys)
					goto done;
		}

		if (bcmtx->cyclic) {
			list_del(&bcmtx->list);
			list_add_tail(&bcmtx->list, &bcmchan->pending);
			list_add_tail(&bcmtx->list, &cyclic);
		} else {
			dma_cookie_complete(&bcmtx->dmatx);
			last = bcmtx;
		}
	}

done:
	if (last != NULL) {
		LIST_HEAD(completed);
		list_cut_position(&completed, &bcmchan->running, &last->list);
		list_splice_tail(&completed, &bcmchan->completed);
	}
	list_splice_tail(&cyclic, &bcmchan->completed);
	if (issue)
		__bcm2708_dma_issue_pending(bcmchan);
	spin_unlock(&bcmchan->lock);
}

static inline void __bcm2708_dma_cleanup_dst(struct bcm2708_dmachan *bcmchan,
	struct bcm2708_dmatx *bcmtx, int i)
{
	if (bcmtx->dmatx.flags & DMA_COMPL_DEST_UNMAP_SINGLE)
		dma_unmap_single(bcmchan->dev, bcmtx->desc[i].cb->dst,
			bcmtx->desc[i].cb->len, DMA_FROM_DEVICE);
	else
		dma_unmap_page(bcmchan->dev, bcmtx->desc[i].cb->dst,
			bcmtx->desc[i].cb->len, DMA_FROM_DEVICE);
}

static inline void __bcm2708_dma_cleanup_src(struct bcm2708_dmachan *bcmchan,
	struct bcm2708_dmatx *bcmtx, int i)
{
	if (bcmtx->dmatx.flags & DMA_COMPL_SRC_UNMAP_SINGLE)
		dma_unmap_single(bcmchan->dev, bcmtx->desc[i].cb->src,
			bcmtx->desc[i].cb->len, DMA_TO_DEVICE);
	else
		dma_unmap_page(bcmchan->dev, bcmtx->desc[i].cb->src,
			bcmtx->desc[i].cb->len, DMA_TO_DEVICE);
}

static void __bcm2708_dma_cleanup(struct bcm2708_dmachan *bcmchan,
	struct bcm2708_dmatx *bcmtx)
{
	int i;

	dev_dbg(bcmtx->chan->dev, "%s: %d: %08x\n", __func__,
		bcmtx->chan->id, bcmtx->dmatx.cookie);

	if (bcmtx->dmatx.callback)
		bcmtx->dmatx.callback(bcmtx->dmatx.callback_param);

	if (!(bcmtx->dmatx.flags & DMA_COMPL_SKIP_DEST_UNMAP))
		for (i = 0; i < bcmtx->count; i++)
			__bcm2708_dma_cleanup_dst(bcmchan, bcmtx, i);

	if (!(bcmtx->dmatx.flags & DMA_COMPL_SKIP_SRC_UNMAP))
		for (i = 0; i < bcmtx->count; i++)
			__bcm2708_dma_cleanup_src(bcmchan, bcmtx, i);
}

static inline void bcm2708_dma_cleanup(struct bcm2708_dmachan *bcmchan)
{
	LIST_HEAD(completed);
	struct bcm2708_dmatx *bcmtx;
	struct bcm2708_dmatx *tmp;
	unsigned long flags;

	spin_lock_irqsave(&bcmchan->lock, flags);
	list_splice_tail_init(&bcmchan->completed, &completed);
	spin_unlock_irqrestore(&bcmchan->lock, flags);

	list_for_each_entry_safe(bcmtx, tmp, &completed, list) {
		__bcm2708_dma_cleanup(bcmchan, bcmtx);
		list_del(&bcmtx->list);
		if (!bcmtx->cyclic)
			bcm2708_dma_free_tx(bcmtx);
	}
}

static void bcm2708_dma_tasklet(unsigned long data)
{
	bcm2708_dma_cleanup((struct bcm2708_dmachan *)data);
}

static void bcm2708_dma_abort(struct bcm2708_dmachan *bcmchan, u32 block)
{
	struct bcm2708_dmatx *bcmtx;
	bool found = false;
	int i;

	spin_lock(&bcmchan->lock);
	if (!list_empty(&bcmchan->running)) {
		bcmtx = list_first_entry(&bcmchan->running,
				struct bcm2708_dmatx, list);

		for (i = 0; i < bcmtx->count; i++)
			if (block == bcmtx->desc[i].phys)
				found = true;

		if (found) {
			list_del(&bcmtx->list);
			list_add_tail(&bcmtx->list, &bcmchan->completed);
		}
	}
	spin_unlock(&bcmchan->lock);

	if (!found)
		dev_warn(bcmchan->dev, "unable to find CB in error\n");
}

static irqreturn_t bcm2708_dma_irq_handler(int irq, void *dev_id)
{
	struct dma_chan *dmachan = dev_id;
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	u32 status = readl(bcmchan->base + REG_CS);
	u32 block;

	if (!(status & BCM_CS_INT))
		return IRQ_NONE;

	block = readl(bcmchan->base + REG_CONBLK_AD);
	if (status & BCM_CS_END)
		writel(status & BCM_CS_END, bcmchan->base + REG_CS);

	bcm2708_dma_update_progress(bcmchan, block, true);

	if (status & BCM_CS_ERROR) {
		u32 debug = readl(bcmchan->base + REG_DEBUG);

		dev_warn(bcmchan->dev, "error on chan %d:%s%s%s %08x at %08x\n",
			bcmchan->id,
			BCM_DEBUG_RLSN_ERR(debug) ? " rlsn" : "",
			BCM_DEBUG_FIFO_ERR(debug) ? " fifo" : "",
			BCM_DEBUG_READ_ERR(debug) ? " read" : "",
			BCM_DEBUG_DMA_STATE(debug),
			block);

		writel(BCM_DEBUG_RLSN_ERR(debug)
			| BCM_DEBUG_FIFO_ERR(debug)
			| BCM_DEBUG_READ_ERR(debug),
			bcmchan->base + REG_DEBUG);

		bcm2708_dma_abort(bcmchan, block);

		writel(BCM_CS_ABORT, bcmchan->base + REG_CS);
		writel(BCM_CS_ACTIVE, bcmchan->base + REG_CS);
	}

	tasklet_schedule(&bcmchan->tasklet);

	writel(status & BCM_CS_INT, bcmchan->base + REG_CS);
	return IRQ_HANDLED;
}

static inline u32 bcm2708_dma_make_cfg(struct bcm2708_dmacfg *cfg)
{
	return (cfg->src_dreq ? BCM_TI_SRC_DREQ : 0)
		| (cfg->dst_dreq ? BCM_TI_DST_DREQ : 0)
		| BCM_TI_PERMAP_SET(cfg->per)
		| BCM_TI_WAITS_SET(cfg->waits);
}

static int bcm2708_dma_control(struct dma_chan *dmachan,
		enum dma_ctrl_cmd cmd, unsigned long arg)
{
	struct bcm2708_dmachan *bcmchan = to_bcmchan(dmachan);
	unsigned long flags;

	dev_vdbg(bcmchan->dev, "%s: %d: %d %lu\n", __func__,
		bcmchan->id, cmd, arg);

	switch (cmd) {
	case DMA_SLAVE_CONFIG: {
		struct bcm2708_dmaslcfg *cfg = (struct bcm2708_dmaslcfg *)arg;

		if (cfg == NULL || cfg->other.waits > MAX_WAITS
				|| cfg->from.waits > MAX_WAITS
				|| cfg->to.waits > MAX_WAITS)
			return -EINVAL;

		bcmchan->cfg = bcm2708_dma_make_cfg(&cfg->other);
		bcmchan->slave_cfg_from = bcm2708_dma_make_cfg(&cfg->from);
		bcmchan->slave_cfg_to = bcm2708_dma_make_cfg(&cfg->to);
		bcmchan->slave_addr_from = cfg->from.dev_addr;
		bcmchan->slave_addr_to = cfg->to.dev_addr;
		return 0;
	}

	case DMA_TERMINATE_ALL:
		spin_lock_irqsave(&bcmchan->lock, flags);
		if (bcmchan->active) {
			u32 block;

			writel(0, bcmchan->base + REG_CS);
			block = readl(bcmchan->base + REG_CONBLK_AD);
			writel(0, bcmchan->base + REG_CONBLK_AD);
			writel(BCM_CS_ABORT, bcmchan->base + REG_CS);
			writel(0, bcmchan->base + REG_CONBLK_AD);
			bcmchan->active = false;

			bcm2708_dma_update_progress(bcmchan, block, false);
			tasklet_schedule(&bcmchan->tasklet);
		}
		bcm2708_dma_delete(&bcmchan->pending);
		bcm2708_dma_delete(&bcmchan->running);
		spin_unlock_irqrestore(&bcmchan->lock, flags);
		return 0;

	case DMA_PAUSE:
		spin_lock_irqsave(&bcmchan->lock, flags);
		if (bcmchan->active && !bcmchan->paused) {
			writel(0, bcmchan->base + REG_CS);
			bcmchan->paused = true;
		}
		spin_unlock_irqrestore(&bcmchan->lock, flags);
		return 0;

	case DMA_RESUME:
		spin_lock_irqsave(&bcmchan->lock, flags);
		if (bcmchan->active && bcmchan->paused) {
			bcmchan->paused = false;
			writel(BCM_CS_ACTIVE, bcmchan->base + REG_CS);
		}
		spin_unlock_irqrestore(&bcmchan->lock, flags);
		return 0;

	default:
		return -ENOSYS;
	}
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
	u32 channels;
	int ret, i;

	if (bcmdev == NULL)
		return -ENOMEM;
	bcmdev->dev = &pdev->dev;
	bcmdev->pool = dmam_pool_create(dev_name(bcmdev->dev), bcmdev->dev,
		sizeof(struct bcm2708_dmacb),
		sizeof(struct bcm2708_dmacb), 32);
	if (!bcmdev->pool) {
		dev_err(bcmdev->dev, "unable to create dma pool\n");
		return -ENOMEM;
	}

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

	if (of_property_read_u32(np, "broadcom,channels", &channels)) {
		dev_err(bcmdev->dev, "missing channels mask\n");
		return -EINVAL;
	}

	for (i = 0; i < D_MAX; i++) {
		dmadev[i].dev = bcmdev->dev;
		dmadev[i].chancnt = 0;
		INIT_LIST_HEAD(&dmadev[i].channels);
	}

	channels &= readl(bcmdev->base[0] + REG_ENABLE);

	for (i = 0; i < MAX_CHANS; i++) {
		u32 debug;
		char *tmp;
		int type, len;

		if (!(channels & BIT(i)))
			continue;

		bcmchan = devm_kzalloc(bcmdev->dev,
			sizeof(*bcmchan), GFP_KERNEL);
		if (bcmchan == NULL)
			return -ENOMEM;
		dmachan = &bcmchan->dmachan;
		bcmchan->dev = bcmdev->dev;
		bcmchan->pool = bcmdev->pool;
		bcmchan->id = i;
		mutex_init(&bcmchan->prep_lock);
		INIT_LIST_HEAD(&bcmchan->prep_list);
		spin_lock_init(&bcmchan->lock);
		INIT_LIST_HEAD(&bcmchan->pending);
		INIT_LIST_HEAD(&bcmchan->running);
		INIT_LIST_HEAD(&bcmchan->completed);
		tasklet_init(&bcmchan->tasklet, bcm2708_dma_tasklet,
			(unsigned long)bcmchan);

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
		writel(BCM_CS_RESET, bcmchan->base + REG_CS);

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
			bcm2708_dma_irq_handler, IRQF_SHARED, tmp, dmachan);
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
	dma_cap_set(DMA_MEMSET, dmadev[D_FULL].cap_mask);
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
		if (list_empty(&dmadev[i].channels))
			continue;

		dmadev[i].copy_align = CACHE_LINE_SIZE;
		dmadev[i].fill_align = CACHE_LINE_SIZE;

		dmadev[i].device_alloc_chan_resources = bcm2708_dma_alloc_chan;
		dmadev[i].device_free_chan_resources = bcm2708_dma_free_chan;

		dmadev[i].device_prep_dma_memcpy = bcm2708_dma_prep_memcpy;
		dmadev[i].device_prep_dma_memset = bcm2708_dma_prep_memset;
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
	struct dma_chan *dmachan;
	struct bcm2708_dmachan *bcmchan;
	int i;

	for (i = 0; i < D_MAX; i++) {
		if (list_empty(&bcmdev->dmadev[i].channels))
			continue;

		list_for_each_entry(dmachan,
				&bcmdev->dmadev[i].channels, device_node) {
			bcmchan = to_bcmchan(dmachan);
			tasklet_kill(&bcmchan->tasklet);
		}

		dma_async_device_unregister(&bcmdev->dmadev[i]);
	}
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

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("Broadcom BCM2708 DMA engine driver");
MODULE_LICENSE("GPL");
