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
 * Copyright 2000-2010 Broadcom Corporation
 *
 * Derived from bcmdrivers/opensource/net/enet/impl4/bcmenet.c:
 * Copyright 2010 Broadcom Corporation
 *
 * The DMA controller processes requests from an arbitrarily sized
 * ring buffer in CPU memory. By default this is set to the size of
 * a typical page (4KB). This will be the maximum number of active
 * requests per channel for this device, but a client driver can opt
 * to limit the number of active requests itself if this buffer size
 * would otherwise be too large (e.g. a network transmit buffer).
 *
 * There is no processing overhead to doing this as the controller
 * won't move around the ring unless there are active requests to
 * process.
 */

#include <linux/delay.h>
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

#include "virt-dma.h"


/* Check that the right register access functions are used */
#define IUDMA_EXTRA_ASSERTS		1


/* at 8B per hw descriptor => one 4KB PAGE_SIZE per channel */
#define IUDMA_DEFAULT_NUM_REQUESTS	512
#define IUDMA_MAX_NUM_REQUESTS		8192
#define IUDMA_MAX_BURST			16


/* Global configuration */
#define IUDMA_G_CFG_REG			0x00
#define   IUDMA_G_CFG_ENABLE			BIT(0)
#define   IUDMA_G_FLOWC_ENABLE(x)		BIT(((x) >> 1) + 1)
#define IUDMA_G_FLOWC_LO_THRESH_REG(x)	(0x04 + (x) * 0x06)
#define IUDMA_G_FLOWC_HI_THRESH_REG(x)	(0x08 + (x) * 0x06)
#define IUDMA_G_FLOWC_BUFALLOC_REG(x)	(0x0c + (x) * 0x06)
#define   IUDMA_G_FLOWC_BUFALLOC_FORCE		BIT(31)
#define IUDMA_G_CHANNEL_RESET_REG	0x34
#define IUDMA_G_CHANNEL_DEBUG_REG	0x38
#define IUDMA_G_INT_STATUS_REG		0x40
#define IUDMA_G_INT_ENABLE_REG		0x44

#define IUDMA_G_FLOWC_MAX		8


/* Channel configuration (with state RAM) */
#define IUDMA_C_CHANNEL_BASE_REG(x)	(0x10 * (x))
#define IUDMA_C_CFG_REG			0x00
#define   IUDMA_C_CFG_ENABLE			BIT(0)
	  /* idle after an EOP flag is detected */
#define   IUDMA_C_CFG_PKT_HALT			BIT(1)
	  /* idle after finish current memory burst */
#define   IUDMA_C_CFG_BURST_HALT		BIT(2)
#define IUDMA_C_INT_STATUS_REG		0x04
#define IUDMA_C_INT_ENABLE_REG		0x08
	  /* buffer done */
#define   IUDMA_C_INT_PKT_DONE			BIT(0)
	  /* transfer complete */
#define   IUDMA_C_INT_BUF_DONE			BIT(1)
	  /* no valid descriptors */
#define   IUDMA_C_INT_NO_DESC			BIT(2)
	  /* rxdma detect client protocol error */
#define   IUDMA_C_INT_RX_ERROR			BIT(3)
#define IUDMA_C_MAX_BURST_REG		0x0c


/* Channel configuration (no state RAM) */
#define IUDMA_N_CHANNEL_BASE_REG(x)	(0x40 * (x))
#define IUDMA_N_CFG_REG			0x00
#define   IUDMA_N_CFG_ENABLE			BIT(0)
	  /* idle after an EOP flag is detected */
#define   IUDMA_N_CFG_PKT_HALT			BIT(1)
	  /* idle after finish current memory burst */
#define   IUDMA_N_CFG_BURST_HALT		BIT(2)
#define   IUDMA_N_CFG_CHAINING			BIT(3)
#define   IUDMA_N_CFG_WRAP_ENABLE		BIT(4)
#define   IUDMA_N_CFG_FLOWC_ENABLE		BIT(5)
#define IUDMA_N_MAX_BURST_REG		0x04
#define IUDMA_N_RSTART_REG		0x08
#define IUDMA_N_LEN_REG			0x0c
#define IUDMA_N_INT_STATUS_REG		0x14
#define IUDMA_N_INT_ENABLE_REG		0x18
	  /* buffer done */
#define   IUDMA_N_INT_PKT_DONE			BIT(0)
	  /* transfer complete */
#define   IUDMA_N_INT_BUF_DONE			BIT(1)
	  /* no valid descriptors */
#define   IUDMA_N_INT_NO_DESC			BIT(2)
	  /* rxdma detect client protocol error */
#define   IUDMA_N_INT_RX_ERROR			BIT(3)
#define IUDMA_N_FLOWC_REG		0x1c
#define IUDMA_N_BUFALLOC_REG		0x20
#define   IUDMA_N_BUFALLOC_FORCE		BIT(31)


/* State RAM */
#define IUDMA_S_CHANNEL_BASE_REG(x)	(0x10 * (x))
	/* descriptor ring start address */
#define IUDMA_S_RSTART_REG		0x00
	/* state/bytes done/ring offset */
#define IUDMA_S_STATE_DATA_REG		0x04
	/* buffer descriptor status and len */
#define IUDMA_S_DESC_LEN_STATUS_REG	0x08
	/* buffer descriptor current processing */
#define IUDMA_S_DESC_BASE_BUFPTR_REG	0x0c


struct bcm63xx_iudma_hw_desc {
	u16 length;
#define IUDMA_D_LEN_MAX			0xfff

	union {
		u16 control;
/* control: client device specific */
	/* usbd: send zero length packet */
#define IUDMA_D_CTL_USBD_ZERO		BIT(0)
	/* enetsw: tx port 0-7 */
#define IUDMA_D_CTL_ENETSW_PORT(x)	BIT(x)
	/* enetsw: append CRC */
#define IUDMA_D_CTL_ENETSW_APPEND_CRC	BIT(8)
	/* enetsw: append BRCM tag */
#define IUDMA_D_CTL_ENETSW_APPEND_TAG	BIT(9)
	/* enetsw: tx priority 0-3 */
#define IUDMA_D_CTL_ENETSW_PRIO(x)	((x) << 10)

/* control: DMA controller */
	/* last descriptor in ring */
#define IUDMA_D_CTL_WRAP		BIT(12)
	/* first buffer in packet */
#define IUDMA_D_CTL_SOP			BIT(13)
	/* last buffer in packet */
#define IUDMA_D_CTL_EOP			BIT(14)
#define IUDMA_D_CTL_ESOP_MASK		(IUDMA_D_CTL_EOP | IUDMA_D_CTL_SOP)
	/* cleared by hw, set by cpu */
#define IUDMA_D_CTL_OWNER		BIT(15)

		u16 status;
/* status: client device specific */
	};

	u32 address;
} __attribute__((packed));

struct bcm63xx_iudma;

struct bcm63xx_iudma_chan {
	struct bcm63xx_iudma *ctrl;

	unsigned int id;
	char *name;
	union {
		void __iomem *base_c; /* with state RAM */
		void __iomem *base_n; /* no state RAM */
	};
	void __iomem *base_s;
	int irq;

	struct virt_dma_chan vc;
	struct dma_slave_config cfg;

	spinlock_t hw_ring_lock;
	dma_addr_t hw_ring_base;
	unsigned int hw_ring_alloc;
	unsigned int hw_ring_size;

	struct bcm63xx_iudma_hw_desc *hw_desc;
	/* current number of armed descriptor given to hardware */
	unsigned int desc_count;
	/* next descriptor to fetch from hardware */
	unsigned int read_desc;
	/* next dirty descriptor to use */
	unsigned int write_desc;
};

struct bcm63xx_iudma {
	void __iomem *base_g;
	spinlock_t lock;

	struct dma_device ddev;
	unsigned int n_channels;
	struct bcm63xx_iudma_chan chan[];
};

struct bcm63xx_iudma_desc {
	struct virt_dma_desc vd;
	enum dma_transfer_direction dir;
};

struct bcm63xx_iudma_of_data {
	unsigned int n_channels;
	unsigned int dmac_offset;
	unsigned int dmas_offset;
};


static inline __pure struct bcm63xx_iudma *to_bcm63xx_iduma(struct dma_device *d)
{
	return container_of(d, struct bcm63xx_iudma, ddev);
}

static inline __pure struct bcm63xx_iudma_chan *to_bcm63xx_iudma_chan(struct dma_chan *c)
{
	return container_of(c, struct bcm63xx_iudma_chan, vc.chan);
}

static inline __pure struct bcm63xx_iudma_desc *to_bcm63xx_iudma_desc(
	struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct bcm63xx_iudma_desc, vd.tx);
}


static inline __pure bool bcm63xx_iudma_chan_has_sram(
	struct bcm63xx_iudma_chan *chan)
{
	return chan->base_s != NULL;
}

/* Flow control is configurable for channels 0, 2, 4, 6
 * (used on Ethernet to enable PAUSE frame generation)
 */
static inline __pure bool bcm63xx_iudma_chan_has_flowc(
	struct bcm63xx_iudma_chan *chan)
{
	return chan->id < IUDMA_G_FLOWC_MAX && !(chan->id & 1);
}


/* Global configuration */
static inline u32 g_readl(struct bcm63xx_iudma *iudma, int reg)
{
	return __raw_readl(iudma->base_g + reg);
}

static inline void g_writel(u32 val, struct bcm63xx_iudma *iudma, int reg)
{
	return __raw_writel(val, iudma->base_g + reg);
}

/* Channel configuration (with state RAM) */
static inline u32 c_readl(struct bcm63xx_iudma_chan *chan, int reg)
{
#if IUDMA_EXTRA_ASSERTS
	BUG_ON(!bcm63xx_iudma_chan_has_sram(chan));
#endif
	return __raw_readl(chan->base_c + reg);
}

static inline void c_writel(u32 val, struct bcm63xx_iudma_chan *chan, int reg)
{
#if IUDMA_EXTRA_ASSERTS
	BUG_ON(!bcm63xx_iudma_chan_has_sram(chan));
#endif
	return __raw_writel(val, chan->base_c + reg);
}

/* Channel configuration (no state RAM) */
static inline u32 n_readl(struct bcm63xx_iudma_chan *chan, int reg)
{
#if IUDMA_EXTRA_ASSERTS
	BUG_ON(bcm63xx_iudma_chan_has_sram(chan));
#endif
	return __raw_readl(chan->base_n + reg);
}

static inline void n_writel(u32 val, struct bcm63xx_iudma_chan *chan, int reg)
{
#if IUDMA_EXTRA_ASSERTS
	BUG_ON(bcm63xx_iudma_chan_has_sram(chan));
#endif
	return __raw_writel(val, chan->base_n + reg);
}

/* State RAM */
static inline u32 s_readl(struct bcm63xx_iudma_chan *chan, int reg)
{
#if IUDMA_EXTRA_ASSERTS
	BUG_ON(!bcm63xx_iudma_chan_has_sram(chan));
#endif
	return __raw_readl(chan->base_s + reg);
}

static inline void s_writel(u32 val, struct bcm63xx_iudma_chan *chan, int reg)
{
#if IUDMA_EXTRA_ASSERTS
	BUG_ON(!bcm63xx_iudma_chan_has_sram(chan));
#endif
	return __raw_writel(val, chan->base_s + reg);
}


/* Controller functions */

static inline void bcm63xx_iudma_disable_ctl(struct bcm63xx_iudma *iudma)
{
	g_writel(0, iudma, IUDMA_G_CFG_REG);
}

static inline void bcm63xx_iudma_enable_ctl(struct bcm63xx_iudma *iudma)
{
	g_writel(IUDMA_G_CFG_ENABLE, iudma, IUDMA_G_CFG_REG);
}


/* Channel functions */

static inline void bcm63xx_iudma_reset_chan(struct bcm63xx_iudma_chan *chan)
{
	struct bcm63xx_iudma *iudma = chan->ctrl;
	unsigned long flags;

	spin_lock_irqsave(&iudma->lock, flags);
	g_writel(BIT(chan->id), iudma, IUDMA_G_CHANNEL_RESET_REG);
	g_writel(0, iudma, IUDMA_G_CHANNEL_RESET_REG);
	spin_unlock_irqrestore(&iudma->lock, flags);
}

static inline void bcm63xx_iudma_bufalloc_chan(struct bcm63xx_iudma_chan *chan,
	u32 val)
{
	struct bcm63xx_iudma *iudma = chan->ctrl;

	if (bcm63xx_iudma_chan_has_flowc(chan)) {
		if (bcm63xx_iudma_chan_has_sram(chan))
			g_writel(val, iudma,
				IUDMA_G_FLOWC_BUFALLOC_REG(chan->id));
	} else if (!bcm63xx_iudma_chan_has_sram(chan)) {
		n_writel(val, chan, IUDMA_N_BUFALLOC_REG);
	}
}

static inline void bcm63xx_iudma_disable_chan(struct bcm63xx_iudma_chan *chan)
{
	unsigned int limit = 1000;

	if (bcm63xx_iudma_chan_has_sram(chan))
		c_writel(0, chan, IUDMA_C_INT_ENABLE_REG);
	else
		c_writel(0, chan, IUDMA_N_INT_ENABLE_REG);

	do {
		u32 val;

		if (bcm63xx_iudma_chan_has_sram(chan)) {
			c_writel(IUDMA_C_CFG_PKT_HALT, chan, IUDMA_C_CFG_REG);
			val = c_readl(chan, IUDMA_C_CFG_REG);
			if (!(val & IUDMA_C_CFG_ENABLE))
				break;
		} else {
			n_writel(IUDMA_N_CFG_PKT_HALT, chan, IUDMA_N_CFG_REG);
			val = n_readl(chan, IUDMA_N_CFG_REG);
			if (!(val & IUDMA_N_CFG_ENABLE))
				break;
		}

		udelay(1);
	} while (--limit);
	WARN_ON(!limit);

	bcm63xx_iudma_reset_chan(chan);
}

static inline void bcm63xx_iudma_enable_chan(struct bcm63xx_iudma_chan *chan)
{
	bcm63xx_iudma_reset_chan(chan);

	/* initialize flow control buffer allocation */
	if (bcm63xx_iudma_chan_has_sram(chan))
		bcm63xx_iudma_bufalloc_chan(chan, IUDMA_G_FLOWC_BUFALLOC_FORCE);
	else
		bcm63xx_iudma_bufalloc_chan(chan, IUDMA_N_BUFALLOC_FORCE);

	/* write ring address */
	if (bcm63xx_iudma_chan_has_sram(chan))
		s_writel(chan->hw_ring_base, chan, IUDMA_S_RSTART_REG);
	else
		n_writel(chan->hw_ring_base, chan, IUDMA_N_RSTART_REG);

	/* clear remaining state ram */
	if (bcm63xx_iudma_chan_has_sram(chan)) {
		s_writel(0, chan, IUDMA_S_STATE_DATA_REG);
		s_writel(0, chan, IUDMA_S_DESC_LEN_STATUS_REG);
		s_writel(0, chan, IUDMA_S_DESC_BASE_BUFPTR_REG);
	} else {
		n_writel(0, chan, IUDMA_N_FLOWC_REG);
	}

	/* set dma maximum burst len */
	if (bcm63xx_iudma_chan_has_sram(chan))
		c_writel(IUDMA_MAX_BURST, chan, IUDMA_C_MAX_BURST_REG);
	else
		n_writel(IUDMA_MAX_BURST, chan, IUDMA_N_MAX_BURST_REG);

	/* set flow control low/high threshold to 1/3 / 2/3 */
	if (bcm63xx_iudma_chan_has_flowc(chan)) {
		if (bcm63xx_iudma_chan_has_sram(chan)) {
			c_writel(chan->hw_ring_size / 3, chan,
				IUDMA_G_FLOWC_LO_THRESH_REG(chan->id));
			c_writel((chan->hw_ring_size * 2) / 3, chan,
				IUDMA_G_FLOWC_HI_THRESH_REG(chan->id));
		}
	} else if (!bcm63xx_iudma_chan_has_sram(chan)) {
		n_writel(5, chan, IUDMA_N_FLOWC_REG);
		n_writel(chan->hw_ring_size, chan, IUDMA_N_LEN_REG);
	}

	wmb();

	if (bcm63xx_iudma_chan_has_sram(chan))
		c_writel(IUDMA_C_CFG_ENABLE, chan, IUDMA_C_CFG_REG);
	else
		n_writel(IUDMA_N_CFG_ENABLE | IUDMA_N_CFG_CHAINING |
			IUDMA_N_CFG_WRAP_ENABLE | IUDMA_N_CFG_FLOWC_ENABLE,
			chan, IUDMA_N_CFG_REG);

	if (bcm63xx_iudma_chan_has_sram(chan))
		c_writel(IUDMA_C_INT_PKT_DONE, chan, IUDMA_C_INT_ENABLE_REG);
	else
		n_writel(IUDMA_N_INT_PKT_DONE | IUDMA_N_INT_BUF_DONE |
			IUDMA_C_INT_NO_DESC, chan, IUDMA_N_INT_ENABLE_REG);
}

static irqreturn_t bcm63xx_iudma_interrupt(int irq, void *data)
{
	struct bcm63xx_iudma_chan *ch = data;
	struct device *dev = ch->vc.chan.device->dev;

	dev_info(dev, "%s(%u)\n", __func__, ch->id);

	return IRQ_HANDLED;
}


/* DMA engine interface functions */

static int bcm63xx_iudma_alloc_chan(struct dma_chan *dchan)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct device *dev = dchan->device->dev;
	int ret;

	dev_info(dev, "%s(%u)\n", __func__, ch->id);

	ch->hw_desc = dma_zalloc_coherent(dev, ch->hw_ring_alloc,
					&ch->hw_ring_base, GFP_KERNEL);
	if (!ch->hw_desc)
		return -ENOMEM;

	ch->desc_count = 0;
	ch->read_desc = 0;
	ch->write_desc = 0;

	ret = request_irq(ch->irq, bcm63xx_iudma_interrupt, 0, ch->name, ch);
	if (ret)
		goto free_hw_desc;

	bcm63xx_iudma_enable_chan(ch);
	return 0;

free_hw_desc:
	dma_free_coherent(dev, ch->hw_ring_alloc,
		ch->hw_desc, ch->hw_ring_base);
	return ret;
}

static void bcm63xx_iudma_free_chan(struct dma_chan *dchan)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct device *dev = dchan->device->dev;

	dev_info(dev, "%s(%u)\n", __func__, ch->id);

	bcm63xx_iudma_disable_chan(ch);
	vchan_free_chan_resources(&ch->vc);
	free_irq(ch->irq, ch);
	dma_free_coherent(dev, ch->hw_ring_alloc,
		ch->hw_desc, ch->hw_ring_base);
}

static struct dma_async_tx_descriptor *bcm63xx_iudma_prep_slave_sg(
	struct dma_chan *dchan,
	struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct device *dev = dchan->device->dev;

	dev_info(dev, "%s(%u)\n", __func__, ch->id);

	return ERR_PTR(-ENOSYS);
}

static void bcm63xx_iudma_issue_pending(struct dma_chan *dchan)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct device *dev = dchan->device->dev;

	dev_info(dev, "%s(%u)\n", __func__, ch->id);
}

static enum dma_status bcm63xx_iudma_tx_status(struct dma_chan *dchan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct device *dev = dchan->device->dev;

	dev_info(dev, "%s(%u)\n", __func__, ch->id);

	return DMA_ERROR;
}

static int bcm63xx_iudma_config(struct dma_chan *dchan,
	struct dma_slave_config *config)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct device *dev = dchan->device->dev;

	dev_info(dev, "%s(%u)\n", __func__, ch->id);

	return -ENOSYS;
}

static int bcm63xx_iudma_terminate_all(struct dma_chan *dchan)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct device *dev = dchan->device->dev;

	dev_info(dev, "%s(%u)\n", __func__, ch->id);

	return -ENOSYS;
}

static void bcm63xx_iudma_desc_free(struct virt_dma_desc *vd)
{
//	struct bcm63xx_iudma_desc *desc = container_of(vd,
//					struct bcm63xx_iudma_desc, vd);

}


/* Driver functions */

static int bcm63xx_iudma_set_chan_name(struct bcm63xx_iudma_chan *ch,
	struct device *dev)
{
	const char *prefix = dev_name(dev);
	size_t name_len = strlen(prefix) + 5 /* ":chan" */ + 32 /* "%u" */ + 1;

	ch->name = devm_kzalloc(dev, name_len, GFP_KERNEL);
	if (!ch->name)
		return -ENOMEM;

	return snprintf(ch->name, name_len, "%s:chan%u", prefix, ch->id);
}

static int bcm63xx_iudma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct bcm63xx_iudma *iudma;
	struct dma_device *ddev;
	const struct bcm63xx_iudma_of_data *of_data;
	struct resource *res;
	u32 dma_requests;
	unsigned int i;
	int ret;

	of_data = of_device_get_match_data(dev);
	if (!of_data)
		return -EINVAL;

	iudma = devm_kzalloc(dev,
		sizeof(*iudma) + sizeof(*iudma->chan) * of_data->n_channels,
		GFP_KERNEL);
	if (!iudma)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iudma->base_g = devm_ioremap_resource(dev, res);
	if (IS_ERR(iudma->base_g))
		return PTR_ERR(iudma->base_g);

	ret = of_property_read_u32(np, "dma-requests", &dma_requests);
	if (ret || dma_requests == 0 || dma_requests > IUDMA_MAX_NUM_REQUESTS)
		dma_requests = IUDMA_DEFAULT_NUM_REQUESTS;

	spin_lock_init(&iudma->lock);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	dma_set_max_seg_size(dev, IUDMA_D_LEN_MAX);

	ddev = &iudma->ddev;
	ddev->dev = dev;
	INIT_LIST_HEAD(&ddev->channels);
	ddev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	ddev->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	ddev->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	ddev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;

	dma_cap_set(DMA_PRIVATE, ddev->cap_mask);
	dma_cap_set(DMA_SLAVE, ddev->cap_mask);

	ddev->device_alloc_chan_resources = bcm63xx_iudma_alloc_chan;
	ddev->device_free_chan_resources = bcm63xx_iudma_free_chan;
	ddev->device_prep_slave_sg = bcm63xx_iudma_prep_slave_sg;
	ddev->device_issue_pending = bcm63xx_iudma_issue_pending;
	ddev->device_tx_status = bcm63xx_iudma_tx_status;
	ddev->device_config = bcm63xx_iudma_config;
	ddev->device_terminate_all = bcm63xx_iudma_terminate_all;

	bcm63xx_iudma_disable_ctl(iudma);

	for (i = 0; i < of_data->n_channels; i++) {
		struct bcm63xx_iudma_chan *ch = &iudma->chan[i];

		ch->ctrl = iudma;
		ch->id = i;

		ret = bcm63xx_iudma_set_chan_name(ch, dev);
		if (ret)
			return ret;

		ch->irq = platform_get_irq(pdev, i);
		if (ch->irq < 0)
			break;

		if (of_data->dmas_offset) {
			/* With state RAM */
			ch->base_c = iudma->base_g + of_data->dmac_offset +
					IUDMA_C_CHANNEL_BASE_REG(i);
			ch->base_s = iudma->base_g + of_data->dmas_offset +
					IUDMA_S_CHANNEL_BASE_REG(i);
		} else {
			/* No state RAM */
			ch->base_c = iudma->base_g + of_data->dmac_offset +
					IUDMA_N_CHANNEL_BASE_REG(i);
			ch->base_s = 0;
		}

		spin_lock_init(&ch->hw_ring_lock);
		ch->hw_ring_size = dma_requests;
		ch->hw_ring_alloc = sizeof(*ch->hw_desc) * ch->hw_ring_size;

		bcm63xx_iudma_disable_chan(ch);

		ch->vc.desc_free = bcm63xx_iudma_desc_free;
		vchan_init(&ch->vc, ddev);

		iudma->n_channels++;
	}

	if (!iudma->n_channels) {
		dev_err(dev, "device has no channel IRQs\n");
		return -EINVAL;
	}

	bcm63xx_iudma_enable_ctl(iudma);

	dev_info(dev, "registered BCM63xx IUDMA controller at MMIO 0x%p (%u channels)",
			iudma->base_g, iudma->n_channels);
	return 0;
}

static int bcm63xx_iudma_remove(struct platform_device *pdev)
{
	struct bcm63xx_iudma *iudma = platform_get_drvdata(pdev);

	dma_async_device_unregister(&iudma->ddev);
	bcm63xx_iudma_disable_ctl(iudma);

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

