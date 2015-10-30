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
 * Derived from bcm963xx_4.12L.06B_consumer/shared/opensource/include/bcm963xx/63268_map_part.h:
 * Copyright 2000-2010 Broadcom Corporation
 *
 * Derived from bcm963xx_4.12L.06B_consumer/bcmdrivers/opensource/net/enet/impl4/bcmenet.c:
 * Copyright 2010 Broadcom Corporation
 *
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
 *
 *
 * WARNING: lock debugging increases softirq CPU load, reduces network
 * throughput by up to 50% and maximum packets per second by up to 75%
 */

#include <linux/bcm63xx_iudma.h>
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


/* at 8B per hw descriptor => one 4KB PAGE_SIZE per channel */
#define IUDMA_DEFAULT_NUM_REQUESTS	512
/* Maximum ring size supported by the hardware */
#define IUDMA_MAX_NUM_REQUESTS		8192
/* Maximum rx buffers to process before returning from tasklet
 * (if this is too high it starves tx completion) */
#define IUDMA_MAX_RX_BUF_BURST		16


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
#define   IUDMA_C_INT_BUF_DONE			BIT(0)
	  /* transfer complete */
#define   IUDMA_C_INT_PKT_DONE			BIT(1)
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
#define   IUDMA_N_INT_BUF_DONE			BIT(0)
	  /* transfer complete */
#define   IUDMA_N_INT_PKT_DONE			BIT(1)
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
/* control/status: DMA controller */
	/* last descriptor in ring */
#define IUDMA_D_CS_WRAP		BIT(12)
	/* cleared by hw, set by cpu */
#define IUDMA_D_CS_OWNER		BIT(15)

/* control: client device specific */
#define IUDMA_D_CTL_CLIENT_MASK		(~(IUDMA_D_CS_WRAP|IUDMA_D_CS_OWNER))

		u16 status;
/* status: client device specific */
#define IUDMA_D_STA_CLIENT_MASK		(~(IUDMA_D_CS_WRAP|IUDMA_D_CS_OWNER))
	};

	u32 address;
} __attribute__((packed));

struct bcm63xx_iudma;

struct bcm63xx_iudma_desc {
	struct virt_dma_desc vd;

	struct list_head node;
	dma_addr_t address;
	size_t length;
	struct bcm63xx_iudma_context *context;
};

struct bcm63xx_iudma_chan {
	struct bcm63xx_iudma *ctrl;
	struct bcm63xx_iudma_chan *pair; /* opposite transfer direction */

	unsigned int id;
	const char *name;
	union {
		void __iomem *base_c; /* with state RAM */
		void __iomem *base_n; /* no state RAM */
	};
	void __iomem *base_s;
	int irq;

	struct dma_chan dchan;
	struct virt_dma_chan vc;
	struct tasklet_struct task; /* rx chan only */
	bool running;

	unsigned int maxburst;
	bool flowc;
	unsigned int flowc_ring_size;

	dma_addr_t hw_ring_base;
	unsigned int hw_ring_alloc;
	unsigned int hw_ring_size;

	struct bcm63xx_iudma_hw_desc *hw_desc;
	struct bcm63xx_iudma_desc **desc;

	spinlock_t pool_lock;
	struct list_head desc_pool;

	/* current number of armed descriptor given to hardware */
	unsigned int desc_count;
	/* next descriptor to fetch from hardware */
	unsigned int read_desc;
	/* next dirty descriptor to use */
	unsigned int write_desc;
};

struct bcm63xx_iudma {
	struct device *dev;
	void __iomem *base_g;
	spinlock_t lock;

	struct dma_device ddev;
	unsigned int n_channels;
	struct bcm63xx_iudma_chan chan[];
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
	struct bcm63xx_iudma_chan *ch)
{
	return ch->base_s != NULL;
}

/* Even numbered channels are for receiving */
static inline __pure bool bcm63xx_iudma_chan_is_rx(
	struct bcm63xx_iudma_chan *ch)
{
	return !(ch->id & 1);
}

/* Odd numbered channels are for transmitting */
static inline __pure bool bcm63xx_iudma_chan_is_tx(
	struct bcm63xx_iudma_chan *ch)
{
	return (ch->id & 1);
}

/* Flow control is configurable for channels 0, 2, 4, 6
 * (used on Ethernet to enable PAUSE frame generation)
 */
static inline __pure bool bcm63xx_iudma_chan_has_flowc(
	struct bcm63xx_iudma_chan *ch)
{
	return bcm63xx_iudma_chan_is_rx(ch) &&
		bcm63xx_iudma_chan_has_sram(ch) &&
		ch->id < IUDMA_G_FLOWC_MAX;
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
static inline u32 c_readl(struct bcm63xx_iudma_chan *ch, int reg)
{
	return __raw_readl(ch->base_c + reg);
}

static inline void c_writel(u32 val, struct bcm63xx_iudma_chan *ch, int reg)
{
	return __raw_writel(val, ch->base_c + reg);
}

/* Channel configuration (no state RAM) */
static inline u32 n_readl(struct bcm63xx_iudma_chan *ch, int reg)
{
	return __raw_readl(ch->base_n + reg);
}

static inline void n_writel(u32 val, struct bcm63xx_iudma_chan *ch, int reg)
{
	return __raw_writel(val, ch->base_n + reg);
}

/* State RAM */
static inline u32 s_readl(struct bcm63xx_iudma_chan *ch, int reg)
{
	return __raw_readl(ch->base_s + reg);
}

static inline void s_writel(u32 val, struct bcm63xx_iudma_chan *ch, int reg)
{
	return __raw_writel(val, ch->base_s + reg);
}


/* Controller functions */

static inline void bcm63xx_iudma_disable_ctl(struct bcm63xx_iudma *iudma)
{
	u32 val;

	spin_lock_irq(&iudma->lock);
	val = g_readl(iudma, IUDMA_G_CFG_REG);
	val &= ~IUDMA_G_CFG_ENABLE;
	g_writel(val, iudma, IUDMA_G_CFG_REG);
	spin_unlock_irq(&iudma->lock);
}

static inline void bcm63xx_iudma_enable_ctl(struct bcm63xx_iudma *iudma)
{
	u32 val;

	spin_lock_irq(&iudma->lock);
	val = g_readl(iudma, IUDMA_G_CFG_REG);
	val |= IUDMA_G_CFG_ENABLE;
	g_writel(val, iudma, IUDMA_G_CFG_REG);
	spin_unlock_irq(&iudma->lock);
}


/* Channel functions */

static inline struct bcm63xx_iudma_desc *bcm63xx_iudma_desc_get(
	struct bcm63xx_iudma_chan *ch)
{
	struct bcm63xx_iudma_desc *desc;

	spin_lock_bh(&ch->pool_lock);
	desc = list_first_entry_or_null(&ch->desc_pool,
		struct bcm63xx_iudma_desc, node);
	if (desc)
		list_del(&desc->node);
	else
		WARN_ONCE(1, "%s: desc pool empty on channel %u (hw_ring_size=%u, desc_count=%u, read_desc=%u, write_desc=%u)\n",
			__func__, ch->id, ch->hw_ring_size, ch->desc_count,
			ch->read_desc, ch->write_desc);
	spin_unlock_bh(&ch->pool_lock);

	return desc;
}

static void bcm63xx_iudma_desc_put(struct virt_dma_desc *vd)
{
	struct bcm63xx_iudma_desc *desc = container_of(vd,
					struct bcm63xx_iudma_desc, vd);
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(vd->tx.chan);

	spin_lock_bh(&ch->pool_lock);
	list_add_tail(&desc->node, &ch->desc_pool);
	spin_unlock_bh(&ch->pool_lock);
}

static inline void bcm63xx_iudma_reset_chan(struct bcm63xx_iudma_chan *ch)
{
	struct bcm63xx_iudma *iudma = ch->ctrl;

	spin_lock_irq(&iudma->lock);
	g_writel(BIT(ch->id), iudma, IUDMA_G_CHANNEL_RESET_REG);
	g_writel(0, iudma, IUDMA_G_CHANNEL_RESET_REG);
	spin_unlock_irq(&iudma->lock);
}

static inline void bcm63xx_iudma_reset_ring(struct bcm63xx_iudma_chan *ch)
{
	memset(ch->hw_desc, 0, ch->hw_ring_alloc);
	ch->hw_desc[ch->hw_ring_size - 1].control = IUDMA_D_CS_WRAP;

	while (ch->desc_count > 0) {
		struct bcm63xx_iudma_desc *desc = ch->desc[ch->read_desc];

		if (unlikely(desc == NULL))
			WARN(1, "%s: read desc == NULL on channel %u (hw_ring_size=%u, desc_count=%u, read_desc=%u, write_desc=%u)\n",
				__func__, ch->id, ch->hw_ring_size,
				ch->desc_count, ch->read_desc,
				ch->write_desc);
		else
			bcm63xx_iudma_desc_put(&desc->vd);
		ch->desc[ch->read_desc] = NULL;

		ch->read_desc++;
		if (ch->read_desc == ch->hw_ring_size)
			ch->read_desc = 0;
		ch->desc_count--;
	}

	ch->flowc_ring_size = 0;
	wmb();
}

static inline void bcm63xx_iudma_bufalloc_chan(struct bcm63xx_iudma_chan *ch,
	u32 val)
{
	struct bcm63xx_iudma *iudma = ch->ctrl;

	if (!bcm63xx_iudma_chan_has_sram(ch))
		n_writel(val, ch, IUDMA_N_BUFALLOC_REG);
	else if (bcm63xx_iudma_chan_has_flowc(ch))
		g_writel(val, iudma, IUDMA_G_FLOWC_BUFALLOC_REG(ch->id));
}

static inline void bcm63xx_iudma_chan_set_flowc(struct bcm63xx_iudma_chan *ch)
{
	struct bcm63xx_iudma *iudma = ch->ctrl;

	if (bcm63xx_iudma_chan_has_flowc(ch)) {
		u32 val;

		spin_lock_irq(&iudma->lock);
		val = g_readl(iudma, IUDMA_G_CFG_REG);
		if (ch->flowc)
			val |= IUDMA_G_FLOWC_ENABLE(ch->id);
		else
			val &= ~IUDMA_G_FLOWC_ENABLE(ch->id);
		g_writel(val, iudma, IUDMA_G_CFG_REG);
		spin_unlock_irq(&iudma->lock);
	}
}

static inline void bcm63xx_iudma_chan_set_flowc_thresh(
	struct bcm63xx_iudma_chan *ch)
{
	struct bcm63xx_iudma *iudma = ch->ctrl;

	if (!bcm63xx_iudma_chan_has_sram(ch)) {
		n_writel(5, ch, IUDMA_N_FLOWC_REG);
		n_writel(ch->flowc_ring_size, ch, IUDMA_N_LEN_REG);
	} else if (bcm63xx_iudma_chan_has_flowc(ch)) {
		/* set flow control low/high threshold to 1/3 / 2/3 */
		g_writel(ch->flowc_ring_size / 3, iudma,
			IUDMA_G_FLOWC_LO_THRESH_REG(ch->id));
		g_writel((ch->flowc_ring_size * 2) / 3, iudma,
			IUDMA_G_FLOWC_HI_THRESH_REG(ch->id));
	}
}

static inline void bcm63xx_iudma_configure_chan(struct bcm63xx_iudma_chan *ch)
{
	WARN(ch->desc_count != 0, "%s: desc_count=%u on channel %u\n",
		__func__, ch->desc_count, ch->id);
	ch->desc_count = 0;
	ch->read_desc = 0;
	ch->write_desc = 0;

	/* initialize flow control buffer allocation */
	if (bcm63xx_iudma_chan_has_sram(ch))
		bcm63xx_iudma_bufalloc_chan(ch, IUDMA_G_FLOWC_BUFALLOC_FORCE);
	else
		bcm63xx_iudma_bufalloc_chan(ch, IUDMA_N_BUFALLOC_FORCE);

	/* write ring address */
	if (bcm63xx_iudma_chan_has_sram(ch))
		s_writel(ch->hw_ring_base, ch, IUDMA_S_RSTART_REG);
	else
		n_writel(ch->hw_ring_base, ch, IUDMA_N_RSTART_REG);

	/* clear remaining state ram */
	if (bcm63xx_iudma_chan_has_sram(ch)) {
		s_writel(0, ch, IUDMA_S_STATE_DATA_REG);
		s_writel(0, ch, IUDMA_S_DESC_LEN_STATUS_REG);
		s_writel(0, ch, IUDMA_S_DESC_BASE_BUFPTR_REG);
	} else {
		n_writel(0, ch, IUDMA_N_FLOWC_REG);
	}

	/* set dma maximum burst len */
	if (bcm63xx_iudma_chan_has_sram(ch))
		c_writel(ch->maxburst, ch, IUDMA_C_MAX_BURST_REG);
	else
		n_writel(ch->maxburst, ch, IUDMA_N_MAX_BURST_REG);

	bcm63xx_iudma_chan_set_flowc(ch);
	bcm63xx_iudma_chan_set_flowc_thresh(ch);

	wmb();
}

static inline void bcm63xx_iudma_ack_chan_int(struct bcm63xx_iudma_chan *ch)
{
	u32 status;

	if (bcm63xx_iudma_chan_has_sram(ch))
		status = c_readl(ch, IUDMA_C_INT_STATUS_REG);
	else
		status = n_readl(ch, IUDMA_N_INT_STATUS_REG);

	if (bcm63xx_iudma_chan_has_sram(ch))
		c_writel(status, ch, IUDMA_C_INT_STATUS_REG);
	else
		n_writel(status, ch, IUDMA_N_INT_STATUS_REG);
}

static inline void bcm63xx_iudma_disable_chan_int(struct bcm63xx_iudma_chan *ch)
{
	if (bcm63xx_iudma_chan_has_sram(ch))
		c_writel(0, ch, IUDMA_C_INT_ENABLE_REG);
	else
		c_writel(0, ch, IUDMA_N_INT_ENABLE_REG);
}

static inline void bcm63xx_iudma_enable_chan_int(struct bcm63xx_iudma_chan *ch)
{
	if (bcm63xx_iudma_chan_has_sram(ch))
		c_writel(IUDMA_C_INT_BUF_DONE, ch, IUDMA_C_INT_ENABLE_REG);
	else
		n_writel(IUDMA_N_INT_BUF_DONE, ch, IUDMA_N_INT_ENABLE_REG);
}

#define IUDMA_STOP_TIMEOUT_US 500

static inline void bcm63xx_iudma_stop_chan(struct bcm63xx_iudma_chan *ch)
{
	unsigned int timeout = IUDMA_STOP_TIMEOUT_US;
	u32 cfg;

	bcm63xx_iudma_disable_chan_int(ch);

	do {
		u32 halt = (timeout > IUDMA_STOP_TIMEOUT_US / 2)
			? IUDMA_C_CFG_PKT_HALT : IUDMA_C_CFG_BURST_HALT;

		if (bcm63xx_iudma_chan_has_sram(ch)) {
			c_writel(halt, ch, IUDMA_C_CFG_REG);
			mb();
			cfg = c_readl(ch, IUDMA_C_CFG_REG);
			if (!(cfg & IUDMA_C_CFG_ENABLE))
				break;
		} else {
			n_writel(halt, ch, IUDMA_N_CFG_REG);
			mb();
			cfg = n_readl(ch, IUDMA_N_CFG_REG);
			if (!(cfg & IUDMA_N_CFG_ENABLE))
				break;
		}

		udelay(1);
	} while (--timeout);

	if (!timeout) {
		u32 hw_pos;

		if (bcm63xx_iudma_chan_has_sram(ch))
			hw_pos = s_readl(ch, IUDMA_S_STATE_DATA_REG);
		else
			hw_pos = ~0;

		dev_err(ch->ctrl->dev,
			"Unable to stop channel %u (hw_ring_size=%u, desc_count=%u, read_desc=%u, write_desc=%u, hw_pos=%08x, cfg=%08x)\n",
			ch->id, ch->hw_ring_size, ch->desc_count,
			ch->read_desc, ch->write_desc, hw_pos, cfg);
	}

	ch->running = false;

	bcm63xx_iudma_reset_chan(ch);
}

static inline void bcm63xx_iudma_start_chan(struct bcm63xx_iudma_chan *ch)
{
	if (bcm63xx_iudma_chan_has_sram(ch))
		c_writel(IUDMA_C_CFG_ENABLE, ch, IUDMA_C_CFG_REG);
	else
		n_writel(IUDMA_N_CFG_ENABLE | IUDMA_N_CFG_CHAINING |
			IUDMA_N_CFG_WRAP_ENABLE | IUDMA_N_CFG_FLOWC_ENABLE,
			ch, IUDMA_N_CFG_REG);

	if (!ch->running) {
		bcm63xx_iudma_enable_chan_int(ch);
		ch->running = true;
	}
}

static bool bcm63xx_iudma_complete_transactions(struct bcm63xx_iudma_chan *ch, bool all)
{
	unsigned int buffers = 0;

	while (ch->desc_count > 0 &&
			(bcm63xx_iudma_chan_is_tx(ch) ||
				buffers < IUDMA_MAX_RX_BUF_BURST || all)) {
		struct bcm63xx_iudma_hw_desc *hwd;
		struct bcm63xx_iudma_desc *desc;

		hwd = &ch->hw_desc[ch->read_desc];

		/* make sure we actually read the descriptor status at
		 * each loop */
		rmb();

		if (hwd->status & IUDMA_D_CS_OWNER)
			break;

		/* ensure other fields of the descriptor were not read
		 * before we checked ownership */
		rmb();

		desc = ch->desc[ch->read_desc];
		if (unlikely(desc == NULL)) {
			WARN(1, "%s: read desc == NULL on channel %u (hw_ring_size=%u, desc_count=%u, read_desc=%u, write_desc=%u)\n",
				__func__, ch->id, ch->hw_ring_size,
				ch->desc_count, ch->read_desc,
				ch->write_desc);
		} else {
			if (desc->context) {
				desc->context->length = hwd->length;
				desc->context->status = hwd->status &
					IUDMA_D_STA_CLIENT_MASK;
			}

			vchan_cookie_complete_notask(&desc->vd);
			ch->desc[ch->read_desc] = NULL;
		}

		ch->read_desc++;
		if (ch->read_desc == ch->hw_ring_size)
			ch->read_desc = 0;
		ch->desc_count--;

		buffers++;
	}

	if (bcm63xx_iudma_chan_is_rx(ch)) {
		/* RX is finished if fewer buffers than the burst limit were
		 * processed and there are some descriptors submitted
		 */
		return buffers < IUDMA_MAX_RX_BUF_BURST && ch->desc_count > 0;
	} else {
		/* TX is finished if no buffers were processed
		 * or there are no descriptors submitted
		 */
		return buffers == 0 || ch->desc_count == 0;
	}
}

static void bcm63xx_iudma_issue_transactions(struct bcm63xx_iudma_chan *ch)
{
	while (ch->desc_count < ch->hw_ring_size) {
		struct virt_dma_desc *vd = vchan_next_desc(&ch->vc);
		struct bcm63xx_iudma_desc *desc;
		struct bcm63xx_iudma_hw_desc *hwd;
		u16 control = IUDMA_D_CS_OWNER;

		if (!vd)
			break;

		list_del(&vd->node);

		desc = to_bcm63xx_iudma_desc(&vd->tx);
		WARN(unlikely(ch->desc[ch->write_desc] != NULL),
			"%s: write desc != NULL on channel %u (hw_ring_size=%u, desc_count=%u, read_desc=%u, write_desc=%u)\n",
				__func__, ch->id, ch->hw_ring_size,
				ch->desc_count, ch->read_desc,
				ch->write_desc);
		ch->desc[ch->write_desc] = desc;

		if (desc->context)
			control |= desc->context->control
					& IUDMA_D_CTL_CLIENT_MASK;

		if (ch->write_desc == ch->hw_ring_size - 1)
			control |= IUDMA_D_CS_WRAP;

		/* dma might be already polling, make sure we update desc
		 * fields in correct order */
		hwd = &ch->hw_desc[ch->write_desc];
		hwd->address = desc->address;
		hwd->length = desc->length;
		wmb();
		hwd->control = control;
		wmb();

		/* tell dma engine we allocated one buffer */
		if (bcm63xx_iudma_chan_is_rx(ch))
			bcm63xx_iudma_bufalloc_chan(ch, 1);

		ch->write_desc++;
		if (ch->write_desc == ch->hw_ring_size)
			ch->write_desc = 0;
		ch->desc_count++;
	}

	if (ch->desc_count)
		bcm63xx_iudma_start_chan(ch);
}

static void bcm63xx_iudma_tasklet(unsigned long data)
{
	struct bcm63xx_iudma_chan *rx_ch = (struct bcm63xx_iudma_chan *)data;
	struct bcm63xx_iudma_chan *tx_ch = rx_ch->pair;
	bool finished;

	bcm63xx_iudma_ack_chan_int(rx_ch);
	bcm63xx_iudma_ack_chan_int(tx_ch);

	spin_lock(&rx_ch->vc.lock);
	finished = bcm63xx_iudma_complete_transactions(rx_ch, false);
	spin_unlock(&rx_ch->vc.lock);

	vchan_complete_task(&rx_ch->vc);

	spin_lock(&tx_ch->vc.lock);
	finished &= bcm63xx_iudma_complete_transactions(tx_ch, false);
	spin_unlock(&tx_ch->vc.lock);

	vchan_complete_task(&tx_ch->vc);

	if (finished) {
		/* re-enable interrupts */

		spin_lock(&rx_ch->vc.lock);
		rx_ch->running = false;
		if (rx_ch->desc_count)
			bcm63xx_iudma_start_chan(rx_ch);
		spin_unlock(&rx_ch->vc.lock);

		spin_lock(&tx_ch->vc.lock);
		tx_ch->running = false;
		if (tx_ch->desc_count)
			bcm63xx_iudma_start_chan(tx_ch);
		spin_unlock(&tx_ch->vc.lock);
	} else {
		tasklet_schedule(&rx_ch->task);
	}
}

static irqreturn_t bcm63xx_iudma_rx_interrupt(int irq, void *data)
{
	struct bcm63xx_iudma_chan *rx_ch = data;
	struct bcm63xx_iudma_chan *tx_ch = rx_ch->pair;

	bcm63xx_iudma_disable_chan_int(tx_ch);
	bcm63xx_iudma_disable_chan_int(rx_ch);
	tasklet_schedule(&rx_ch->task);

	return IRQ_HANDLED;
}

static irqreturn_t bcm63xx_iudma_tx_interrupt(int irq, void *data)
{
	struct bcm63xx_iudma_chan *tx_ch = data;
	struct bcm63xx_iudma_chan *rx_ch = tx_ch->pair;

	bcm63xx_iudma_disable_chan_int(rx_ch);
	bcm63xx_iudma_disable_chan_int(tx_ch);
	tasklet_schedule(&rx_ch->task);

	return IRQ_HANDLED;
}


/* DMA engine interface functions */

static int bcm63xx_iudma_alloc_chan(struct dma_chan *dchan)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct device *dev = ch->ctrl->dev;
	LIST_HEAD(alloc_desc_pool);
	int ret, i;

	ch->hw_desc = dma_alloc_coherent(dev, ch->hw_ring_alloc,
					&ch->hw_ring_base, GFP_KERNEL);
	if (!ch->hw_desc)
		return -ENOMEM;

	ch->desc = devm_kcalloc(dev, ch->hw_ring_size,
					sizeof(*ch->desc), GFP_KERNEL);
	if (!ch->desc) {
		ret = -ENOMEM;
		goto free_hw_desc;
	}

	for (i = 0; i < ch->hw_ring_size; i++) {
		struct bcm63xx_iudma_desc *desc;

		desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
		if (!desc) {
			ret = -ENOMEM;
			goto free_pool;
		}
		list_add_tail(&desc->node, &alloc_desc_pool);
	}

	spin_lock_bh(&ch->vc.lock);
	ch->desc_count = 0;
	ch->read_desc = 0;
	ch->write_desc = 0;
	ch->running = false;

	/* set safe initial dma maximum burst len */
	ch->maxburst = 1;

	/* disable flow control unless requested by slave config */
	ch->flowc = false;
	ch->flowc_ring_size = 0;

	bcm63xx_iudma_reset_chan(ch);
	bcm63xx_iudma_reset_ring(ch);
	bcm63xx_iudma_configure_chan(ch);
	spin_unlock_bh(&ch->vc.lock);

	if (bcm63xx_iudma_chan_is_rx(ch))
		ret = request_irq(ch->irq, bcm63xx_iudma_rx_interrupt,
			0, ch->name, ch);
	else
		ret = request_irq(ch->irq, bcm63xx_iudma_tx_interrupt,
			0, ch->name, ch);
	if (ret)
		goto free_pool;

	spin_lock_bh(&ch->pool_lock);
	list_splice_init(&alloc_desc_pool, &ch->desc_pool);
	spin_unlock_bh(&ch->pool_lock);
	return 0;

free_pool:
	while (!list_empty(&alloc_desc_pool)) {
		struct bcm63xx_iudma_desc *desc = list_first_entry(
			&alloc_desc_pool, struct bcm63xx_iudma_desc, node);

		list_del(&desc->node);
		devm_kfree(dev, desc);
	}

	devm_kfree(dev, ch->desc);
	ch->desc = NULL;

free_hw_desc:
	dma_free_coherent(dev, ch->hw_ring_alloc,
		ch->hw_desc, ch->hw_ring_base);
	ch->hw_desc = NULL;
	ch->hw_ring_base = 0;
	return ret;
}

static void bcm63xx_iudma_free_chan(struct dma_chan *dchan)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct bcm63xx_iudma_chan *rx_ch;
	struct device *dev = ch->ctrl->dev;

	if (bcm63xx_iudma_chan_is_rx(ch))
		rx_ch = ch;
	else
		rx_ch = ch->pair;

	/* stop tasklet from calling completion callbacks after the channel
	 * is stopped (see bcm63xx_iudma_terminate_all)
	 */
	tasklet_disable(&rx_ch->task);

	spin_lock_bh(&ch->vc.lock);
	bcm63xx_iudma_stop_chan(ch);
	bcm63xx_iudma_reset_ring(ch);

	devm_kfree(dev, ch->desc);
	dma_free_coherent(dev, ch->hw_ring_alloc,
		ch->hw_desc, ch->hw_ring_base);

	ch->desc_count = 0;
	ch->read_desc = 0;
	ch->write_desc = 0;
	ch->running = false;

	ch->desc = NULL;
	ch->hw_desc = NULL;
	ch->hw_ring_base = 0;
	spin_unlock_bh(&ch->vc.lock);

	tasklet_enable(&rx_ch->task);

	/* run the tasklet in case the other channel was using it with
	 * interrupts disabled
	 */
	tasklet_schedule(&rx_ch->task);

	free_irq(ch->irq, ch);
	vchan_free_chan_resources(&ch->vc);

	spin_lock_bh(&ch->pool_lock);
	while (!list_empty(&ch->desc_pool)) {
		struct bcm63xx_iudma_desc *desc = list_first_entry(
			&ch->desc_pool, struct bcm63xx_iudma_desc, node);
		list_del(&desc->node);
		devm_kfree(dev, desc);
	}
	spin_unlock_bh(&ch->pool_lock);
}

static struct dma_async_tx_descriptor *bcm63xx_iudma_prep_slave_sg(
	struct dma_chan *dchan,
	struct scatterlist *sgl, unsigned int sg_len,
	enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct bcm63xx_iudma_desc *desc = NULL;

	switch (direction) {
	case DMA_DEV_TO_MEM:
		if (!bcm63xx_iudma_chan_is_rx(ch))
			return ERR_PTR(-EINVAL);
		break;

	case DMA_MEM_TO_DEV:
		if (!bcm63xx_iudma_chan_is_tx(ch))
			return ERR_PTR(-EINVAL);
		break;

	default:
		return ERR_PTR(-EINVAL);
	}

	if (!sgl || !sg_len || !context)
		return ERR_PTR(-EINVAL);

	if (sg_len != 1) {
		/* It is not possible to support sg_len > 1 on
		 * receive channels because the data is variable
		 * length packets and there's no way to tell the
		 * hardware that any sg descriptors after the actual
		 * packet are to be skipped without stalling after
		 * every packet.
		 *
		 * It is possible to support sg_len > 1 on transmit
		 * channels but descriptor submission to the hardware
		 * ring would need to ensure that all the descriptors
		 * will fit on the ring. Start/end packet flags would
		 * need to be adjusted for each hardware descriptor.
		 */
		return ERR_PTR(-ENOSYS);
	}

	if (sg_dma_len(sgl) > IUDMA_D_LEN_MAX)
		return ERR_PTR(-EINVAL);

	/* receive buffer must be aligned to double the burst size */
	if (bcm63xx_iudma_chan_is_rx(ch) &&
			sg_dma_len(sgl) % (ch->maxburst * 2) != 0)
		return ERR_PTR(-EINVAL);

	desc = bcm63xx_iudma_desc_get(ch);
	if (!desc)
		return ERR_PTR(-EBUSY);

	desc->address = sg_dma_address(sgl);
	desc->length = sg_dma_len(sgl);
	desc->context = context;

	return vchan_tx_prep(&ch->vc, &desc->vd, flags);
}

static void bcm63xx_iudma_issue_pending(struct dma_chan *dchan)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);

	spin_lock_bh(&ch->vc.lock);
	if (vchan_issue_pending(&ch->vc)) {
		bcm63xx_iudma_issue_transactions(ch);
		if (ch->desc_count > ch->flowc_ring_size) {
			ch->flowc_ring_size = ch->desc_count;
			bcm63xx_iudma_chan_set_flowc_thresh(ch);
		}
	}
	spin_unlock_bh(&ch->vc.lock);
}

static int bcm63xx_iudma_config(struct dma_chan *dchan,
	struct dma_slave_config *config)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	unsigned int maxburst;

	if (bcm63xx_iudma_chan_is_rx(ch)) {
		if (config->src_addr)
			return -EINVAL;

		if (config->src_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES)
			return -EINVAL;

		maxburst = config->src_maxburst;
	} else if (bcm63xx_iudma_chan_is_tx(ch)) {
		if (config->dst_addr)
			return -EINVAL;

		if (config->dst_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES)
			return -EINVAL;

		maxburst = config->dst_maxburst;
	}

	/* it must be possible to fit two bursts into the receive buffer */
	if (maxburst < 1 || maxburst * 4 * 2 > IUDMA_D_LEN_MAX)
		return -EINVAL;

	spin_lock_bh(&ch->vc.lock);
	ch->flowc = config->device_fc;
	ch->flowc_ring_size = 0;
	ch->maxburst = maxburst;

	bcm63xx_iudma_chan_set_flowc(ch);
	bcm63xx_iudma_chan_set_flowc_thresh(ch);

	if (bcm63xx_iudma_chan_has_sram(ch))
		c_writel(ch->maxburst, ch, IUDMA_C_MAX_BURST_REG);
	else
		n_writel(ch->maxburst, ch, IUDMA_N_MAX_BURST_REG);

	wmb();
	spin_unlock_bh(&ch->vc.lock);

	return 0;
}

static int bcm63xx_iudma_terminate_all(struct dma_chan *dchan)
{
	struct bcm63xx_iudma_chan *ch = to_bcm63xx_iudma_chan(dchan);
	struct bcm63xx_iudma_chan *rx_ch;
	LIST_HEAD(head);

	if (bcm63xx_iudma_chan_is_rx(ch))
		rx_ch = ch;
	else
		rx_ch = ch->pair;

	tasklet_disable(&rx_ch->task);

	spin_lock_bh(&ch->vc.lock);
	bcm63xx_iudma_stop_chan(ch);
	bcm63xx_iudma_complete_transactions(ch, true);
	bcm63xx_iudma_reset_ring(ch);
	bcm63xx_iudma_configure_chan(ch);
	list_splice_tail_init(&ch->vc.desc_submitted, &head);
	list_splice_tail_init(&ch->vc.desc_issued, &head);
	spin_unlock_bh(&ch->vc.lock);

	/* Run the completion task before returning so that the client can
	 * safely free data associated with any completed descriptors.
	 *
	 * Do this with the tasklet disabled, otherwise it can acquire the
	 * completed list before we return and process it after the client
	 * starts freeing data.
	 *
	 * CPU 0                           | CPU 1
	 * =====                           | =====
	 *                                 |
	 *                                 | tasklet:
	 * terminate_all():                |
	 *   spin_lock_bh(&ch->vc.lock)    |
	 *                                 |   vchan_complete_task():
	 *                                 |      spin_lock(&ch->vc.lock)
	 *   spin_unlock_bh(&ch->vc.lock)  |      :: waiting on lock ::
	 *                                 |      <move desc_completed list>
	 *                                 |      spin_unlock_bh(&ch->vc.lock)
	 *   vchan_complete_task():        |
	 *     <desc_completed list empty> |
	 *   return                        |
	 *                                 |     for each completed desc:
	 *                                 |       <client callback>:
	 * spin_lock_bh(<client lock>)     |
	 * <free client's data>            |
	 *                                 |       spin_lock_bh(<client lock>)
	 * spin_unlock_bh(<client lock>)   |       :: waiting on lock ::
	 *                                 |       <callback runs>
	 *                                 |       <access to free'd data>
	 *
	 * The tasklet has been disabled earlier on before we stop the channel
	 * so that we're not waiting for it to finish while everything is
	 * stopped.
	 *
	 * Callbacks are run with BH disabled so that we're in the same context
	 * that a tasklet would have (otherwise it confuses lockdep).
	 */
	local_bh_disable();
	vchan_complete_task(&ch->vc);
	local_bh_enable();

	tasklet_enable(&rx_ch->task);

	/* run the tasklet in case the other channel was using it with
	 * interrupts disabled
	 */
	tasklet_schedule(&rx_ch->task);

	vchan_dma_desc_free_list(&ch->vc, &head);
	return 0;
}


/* Driver functions */

static void bcm63xx_iudma_set_chan_name(struct bcm63xx_iudma_chan *ch,
	struct device *dev)
{
	const char *prefix = dev_name(dev);
	char *name;
	size_t name_len = strlen(prefix) + 5 /* ":chan" */ + 32 /* "%u" */ + 1;
	int ret;

	name = devm_kzalloc(dev, name_len, GFP_KERNEL);
	if (!name) {
		ch->name = prefix;
		return;
	}

	ret = snprintf(name, name_len, "%s:chan%u", prefix, ch->id);
	if (ret < 0) {
		devm_kfree(dev, name);
		ch->name = prefix;
		return;
	}

	ch->name = name;
}

static int bcm63xx_iudma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct bcm63xx_iudma *iudma;
	struct dma_device *ddev;
	const struct bcm63xx_iudma_platform_data *pd;
	struct resource *res;
	u32 dma_requests;
	unsigned int i;
	int ret;

	pd = dev_get_platdata(dev);
	if (!pd)
		pd = of_device_get_match_data(dev);
	if (!pd)
		return -EINVAL;

	iudma = devm_kzalloc(dev,
		sizeof(*iudma) + sizeof(*iudma->chan) * pd->n_channels,
		GFP_KERNEL);
	if (!iudma)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iudma->base_g = devm_ioremap_resource(dev, res);
	if (IS_ERR(iudma->base_g)) {
		ret = PTR_ERR(iudma->base_g);
		dev_err(dev, "failed to get register base: %d\n", ret);
		return ret;
	}

	if (pd->n_requests) {
		dma_requests = pd->n_requests;
	} else {
		ret = of_property_read_u32(np, "dma-requests", &dma_requests);
		if (ret || dma_requests == 0)
			dma_requests = IUDMA_DEFAULT_NUM_REQUESTS;
	}

	if (dma_requests > IUDMA_MAX_NUM_REQUESTS)
		dma_requests = IUDMA_MAX_NUM_REQUESTS;

	spin_lock_init(&iudma->lock);

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "failed to set DMA bit mask: %d\n", ret);
		return ret;
	}

	dma_set_max_seg_size(dev, IUDMA_D_LEN_MAX);

	iudma->dev = dev;
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
	ddev->device_tx_status = dma_cookie_status;
	ddev->device_config = bcm63xx_iudma_config;
	ddev->device_terminate_all = bcm63xx_iudma_terminate_all;

	bcm63xx_iudma_disable_ctl(iudma);

	for (i = 0; i < pd->n_channels; i++) {
		struct bcm63xx_iudma_chan *ch = &iudma->chan[i];

		ch->ctrl = iudma;
		ch->id = i;
		if (bcm63xx_iudma_chan_is_rx(ch))
			ch->pair = &iudma->chan[i + 1];
		else
			ch->pair = &iudma->chan[i - 1];

		bcm63xx_iudma_set_chan_name(ch, dev);

		ch->irq = platform_get_irq(pdev, i);
		if (ch->irq < 0)
			break;

		if (pd->dmas_offset) {
			/* With state RAM */
			ch->base_c = iudma->base_g + pd->dmac_offset +
					IUDMA_C_CHANNEL_BASE_REG(i);
			ch->base_s = iudma->base_g + pd->dmas_offset +
					IUDMA_S_CHANNEL_BASE_REG(i);
		} else {
			/* No state RAM */
			ch->base_c = iudma->base_g + pd->dmac_offset +
					IUDMA_N_CHANNEL_BASE_REG(i);
			ch->base_s = 0;
		}

		ch->hw_ring_size = dma_requests;
		ch->hw_ring_alloc = sizeof(*ch->hw_desc) * ch->hw_ring_size;

		spin_lock_init(&ch->pool_lock);
		INIT_LIST_HEAD(&ch->desc_pool);
		if (bcm63xx_iudma_chan_is_rx(ch))
			tasklet_init(&ch->task, bcm63xx_iudma_tasklet,
				(unsigned long)ch);

		ch->vc.desc_free = bcm63xx_iudma_desc_put;

		vchan_init(&ch->vc, ddev);

		bcm63xx_iudma_stop_chan(ch);

		iudma->n_channels++;
	}

	if (iudma->n_channels % 2) {
		dev_warn(dev, "ignoring odd numbered channel, only pairs of channels are supported\n");
		iudma->n_channels--;
		list_del(&iudma->chan[iudma->n_channels].vc.chan.device_node);
	}

	if (!iudma->n_channels) {
		dev_err(dev, "device has no channel IRQs\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, iudma);

	bcm63xx_iudma_enable_ctl(iudma);

	if (np) {
		ret = of_dma_controller_register(np,
			of_dma_xlate_by_chan_id, ddev);
		if (ret) {
			dev_err(dev, "failed to register DMA controller: %d\n", ret);
			return ret;
		}
	}

	ret = dma_async_device_register(ddev);
	if (ret) {
		dev_err(dev, "failed to register DMA engine device: %d\n", ret);
		goto of_dma_unregister;
	}

	dev_info(dev, "registered BCM63xx IUDMA controller at MMIO 0x%p (%u channels)",
		iudma->base_g, iudma->n_channels);
	return 0;

of_dma_unregister:
	if (np)
		of_dma_controller_free(np);
	return ret;
}

static int bcm63xx_iudma_remove(struct platform_device *pdev)
{
	struct bcm63xx_iudma *iudma = platform_get_drvdata(pdev);

	dma_async_device_unregister(&iudma->ddev);
	of_dma_controller_free(pdev->dev.of_node);
	bcm63xx_iudma_disable_ctl(iudma);

	return 0;
}

static const struct bcm63xx_iudma_platform_data bcm63xx_iudma_8ch = {
	.n_channels = 8,
	.dmac_offset = 0x40,
	.dmas_offset = 0,
};

static const struct bcm63xx_iudma_platform_data bcm63xx_iudma_16ch = {
	.n_channels = 16,
	.dmac_offset = 0x100,
	.dmas_offset = 0x200,
};

static const struct bcm63xx_iudma_platform_data bcm63xx_iudma_32ch = {
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
MODULE_DEVICE_TABLE(of, bcm63xx_iudma_of_match);

static struct platform_driver bcm63xx_iudma_driver = {
	.probe  = bcm63xx_iudma_probe,
	.remove = bcm63xx_iudma_remove,
	.driver = {
		.name = "bcm63xx-iudma",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bcm63xx_iudma_of_match),
	},
};

module_platform_driver(bcm63xx_iudma_driver);

MODULE_ALIAS("platform:bcm63xx-iudma");
MODULE_DESCRIPTION("BCM63xx IUDMA engine driver");
MODULE_AUTHOR("Simon Arlott");
MODULE_LICENSE("GPL");

