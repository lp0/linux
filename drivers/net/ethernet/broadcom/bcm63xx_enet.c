/*
 * Driver for BCM963xx builtin Ethernet mac
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/etherdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/crc32.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/if_vlan.h>
#include <linux/of_dma.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>

#include <linux/bcm63xx_dev_enet.h>
#include "bcm63xx_enet.h"

static char bcm_enet_driver_name[] = "bcm63xx_enet";
static char bcm_enet_driver_version[] = "2.0";

static int copybreak __read_mostly = 128;
module_param(copybreak, int, 0);
MODULE_PARM_DESC(copybreak, "Receive copy threshold");

static void bcm_enet_rx_complete(void *data);
static void bcm_enet_tx_complete(void *data);

/*
 * io helpers to access mac registers
 */
static inline u32 enet_readl(struct bcm_enet_priv *priv, u32 off)
{
	return __raw_readl(priv->base + off);
}

static inline void enet_writel(struct bcm_enet_priv *priv,
			       u32 val, u32 off)
{
	__raw_writel(val, priv->base + off);
}

/*
 * io helpers to access switch registers
 */
static inline u32 enetsw_readl(struct bcm_enet_priv *priv, u32 off)
{
	return __raw_readl(priv->base + off);
}

static inline void enetsw_writel(struct bcm_enet_priv *priv,
				 u32 val, u32 off)
{
	__raw_writel(val, priv->base + off);
}

static inline u16 enetsw_readw(struct bcm_enet_priv *priv, u32 off)
{
	return __raw_readw(priv->base + off);
}

static inline void enetsw_writew(struct bcm_enet_priv *priv,
				 u16 val, u32 off)
{
	__raw_writew(val, priv->base + off);
}

static inline u8 enetsw_readb(struct bcm_enet_priv *priv, u32 off)
{
	return __raw_readb(priv->base + off);
}

static inline void enetsw_writeb(struct bcm_enet_priv *priv,
				 u8 val, u32 off)
{
	__raw_writeb(val, priv->base + off);
}

/*
 * write given data into mii register and wait for transfer to end
 * with timeout (average measured transfer time is 25us)
 */
static int do_mdio_op(struct bcm_enet_priv *priv, unsigned int data)
{
	int limit;

	/* make sure mii interrupt status is cleared */
	enet_writel(priv, ENET_IR_MII, ENET_IR_REG);

	enet_writel(priv, data, ENET_MIIDATA_REG);
	wmb();

	/* busy wait on mii interrupt bit, with timeout */
	limit = 1000;
	do {
		if (enet_readl(priv, ENET_IR_REG) & ENET_IR_MII)
			break;
		udelay(1);
	} while (limit-- > 0);

	return (limit < 0) ? 1 : 0;
}

/*
 * MII internal read callback
 */
static int bcm_enet_mdio_read(struct bcm_enet_priv *priv, int mii_id,
			      int regnum)
{
	u32 tmp, val;

	tmp = regnum << ENET_MIIDATA_REG_SHIFT;
	tmp |= 0x2 << ENET_MIIDATA_TA_SHIFT;
	tmp |= mii_id << ENET_MIIDATA_PHYID_SHIFT;
	tmp |= ENET_MIIDATA_OP_READ_MASK;

	if (do_mdio_op(priv, tmp))
		return -1;

	val = enet_readl(priv, ENET_MIIDATA_REG);
	val &= 0xffff;
	return val;
}

/*
 * MII internal write callback
 */
static int bcm_enet_mdio_write(struct bcm_enet_priv *priv, int mii_id,
			       int regnum, u16 value)
{
	u32 tmp;

	tmp = (value & 0xffff) << ENET_MIIDATA_DATA_SHIFT;
	tmp |= 0x2 << ENET_MIIDATA_TA_SHIFT;
	tmp |= regnum << ENET_MIIDATA_REG_SHIFT;
	tmp |= mii_id << ENET_MIIDATA_PHYID_SHIFT;
	tmp |= ENET_MIIDATA_OP_WRITE_MASK;

	(void)do_mdio_op(priv, tmp);
	return 0;
}

/*
 * MII read callback from phylib
 */
static int bcm_enet_mdio_read_phylib(struct mii_bus *bus, int mii_id,
				     int regnum)
{
	return bcm_enet_mdio_read(bus->priv, mii_id, regnum);
}

/*
 * MII write callback from phylib
 */
static int bcm_enet_mdio_write_phylib(struct mii_bus *bus, int mii_id,
				      int regnum, u16 value)
{
	return bcm_enet_mdio_write(bus->priv, mii_id, regnum, value);
}

/*
 * MII read callback from mii core
 */
static int bcm_enet_mdio_read_mii(struct net_device *dev, int mii_id,
				  int regnum)
{
	return bcm_enet_mdio_read(netdev_priv(dev), mii_id, regnum);
}

/*
 * MII write callback from mii core
 */
static void bcm_enet_mdio_write_mii(struct net_device *dev, int mii_id,
				    int regnum, int value)
{
	bcm_enet_mdio_write(netdev_priv(dev), mii_id, regnum, value);
}

/*
 * refill rx queue
 */
static void bcm_enet_refill_rx(struct net_device *dev)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);
	struct device *kdev = &priv->pdev->dev;

	if (!priv->rx_running)
		return;

	while (!list_empty(&priv->rx_inactive)) {
		struct bcm_enet_pkt *pkt;
		struct dma_async_tx_descriptor *desc;
		dma_cookie_t cookie;

		pkt = list_first_entry(&priv->rx_inactive,
			struct bcm_enet_pkt, node);

		if (!pkt->skb) {
			pkt->skb = netdev_alloc_skb(dev, priv->rx_skb_size);
			if (!pkt->skb)
				break;

			pkt->buf = dma_map_single(kdev, pkt->skb->data,
				priv->rx_skb_size, DMA_FROM_DEVICE);
			if (unlikely(dma_mapping_error(kdev, pkt->buf))) {
				dev_err_ratelimited(kdev, "rx dma map error\n");
				dev_kfree_skb(pkt->skb);
				pkt->skb = NULL;
				break;
			}
		}

		pkt->ctx.control = 0;

		desc = dmaengine_prep_bcm63xx_single(priv->rx_dma,
				pkt->buf, priv->rx_skb_size, DMA_DEV_TO_MEM,
				DMA_PREP_INTERRUPT, &pkt->ctx);
		if (unlikely(IS_ERR_OR_NULL(desc))) {
			dev_err_ratelimited(kdev,
				"rx dma prep err=%ld\n", PTR_ERR(desc));
			break;
		}

		desc->callback = bcm_enet_rx_complete;
		desc->callback_param = pkt;
		cookie = dmaengine_submit(desc);
		if (unlikely(dma_submit_error(cookie))) {
			dev_err_ratelimited(kdev,
				"rx dma submit err=%ld\n", PTR_ERR(desc));
			break;
		}

		list_move_tail(&pkt->node, &priv->rx_active);
	}

	dma_async_issue_pending(priv->rx_dma);

	/* If rx queue is still empty, set a timer to try allocating
	 * again at a later time. */
	if (list_empty(&priv->rx_active) && netif_running(dev)) {
		dev_warn(&priv->pdev->dev, "unable to refill rx queue\n");
		priv->rx_timeout.expires = jiffies + HZ;

		/* Timer could already be scheduled during a call to
		 * adjust the mtu or ring size
		 */
		del_timer_sync(&priv->rx_timeout);
		add_timer(&priv->rx_timeout);
	}
}

/*
 * timer callback to defer refill rx queue in case we're OOM
 */
static void bcm_enet_refill_rx_timer(unsigned long data)
{
	struct net_device *dev;
	struct bcm_enet_priv *priv;

	dev = (struct net_device *)data;
	priv = netdev_priv(dev);

	spin_lock(&priv->rx_lock);
	bcm_enet_refill_rx((struct net_device *)data);
	spin_unlock(&priv->rx_lock);
}

static void bcm_enet_rx_complete(void *data)
{
	struct bcm_enet_pkt *pkt = data;
	struct sk_buff *skb = pkt->skb;
	struct net_device *dev = skb->dev;
	struct bcm_enet_priv *priv = netdev_priv(dev);
	struct device *kdev = &priv->pdev->dev;
	unsigned int len;
	struct sk_buff *nskb;

	/* if the packet does not have start of packet _and_
	 * end of packet flag set, then just discard it */
	if (!(pkt->ctx.status & IUDMA_D_CS_SOP))
		goto out;

	if (!(pkt->ctx.status & IUDMA_D_CS_EOP)) {
		dev->stats.rx_errors++;
		dev->stats.rx_over_errors++;
		goto out;
	}

	/* discard packet if it's marked as bad */
	if (!priv->enet_is_sw &&
	    unlikely(pkt->ctx.status & IUDMA_D_STA_ENET_ERR_MASK)) {
		dev->stats.rx_errors++;

		if (pkt->ctx.status & IUDMA_D_STA_ENET_OVSIZE)
			dev->stats.rx_over_errors++;
		if (pkt->ctx.status & IUDMA_D_STA_ENET_CRC)
			dev->stats.rx_crc_errors++;
		if (pkt->ctx.status & IUDMA_D_STA_ENET_UNDER)
			dev->stats.rx_frame_errors++;
		if (pkt->ctx.status & IUDMA_D_STA_ENET_OV)
			dev->stats.rx_fifo_errors++;
		goto out;
	}

	/* valid packet */
	len = pkt->ctx.length;
	if (len > priv->rx_skb_size) {
		/* RX stops working when this happens */
		dev_emerg_ratelimited(kdev, "receive buffer overflow: %u > %u\n",
			len, priv->rx_skb_size);
		dev->stats.rx_errors++;
		dev->stats.rx_over_errors++;
		goto out;
	}
	/* don't include FCS */
	len -= ETH_FCS_LEN;

	if (len < copybreak && copybreak < priv->rx_skb_size)
		nskb = netdev_alloc_skb(dev, len);
	else
		nskb = NULL;

	if (nskb) {
		dma_sync_single_for_cpu(kdev, pkt->buf,
					len, DMA_FROM_DEVICE);
		memcpy(nskb->data, skb->data, len);
		dma_sync_single_for_device(kdev, pkt->buf,
					len, DMA_FROM_DEVICE);
		skb = nskb;
	} else {
		dma_unmap_single(&priv->pdev->dev, pkt->buf,
				 priv->rx_skb_size, DMA_FROM_DEVICE);
		pkt->skb = NULL;
	}

	skb_put(skb, len);
	skb->protocol = eth_type_trans(skb, dev);
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += len;
	netif_receive_skb(skb);

out:
	spin_lock(&priv->rx_lock);
	list_move_tail(&pkt->node, &priv->rx_inactive);
	bcm_enet_refill_rx(dev);
	spin_unlock(&priv->rx_lock);
}

static void bcm_enet_tx_complete(void *data)
{
	struct bcm_enet_pkt *pkt = data;
	struct sk_buff *skb = pkt->skb;
	struct net_device *dev = skb->dev;
	struct bcm_enet_priv *priv = netdev_priv(dev);
	struct device *kdev = &priv->pdev->dev;

	if (pkt->ctx.status & IUDMA_D_STA_ENET_UNDER)
		dev->stats.tx_errors++;

	dma_unmap_single(kdev, pkt->buf, skb->len, DMA_TO_DEVICE);
	dev_kfree_skb(pkt->skb);

	spin_lock(&priv->tx_lock);
	pkt->skb = NULL;
	list_move_tail(&pkt->node, &priv->tx_inactive);

	if (priv->tx_running && netif_queue_stopped(dev))
		netif_wake_queue(dev);
	spin_unlock(&priv->tx_lock);
}

static void bcm_enet_stop_rx_dma(struct net_device *dev)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);

	spin_lock_bh(&priv->rx_lock);
	priv->rx_running = false;
	spin_unlock_bh(&priv->rx_lock);

	dmaengine_terminate_all(priv->rx_dma);

	spin_lock_bh(&priv->rx_lock);
	list_splice_tail_init(&priv->rx_active, &priv->rx_inactive);
	spin_unlock_bh(&priv->rx_lock);
}

static void bcm_enet_start_rx_dma(struct net_device *dev)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);

	spin_lock_bh(&priv->rx_lock);
	priv->rx_running = true;
	bcm_enet_refill_rx(dev);
	spin_unlock_bh(&priv->rx_lock);
}

static void bcm_enet_stop_tx_dma(struct net_device *dev)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);
	struct device *kdev = &priv->pdev->dev;
	struct bcm_enet_pkt *pkt;

	spin_lock_bh(&priv->tx_lock);
	priv->tx_running = false;
	netif_stop_queue(dev);
	spin_unlock_bh(&priv->tx_lock);

	dmaengine_terminate_all(priv->tx_dma);

	/* discard active tx packets */
	spin_lock_bh(&priv->tx_lock);
	while (!list_empty(&priv->tx_active)) {
		pkt = list_first_entry(&priv->tx_active,
			struct bcm_enet_pkt, node);

		dma_unmap_single(kdev, pkt->buf, pkt->skb->len, DMA_TO_DEVICE);
		kfree_skb(pkt->skb);

		pkt->skb = NULL;
		list_move_tail(&pkt->node, &priv->tx_inactive);
	}
	spin_unlock_bh(&priv->tx_lock);
}

static void bcm_enet_start_tx_dma(struct net_device *dev)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);

	spin_lock_bh(&priv->tx_lock);
	priv->tx_running = true;
	netif_start_queue(dev);
	spin_unlock_bh(&priv->tx_lock);
}

static bool bcm_enet_dma_chan_filter(struct dma_chan *chan, void *param)
{
	const struct bcm63xx_enet_platform_dma_data *pd = param;

	return chan->device->dev == pd->dma_dev && chan->chan_id == pd->chan_id;
}

static int bcm_enet_request_dma(struct net_device *dev)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);
	struct device *kdev = &priv->pdev->dev;
	struct device_node *np = kdev->of_node;
	dma_cap_mask_t dma_cap_mask;
	struct dma_slave_config dma_cfg;
	int i, ret;

	INIT_LIST_HEAD(&priv->rx_active);
	INIT_LIST_HEAD(&priv->rx_inactive);
	INIT_LIST_HEAD(&priv->tx_active);
	INIT_LIST_HEAD(&priv->tx_inactive);

	dma_cap_zero(dma_cap_mask);
	dma_cap_set(DMA_SLAVE, dma_cap_mask);

	if (np) {
		priv->rx_dma = dma_request_slave_channel_reason(kdev, "rx");
	} else {
		priv->rx_dma = dma_request_channel(dma_cap_mask,
					bcm_enet_dma_chan_filter,
					&priv->pd_rx_dma);
		if (!priv->rx_dma)
			priv->rx_dma = ERR_PTR(-ENODEV);
	}
	if (IS_ERR(priv->rx_dma)) {
		ret = PTR_ERR(priv->rx_dma);
		netdev_info(dev, "unable to request RX DMA channel: %d\n",
			ret);
		return ret;
	}

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dma_cfg.src_maxburst = priv->dma_maxburst;

	ret = dmaengine_slave_config(priv->rx_dma, &dma_cfg);
	if (ret) {
		netdev_info(dev,
			"unable to configure RX DMA channel: %d\n", ret);
		goto out_release_rx_dma;
	}

	if (np) {
		priv->tx_dma = dma_request_slave_channel_reason(kdev, "tx");
	} else {
		priv->tx_dma = dma_request_channel(dma_cap_mask,
					bcm_enet_dma_chan_filter,
					&priv->pd_tx_dma);
		if (!priv->tx_dma)
			priv->tx_dma = ERR_PTR(-ENODEV);
	}
	if (IS_ERR(priv->tx_dma)) {
		ret = PTR_ERR(priv->tx_dma);
		netdev_info(dev, "unable to request TX DMA channel: %d\n", ret);
		goto out_release_rx_dma;
	}

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dma_cfg.dst_maxburst = priv->dma_maxburst;

	ret = dmaengine_slave_config(priv->tx_dma, &dma_cfg);
	if (ret) {
		netdev_info(dev,
			"unable to configure TX DMA channel: %d\n", ret);
		goto out_release_tx_dma;
	}

	if (priv->rx_dma->device->dev && priv->rx_dma->device->dev->of_node) {
		u32 dma_requests;

		if (!of_property_read_u32(priv->rx_dma->device->dev->of_node,
				"dma-requests", &dma_requests))
			priv->rx_max_ring_size = dma_requests;
		else
			priv->rx_max_ring_size = BCMENET_MAX_RX_DESC;
	}

	if (priv->tx_dma->device->dev && priv->tx_dma->device->dev->of_node) {
		u32 dma_requests;

		if (!of_property_read_u32(priv->tx_dma->device->dev->of_node,
				"dma-requests", &dma_requests))
			priv->tx_max_ring_size = dma_requests;
		else
			priv->tx_max_ring_size = BCMENET_MAX_TX_DESC;
	}

	if (priv->rx_ring_size > priv->rx_max_ring_size)
		priv->rx_ring_size = priv->rx_max_ring_size;

	if (priv->tx_ring_size > priv->tx_max_ring_size)
		priv->tx_ring_size = priv->tx_max_ring_size;

	for (i = 0; i < priv->rx_ring_size; i++) {
		struct bcm_enet_pkt *pkt = devm_kzalloc(kdev,
			sizeof(*pkt), GFP_KERNEL);

		if (!pkt) {
			ret = -ENOMEM;
			goto out_free_pkt;
		}

		list_add_tail(&pkt->node, &priv->rx_inactive);
	}

	for (i = 0; i < priv->tx_ring_size; i++) {
		struct bcm_enet_pkt *pkt = devm_kzalloc(kdev,
			sizeof(*pkt), GFP_KERNEL);

		if (!pkt) {
			ret = -ENOMEM;
			goto out_free_pkt;
		}

		list_add_tail(&pkt->node, &priv->tx_inactive);
	}
	return 0;

out_free_pkt:
	while (!list_empty(&priv->rx_inactive)) {
		struct bcm_enet_pkt *pkt;

		pkt = list_first_entry(&priv->rx_inactive,
			struct bcm_enet_pkt, node);
		devm_kfree(kdev, pkt);

		list_del(&pkt->node);
	}

	while (!list_empty(&priv->tx_inactive)) {
		struct bcm_enet_pkt *pkt;

		pkt = list_first_entry(&priv->tx_inactive,
			struct bcm_enet_pkt, node);
		devm_kfree(kdev, pkt);

		list_del(&pkt->node);
	}

out_release_tx_dma:
	dma_release_channel(priv->tx_dma);

out_release_rx_dma:
	dma_release_channel(priv->rx_dma);

	return ret;
}

/* free only the skbs in the RX inactive queue
 * (if the rx_skb_size is to be changed) so that
 * bcm_enet_refill_rx reallocates them
 */
static void bcm_enet_free_rx_queue_skbs(struct bcm_enet_priv *priv)
{
	struct device *kdev = &priv->pdev->dev;
	struct bcm_enet_pkt *pkt;

	spin_lock_bh(&priv->rx_lock);
	WARN(!list_empty(&priv->rx_active), "%s: rx active queue not empty\n",
		__func__);

	list_for_each_entry(pkt, &priv->rx_inactive, node) {
		if (pkt->skb) {
			dma_unmap_single(kdev, pkt->buf, priv->rx_skb_size,
				DMA_FROM_DEVICE);
			dev_kfree_skb(pkt->skb);
			pkt->skb = NULL;
		}
	}
	spin_unlock_bh(&priv->rx_lock);
}


/* free the RX and TX queues */
static void bcm_enet_free_queues(struct bcm_enet_priv *priv)
{
	struct device *kdev = &priv->pdev->dev;
	LIST_HEAD(pkts);

	bcm_enet_free_rx_queue_skbs(priv);

	spin_lock_bh(&priv->rx_lock);
	WARN(!list_empty(&priv->rx_active), "%s: rx active queue not empty\n",
		__func__);
	list_splice_tail_init(&priv->rx_inactive, &pkts);
	spin_unlock_bh(&priv->rx_lock);

	spin_lock_bh(&priv->tx_lock);
	WARN(!list_empty(&priv->tx_active), "%s: tx active queue not empty\n",
		__func__);
	list_splice_tail_init(&priv->tx_inactive, &pkts);
	spin_unlock_bh(&priv->tx_lock);

	while (!list_empty(&pkts)) {
		struct bcm_enet_pkt *pkt;

		pkt = list_first_entry(&pkts, struct bcm_enet_pkt, node);
		list_del(&pkt->node);
		devm_kfree(kdev, pkt);
	}
}

static void bcm_enet_release_dma(struct net_device *dev)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);

	bcm_enet_stop_rx_dma(dev);
	bcm_enet_stop_tx_dma(dev);

	dma_release_channel(priv->rx_dma);
	dma_release_channel(priv->tx_dma);

	priv->rx_dma = NULL;
	priv->tx_dma = NULL;

	bcm_enet_free_queues(priv);
}

/*
 * mac interrupt handler
 */
static irqreturn_t bcm_enet_isr_mac(int irq, void *dev_id)
{
	struct net_device *dev;
	struct bcm_enet_priv *priv;
	u32 stat;

	dev = dev_id;
	priv = netdev_priv(dev);

	stat = enet_readl(priv, ENET_IR_REG);
	if (!(stat & ENET_IR_MIB))
		return IRQ_NONE;

	/* clear & mask interrupt */
	enet_writel(priv, ENET_IR_MIB, ENET_IR_REG);
	enet_writel(priv, 0, ENET_IRMASK_REG);

	/* read mib registers in workqueue */
	schedule_work(&priv->mib_update_task);

	return IRQ_HANDLED;
}

/*
 * tx request callback
 */
static int bcm_enet_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);
	struct device *kdev = &priv->pdev->dev;
	struct bcm_enet_pkt *pkt = NULL;
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	int ret;

	/* lock against tx reclaim */
	spin_lock(&priv->tx_lock);

	/* make sure  the tx hw queue  is not full,  should not happen
	 * since we stop queue before it's the case */
	if (unlikely(list_empty(&priv->tx_inactive) || !priv->tx_running)) {
		netif_stop_queue(dev);
		dev_err(kdev, "xmit called with no tx desc available?\n");
		ret = NETDEV_TX_BUSY;
		goto out_unlock;
	}

	/* pad small packets sent on a switch device */
	if (priv->enet_is_sw && skb->len < VLAN_ETH_ZLEN) {
		int needed = VLAN_ETH_ZLEN - skb->len;
		char *data;

		if (unlikely(skb_tailroom(skb) < needed)) {
			struct sk_buff *nskb;

			nskb = skb_copy_expand(skb, 0, needed, GFP_ATOMIC);
			if (!nskb) {
				ret = NETDEV_TX_BUSY;
				goto out_unlock;
			}
			dev_kfree_skb(skb);
			skb = nskb;
		}
		data = skb_put(skb, needed);
		memset(data, 0, needed);
	}

	ret = skb_linearize(skb);
	if (unlikely(ret)) {
		dev_err_ratelimited(kdev, "tx skb linearize err=%d\n", ret);
		ret = NETDEV_TX_BUSY;
		goto out_unlock;
	}

	pkt = list_first_entry(&priv->tx_inactive, struct bcm_enet_pkt, node);

	pkt->skb = skb;
	pkt->buf = dma_map_single(kdev, skb->data, skb->len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(kdev, pkt->buf))) {
		dev_err_ratelimited(kdev, "tx dma map error\n");
		ret = NETDEV_TX_BUSY;
		goto out_unlock;
	}
	pkt->ctx.control = IUDMA_D_CS_ESOP_MASK | IUDMA_D_CTL_ENET_APPEND_CRC;

	desc = dmaengine_prep_bcm63xx_single(priv->tx_dma, pkt->buf,
		skb->len, DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT, &pkt->ctx);
	if (unlikely(IS_ERR_OR_NULL(desc))) {
		dev_err_ratelimited(kdev,
			"tx dma prep err=%ld\n", PTR_ERR(desc));
		ret = NETDEV_TX_BUSY;
		goto out_unmap;
	}

	desc->callback = bcm_enet_tx_complete;
	desc->callback_param = pkt;
	cookie = dmaengine_submit(desc);
	if (unlikely(dma_submit_error(cookie))) {
		dev_err_ratelimited(kdev,
			"tx dma submit err=%ld\n", PTR_ERR(desc));
		ret = NETDEV_TX_BUSY;
		goto out_unmap;
	}

	list_move_tail(&pkt->node, &priv->tx_active);
	pkt = NULL;

	dma_async_issue_pending(priv->tx_dma);

	/* stop queue if no more desc available */
	if (list_empty(&priv->tx_inactive))
		netif_stop_queue(dev);

	dev->stats.tx_bytes += skb->len;
	dev->stats.tx_packets++;
	ret = NETDEV_TX_OK;

out_unmap:
	if (pkt) {
		/* DMA submit failed, cleanup pkt */
		dma_unmap_single(kdev, pkt->buf, pkt->skb->len, DMA_TO_DEVICE);
		pkt->skb = NULL;
	}

out_unlock:
	spin_unlock(&priv->tx_lock);
	return ret;
}

/*
 * Change the interface's mac address.
 */
static int bcm_enet_set_mac_address(struct net_device *dev, void *p)
{
	struct bcm_enet_priv *priv;
	struct sockaddr *addr = p;
	u32 val;

	priv = netdev_priv(dev);
	memcpy(dev->dev_addr, addr->sa_data, ETH_ALEN);

	/* use perfect match register 0 to store my mac address */
	val = (dev->dev_addr[2] << 24) | (dev->dev_addr[3] << 16) |
		(dev->dev_addr[4] << 8) | dev->dev_addr[5];
	enet_writel(priv, val, ENET_PML_REG(0));

	val = (dev->dev_addr[0] << 8 | dev->dev_addr[1]);
	val |= ENET_PMH_DATAVALID_MASK;
	enet_writel(priv, val, ENET_PMH_REG(0));

	return 0;
}

/*
 * Change rx mode (promiscuous/allmulti) and update multicast list
 */
static void bcm_enet_set_multicast_list(struct net_device *dev)
{
	struct bcm_enet_priv *priv;
	struct netdev_hw_addr *ha;
	u32 val;
	int i;

	priv = netdev_priv(dev);

	val = enet_readl(priv, ENET_RXCFG_REG);

	if (dev->flags & IFF_PROMISC)
		val |= ENET_RXCFG_PROMISC_MASK;
	else
		val &= ~ENET_RXCFG_PROMISC_MASK;

	/* only 3 perfect match registers left, first one is used for
	 * own mac address */
	if ((dev->flags & IFF_ALLMULTI) || netdev_mc_count(dev) > 3)
		val |= ENET_RXCFG_ALLMCAST_MASK;
	else
		val &= ~ENET_RXCFG_ALLMCAST_MASK;

	/* no need to set perfect match registers if we catch all
	 * multicast */
	if (val & ENET_RXCFG_ALLMCAST_MASK) {
		enet_writel(priv, val, ENET_RXCFG_REG);
		return;
	}

	i = 0;
	netdev_for_each_mc_addr(ha, dev) {
		u8 *dmi_addr;
		u32 tmp;

		if (i == 3)
			break;
		/* update perfect match registers */
		dmi_addr = ha->addr;
		tmp = (dmi_addr[2] << 24) | (dmi_addr[3] << 16) |
			(dmi_addr[4] << 8) | dmi_addr[5];
		enet_writel(priv, tmp, ENET_PML_REG(i + 1));

		tmp = (dmi_addr[0] << 8 | dmi_addr[1]);
		tmp |= ENET_PMH_DATAVALID_MASK;
		enet_writel(priv, tmp, ENET_PMH_REG(i++ + 1));
	}

	for (; i < 3; i++) {
		enet_writel(priv, 0, ENET_PML_REG(i + 1));
		enet_writel(priv, 0, ENET_PMH_REG(i + 1));
	}

	enet_writel(priv, val, ENET_RXCFG_REG);
}

/*
 * set mac duplex parameters
 */
static void bcm_enet_set_duplex(struct bcm_enet_priv *priv, int fullduplex)
{
	u32 val;

	val = enet_readl(priv, ENET_TXCTL_REG);
	if (fullduplex)
		val |= ENET_TXCTL_FD_MASK;
	else
		val &= ~ENET_TXCTL_FD_MASK;
	enet_writel(priv, val, ENET_TXCTL_REG);
}

/*
 * set mac flow control parameters
 */
static void bcm_enet_set_flow(struct bcm_enet_priv *priv, int rx_en, int tx_en)
{
	u32 val;
	struct dma_slave_config dma_cfg;

	/* rx flow control (pause frame handling) */
	val = enet_readl(priv, ENET_RXCFG_REG);
	if (rx_en)
		val |= ENET_RXCFG_ENFLOW_MASK;
	else
		val &= ~ENET_RXCFG_ENFLOW_MASK;
	enet_writel(priv, val, ENET_RXCFG_REG);

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	dma_cfg.src_maxburst = priv->dma_maxburst;
	dma_cfg.device_fc = tx_en;

	dmaengine_slave_config(priv->rx_dma, &dma_cfg);
}

/*
 * link changed callback (from phylib)
 */
static void bcm_enet_adjust_phy_link(struct net_device *dev)
{
	struct bcm_enet_priv *priv;
	struct phy_device *phydev;
	int status_changed;

	priv = netdev_priv(dev);
	phydev = priv->phydev;
	status_changed = 0;

	if (priv->old_link != phydev->link) {
		status_changed = 1;
		priv->old_link = phydev->link;
	}

	/* reflect duplex change in mac configuration */
	if (phydev->link && phydev->duplex != priv->old_duplex) {
		bcm_enet_set_duplex(priv,
				    (phydev->duplex == DUPLEX_FULL) ? 1 : 0);
		status_changed = 1;
		priv->old_duplex = phydev->duplex;
	}

	/* enable flow control if remote advertise it (trust phylib to
	 * check that duplex is full */
	if (phydev->link && phydev->pause != priv->old_pause) {
		int rx_pause_en, tx_pause_en;

		if (phydev->pause) {
			/* pause was advertised by lpa and us */
			rx_pause_en = 1;
			tx_pause_en = 1;
		} else if (!priv->pause_auto) {
			/* pause setting overrided by user */
			rx_pause_en = priv->pause_rx;
			tx_pause_en = priv->pause_tx;
		} else {
			rx_pause_en = 0;
			tx_pause_en = 0;
		}

		bcm_enet_set_flow(priv, rx_pause_en, tx_pause_en);
		status_changed = 1;
		priv->old_pause = phydev->pause;
	}

	if (status_changed) {
		pr_info("%s: link %s", dev->name, phydev->link ?
			"UP" : "DOWN");
		if (phydev->link)
			pr_cont(" - %d/%s - flow control %s", phydev->speed,
			       DUPLEX_FULL == phydev->duplex ? "full" : "half",
			       phydev->pause == 1 ? "rx&tx" : "off");

		pr_cont("\n");
	}
}

/*
 * link changed callback (if phylib is not used)
 */
static void bcm_enet_adjust_link(struct net_device *dev)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);
	bcm_enet_set_duplex(priv, priv->force_duplex_full);
	bcm_enet_set_flow(priv, priv->pause_rx, priv->pause_tx);
	netif_carrier_on(dev);

	pr_info("%s: link forced UP - %d/%s - flow control %s/%s\n",
		dev->name,
		priv->force_speed_100 ? 100 : 10,
		priv->force_duplex_full ? "full" : "half",
		priv->pause_rx ? "rx" : "off",
		priv->pause_tx ? "tx" : "off");
}

/*
 * open callback, start rx operation
 */
static int bcm_enet_open(struct net_device *dev)
{
	struct bcm_enet_priv *priv;
	struct sockaddr addr;
	struct device *kdev;
	struct phy_device *phydev;
	int i, ret;
	char phy_id[MII_BUS_ID_SIZE + 3];
	u32 val;

	priv = netdev_priv(dev);
	kdev = &priv->pdev->dev;

	if (priv->has_phy) {
		/* connect to PHY */
		snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT,
			 priv->mii_bus->id, priv->phy_id);

		phydev = phy_connect(dev, phy_id, bcm_enet_adjust_phy_link,
				     PHY_INTERFACE_MODE_MII);

		if (IS_ERR(phydev)) {
			dev_err(kdev, "could not attach to PHY\n");
			return PTR_ERR(phydev);
		}

		/* mask with MAC supported features */
		phydev->supported &= (SUPPORTED_10baseT_Half |
				      SUPPORTED_10baseT_Full |
				      SUPPORTED_100baseT_Half |
				      SUPPORTED_100baseT_Full |
				      SUPPORTED_Autoneg |
				      SUPPORTED_Pause |
				      SUPPORTED_MII);
		phydev->advertising = phydev->supported;

		if (priv->pause_auto && priv->pause_rx && priv->pause_tx)
			phydev->advertising |= SUPPORTED_Pause;
		else
			phydev->advertising &= ~SUPPORTED_Pause;

		dev_info(kdev, "attached PHY at address %d [%s]\n",
			 phydev->addr, phydev->drv->name);

		priv->old_link = 0;
		priv->old_duplex = -1;
		priv->old_pause = -1;
		priv->phydev = phydev;
	}

	ret = bcm_enet_request_dma(dev);
	if (ret)
		goto out_phy_disconnect;

	/* mask all interrupts and request them */
	enet_writel(priv, 0, ENET_IRMASK_REG);

	ret = request_irq(dev->irq, bcm_enet_isr_mac, 0, dev->name, dev);
	if (ret)
		goto out_phy_disconnect;

	/* initialize perfect match registers */
	for (i = 0; i < 4; i++) {
		enet_writel(priv, 0, ENET_PML_REG(i));
		enet_writel(priv, 0, ENET_PMH_REG(i));
	}

	/* write device mac address */
	memcpy(addr.sa_data, dev->dev_addr, ETH_ALEN);
	bcm_enet_set_mac_address(dev, &addr);

	/* set correct transmit fifo watermark */
	enet_writel(priv, BCMENET_TX_FIFO_TRESH, ENET_TXWMARK_REG);

	/* all set, enable mac and interrupts, start dma engine and
	 * kick rx dma channel */
	wmb();
	val = enet_readl(priv, ENET_CTL_REG);
	val |= ENET_CTL_ENABLE_MASK;
	enet_writel(priv, val, ENET_CTL_REG);

	/* watch "mib counters about to overflow" interrupt */
	enet_writel(priv, ENET_IR_MIB, ENET_IR_REG);
	enet_writel(priv, ENET_IR_MIB, ENET_IRMASK_REG);

	if (priv->has_phy)
		phy_start(priv->phydev);
	else
		bcm_enet_adjust_link(dev);

	bcm_enet_start_rx_dma(dev);
	bcm_enet_start_tx_dma(dev);
	return 0;

out_phy_disconnect:
	phy_disconnect(priv->phydev);

	return ret;
}

/*
 * disable mac
 */
static void bcm_enet_disable_mac(struct bcm_enet_priv *priv)
{
	int limit;
	u32 val;

	val = enet_readl(priv, ENET_CTL_REG);
	val |= ENET_CTL_DISABLE_MASK;
	enet_writel(priv, val, ENET_CTL_REG);

	limit = 1000;
	do {
		u32 val;

		val = enet_readl(priv, ENET_CTL_REG);
		if (!(val & ENET_CTL_DISABLE_MASK))
			break;
		udelay(1);
	} while (limit--);
}

/*
 * stop callback
 */
static int bcm_enet_stop(struct net_device *dev)
{
	struct bcm_enet_priv *priv;
	struct device *kdev;

	priv = netdev_priv(dev);
	kdev = &priv->pdev->dev;

	if (priv->has_phy)
		phy_stop(priv->phydev);
	del_timer_sync(&priv->rx_timeout);

	bcm_enet_release_dma(dev);

	/* mask all interrupts */
	enet_writel(priv, 0, ENET_IRMASK_REG);

	/* make sure no mib update is scheduled */
	cancel_work_sync(&priv->mib_update_task);

	/* disable mac */
	bcm_enet_disable_mac(priv);

	free_irq(dev->irq, dev);

	/* release phy */
	if (priv->has_phy) {
		phy_disconnect(priv->phydev);
		priv->phydev = NULL;
	}

	return 0;
}

/*
 * ethtool callbacks
 */
struct bcm_enet_stats {
	char stat_string[ETH_GSTRING_LEN];
	int sizeof_stat;
	int stat_offset;
	int mib_reg;
};

#define GEN_STAT(m) sizeof(((struct bcm_enet_priv *)0)->m),		\
		     offsetof(struct bcm_enet_priv, m)
#define DEV_STAT(m) sizeof(((struct net_device_stats *)0)->m),		\
		     offsetof(struct net_device_stats, m)

static const struct bcm_enet_stats bcm_enet_gstrings_stats[] = {
	{ "rx_packets", DEV_STAT(rx_packets), -1 },
	{ "tx_packets",	DEV_STAT(tx_packets), -1 },
	{ "rx_bytes", DEV_STAT(rx_bytes), -1 },
	{ "tx_bytes", DEV_STAT(tx_bytes), -1 },
	{ "rx_errors", DEV_STAT(rx_errors), -1 },
	{ "tx_errors", DEV_STAT(tx_errors), -1 },
	{ "rx_dropped",	DEV_STAT(rx_dropped), -1 },
	{ "tx_dropped",	DEV_STAT(tx_dropped), -1 },

	{ "rx_good_octets", GEN_STAT(mib.rx_gd_octets), ETH_MIB_RX_GD_OCTETS},
	{ "rx_good_pkts", GEN_STAT(mib.rx_gd_pkts), ETH_MIB_RX_GD_PKTS },
	{ "rx_broadcast", GEN_STAT(mib.rx_brdcast), ETH_MIB_RX_BRDCAST },
	{ "rx_multicast", GEN_STAT(mib.rx_mult), ETH_MIB_RX_MULT },
	{ "rx_64_octets", GEN_STAT(mib.rx_64), ETH_MIB_RX_64 },
	{ "rx_65_127_oct", GEN_STAT(mib.rx_65_127), ETH_MIB_RX_65_127 },
	{ "rx_128_255_oct", GEN_STAT(mib.rx_128_255), ETH_MIB_RX_128_255 },
	{ "rx_256_511_oct", GEN_STAT(mib.rx_256_511), ETH_MIB_RX_256_511 },
	{ "rx_512_1023_oct", GEN_STAT(mib.rx_512_1023), ETH_MIB_RX_512_1023 },
	{ "rx_1024_max_oct", GEN_STAT(mib.rx_1024_max), ETH_MIB_RX_1024_MAX },
	{ "rx_jabber", GEN_STAT(mib.rx_jab), ETH_MIB_RX_JAB },
	{ "rx_oversize", GEN_STAT(mib.rx_ovr), ETH_MIB_RX_OVR },
	{ "rx_fragment", GEN_STAT(mib.rx_frag), ETH_MIB_RX_FRAG },
	{ "rx_dropped",	GEN_STAT(mib.rx_drop), ETH_MIB_RX_DROP },
	{ "rx_crc_align", GEN_STAT(mib.rx_crc_align), ETH_MIB_RX_CRC_ALIGN },
	{ "rx_undersize", GEN_STAT(mib.rx_und), ETH_MIB_RX_UND },
	{ "rx_crc", GEN_STAT(mib.rx_crc), ETH_MIB_RX_CRC },
	{ "rx_align", GEN_STAT(mib.rx_align), ETH_MIB_RX_ALIGN },
	{ "rx_symbol_error", GEN_STAT(mib.rx_sym), ETH_MIB_RX_SYM },
	{ "rx_pause", GEN_STAT(mib.rx_pause), ETH_MIB_RX_PAUSE },
	{ "rx_control", GEN_STAT(mib.rx_cntrl), ETH_MIB_RX_CNTRL },

	{ "tx_good_octets", GEN_STAT(mib.tx_gd_octets), ETH_MIB_TX_GD_OCTETS },
	{ "tx_good_pkts", GEN_STAT(mib.tx_gd_pkts), ETH_MIB_TX_GD_PKTS },
	{ "tx_broadcast", GEN_STAT(mib.tx_brdcast), ETH_MIB_TX_BRDCAST },
	{ "tx_multicast", GEN_STAT(mib.tx_mult), ETH_MIB_TX_MULT },
	{ "tx_64_oct", GEN_STAT(mib.tx_64), ETH_MIB_TX_64 },
	{ "tx_65_127_oct", GEN_STAT(mib.tx_65_127), ETH_MIB_TX_65_127 },
	{ "tx_128_255_oct", GEN_STAT(mib.tx_128_255), ETH_MIB_TX_128_255 },
	{ "tx_256_511_oct", GEN_STAT(mib.tx_256_511), ETH_MIB_TX_256_511 },
	{ "tx_512_1023_oct", GEN_STAT(mib.tx_512_1023), ETH_MIB_TX_512_1023},
	{ "tx_1024_max_oct", GEN_STAT(mib.tx_1024_max), ETH_MIB_TX_1024_MAX },
	{ "tx_jabber", GEN_STAT(mib.tx_jab), ETH_MIB_TX_JAB },
	{ "tx_oversize", GEN_STAT(mib.tx_ovr), ETH_MIB_TX_OVR },
	{ "tx_fragment", GEN_STAT(mib.tx_frag), ETH_MIB_TX_FRAG },
	{ "tx_underrun", GEN_STAT(mib.tx_underrun), ETH_MIB_TX_UNDERRUN },
	{ "tx_collisions", GEN_STAT(mib.tx_col), ETH_MIB_TX_COL },
	{ "tx_single_collision", GEN_STAT(mib.tx_1_col), ETH_MIB_TX_1_COL },
	{ "tx_multiple_collision", GEN_STAT(mib.tx_m_col), ETH_MIB_TX_M_COL },
	{ "tx_excess_collision", GEN_STAT(mib.tx_ex_col), ETH_MIB_TX_EX_COL },
	{ "tx_late_collision", GEN_STAT(mib.tx_late), ETH_MIB_TX_LATE },
	{ "tx_deferred", GEN_STAT(mib.tx_def), ETH_MIB_TX_DEF },
	{ "tx_carrier_sense", GEN_STAT(mib.tx_crs), ETH_MIB_TX_CRS },
	{ "tx_pause", GEN_STAT(mib.tx_pause), ETH_MIB_TX_PAUSE },

};

#define BCM_ENET_STATS_LEN	ARRAY_SIZE(bcm_enet_gstrings_stats)

static const u32 unused_mib_regs[] = {
	ETH_MIB_TX_ALL_OCTETS,
	ETH_MIB_TX_ALL_PKTS,
	ETH_MIB_RX_ALL_OCTETS,
	ETH_MIB_RX_ALL_PKTS,
};


static void bcm_enet_get_drvinfo(struct net_device *netdev,
				 struct ethtool_drvinfo *drvinfo)
{
	strlcpy(drvinfo->driver, bcm_enet_driver_name, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, bcm_enet_driver_version,
		sizeof(drvinfo->version));
	strlcpy(drvinfo->fw_version, "N/A", sizeof(drvinfo->fw_version));
	strlcpy(drvinfo->bus_info, "bcm63xx", sizeof(drvinfo->bus_info));
}

static int bcm_enet_get_sset_count(struct net_device *netdev,
					int string_set)
{
	switch (string_set) {
	case ETH_SS_STATS:
		return BCM_ENET_STATS_LEN;
	default:
		return -EINVAL;
	}
}

static void bcm_enet_get_strings(struct net_device *netdev,
				 u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < BCM_ENET_STATS_LEN; i++) {
			memcpy(data + i * ETH_GSTRING_LEN,
			       bcm_enet_gstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
		}
		break;
	}
}

static void update_mib_counters(struct bcm_enet_priv *priv)
{
	int i;

	for (i = 0; i < BCM_ENET_STATS_LEN; i++) {
		const struct bcm_enet_stats *s;
		u32 val;
		char *p;

		s = &bcm_enet_gstrings_stats[i];
		if (s->mib_reg == -1)
			continue;

		val = enet_readl(priv, ENET_MIB_REG(s->mib_reg));
		p = (char *)priv + s->stat_offset;

		if (s->sizeof_stat == sizeof(u64))
			*(u64 *)p += val;
		else
			*(u32 *)p += val;
	}

	/* also empty unused mib counters to make sure mib counter
	 * overflow interrupt is cleared */
	for (i = 0; i < ARRAY_SIZE(unused_mib_regs); i++)
		(void)enet_readl(priv, ENET_MIB_REG(unused_mib_regs[i]));
}

static void bcm_enet_update_mib_counters_defer(struct work_struct *t)
{
	struct bcm_enet_priv *priv;

	priv = container_of(t, struct bcm_enet_priv, mib_update_task);
	mutex_lock(&priv->mib_update_lock);
	update_mib_counters(priv);
	mutex_unlock(&priv->mib_update_lock);

	/* reenable mib interrupt */
	if (netif_running(priv->net_dev))
		enet_writel(priv, ENET_IR_MIB, ENET_IRMASK_REG);
}

static void bcm_enet_get_ethtool_stats(struct net_device *netdev,
				       struct ethtool_stats *stats,
				       u64 *data)
{
	struct bcm_enet_priv *priv;
	int i;

	priv = netdev_priv(netdev);

	mutex_lock(&priv->mib_update_lock);
	update_mib_counters(priv);

	for (i = 0; i < BCM_ENET_STATS_LEN; i++) {
		const struct bcm_enet_stats *s;
		char *p;

		s = &bcm_enet_gstrings_stats[i];
		if (s->mib_reg == -1)
			p = (char *)&netdev->stats;
		else
			p = (char *)priv;
		p += s->stat_offset;
		data[i] = (s->sizeof_stat == sizeof(u64)) ?
			*(u64 *)p : *(u32 *)p;
	}
	mutex_unlock(&priv->mib_update_lock);
}

static int bcm_enet_nway_reset(struct net_device *dev)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);
	if (priv->has_phy) {
		if (!priv->phydev)
			return -ENODEV;
		return genphy_restart_aneg(priv->phydev);
	}

	return -EOPNOTSUPP;
}

static int bcm_enet_get_settings(struct net_device *dev,
				 struct ethtool_cmd *cmd)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);

	cmd->maxrxpkt = 0;
	cmd->maxtxpkt = 0;

	if (priv->has_phy) {
		if (!priv->phydev)
			return -ENODEV;
		return phy_ethtool_gset(priv->phydev, cmd);
	} else {
		cmd->autoneg = 0;
		ethtool_cmd_speed_set(cmd, ((priv->force_speed_100)
					    ? SPEED_100 : SPEED_10));
		cmd->duplex = (priv->force_duplex_full) ?
			DUPLEX_FULL : DUPLEX_HALF;
		cmd->supported = ADVERTISED_10baseT_Half  |
			ADVERTISED_10baseT_Full |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full;
		cmd->advertising = 0;
		cmd->port = PORT_MII;
		cmd->transceiver = XCVR_EXTERNAL;
	}
	return 0;
}

static int bcm_enet_set_settings(struct net_device *dev,
				 struct ethtool_cmd *cmd)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);
	if (priv->has_phy) {
		if (!priv->phydev)
			return -ENODEV;
		return phy_ethtool_sset(priv->phydev, cmd);
	} else {

		if (cmd->autoneg ||
		    (cmd->speed != SPEED_100 && cmd->speed != SPEED_10) ||
		    cmd->port != PORT_MII)
			return -EINVAL;

		priv->force_speed_100 = (cmd->speed == SPEED_100) ? 1 : 0;
		priv->force_duplex_full = (cmd->duplex == DUPLEX_FULL) ? 1 : 0;

		if (netif_running(dev))
			bcm_enet_adjust_link(dev);
		return 0;
	}
}

static void bcm_enet_get_ringparam(struct net_device *dev,
				   struct ethtool_ringparam *ering)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);

	ering->rx_max_pending = priv->rx_max_ring_size;
	ering->tx_max_pending = priv->tx_max_ring_size;
	ering->rx_mini_max_pending = 0;
	ering->rx_jumbo_max_pending = 0;
	ering->rx_pending = priv->rx_ring_size;
	ering->tx_pending = priv->tx_ring_size;
}

static int bcm_enet_set_ringparam(struct net_device *dev,
				  struct ethtool_ringparam *ering)
{
	struct bcm_enet_priv *priv;
	struct device *kdev;
	int adjust_rx;
	int adjust_tx;
	int ret, i;

	priv = netdev_priv(dev);
	kdev = &priv->pdev->dev;

	if (ering->rx_pending < 1)
		ering->rx_pending = BCMENET_DEF_RX_DESC;
	if (ering->rx_pending > priv->rx_max_ring_size)
		ering->rx_pending = priv->rx_max_ring_size;

	if (ering->tx_pending < 1)
		ering->tx_pending = BCMENET_DEF_TX_DESC;
	if (ering->tx_pending > priv->tx_max_ring_size)
		ering->tx_pending = priv->tx_max_ring_size;

	if (!netif_running(dev)) {
		priv->rx_ring_size = ering->rx_pending;
		priv->tx_ring_size = ering->tx_pending;
		return 0;
	}

	adjust_rx = ering->rx_pending - priv->rx_ring_size;
	adjust_tx = ering->tx_pending - priv->tx_ring_size;

	ret = 0;
	if (adjust_rx > 0) {
		/* increase size of rx ring:
		 * - add new packets to inactive queue
		 * - refill active queue
		 */

		for (i = 0; i < adjust_rx; i++) {
			struct bcm_enet_pkt *pkt = devm_kzalloc(kdev,
				sizeof(*pkt), GFP_KERNEL);

			if (!pkt) {
				dev_warn(kdev, "out of memory adding to rx ring\n");
				ret = -ENOMEM;
				break;
			}

			spin_lock_bh(&priv->rx_lock);
			list_add_tail(&pkt->node, &priv->rx_inactive);
			priv->rx_ring_size++;
			spin_unlock_bh(&priv->rx_lock);
		}

		spin_lock_bh(&priv->rx_lock);
		bcm_enet_refill_rx(dev);
		spin_unlock_bh(&priv->rx_lock);
	} else if (adjust_rx < 0) {
		/* decrease size of rx ring:
		 * - stop RX DMA (moves unused active packets to inactive queue)
		 * - remove packets from inactive queue
		 * - start RX DMA (refills active queue)
		 */

		bcm_enet_stop_rx_dma(dev);
		spin_lock_bh(&priv->rx_lock);
		for (i = adjust_rx; i < 0; i++) {
			struct bcm_enet_pkt *pkt;

			pkt = list_first_entry_or_null(&priv->rx_inactive,
				struct bcm_enet_pkt, node);
			if (!pkt) {
				WARN_ONCE(1, "%s: rx inactive queue empty",
					__func__);
				ret = -EIO;
				break;
			}

			if (pkt->skb) {
				dma_unmap_single(kdev, pkt->buf,
					priv->rx_skb_size, DMA_FROM_DEVICE);
				dev_kfree_skb(pkt->skb);
			}

			list_del(&pkt->node);
			devm_kfree(kdev, pkt);

			priv->rx_ring_size--;
		}
		spin_unlock_bh(&priv->rx_lock);
		bcm_enet_start_rx_dma(dev);
	}

	if (adjust_tx > 0) {
		/* increase size of tx ring:
		 * - add new packets to inactive queue
		 */

		for (i = 0; i < adjust_tx; i++) {
			struct bcm_enet_pkt *pkt = devm_kzalloc(kdev,
				sizeof(*pkt), GFP_KERNEL);

			if (!pkt) {
				dev_warn(kdev, "out of memory adding to tx ring\n");
				ret = -ENOMEM;
				break;
			}

			spin_lock_bh(&priv->tx_lock);
			list_add_tail(&pkt->node, &priv->tx_inactive);
			priv->tx_ring_size++;
			if (priv->tx_running && netif_queue_stopped(dev))
				netif_wake_queue(dev);
			spin_unlock_bh(&priv->tx_lock);
		}
	} else if (adjust_tx < 0) {
		bool queue_stopped;

		/* decrease size of rx ring:
		 * - remove packets from inactive queue (if possible)
		 */

		spin_lock_bh(&priv->tx_lock);
		queue_stopped = netif_queue_stopped(dev);
		netif_stop_queue(dev);

		for (i = adjust_tx; i < 0; i++) {
			struct bcm_enet_pkt *pkt;

			pkt = list_first_entry_or_null(&priv->tx_inactive,
				struct bcm_enet_pkt, node);
			if (!pkt) {
				dev_warn(kdev, "unable to reduce size of busy tx queue\n");
				ret = -EBUSY;
				break;
			}

			list_del(&pkt->node);
			devm_kfree(kdev, pkt);

			priv->tx_ring_size--;
		}

		if (!list_empty(&priv->tx_inactive) && !queue_stopped)
			netif_wake_queue(dev);
		spin_unlock_bh(&priv->tx_lock);
	}

	return ret;
}

static void bcm_enet_get_pauseparam(struct net_device *dev,
				    struct ethtool_pauseparam *ecmd)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);
	ecmd->autoneg = priv->pause_auto;
	ecmd->rx_pause = priv->pause_rx;
	ecmd->tx_pause = priv->pause_tx;
}

static int bcm_enet_set_pauseparam(struct net_device *dev,
				   struct ethtool_pauseparam *ecmd)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);

	if (priv->has_phy) {
		if (ecmd->autoneg && (ecmd->rx_pause != ecmd->tx_pause)) {
			/* asymetric pause mode not supported,
			 * actually possible but integrated PHY has RO
			 * asym_pause bit */
			return -EINVAL;
		}
	} else {
		/* no pause autoneg on direct mii connection */
		if (ecmd->autoneg)
			return -EINVAL;
	}

	priv->pause_auto = ecmd->autoneg;
	priv->pause_rx = ecmd->rx_pause;
	priv->pause_tx = ecmd->tx_pause;

	return 0;
}

static const struct ethtool_ops bcm_enet_ethtool_ops = {
	.get_strings		= bcm_enet_get_strings,
	.get_sset_count		= bcm_enet_get_sset_count,
	.get_ethtool_stats      = bcm_enet_get_ethtool_stats,
	.nway_reset		= bcm_enet_nway_reset,
	.get_settings		= bcm_enet_get_settings,
	.set_settings		= bcm_enet_set_settings,
	.get_drvinfo		= bcm_enet_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_ringparam		= bcm_enet_get_ringparam,
	.set_ringparam		= bcm_enet_set_ringparam,
	.get_pauseparam		= bcm_enet_get_pauseparam,
	.set_pauseparam		= bcm_enet_set_pauseparam,
};

static int bcm_enet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);
	if (priv->has_phy) {
		if (!priv->phydev)
			return -ENODEV;
		return phy_mii_ioctl(priv->phydev, rq, cmd);
	} else {
		struct mii_if_info mii;

		mii.dev = dev;
		mii.mdio_read = bcm_enet_mdio_read_mii;
		mii.mdio_write = bcm_enet_mdio_write_mii;
		mii.phy_id = 0;
		mii.phy_id_mask = 0x3f;
		mii.reg_num_mask = 0x1f;
		return generic_mii_ioctl(&mii, if_mii(rq), cmd, NULL);
	}
}

/*
 * calculate actual hardware mtu
 */
static unsigned int __pure compute_hw_mtu(int mtu)
{
	unsigned int actual_mtu;

	if (mtu < 0)
		return 0;

	actual_mtu = mtu;

	/* add ethernet header + vlan tag size */
	actual_mtu += VLAN_ETH_HLEN;

	if (actual_mtu < ETH_ZLEN || actual_mtu > BCMENET_MAX_MTU)
		return 0;

	return actual_mtu;
}

/*
 * apply actual hardware mtu
 */
static void set_hw_mtu(struct bcm_enet_priv *priv, unsigned int actual_mtu)
{
	/*
	 * setup maximum size before we get overflow mark in
	 * descriptor, note that this will not prevent reception of
	 * big frames, they will be split into multiple buffers
	 * anyway
	 */
	priv->hw_mtu = actual_mtu;

	/*
	 * align rx buffer size to double the dma burst len, and account
	 * for the FCS as it's appended
	 */
	priv->rx_skb_size = ALIGN(actual_mtu + ETH_FCS_LEN,
				  priv->dma_maxburst * 4 * 2);

	if (!priv->enet_is_sw) {
		/* set max rx/tx length */
		enet_writel(priv, priv->hw_mtu, ENET_RXMAXLEN_REG);
		enet_writel(priv, priv->hw_mtu, ENET_TXMAXLEN_REG);
	}
}

/*
 * adjust mtu, temporarily stop rx if device is running
 */
static int bcm_enet_change_mtu(struct net_device *dev, int new_mtu)
{
	struct bcm_enet_priv *priv = netdev_priv(dev);
	unsigned int hw_mtu;

	hw_mtu = compute_hw_mtu(new_mtu);
	if (!hw_mtu)
		return -EINVAL;

	if (!netif_running(dev)) {
		dev->mtu = new_mtu;
		return 0;
	}

	bcm_enet_stop_rx_dma(dev);
	bcm_enet_free_rx_queue_skbs(priv);

	spin_lock_bh(&priv->rx_lock);
	set_hw_mtu(priv, hw_mtu);
	spin_unlock_bh(&priv->rx_lock);

	dev->mtu = new_mtu;

	bcm_enet_start_rx_dma(dev);

	return 0;
}

/*
 * preinit hardware to allow mii operation while device is down
 */
static void bcm_enet_hw_preinit(struct bcm_enet_priv *priv)
{
	u32 val;
	int limit;

	/* make sure mac is disabled */
	bcm_enet_disable_mac(priv);

	/* soft reset mac */
	val = ENET_CTL_SRESET_MASK;
	enet_writel(priv, val, ENET_CTL_REG);
	wmb();

	limit = 1000;
	do {
		val = enet_readl(priv, ENET_CTL_REG);
		if (!(val & ENET_CTL_SRESET_MASK))
			break;
		udelay(1);
	} while (limit--);

	/* select correct mii interface */
	val = enet_readl(priv, ENET_CTL_REG);
	if (priv->use_external_mii)
		val |= ENET_CTL_EPHYSEL_MASK;
	else
		val &= ~ENET_CTL_EPHYSEL_MASK;
	enet_writel(priv, val, ENET_CTL_REG);

	/* turn on mdc clock */
	enet_writel(priv, (0x1f << ENET_MIISC_MDCFREQDIV_SHIFT) |
		    ENET_MIISC_PREAMBLEEN_MASK, ENET_MIISC_REG);

	/* set mib counters to self-clear when read */
	val = enet_readl(priv, ENET_MIBCTL_REG);
	val |= ENET_MIBCTL_RDCLEAR_MASK;
	enet_writel(priv, val, ENET_MIBCTL_REG);
}

static const struct net_device_ops bcm_enet_ops = {
	.ndo_open		= bcm_enet_open,
	.ndo_stop		= bcm_enet_stop,
	.ndo_start_xmit		= bcm_enet_start_xmit,
	.ndo_set_mac_address	= bcm_enet_set_mac_address,
	.ndo_set_rx_mode	= bcm_enet_set_multicast_list,
	.ndo_do_ioctl		= bcm_enet_ioctl,
	.ndo_change_mtu		= bcm_enet_change_mtu,
};

/*
 * allocate netdevice, request register memory and register device.
 */
static int bcm_enet_probe(struct platform_device *pdev)
{
	struct regulator *regulator = NULL;
	struct reset_control *reset = NULL;
	struct bcm_enet_priv *priv;
	struct device_node *np = pdev->dev.of_node;
	struct net_device *dev;
	struct bcm63xx_enet_platform_data *pd;
	struct resource *res_mem, *res_irq;
	struct mii_bus *bus;
	unsigned int hw_mtu;
	const char *clk_name = NULL;
	int i, clk, ret;

	pd = dev_get_platdata(&pdev->dev);

	if (!pd) {
		regulator = devm_regulator_get(&pdev->dev, "power");
		if (IS_ERR(regulator)) {
			ret = PTR_ERR(regulator);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev,
					"failed to get regulator: %d\n", ret);
			return ret;
		}

		reset = devm_reset_control_get(&pdev->dev, NULL);
		if (IS_ERR(reset)) {
			ret = PTR_ERR(reset);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev,
					"failed to get reset controller: %d\n",
					ret);
			return ret;
		}
	}

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -ENODEV;


	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res_irq)
		return -ENODEV;

	ret = 0;
	dev = alloc_etherdev(sizeof(*priv));
	if (!dev)
		return -ENOMEM;
	priv = netdev_priv(dev);

	if (!pd) {
		priv->regulator = regulator;
		priv->reset = reset;
	}

	priv->enet_is_sw = false;
	priv->dma_maxburst = BCMENET_DMA_MAXBURST;

	hw_mtu = compute_hw_mtu(dev->mtu);
	if (!hw_mtu) {
		ret = -EINVAL;
		goto out;
	}

	priv->base = devm_ioremap_resource(&pdev->dev, res_mem);
	if (IS_ERR(priv->base)) {
		ret = PTR_ERR(priv->base);
		goto out;
	}

	dev->irq = priv->irq = res_irq->start;
	priv->mac_id = pdev->id;

	/* get rx & tx dma channel id for this mac */
	if (priv->mac_id == 0) {
		clk_name = "enet0";
	} else if (priv->mac_id == 1) {
		clk_name = "enet1";
	}

	if (priv->regulator) {
		ret = regulator_enable(priv->regulator);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to power on device: %d\n", ret);
			goto out;
		}
	}

	for (clk = 0; clk < BCMENET_MAX_CLKS; clk++) {
		priv->mac_clks[clk] = of_clk_get(np, clk);
		if (IS_ERR(priv->mac_clks[clk])) {
			ret = PTR_ERR(priv->mac_clks[clk]);
			if (ret == -EPROBE_DEFER)
				goto out_put_clk_mac;

			priv->mac_clks[clk] = NULL;
			break;
		}
	}

	if (clk == 0 && priv->mac_clks[clk] == NULL) {
		priv->mac_clks[clk] = clk_get(&pdev->dev, clk_name);
		if (IS_ERR(priv->mac_clks[clk])) {
			ret = PTR_ERR(priv->mac_clks[clk]);
			goto out_put_clk_mac;
		}
	}

	for (clk = 0; clk < BCMENET_MAX_CLKS && priv->mac_clks[clk]; clk++) {
		ret = clk_prepare_enable(priv->mac_clks[clk]);
		if (ret) {
			dev_err(&pdev->dev, "unable to enable %s clock: %d\n",
#ifdef CONFIG_COMMON_CLK
				__clk_get_name(priv->mac_clks[clk]),
#else
				clk_name,
#endif
				ret);
			goto out_disable_clk_mac;
		}
	}

	if (priv->reset) {
		ret = reset_control_reset(priv->reset);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to reset device: %d\n", ret);
			goto out_disable_clk_mac;
		}
	}

	spin_lock_init(&priv->rx_lock);
	spin_lock_init(&priv->tx_lock);

	if (pd) {
		memcpy(dev->dev_addr, pd->mac_addr, ETH_ALEN);
		priv->has_phy = pd->has_phy;
		priv->phy_id = pd->phy_id;
		priv->has_phy_interrupt = pd->has_phy_interrupt;
		priv->phy_interrupt = pd->phy_interrupt;
		priv->use_external_mii = !pd->use_internal_phy;
		priv->pause_auto = pd->pause_auto;
		priv->pause_rx = pd->pause_rx;
		priv->pause_tx = pd->pause_tx;
		priv->force_duplex_full = pd->force_duplex_full;
		priv->force_speed_100 = pd->force_speed_100;
		priv->rx_max_ring_size = pd->rx_max_ring_size;
		priv->tx_max_ring_size = pd->tx_max_ring_size;
	} else {
		/* limited by DMA controller */
		priv->rx_max_ring_size = BCMENET_MAX_RX_DESC;
		priv->tx_max_ring_size = BCMENET_MAX_TX_DESC;
	}

	priv->rx_ring_size = BCMENET_DEF_RX_DESC;
	priv->tx_ring_size = BCMENET_DEF_TX_DESC;

	if (priv->mac_id == 0 && priv->has_phy && !priv->use_external_mii) {
		/* using internal PHY, enable clock */
		priv->phy_clk = clk_get(&pdev->dev, "ephy");
		if (IS_ERR(priv->phy_clk)) {
			ret = PTR_ERR(priv->phy_clk);
			priv->phy_clk = NULL;
			goto out_disable_clk_mac;
		}
		clk_prepare_enable(priv->phy_clk);
	}

	/* do minimal hardware init to be able to probe mii bus */
	bcm_enet_hw_preinit(priv);

	/* MII bus registration */
	if (priv->has_phy) {

		priv->mii_bus = mdiobus_alloc();
		if (!priv->mii_bus) {
			ret = -ENOMEM;
			goto out_uninit_hw;
		}

		bus = priv->mii_bus;
		bus->name = "bcm63xx_enet MII bus";
		bus->parent = &pdev->dev;
		bus->priv = priv;
		bus->read = bcm_enet_mdio_read_phylib;
		bus->write = bcm_enet_mdio_write_phylib;
		sprintf(bus->id, "%s-%d", pdev->name, priv->mac_id);

		/* only probe bus where we think the PHY is, because
		 * the mdio read operation return 0 instead of 0xffff
		 * if a slave is not present on hw */
		bus->phy_mask = ~(1 << priv->phy_id);

		bus->irq = devm_kzalloc(&pdev->dev, sizeof(int) * PHY_MAX_ADDR,
					GFP_KERNEL);
		if (!bus->irq) {
			ret = -ENOMEM;
			goto out_free_mdio;
		}

		if (priv->has_phy_interrupt)
			bus->irq[priv->phy_id] = priv->phy_interrupt;
		else
			bus->irq[priv->phy_id] = PHY_POLL;

		ret = mdiobus_register(bus);
		if (ret) {
			dev_err(&pdev->dev, "unable to register mdio bus\n");
			goto out_free_mdio;
		}
	} else {

		/* run platform code to initialize PHY device */
		if (pd->mii_config &&
		    pd->mii_config(dev, 1, bcm_enet_mdio_read_mii,
				   bcm_enet_mdio_write_mii)) {
			dev_err(&pdev->dev, "unable to configure mdio bus\n");
			goto out_uninit_hw;
		}
	}

	set_hw_mtu(priv, hw_mtu);

	/* init rx timeout (used for oom) */
	init_timer(&priv->rx_timeout);
	priv->rx_timeout.function = bcm_enet_refill_rx_timer;
	priv->rx_timeout.data = (unsigned long)dev;

	/* init the mib update lock&work */
	mutex_init(&priv->mib_update_lock);
	INIT_WORK(&priv->mib_update_task, bcm_enet_update_mib_counters_defer);

	/* zero mib counters */
	for (i = 0; i < ENET_MIB_REG_COUNT; i++)
		enet_writel(priv, 0, ENET_MIB_REG(i));

	/* register netdevice */
	dev->netdev_ops = &bcm_enet_ops;

	dev->ethtool_ops = &bcm_enet_ethtool_ops;
	SET_NETDEV_DEV(dev, &pdev->dev);

	platform_set_drvdata(pdev, dev);
	priv->pdev = pdev;
	priv->net_dev = dev;

	ret = register_netdev(dev);
	if (ret)
		goto out_unregister_mdio;

	netif_carrier_off(dev);

	dev_info(&priv->pdev->dev, "%s at MMIO 0x%08x, %pM\n", dev->name,
		res_mem->start, dev->dev_addr);
	return 0;

out_unregister_mdio:
	if (priv->mii_bus)
		mdiobus_unregister(priv->mii_bus);

out_free_mdio:
	if (priv->mii_bus)
		mdiobus_free(priv->mii_bus);

out_uninit_hw:
	/* turn off mdc clock */
	enet_writel(priv, 0, ENET_MIISC_REG);
	if (priv->phy_clk) {
		clk_disable_unprepare(priv->phy_clk);
		clk_put(priv->phy_clk);
	}

out_disable_clk_mac:
	while (--clk >= 0) {
		clk_disable_unprepare(priv->mac_clks[clk]);
		clk_put(priv->mac_clks[clk]);
	}

out_put_clk_mac:
	while (--clk >= 0)
		clk_put(priv->mac_clks[clk]);

	if (priv->regulator)
		regulator_disable(priv->regulator);

out:
	free_netdev(dev);
	return ret;
}


/*
 * exit func, stops hardware and unregisters netdevice
 */
static int bcm_enet_remove(struct platform_device *pdev)
{
	struct bcm_enet_priv *priv;
	struct net_device *dev;
	int clk;

	/* stop netdevice */
	dev = platform_get_drvdata(pdev);
	priv = netdev_priv(dev);
	unregister_netdev(dev);

	/* turn off mdc clock */
	enet_writel(priv, 0, ENET_MIISC_REG);

	if (priv->has_phy) {
		mdiobus_unregister(priv->mii_bus);
		mdiobus_free(priv->mii_bus);
	} else {
		struct bcm63xx_enet_platform_data *pd;

		pd = dev_get_platdata(&pdev->dev);
		if (pd && pd->mii_config)
			pd->mii_config(dev, 0, bcm_enet_mdio_read_mii,
				       bcm_enet_mdio_write_mii);
	}

	/* disable hw block clocks */
	if (priv->phy_clk) {
		clk_disable_unprepare(priv->phy_clk);
		clk_put(priv->phy_clk);
	}

	for (clk = 0; clk < BCMENET_MAX_CLKS && priv->mac_clks[clk]; clk++) {
		clk_disable_unprepare(priv->mac_clks[clk]);
		clk_put(priv->mac_clks[clk]);
	}

	if (priv->regulator)
		regulator_disable(priv->regulator);

	free_netdev(dev);
	return 0;
}

struct platform_driver bcm63xx_enet_driver = {
	.probe	= bcm_enet_probe,
	.remove	= bcm_enet_remove,
	.driver	= {
		.name	= "bcm63xx_enet",
		.owner  = THIS_MODULE,
	},
};

/*
 * switch mii access callbacks
 */
static int bcmenet_sw_mdio_read(struct bcm_enet_priv *priv,
				int ext, int phy_id, int location)
{
	u32 reg;
	int ret;

	spin_lock_bh(&priv->enetsw_mdio_lock);
	enetsw_writel(priv, 0, ENETSW_MDIOC_REG);

	reg = ENETSW_MDIOC_RD_MASK |
		(phy_id << ENETSW_MDIOC_PHYID_SHIFT) |
		(location << ENETSW_MDIOC_REG_SHIFT);

	if (ext)
		reg |= ENETSW_MDIOC_EXT_MASK;

	enetsw_writel(priv, reg, ENETSW_MDIOC_REG);
	udelay(50);
	ret = enetsw_readw(priv, ENETSW_MDIOD_REG);
	spin_unlock_bh(&priv->enetsw_mdio_lock);
	return ret;
}

static void bcmenet_sw_mdio_write(struct bcm_enet_priv *priv,
				 int ext, int phy_id, int location,
				 uint16_t data)
{
	u32 reg;

	spin_lock_bh(&priv->enetsw_mdio_lock);
	enetsw_writel(priv, 0, ENETSW_MDIOC_REG);

	reg = ENETSW_MDIOC_WR_MASK |
		(phy_id << ENETSW_MDIOC_PHYID_SHIFT) |
		(location << ENETSW_MDIOC_REG_SHIFT);

	if (ext)
		reg |= ENETSW_MDIOC_EXT_MASK;

	reg |= data;

	enetsw_writel(priv, reg, ENETSW_MDIOC_REG);
	udelay(50);
	spin_unlock_bh(&priv->enetsw_mdio_lock);
}

static inline int bcm_enet_port_is_rgmii(int portid)
{
	return portid >= ENETSW_RGMII_PORT0;
}

/*
 * enet sw PHY polling
 */
static void swphy_poll_timer(unsigned long data)
{
	struct bcm_enet_priv *priv = (struct bcm_enet_priv *)data;
	unsigned int i;

	for (i = 0; i < priv->num_ports; i++) {
		struct bcm63xx_enetsw_port *port;
		int val, j, up, advertise, lpa, speed, duplex, media;
		int external_phy = bcm_enet_port_is_rgmii(i);
		u8 override;

		port = &priv->used_ports[i];
		if (!port->used)
			continue;

		if (port->bypass_link)
			continue;

		/* dummy read to clear */
		for (j = 0; j < 2; j++)
			val = bcmenet_sw_mdio_read(priv, external_phy,
						   port->phy_id, MII_BMSR);

		if (val == 0xffff)
			continue;

		up = (val & BMSR_LSTATUS) ? 1 : 0;
		if (!(up ^ priv->sw_port_link[i]))
			continue;

		priv->sw_port_link[i] = up;

		/* link changed */
		if (!up) {
			dev_info(&priv->pdev->dev, "link DOWN on %s\n",
				 port->name);
			enetsw_writeb(priv, ENETSW_PORTOV_ENABLE_MASK,
				      ENETSW_PORTOV_REG(i));
			enetsw_writeb(priv, ENETSW_PTCTRL_RXDIS_MASK |
				      ENETSW_PTCTRL_TXDIS_MASK,
				      ENETSW_PTCTRL_REG(i));
			continue;
		}

		advertise = bcmenet_sw_mdio_read(priv, external_phy,
						 port->phy_id, MII_ADVERTISE);

		lpa = bcmenet_sw_mdio_read(priv, external_phy, port->phy_id,
					   MII_LPA);

		/* figure out media and duplex from advertise and LPA values */
		media = mii_nway_result(lpa & advertise);
		duplex = (media & ADVERTISE_FULL) ? 1 : 0;

		if (media & (ADVERTISE_100FULL | ADVERTISE_100HALF))
			speed = 100;
		else
			speed = 10;

		if (val & BMSR_ESTATEN) {
			advertise = bcmenet_sw_mdio_read(priv, external_phy,
						port->phy_id, MII_CTRL1000);

			lpa = bcmenet_sw_mdio_read(priv, external_phy,
						port->phy_id, MII_STAT1000);

			if (advertise & (ADVERTISE_1000FULL | ADVERTISE_1000HALF)
					&& lpa & (LPA_1000FULL | LPA_1000HALF)) {
				speed = 1000;
				duplex = (lpa & LPA_1000FULL);
			}
		}

		dev_info(&priv->pdev->dev,
			 "link UP on %s, %dMbps, %s-duplex\n",
			 port->name, speed, duplex ? "full" : "half");

		override = ENETSW_PORTOV_ENABLE_MASK |
			ENETSW_PORTOV_LINKUP_MASK;

		if (speed == 1000)
			override |= ENETSW_IMPOV_1000_MASK;
		else if (speed == 100)
			override |= ENETSW_IMPOV_100_MASK;
		if (duplex)
			override |= ENETSW_IMPOV_FDX_MASK;

		enetsw_writeb(priv, override, ENETSW_PORTOV_REG(i));
		enetsw_writeb(priv, 0, ENETSW_PTCTRL_REG(i));
	}

	priv->swphy_poll.expires = jiffies + HZ;
	add_timer(&priv->swphy_poll);
}

/*
 * open callback, start rx operation
 */
static int bcm_enetsw_open(struct net_device *dev)
{
	struct bcm_enet_priv *priv;
	struct device *kdev;
	int i, ret;
	u32 val;

	priv = netdev_priv(dev);
	kdev = &priv->pdev->dev;

	ret = bcm_enet_request_dma(dev);
	if (ret)
		return ret;

	/* disable all ports */
	for (i = 0; i < ENETSW_MAX_PORT; i++) {
		enetsw_writeb(priv, ENETSW_PORTOV_ENABLE_MASK,
			      ENETSW_PORTOV_REG(i));
		enetsw_writeb(priv, ENETSW_PTCTRL_RXDIS_MASK |
			      ENETSW_PTCTRL_TXDIS_MASK,
			      ENETSW_PTCTRL_REG(i));

		priv->sw_port_link[i] = 0;
	}

	/* reset mib */
	val = enetsw_readb(priv, ENETSW_GMCR_REG);
	val |= ENETSW_GMCR_RST_MIB_MASK;
	enetsw_writeb(priv, val, ENETSW_GMCR_REG);
	mdelay(1);
	val &= ~ENETSW_GMCR_RST_MIB_MASK;
	enetsw_writeb(priv, val, ENETSW_GMCR_REG);
	mdelay(1);

	/* force CPU port state */
	val = enetsw_readb(priv, ENETSW_IMPOV_REG);
	val |= ENETSW_IMPOV_FORCE_MASK | ENETSW_IMPOV_LINKUP_MASK;
	enetsw_writeb(priv, val, ENETSW_IMPOV_REG);

	/* enable switch forward engine */
	val = enetsw_readb(priv, ENETSW_SWMODE_REG);
	val |= ENETSW_SWMODE_FWD_EN_MASK;
	enetsw_writeb(priv, val, ENETSW_SWMODE_REG);

	/* enable jumbo on all ports */
	enetsw_writel(priv, 0x1ff, ENETSW_JMBCTL_PORT_REG);
	enetsw_writew(priv, 9728, ENETSW_JMBCTL_MAXSIZE_REG);

	/* enable all ports and forward all ports to all ports */
	val = 0;
	for (i = 0; i < priv->num_ports; i++)
		if (priv->used_ports[i].used)
			val |= (1 << i);
	enetsw_writeb(priv, val, ENETSW_PORT_ENABLE_REG);

	val = (1 << ENETSW_MAX_PORT); /* host port */
	for (i = 0; i < ENETSW_MAX_PORT; i++) {
		if (priv->used_ports[i].used)
			enetsw_writew(priv, val, ENETSW_PORT_BASED_VLAN(i));
		else
			enetsw_writew(priv, 0, ENETSW_PORT_BASED_VLAN(i));
	}

	/* apply override config for bypass_link ports here. */
	for (i = 0; i < priv->num_ports; i++) {
		struct bcm63xx_enetsw_port *port;
		u8 override;
		port = &priv->used_ports[i];
		if (!port->used)
			continue;

		if (!port->bypass_link)
			continue;

		override = ENETSW_PORTOV_ENABLE_MASK |
			ENETSW_PORTOV_LINKUP_MASK;

		switch (port->force_speed) {
		case 1000:
			override |= ENETSW_IMPOV_1000_MASK;
			break;
		case 100:
			override |= ENETSW_IMPOV_100_MASK;
			break;
		case 10:
			break;
		default:
			pr_warn("invalid forced speed on port %s: assume 10\n",
			       port->name);
			break;
		}

		if (port->force_duplex_full)
			override |= ENETSW_IMPOV_FDX_MASK;


		enetsw_writeb(priv, override, ENETSW_PORTOV_REG(i));
		enetsw_writeb(priv, 0, ENETSW_PTCTRL_REG(i));
	}

	/* start phy polling timer */
	init_timer(&priv->swphy_poll);
	priv->swphy_poll.function = swphy_poll_timer;
	priv->swphy_poll.data = (unsigned long)priv;
	priv->swphy_poll.expires = jiffies;
	add_timer(&priv->swphy_poll);

	netif_carrier_on(dev);
	bcm_enet_start_rx_dma(dev);
	bcm_enet_start_tx_dma(dev);
	return 0;
}

/* stop callback */
static int bcm_enetsw_stop(struct net_device *dev)
{
	struct bcm_enet_priv *priv;
	struct device *kdev;
	LIST_HEAD(pkts);

	priv = netdev_priv(dev);
	kdev = &priv->pdev->dev;

	del_timer_sync(&priv->swphy_poll);
	del_timer_sync(&priv->rx_timeout);

	bcm_enet_release_dma(dev);

	return 0;
}

/* try to sort out phy external status by walking the used_port field
 * in the bcm_enet_priv structure. in case the phy address is not
 * assigned to any physical port on the switch, assume it is external
 * (and yell at the user).
 */
static int bcm_enetsw_phy_is_external(struct bcm_enet_priv *priv, int phy_id)
{
	int i;

	for (i = 0; i < priv->num_ports; ++i) {
		if (!priv->used_ports[i].used)
			continue;
		if (priv->used_ports[i].phy_id == phy_id)
			return bcm_enet_port_is_rgmii(i);
	}

	printk_once(KERN_WARNING  "bcm63xx_enet: could not find a used port with phy_id %i, assuming phy is external\n",
		    phy_id);
	return 1;
}

/* can't use bcmenet_sw_mdio_read directly as we need to sort out
 * external/internal status of the given phy_id first.
 */
static int bcm_enetsw_mii_mdio_read(struct net_device *dev, int phy_id,
				    int location)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);
	return bcmenet_sw_mdio_read(priv,
				    bcm_enetsw_phy_is_external(priv, phy_id),
				    phy_id, location);
}

/* can't use bcmenet_sw_mdio_write directly as we need to sort out
 * external/internal status of the given phy_id first.
 */
static void bcm_enetsw_mii_mdio_write(struct net_device *dev, int phy_id,
				      int location,
				      int val)
{
	struct bcm_enet_priv *priv;

	priv = netdev_priv(dev);
	bcmenet_sw_mdio_write(priv, bcm_enetsw_phy_is_external(priv, phy_id),
			      phy_id, location, val);
}

static int bcm_enetsw_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct mii_if_info mii;

	mii.dev = dev;
	mii.mdio_read = bcm_enetsw_mii_mdio_read;
	mii.mdio_write = bcm_enetsw_mii_mdio_write;
	mii.phy_id = 0;
	mii.phy_id_mask = 0x3f;
	mii.reg_num_mask = 0x1f;
	return generic_mii_ioctl(&mii, if_mii(rq), cmd, NULL);

}

static const struct net_device_ops bcm_enetsw_ops = {
	.ndo_open		= bcm_enetsw_open,
	.ndo_stop		= bcm_enetsw_stop,
	.ndo_start_xmit		= bcm_enet_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_change_mtu		= bcm_enet_change_mtu,
	.ndo_do_ioctl		= bcm_enetsw_ioctl,
};


static const struct bcm_enet_stats bcm_enetsw_gstrings_stats[] = {
	{ "rx_packets", DEV_STAT(rx_packets), -1 },
	{ "tx_packets",	DEV_STAT(tx_packets), -1 },
	{ "rx_bytes", DEV_STAT(rx_bytes), -1 },
	{ "tx_bytes", DEV_STAT(tx_bytes), -1 },
	{ "rx_errors", DEV_STAT(rx_errors), -1 },
	{ "tx_errors", DEV_STAT(tx_errors), -1 },
	{ "rx_dropped",	DEV_STAT(rx_dropped), -1 },
	{ "tx_dropped",	DEV_STAT(tx_dropped), -1 },

	{ "tx_good_octets", GEN_STAT(mib.tx_gd_octets), ETHSW_MIB_RX_GD_OCT },
	{ "tx_unicast", GEN_STAT(mib.tx_unicast), ETHSW_MIB_RX_BRDCAST },
	{ "tx_broadcast", GEN_STAT(mib.tx_brdcast), ETHSW_MIB_RX_BRDCAST },
	{ "tx_multicast", GEN_STAT(mib.tx_mult), ETHSW_MIB_RX_MULT },
	{ "tx_64_octets", GEN_STAT(mib.tx_64), ETHSW_MIB_RX_64 },
	{ "tx_65_127_oct", GEN_STAT(mib.tx_65_127), ETHSW_MIB_RX_65_127 },
	{ "tx_128_255_oct", GEN_STAT(mib.tx_128_255), ETHSW_MIB_RX_128_255 },
	{ "tx_256_511_oct", GEN_STAT(mib.tx_256_511), ETHSW_MIB_RX_256_511 },
	{ "tx_512_1023_oct", GEN_STAT(mib.tx_512_1023), ETHSW_MIB_RX_512_1023},
	{ "tx_1024_1522_oct", GEN_STAT(mib.tx_1024_max),
	  ETHSW_MIB_RX_1024_1522 },
	{ "tx_1523_2047_oct", GEN_STAT(mib.tx_1523_2047),
	  ETHSW_MIB_RX_1523_2047 },
	{ "tx_2048_4095_oct", GEN_STAT(mib.tx_2048_4095),
	  ETHSW_MIB_RX_2048_4095 },
	{ "tx_4096_8191_oct", GEN_STAT(mib.tx_4096_8191),
	  ETHSW_MIB_RX_4096_8191 },
	{ "tx_8192_9728_oct", GEN_STAT(mib.tx_8192_9728),
	  ETHSW_MIB_RX_8192_9728 },
	{ "tx_oversize", GEN_STAT(mib.tx_ovr), ETHSW_MIB_RX_OVR },
	{ "tx_oversize_drop", GEN_STAT(mib.tx_ovr), ETHSW_MIB_RX_OVR_DISC },
	{ "tx_dropped",	GEN_STAT(mib.tx_drop), ETHSW_MIB_RX_DROP },
	{ "tx_undersize", GEN_STAT(mib.tx_underrun), ETHSW_MIB_RX_UND },
	{ "tx_pause", GEN_STAT(mib.tx_pause), ETHSW_MIB_RX_PAUSE },

	{ "rx_good_octets", GEN_STAT(mib.rx_gd_octets), ETHSW_MIB_TX_ALL_OCT },
	{ "rx_broadcast", GEN_STAT(mib.rx_brdcast), ETHSW_MIB_TX_BRDCAST },
	{ "rx_multicast", GEN_STAT(mib.rx_mult), ETHSW_MIB_TX_MULT },
	{ "rx_unicast", GEN_STAT(mib.rx_unicast), ETHSW_MIB_TX_MULT },
	{ "rx_pause", GEN_STAT(mib.rx_pause), ETHSW_MIB_TX_PAUSE },
	{ "rx_dropped", GEN_STAT(mib.rx_drop), ETHSW_MIB_TX_DROP_PKTS },

};

#define BCM_ENETSW_STATS_LEN	\
	(sizeof(bcm_enetsw_gstrings_stats) / sizeof(struct bcm_enet_stats))

static void bcm_enetsw_get_strings(struct net_device *netdev,
				   u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < BCM_ENETSW_STATS_LEN; i++) {
			memcpy(data + i * ETH_GSTRING_LEN,
			       bcm_enetsw_gstrings_stats[i].stat_string,
			       ETH_GSTRING_LEN);
		}
		break;
	}
}

static int bcm_enetsw_get_sset_count(struct net_device *netdev,
				     int string_set)
{
	switch (string_set) {
	case ETH_SS_STATS:
		return BCM_ENETSW_STATS_LEN;
	default:
		return -EINVAL;
	}
}

static void bcm_enetsw_get_drvinfo(struct net_device *netdev,
				   struct ethtool_drvinfo *drvinfo)
{
	strncpy(drvinfo->driver, bcm_enet_driver_name, 32);
	strncpy(drvinfo->version, bcm_enet_driver_version, 32);
	strncpy(drvinfo->fw_version, "N/A", 32);
	strncpy(drvinfo->bus_info, "bcm63xx", 32);
}

static void bcm_enetsw_get_ethtool_stats(struct net_device *netdev,
					 struct ethtool_stats *stats,
					 u64 *data)
{
	struct bcm_enet_priv *priv;
	int i;

	priv = netdev_priv(netdev);

	for (i = 0; i < BCM_ENETSW_STATS_LEN; i++) {
		const struct bcm_enet_stats *s;
		u32 lo, hi;
		char *p;
		int reg;

		s = &bcm_enetsw_gstrings_stats[i];

		reg = s->mib_reg;
		if (reg == -1)
			continue;

		lo = enetsw_readl(priv, ENETSW_MIB_REG(reg));
		p = (char *)priv + s->stat_offset;

		if (s->sizeof_stat == sizeof(u64)) {
			hi = enetsw_readl(priv, ENETSW_MIB_REG(reg + 1));
			*(u64 *)p = ((u64)hi << 32 | lo);
		} else {
			*(u32 *)p = lo;
		}
	}

	for (i = 0; i < BCM_ENETSW_STATS_LEN; i++) {
		const struct bcm_enet_stats *s;
		char *p;

		s = &bcm_enetsw_gstrings_stats[i];

		if (s->mib_reg == -1)
			p = (char *)&netdev->stats + s->stat_offset;
		else
			p = (char *)priv + s->stat_offset;

		data[i] = (s->sizeof_stat == sizeof(u64)) ?
			*(u64 *)p : *(u32 *)p;
	}
}

static struct ethtool_ops bcm_enetsw_ethtool_ops = {
	.get_strings		= bcm_enetsw_get_strings,
	.get_sset_count		= bcm_enetsw_get_sset_count,
	.get_ethtool_stats      = bcm_enetsw_get_ethtool_stats,
	.get_drvinfo		= bcm_enetsw_get_drvinfo,
	.get_ringparam		= bcm_enet_get_ringparam,
	.set_ringparam		= bcm_enet_set_ringparam,
};

/* allocate netdevice, request register memory and register device. */
static int bcm_enetsw_probe(struct platform_device *pdev)
{
	struct regulator *regulator = NULL;
	struct reset_control *reset = NULL;
	struct bcm_enet_priv *priv;
	struct device_node *np = pdev->dev.of_node;
	struct net_device *dev;
	struct bcm63xx_enetsw_platform_data *pd;
	struct resource *res_mem;
	unsigned int hw_mtu;
	int clk, ret;

	pd = dev_get_platdata(&pdev->dev);

	if (!pd) {
		regulator = devm_regulator_get(&pdev->dev, "power");
		if (IS_ERR(regulator)) {
			ret = PTR_ERR(regulator);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev,
					"failed to get regulator: %d\n", ret);
			return ret;
		}

		reset = devm_reset_control_get(&pdev->dev, NULL);
		if (IS_ERR(reset)) {
			ret = PTR_ERR(reset);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev,
					"failed to get reset controller: %d\n",
					ret);
			return ret;
		}
	}

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -ENODEV;

	ret = 0;
	dev = alloc_etherdev(sizeof(*priv));
	if (!dev)
		return -ENOMEM;
	priv = netdev_priv(dev);
	memset(priv, 0, sizeof(*priv));

	if (!pd) {
		priv->regulator = regulator;
		priv->reset = reset;
	}

	/* initialize default and fetch platform data */
	priv->enet_is_sw = true;
	priv->dma_maxburst = BCMENETSW_DMA_MAXBURST;

	spin_lock_init(&priv->rx_lock);
	spin_lock_init(&priv->tx_lock);

	if (pd) {
		memcpy(&priv->pd_rx_dma, &pd->rx_dma, sizeof(pd->rx_dma));
		memcpy(&priv->pd_tx_dma, &pd->tx_dma, sizeof(pd->tx_dma));
		memcpy(dev->dev_addr, pd->mac_addr, ETH_ALEN);
		memcpy(priv->used_ports, pd->used_ports,
		       sizeof(pd->used_ports));
		priv->num_ports = pd->num_ports;
		priv->rx_max_ring_size = pd->rx_max_ring_size;
		priv->tx_max_ring_size = pd->tx_max_ring_size;
	} else {
		int i;

		for (i = 0; i < ENETSW_MAX_PORT; i++) {
			u32 phy_id;

			ret = of_property_read_u32_index(np,
				"brcm,phy-ids", i, &phy_id);
			if (ret)
				break;

			priv->used_ports[i].phy_id = phy_id;

			ret = of_property_read_string_index(np,
				"brcm,port-names", i, &priv->used_ports[i].name);
			if (ret)
				break;

			priv->used_ports[i].used = strlen(
				priv->used_ports[i].name) > 0;

			priv->num_ports++;
		}

		/* limited by DMA controller */
		priv->rx_max_ring_size = BCMENET_MAX_RX_DESC;
		priv->tx_max_ring_size = BCMENET_MAX_TX_DESC;
	}

	priv->rx_ring_size = BCMENET_DEF_RX_DESC;
	priv->tx_ring_size = BCMENET_DEF_TX_DESC;

	hw_mtu = compute_hw_mtu(dev->mtu);
	if (!hw_mtu) {
		ret = -EINVAL;
		goto out;
	}

	priv->base = devm_ioremap_resource(&pdev->dev, res_mem);
	if (IS_ERR(priv->base)) {
		ret = PTR_ERR(priv->base);
		goto out;
	}

	if (priv->regulator) {
		ret = regulator_enable(priv->regulator);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to power on device: %d\n", ret);
			goto out;
		}
	}

	for (clk = 0; clk < BCMENET_MAX_CLKS; clk++) {
		priv->mac_clks[clk] = of_clk_get(np, clk);
		if (IS_ERR(priv->mac_clks[clk])) {
			ret = PTR_ERR(priv->mac_clks[clk]);
			if (ret == -EPROBE_DEFER)
				goto out_put_clk;

			priv->mac_clks[clk] = NULL;
			break;
		}
	}

	if (clk == 0 && priv->mac_clks[0] == NULL) {
		priv->mac_clks[clk] = clk_get(&pdev->dev, "enetsw");
		if (IS_ERR(priv->mac_clks[clk])) {
			ret = PTR_ERR(priv->mac_clks[clk]);
			goto out_put_clk;
		}
	}

	for (clk = 0; clk < BCMENET_MAX_CLKS && priv->mac_clks[clk]; clk++) {
		ret = clk_prepare_enable(priv->mac_clks[clk]);
		if (ret) {
			dev_err(&pdev->dev, "unable to enable %s clock: %d\n",
#ifdef CONFIG_COMMON_CLK
				__clk_get_name(priv->mac_clks[clk]),
#else
				"enetsw",
#endif
				ret);
			goto out_disable_clk;
		}
	}

	if (priv->reset) {
		ret = reset_control_reset(priv->reset);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to reset device: %d\n", ret);
			goto out_disable_clk;
		}
	}

	set_hw_mtu(priv, hw_mtu);

	/* init rx timeout (used for oom) */
	init_timer(&priv->rx_timeout);
	priv->rx_timeout.function = bcm_enet_refill_rx_timer;
	priv->rx_timeout.data = (unsigned long)dev;

	/* register netdevice */
	dev->netdev_ops = &bcm_enetsw_ops;
	dev->ethtool_ops = &bcm_enetsw_ethtool_ops;
	SET_NETDEV_DEV(dev, &pdev->dev);

	spin_lock_init(&priv->enetsw_mdio_lock);

	platform_set_drvdata(pdev, dev);
	priv->pdev = pdev;
	priv->net_dev = dev;

	ret = register_netdev(dev);
	if (ret)
		goto out_disable_clk;

	netif_carrier_off(dev);

	dev_info(&priv->pdev->dev, "%s at MMIO 0x%08x, %pM\n", dev->name,
		res_mem->start, dev->dev_addr);
	return 0;

out_disable_clk:
	while (--clk >= 0) {
		clk_disable_unprepare(priv->mac_clks[clk]);
		clk_put(priv->mac_clks[clk]);
	}

out_put_clk:
	while (--clk >= 0)
		clk_put(priv->mac_clks[clk]);

	if (priv->regulator)
		regulator_disable(priv->regulator);

out:
	free_netdev(dev);
	return ret;
}


/* exit func, stops hardware and unregisters netdevice */
static int bcm_enetsw_remove(struct platform_device *pdev)
{
	struct bcm_enet_priv *priv;
	struct net_device *dev;
	int clk;

	/* stop netdevice */
	dev = platform_get_drvdata(pdev);
	priv = netdev_priv(dev);
	unregister_netdev(dev);

	for (clk = 0; clk < BCMENET_MAX_CLKS && priv->mac_clks[clk]; clk++) {
		clk_disable_unprepare(priv->mac_clks[clk]);
		clk_put(priv->mac_clks[clk]);
	}

	if (priv->regulator)
		regulator_disable(priv->regulator);

	free_netdev(dev);
	return 0;
}

static const struct of_device_id bcm63xx_enetsw_dt_ids[] = {
	{ .compatible = "brcm,bcm63xx-enetsw" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm63xx_enetsw_dt_ids);

struct platform_driver bcm63xx_enetsw_driver = {
	.probe	= bcm_enetsw_probe,
	.remove	= bcm_enetsw_remove,
	.driver	= {
		.name	= "bcm63xx_enetsw",
		.owner  = THIS_MODULE,
		.of_match_table = bcm63xx_enetsw_dt_ids,
	},
};

/* entry point */
static int __init bcm_enet_init(void)
{
	int ret;

	ret = platform_driver_register(&bcm63xx_enet_driver);
	ret = platform_driver_register(&bcm63xx_enetsw_driver);
	if (ret)
		platform_driver_unregister(&bcm63xx_enet_driver);

	return ret;
}

static void __exit bcm_enet_exit(void)
{
	platform_driver_unregister(&bcm63xx_enet_driver);
	platform_driver_unregister(&bcm63xx_enetsw_driver);
}


module_init(bcm_enet_init);
module_exit(bcm_enet_exit);

MODULE_DESCRIPTION("BCM63xx internal ethernet mac driver");
MODULE_AUTHOR("Maxime Bizon <mbizon@freebox.fr>");
MODULE_AUTHOR("Simon Arlott");
MODULE_LICENSE("GPL");
