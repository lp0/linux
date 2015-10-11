/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/export.h>
#include <linux/bcm63xx_dev_enet.h>
#include <linux/bcm63xx_iudma.h>
#include <bcm63xx_io.h>
#include <bcm63xx_regs.h>

static struct resource enetdma_res[] = {
	{
		.start		= -1, /* filled at runtime */
		.end		= -1, /* filled at runtime */
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
};

static struct bcm63xx_iudma_platform_data enetdma_pd;

static struct platform_device bcm63xx_enetdma_device = {
	.name		= "bcm63xx-iudma",
	.num_resources	= ARRAY_SIZE(enetdma_res),
	.resource	= enetdma_res,
	.dev		= {
		.platform_data = &enetdma_pd,
	},
};

static int enetdma_device_registered;

static struct resource enet0_res[] = {
	{
		.start		= -1, /* filled at runtime */
		.end		= -1, /* filled at runtime */
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
};

static struct bcm63xx_enet_platform_data enet0_pd;

static struct platform_device bcm63xx_enet0_device = {
	.name		= "bcm63xx_enet",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(enet0_res),
	.resource	= enet0_res,
	.dev		= {
		.platform_data = &enet0_pd,
	},
};

static struct resource enet1_res[] = {
	{
		.start		= -1, /* filled at runtime */
		.end		= -1, /* filled at runtime */
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= -1, /* filled at runtime */
		.flags		= IORESOURCE_IRQ,
	},
};

static struct bcm63xx_enet_platform_data enet1_pd;

static struct platform_device bcm63xx_enet1_device = {
	.name		= "bcm63xx_enet",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(enet1_res),
	.resource	= enet1_res,
	.dev		= {
		.platform_data = &enet1_pd,
	},
};

static struct resource enetsw_res[] = {
	{
		/* start & end filled at runtime */
		.flags		= IORESOURCE_MEM,
	},
};

static struct bcm63xx_enetsw_platform_data enetsw_pd;

static struct platform_device bcm63xx_enetsw_device = {
	.name		= "bcm63xx_enetsw",
	.num_resources	= ARRAY_SIZE(enetsw_res),
	.resource	= enetsw_res,
	.dev		= {
		.platform_data = &enetsw_pd,
	},
};

static int __init register_enetdma(bool enetsw)
{
	int ret;

	if (enetdma_device_registered)
		return 0;

	enetdma_pd.dmac_offset = bcm63xx_regset_address(RSET_ENETDMAC);
	enetdma_pd.dmac_offset -= bcm63xx_regset_address(RSET_ENETDMA);
	enetdma_pd.dmas_offset = bcm63xx_regset_address(RSET_ENETDMAS);
	enetdma_pd.dmas_offset -= bcm63xx_regset_address(RSET_ENETDMA);

	if (BCMCPU_IS_6328() || BCMCPU_IS_6362() || BCMCPU_IS_6368()
			|| BCMCPU_IS_63168()) {
		enetdma_pd.n_channels = 32;
	} else if (BCMCPU_IS_6345()) {
		enetdma_pd.n_channels = 8;
		enetdma_pd.dmas_offset = 0;
	} else {
		enetdma_pd.n_channels = 16;
	}

	enetdma_pd.n_requests = 512;

	enetdma_res[0].start = bcm63xx_regset_address(RSET_ENETDMA);
	enetdma_res[0].end = enetdma_res[0].start;
	if (BCMCPU_IS_6345()) {
		enetdma_res[0].end += (RSET_6345_ENETDMA_SIZE +
			RSET_ENETDMAC_SIZE(enetdma_pd.n_channels) - 1);
	} else {
		enetdma_res[0].end += (RSET_ENETDMA_SIZE +
			RSET_ENETDMAC_SIZE(enetdma_pd.n_channels) +
			RSET_ENETDMAS_SIZE(enetdma_pd.n_channels) - 1);
	}

	if (enetsw) {
		enetdma_res[1].start = bcm63xx_get_irq_number(IRQ_ENETSW_RXDMA0);
		enetdma_res[2].start = bcm63xx_get_irq_number(IRQ_ENETSW_TXDMA0);
	} else {
		enetdma_res[1].start = bcm63xx_get_irq_number(IRQ_ENET0_RXDMA);
		enetdma_res[2].start = bcm63xx_get_irq_number(IRQ_ENET0_TXDMA);
		enetdma_res[3].start = bcm63xx_get_irq_number(IRQ_ENET1_RXDMA);
		enetdma_res[4].start = bcm63xx_get_irq_number(IRQ_ENET1_TXDMA);
	}

	if (!enetdma_res[1].start)
		enetdma_res[1].start = -1;
	if (!enetdma_res[2].start)
		enetdma_res[2].start = -1;
	if (!enetdma_res[3].start)
		enetdma_res[3].start = -1;
	if (!enetdma_res[4].start)
		enetdma_res[4].start = -1;

	ret = platform_device_register(&bcm63xx_enetdma_device);
	if (ret)
		return ret;
	enetdma_device_registered = 1;

	return 0;
}

int __init bcm63xx_enet_register(int unit,
				 const struct bcm63xx_enet_platform_data *pd)
{
	struct platform_device *pdev;
	struct bcm63xx_enet_platform_data *dpd;
	int ret;

	if (unit > 1)
		return -ENODEV;

	if (unit == 1 && (BCMCPU_IS_6338() || BCMCPU_IS_6345()))
		return -ENODEV;

	ret = register_enetdma(false);
	if (ret)
		return ret;

	if (unit == 0) {
		enet0_res[0].start = bcm63xx_regset_address(RSET_ENET0);
		enet0_res[0].end = enet0_res[0].start;
		enet0_res[0].end += RSET_ENET_SIZE - 1;
		enet0_res[1].start = bcm63xx_get_irq_number(IRQ_ENET0);
		enet0_res[2].start = bcm63xx_get_irq_number(IRQ_ENET0_RXDMA);
		enet0_res[3].start = bcm63xx_get_irq_number(IRQ_ENET0_TXDMA);
		pdev = &bcm63xx_enet0_device;
	} else {
		enet1_res[0].start = bcm63xx_regset_address(RSET_ENET1);
		enet1_res[0].end = enet1_res[0].start;
		enet1_res[0].end += RSET_ENET_SIZE - 1;
		enet1_res[1].start = bcm63xx_get_irq_number(IRQ_ENET1);
		enet1_res[2].start = bcm63xx_get_irq_number(IRQ_ENET1_RXDMA);
		enet1_res[3].start = bcm63xx_get_irq_number(IRQ_ENET1_TXDMA);
		pdev = &bcm63xx_enet1_device;
	}

	/* copy given platform data */
	dpd = pdev->dev.platform_data;
	memcpy(dpd, pd, sizeof(*pd));

	/* adjust them in case internal phy is used */
	if (dpd->use_internal_phy) {

		/* internal phy only exists for enet0 */
		if (unit == 1)
			return -ENODEV;

		dpd->phy_id = 1;
		dpd->has_phy_interrupt = 1;
		dpd->phy_interrupt = bcm63xx_get_irq_number(IRQ_ENET_PHY);
	}

	if (!dpd->rx_max_ring_size)
		dpd->rx_max_ring_size = 512;

	if (!dpd->tx_max_ring_size)
		dpd->tx_max_ring_size = 512;

	dpd->rx_dma.dma_dev = &bcm63xx_enetdma_device.dev;
	dpd->tx_dma.dma_dev = &bcm63xx_enetdma_device.dev;
	if (unit == 0) {
		dpd->rx_dma.chan_id = 0;
		dpd->tx_dma.chan_id = 1;
	} else {
		dpd->rx_dma.chan_id = 2;
		dpd->tx_dma.chan_id = 3;
	}

	ret = platform_device_register(pdev);
	if (ret)
		return ret;
	return 0;
}

int __init
bcm63xx_enetsw_register(const struct bcm63xx_enetsw_platform_data *pd)
{
	int ret;

	if (!BCMCPU_IS_6328() && !BCMCPU_IS_6362() && !BCMCPU_IS_6368()
			&& !BCMCPU_IS_63168())
		return -ENODEV;

	ret = register_enetdma(true);
	if (ret)
		return ret;

	enetsw_res[0].start = bcm63xx_regset_address(RSET_ENETSW);
	enetsw_res[0].end = enetsw_res[0].start;
	enetsw_res[0].end += RSET_ENETSW_SIZE - 1;

	memcpy(bcm63xx_enetsw_device.dev.platform_data, pd, sizeof(*pd));

	if (BCMCPU_IS_6328())
		enetsw_pd.num_ports = ENETSW_PORTS_6328;
	else if (BCMCPU_IS_6362() || BCMCPU_IS_6368())
		enetsw_pd.num_ports = ENETSW_PORTS_6368;
	else if (BCMCPU_IS_63168())
		enetsw_pd.num_ports = ENETSW_PORTS_63168;

	if (!enetsw_pd.rx_max_ring_size)
		enetsw_pd.rx_max_ring_size = 512;

	if (!enetsw_pd.tx_max_ring_size)
		enetsw_pd.tx_max_ring_size = 512;

	enetsw_pd.rx_dma.dma_dev = &bcm63xx_enetdma_device.dev;
	enetsw_pd.tx_dma.dma_dev = &bcm63xx_enetdma_device.dev;
	enetsw_pd.rx_dma.chan_id = 0;
	enetsw_pd.tx_dma.chan_id = 1;

	ret = platform_device_register(&bcm63xx_enetsw_device);
	if (ret)
		return ret;

	return 0;
}
