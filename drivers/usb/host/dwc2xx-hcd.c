/*
 * Copyright 2012  Simon Arlott
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Derived from the Synopsys, Inc. driver (see dwc2xx-hcd.h for licence notice)
 */

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
#include <linux/spinlock.h>

#include "dwc2xx-hcd.h"

#define MODULE_NAME "dwc2xx-hcd"

static irqreturn_t dwc2xx_hcd_irq_handler(int irq, void *dev_id)
{
	struct dwc2xx_hcd *hcd = dev_id;

	dev_dbg(hcd->dev, "irq\n");
	return IRQ_NONE;
}

static void dwc2xx_hcd_free(struct dwc2xx_hcd *hcd)
{
	kfree(hcd);
}

static int __devinit dwc2xx_hcd_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct dwc2xx_hcd *hcd = kzalloc(sizeof(*hcd), GFP_KERNEL);
	int ret;

	if (hcd == NULL)
		return -ENOMEM;

	hcd->dev = &pdev->dev;
	if (of_address_to_resource(np, 0, &hcd->res)) {
		ret = -EINVAL;
		goto err;
	}

	if (resource_size(&hcd->res) < SZ_128K) {
		dev_err(hcd->dev, "resource too small (%#x)\n",
			resource_size(&hcd->res));
		ret = -EINVAL;
		goto err;
	}

	if (!request_region(hcd->res.start, resource_size(&hcd->res),
			np->full_name)) {
		dev_err(hcd->dev, "resource %#lx unavailable\n",
			(unsigned long)hcd->res.start);
		ret = -EBUSY;
		goto err;
	}

	hcd->base = ioremap(hcd->res.start, resource_size(&hcd->res));
	if (!hcd->base) {
		dev_err(hcd->dev, "error mapping io at %#lx\n",
			(unsigned long)hcd->res.start);
		ret = -EIO;
		goto err_release;
	}

	hcd->id = readl(hcd->base + DWC_SNPS_ID_REG);
	if ((hcd->id & DWC_SNPS_ID_MASK) != DWC_SNPS_ID_MATCH) {
		dev_err(hcd->dev, "incompatible device %08x\n", hcd->id);
		ret = -ENODEV;
		goto err_unmap;
	}

	/* TODO: ensure the device won't trigger any interrupts */
	hcd->irq = irq_of_parse_and_map(np, 0);
	if (hcd->irq <= 0) {
		dev_err(hcd->dev, "no IRQ\n");
		ret = -ENXIO;
		goto err_unmap;
	}
	hcd->irqaction.name = dev_name(hcd->dev);
	hcd->irqaction.flags = IRQF_SHARED | IRQF_IRQPOLL;
	hcd->irqaction.dev_id = hcd;
	hcd->irqaction.handler = dwc2xx_hcd_irq_handler;

	ret = setup_irq(hcd->irq, &hcd->irqaction);
	if (ret) {
		dev_err(hcd->dev, "unable to setup irq %d", hcd->irq);
		spin_unlock_irq(&hcd->lock);
		goto err_unmap;
	}

	dev_info(hcd->dev, "HCD v2.%03x at MMIO %#lx (irq = %d)\n",
		hcd->id & ~DWC_SNPS_ID_MASK, (unsigned long)hcd->res.start, hcd->irq);

	dwc2xx_hcd_dump_regs(hcd);

	platform_set_drvdata(pdev, hcd);
	return 0;

/*
err_irq:
	remove_irq(hcd->irq, &hcd->irqaction);
*/
err_unmap:
	iounmap(hcd->base);
err_release:
	release_region(hcd->res.start, resource_size(&hcd->res));
err:
	dwc2xx_hcd_free(hcd);
	return ret;
}

static int dwc2xx_hcd_remove(struct platform_device *pdev)
{
	struct dwc2xx_hcd *hcd = platform_get_drvdata(pdev);

	remove_irq(hcd->irq, &hcd->irqaction);
	iounmap(hcd->base);
	release_region(hcd->res.start, resource_size(&hcd->res));
	dwc2xx_hcd_free(hcd);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct of_device_id dwc2xx_hcd_match[] __devinitconst = {
	{ .compatible = "synopsys,designware-hs-otg2-host" },
	{ .compatible = "broadcom,bcm2708-usb" },
	{}
};
MODULE_DEVICE_TABLE(of, dwc2xx_hcd_match);

static struct platform_driver dwc2xx_hcd_driver = {
	.probe	= dwc2xx_hcd_probe,
	.remove	= dwc2xx_hcd_remove,
	.driver	= {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dwc2xx_hcd_match
	}
};
module_platform_driver(dwc2xx_hcd_driver);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("Synopsys Designware HS OTG 2.xx driver (host mode)");
MODULE_LICENSE("GPL");
