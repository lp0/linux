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

#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "dwc2xx-hcd.h"

#define MODULE_NAME "dwc2xx-hcd"

static irqreturn_t dwc2xx_hcd_irq(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return IRQ_NONE;
}

static int dwc2xx_hcd_reset(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static int dwc2xx_hcd_start(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static void dwc2xx_hcd_stop(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
}

static void dwc2xx_hcd_shutdown(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
}

static int dwc2xx_hcd_get_frame_number(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return -ENOSYS;
}

static int dwc2xx_hcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
	gfp_t mem_flags)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return -ENOSYS;
}

static int dwc2xx_hcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb,
	int status)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return -ENOSYS;
}

static void dwc2xx_hcd_endpoint_disable(struct usb_hcd *hcd,
	struct usb_host_endpoint *ep)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
}

static int dwc2xx_hcd_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return -ENOSYS;
}

static int dwc2xx_hcd_hub_control(struct usb_hcd *hcd,
	u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return -ENOSYS;
}

static int dwc2xx_hcd_bus_suspend(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static int dwc2xx_hcd_bus_resume(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static int dwc2xx_hcd_start_port_reset(struct usb_hcd *hcd, unsigned port_num)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return -ENOSYS;
}

static const struct hc_driver dwc2xx_hcd_hc_driver = {
	.description		= MODULE_NAME,
	.product_desc		= "DWC 2.xx Host Controller",
	.hcd_priv_size		= sizeof(struct dwc2xx_hcd),
	
	.irq			= dwc2xx_hcd_irq,
	.flags			= HCD_MEMORY | HCD_USB2,

	.reset			= dwc2xx_hcd_reset,
	.start			= dwc2xx_hcd_start,
	.stop			= dwc2xx_hcd_stop,
	.shutdown		= dwc2xx_hcd_shutdown,

	.urb_enqueue		= dwc2xx_hcd_urb_enqueue,
	.urb_dequeue		= dwc2xx_hcd_urb_dequeue,
	.endpoint_disable	= dwc2xx_hcd_endpoint_disable,

	.get_frame_number	= dwc2xx_hcd_get_frame_number,

	.hub_status_data	= dwc2xx_hcd_hub_status_data,
	.hub_control		= dwc2xx_hcd_hub_control,

	.bus_suspend		= dwc2xx_hcd_bus_suspend,
	.bus_resume		= dwc2xx_hcd_bus_resume,

	.start_port_reset       = dwc2xx_hcd_start_port_reset
};

static int __devinit dwc2xx_hcd_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct usb_hcd *hcd;
	struct dwc2xx_hcd *dwc;
	int ret;

	hcd = usb_create_hcd(&dwc2xx_hcd_hc_driver,
			&pdev->dev, dev_name(&pdev->dev));
	if (hcd == NULL)
		return -ENOMEM;

	dwc = hcd_to_dwc(hcd);
	dwc->dev = &pdev->dev;
	if (of_address_to_resource(np, 0, &dwc->res)) {
		ret = -EINVAL;
		goto err;
	}

	if (resource_size(&dwc->res) < SZ_128K) {
		dev_err(dwc->dev, "resource too small (%#x)\n",
			resource_size(&dwc->res));
		ret = -EINVAL;
		goto err;
	}

	if (!request_region(dwc->res.start, resource_size(&dwc->res),
			np->full_name)) {
		dev_err(dwc->dev, "resource %#lx unavailable\n",
			(unsigned long)dwc->res.start);
		ret = -EBUSY;
		goto err;
	}

	hcd->regs = ioremap(dwc->res.start, resource_size(&dwc->res));
	if (!hcd->regs) {
		dev_err(dwc->dev, "error mapping io at %#lx\n",
			(unsigned long)dwc->res.start);
		ret = -EIO;
		goto err_release;
	}

	dwc->id = readl(hcd->regs + DWC_SNPS_ID_REG);
	if ((dwc->id & DWC_SNPS_ID_MASK) != DWC_SNPS_ID_MATCH) {
		dev_err(dwc->dev, "incompatible device %08x\n", dwc->id);
		ret = -ENODEV;
		goto err_unmap;
	}

	dwc->irq = irq_of_parse_and_map(np, 0);
	if (dwc->irq <= 0) {
		dev_err(dwc->dev, "no IRQ\n");
		ret = -ENXIO;
		goto err_unmap;
	}

	dev_info(dwc->dev, "HCD v2.%03x at MMIO %#lx (irq = %d)\n",
		dwc->id & ~DWC_SNPS_ID_MASK,
		(unsigned long)dwc->res.start, dwc->irq);

	dwc2xx_hcd_dump_regs(hcd);

	/* TODO: ensure the device won't trigger any interrupts */
	ret = usb_add_hcd(hcd, dwc->irq, 0);
	if (ret) {
		dev_err(dwc->dev, "failed to add HCD (%d)\n", ret);
		goto err_unmap;
	}

	platform_set_drvdata(pdev, hcd);
	return 0;

err_unmap:
	iounmap(hcd->regs);
err_release:
	release_region(dwc->res.start, resource_size(&dwc->res));
err:
	usb_put_hcd(hcd);
	return ret;
}

static int dwc2xx_hcd_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_region(dwc->res.start, resource_size(&dwc->res));
	usb_put_hcd(hcd);
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
