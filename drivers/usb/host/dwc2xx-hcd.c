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
#include <linux/spinlock.h>

#include <linux/usb.h>
#include <linux/usb/hcd.h>

#ifdef CONFIG_BCM_VC_POWER
# include "../../misc/bcm-vc-power.h"
#endif

#include "dwc2xx-hcd.h"

#define MODULE_NAME "dwc2xx-hcd"

static irqreturn_t dwc2xx_hcd_irq(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	unsigned long flags;
	int res = IRQ_NONE;
	u32 status;

	spin_lock_irqsave(&dwc->lock, flags);

	status = readl(hcd->regs + DWC_CORE_INT_STAT_REG);
	dev_dbg(dwc->dev, "%s: status = %08x\n", __func__, status);

	if (status & DWC_CURRENT_MODE_INT)
		dev_dbg(dwc->dev, "%s: DWC_CURRENT_MODE_INT\n", __func__);

	if (status & DWC_MODE_MISMATCH_INT)
		dev_dbg(dwc->dev, "%s: DWC_MODE_MISMATCH_INT\n", __func__);

	if (status & DWC_OTG_INT) {
		u32 otg_status;

		otg_status = readl(hcd->regs + DWC_OTG_INT_REG);
		dev_dbg(dwc->dev, "%s: DWC_OTG_INT otg_status = %08x\n", __func__, otg_status);

		writel(otg_status, hcd->regs + DWC_OTG_INT_REG);
	}

	if (status & DWC_SOF_INT)
		dev_dbg(dwc->dev, "%s: DWC_SOF_INT\n", __func__);

	if (status & DWC_RX_STAT_LEVEL_INT)
		dev_dbg(dwc->dev, "%s: DWC_RX_STAT_LEVEL_INT\n", __func__);

	if (status & DWC_NP_TX_FIFO_EMPTY_INT)
		dev_dbg(dwc->dev, "%s: DWC_NP_TX_FIFO_EMPTY_INT\n", __func__);

	if (status & DWC_GINNAKEFF_INT)
		dev_dbg(dwc->dev, "%s: DWC_GINNAKEFF_INT\n", __func__);

	if (status & DWC_GOUTNAKEFF_INT)
		dev_dbg(dwc->dev, "%s: DWC_GOUTNAKEFF_INT\n", __func__);

	if (status & DWC_ULPICK_INT)
		dev_dbg(dwc->dev, "%s: DWC_ULPICK_INT\n", __func__);

	if (status & DWC_I2C_INT)
		dev_dbg(dwc->dev, "%s: DWC_I2C_INT\n", __func__);

	if (status & DWC_EARLY_SUSPEND_INT)
		dev_dbg(dwc->dev, "%s: DWC_EARLY_SUSPEND_INT\n", __func__);

	if (status & DWC_USB_SUSPEND_INT)
		dev_dbg(dwc->dev, "%s: DWC_USB_SUSPEND_INT\n", __func__);

	if (status & DWC_USB_RESET_INT)
		dev_dbg(dwc->dev, "%s: DWC_USB_RESET_INT\n", __func__);

	if (status & DWC_ENUM_DONE_INT)
		dev_dbg(dwc->dev, "%s: DWC_ENUM_DONE_INT\n", __func__);

	if (status & DWC_ISO_OUT_DROP_INT)
		dev_dbg(dwc->dev, "%s: DWC_ISO_OUT_DROP_INT\n", __func__);

	if (status & DWC_EOP_FRAME_INT)
		dev_dbg(dwc->dev, "%s: DWC_EOP_FRAME_INT\n", __func__);

	if (status & DWC_RESTORE_DONE_INT)
		dev_dbg(dwc->dev, "%s: DWC_RESTORE_DONE_INT\n", __func__);

	if (status & DWC_EP_MISMATCH_INT)
		dev_dbg(dwc->dev, "%s: DWC_EP_MISMATCH_INT\n", __func__);

	if (status & DWC_IN_EP_INT)
		dev_dbg(dwc->dev, "%s: DWC_IN_EP_INT\n", __func__);

	if (status & DWC_OUT_EP_INT)
		dev_dbg(dwc->dev, "%s: DWC_OUT_EP_INT\n", __func__);

	if (status & DWC_INCOMPLETE_ISO_IN_INT)
		dev_dbg(dwc->dev, "%s: DWC_INCOMPLETE_ISO_IN_INT\n", __func__);

	if (status & DWC_INCOMPLETE_ISO_OUT_INT)
		dev_dbg(dwc->dev, "%s: DWC_INCOMPLETE_ISO_OUT_INT\n", __func__);

	if (status & DWC_FETSUSP_INT)
		dev_dbg(dwc->dev, "%s: DWC_FETSUSP_INT\n", __func__);

	if (status & DWC_RESET_DETECT_INT)
		dev_dbg(dwc->dev, "%s: DWC_RESET_DETECT_INT\n", __func__);

	if (status & DWC_PORT_INT)
		dev_dbg(dwc->dev, "%s: DWC_PORT_INT\n", __func__);

	if (status & DWC_HC_INT)
		dev_dbg(dwc->dev, "%s: DWC_HC_INT\n", __func__);

	if (status & DWC_HP_TX_FIFO_EMPTY_INT)
		dev_dbg(dwc->dev, "%s: DWC_HP_TX_FIFO_EMPTY_INT\n", __func__);

	if (status & DWC_LPM_TXN_RCVD_INT)
		dev_dbg(dwc->dev, "%s: DWC_LPM_TXN_RCVD_INT\n", __func__);

	if (status & DWC_CON_ID_STAT_CHG_INT)
		dev_dbg(dwc->dev, "%s: DWC_CON_ID_STAT_CHG_INT\n", __func__);

	if (status & DWC_DISCONNECT_INT)
		dev_dbg(dwc->dev, "%s: DWC_DISCONNECT_INT\n", __func__);

	if (status & DWC_SESS_REQ_INT)
		dev_dbg(dwc->dev, "%s: DWC_SESS_REQ_INT\n", __func__);

	if (status & DWC_WAKEUP_INT)
		dev_dbg(dwc->dev, "%s: DWC_WAKEUP_INT\n", __func__);

	writel(status, hcd->regs + DWC_CORE_INT_STAT_REG);
	res = IRQ_HANDLED;

	spin_unlock_irqrestore(&dwc->lock, flags);
	return res;
}

static int dwc2xx_hcd_do_soft_reset(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	int i;

	/* Perform a soft reset */
	writel(BIT(DWC_CORE_SOFT_RESET), hcd->regs + DWC_CORE_RESET_REG);

	/* Wait for it to complete */
	for (i = 0; i < DWC_SOFT_RESET_TIMEOUT; i++) {
		if (!(readl(hcd->regs + DWC_CORE_RESET_REG)
				& BIT(DWC_CORE_SOFT_RESET)))
			break;
		msleep(1);
	}
	dev_dbg(dwc->dev, "%s: soft reset in %d\n", __func__, i+1);
	if (i == DWC_SOFT_RESET_TIMEOUT) {
		dev_err(dwc->dev, "%s: soft reset did not complete", __func__);
		return -ETIMEDOUT;
	}

	/* Wait for AHB master idle state */
	for (i = 0; i < DWC_AHB_TIMEOUT; i++) {
		if (readl(hcd->regs + DWC_CORE_RESET_REG)
				& BIT(DWC_AHB_MASTER_IDLE))
			break;
		msleep(1);
	}
	dev_dbg(dwc->dev, "%s: master idle in %d\n", __func__, i+1);
	if (i == DWC_AHB_TIMEOUT) {
		dev_err(dwc->dev, "%s: not in AHB master idle state", __func__);
		return -ETIMEDOUT;
	}

	spin_lock_irq(&dwc->lock);
	dwc2xx_hcd_get_cfg(hcd);
	spin_unlock_irq	(&dwc->lock);
	return 0;
}

static int dwc2xx_hcd_reset(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	u32 usb_cfg;
	int ret;

#ifdef CONFIG_BCM_VC_POWER
	if (dwc->power) {
		ret = bcm_vc_power_on(dwc->power);
		if (ret) {
			dev_err(dwc->dev, "unable to power on (%d)\n", ret);
			return ret;
		}
	}
#endif

	ret = dwc2xx_hcd_do_soft_reset(hcd);
	if (ret)
		return ret;

	spin_lock_irq(&dwc->lock);

	/* Configure PHY */
	usb_cfg = dwc->__usb_cfg;
	if (dwc->hw_cfg2.hs_phy >= DWC_CFG2_HS_PHY_ULPI) {
		dwc->usb_cfg.ulpi_utmi_sel = true;
		dwc->usb_cfg.phyif = false;
		dwc->usb_cfg.ddrsel = false;
	} else {
		dwc->usb_cfg.ulpi_utmi_sel = false;
		dwc->usb_cfg.phyif = dwc->hw_cfg4.utmi_phy_data_width
			>= DWC_CFG4_PHY_WIDTH_16BIT;
	}

	/* Don't reset again if the PHY is already configured */
	if (dwc->__usb_cfg != usb_cfg) {
		dwc2xx_hcd_set_usb_cfg(hcd);
		spin_unlock_irq(&dwc->lock);

		ret = dwc2xx_hcd_do_soft_reset(hcd);
	} else {
		spin_unlock_irq(&dwc->lock);
	}
	return ret;
}

static int dwc2xx_hcd_start(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	struct usb_bus *bus = hcd_to_bus(hcd);
	u32 ints = 0;

	dev_dbg(dwc->dev, "%s\n", __func__);
	spin_lock_irq(&dwc->lock);

	/* Configure DMA */
	dwc->ahb_cfg.dma_enable = true;
	dwc->ahb_cfg.dma_single = false;
	dwc->ahb_cfg.dma_burst = DWC_AHB_DMA_BURST_INCR4;

	/* Configure ULPI FSLS */
	if (dwc->hw_cfg2.hs_phy == DWC_CFG2_HS_PHY_ULPI
			&& dwc->hw_cfg2.fs_phy == DWC_CFG2_FS_PHY_FSLS) {
		dwc->usb_cfg.ulpi_fsls = true;
		dwc->usb_cfg.ulpi_clk_sus_m = true;
	} else {
		dwc->usb_cfg.ulpi_fsls = false;
		dwc->usb_cfg.ulpi_clk_sus_m = false;
	}

	/* Configure hnp/srp capablitites */
	switch (dwc->hw_cfg2.op_mode) {
	case DWC_MODE_HNP_SRP_CAPABLE:
		dwc->usb_cfg.hnp_capable = true;
		dwc->usb_cfg.srp_capable = true;
		break;
	case DWC_MODE_SRP_ONLY_CAPABLE:
	case DWC_MODE_SRP_CAPABLE_DEVICE:
	case DWC_MODE_SRP_CAPABLE_HOST:
		dwc->usb_cfg.hnp_capable = false;
		dwc->usb_cfg.srp_capable = true;
		break;
	case DWC_MODE_NO_HNP_SRP_CAPABLE:
	case DWC_MODE_NO_SRP_CAPABLE_DEVICE:
	case DWC_MODE_NO_SRP_CAPABLE_HOST:
		dwc->usb_cfg.hnp_capable = false;
		dwc->usb_cfg.srp_capable = false;
		break;
	}
	dwc2xx_hcd_set_usb_cfg(hcd);

	/* Configure Low Power Mode */
	dwc->lpm_cfg.lpm_cap_en = dwc->hw_cfg3.otg_lpm_en;
	if (dwc->hw_cfg3.otg_lpm_en) {
		dwc->lpm_cfg.appl_resp = true;
		dwc->lpm_cfg.retry_count = 3;
	}

	/* Configure Inter-Chip USB */
	dwc->usb_cfg.ic_usb_capable = dwc->hw_cfg2.otg_enable_ic_usb;

	/* Apply configuration */
	dwc2xx_hcd_set_ahb_cfg(hcd);
	dwc2xx_hcd_set_usb_cfg(hcd);
	dwc2xx_hcd_set_lpm_cfg(hcd);

	/* Clear all pending interrupts */
	writel(~0, hcd->regs + DWC_OTG_INT_REG);
	writel(~0, hcd->regs + DWC_CORE_INT_STAT_REG);

	/* Unmask common interrupts */
	ints |= DWC_MODE_MISMATCH_INT;
	ints |= DWC_OTG_INT;
	if (dwc->ahb_cfg.dma_enable)
		ints |= DWC_RX_STAT_LEVEL_INT;
	ints |= DWC_CON_ID_STAT_CHG_INT;
	ints |= DWC_WAKEUP_INT;
	ints |= DWC_DISCONNECT_INT;
	ints |= DWC_USB_SUSPEND_INT;
	ints |= DWC_SESS_REQ_INT;
	if (dwc->lpm_cfg.lpm_cap_en)
		ints |= DWC_LPM_TXN_RCVD_INT;

	/* Unmask host interrupts */
	ints |= DWC_PORT_INT;
	ints |= DWC_HC_INT;
	writel(ints, hcd->regs + DWC_CORE_INT_MASK_REG);

	/* Enable interrupts */
	dwc->ahb_cfg.int_enable = true;
	dwc2xx_hcd_set_ahb_cfg(hcd);

	spin_unlock_irq(&dwc->lock);

	if (bus->root_hub)
		usb_hcd_resume_root_hub(hcd);
	return 0;
}

static void dwc2xx_hcd_stop(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);

	spin_lock_irq(&dwc->lock);

	/* Disable interrupts */
	dwc->ahb_cfg.int_enable = false;
	dwc2xx_hcd_set_ahb_cfg(hcd);

	/* Mask all interrupts */
	writel(0, hcd->regs + DWC_CORE_INT_MASK_REG);

	spin_unlock_irq(&dwc->lock);

	/* Turn the host port off */
	dwc2xx_hcd_get_hprt(hcd);
	dwc->hprt.power = false;
	dwc2xx_hcd_set_hprt(hcd);
}

static void dwc2xx_hcd_shutdown(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);

#ifdef CONFIG_BCM_VC_POWER
	/* Turn our power off */
	if (dwc->power)
		bcm_vc_power_off(dwc->power);
#endif
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

	BUILD_BUG_ON(sizeof(dwc->__ahb_cfg) != sizeof(dwc->ahb_cfg));
	BUILD_BUG_ON(sizeof(dwc->__usb_cfg) != sizeof(dwc->usb_cfg));
	BUILD_BUG_ON(sizeof(dwc->__hw_cfg1) != sizeof(dwc->hw_cfg1));
	BUILD_BUG_ON(sizeof(dwc->__hw_cfg2) != sizeof(dwc->hw_cfg2));
	BUILD_BUG_ON(sizeof(dwc->__hw_cfg3) != sizeof(dwc->hw_cfg3));
	BUILD_BUG_ON(sizeof(dwc->__hw_cfg4) != sizeof(dwc->hw_cfg4));
	BUILD_BUG_ON(sizeof(dwc->__lpm_cfg) != sizeof(dwc->lpm_cfg));
	BUILD_BUG_ON(sizeof(dwc->__hprt) != sizeof(dwc->hprt));

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

	hcd->rsrc_start = dwc->res.start;
	hcd->rsrc_len = resource_size(&dwc->res);
	hcd->regs = ioremap(dwc->res.start, resource_size(&dwc->res));
	if (!hcd->regs) {
		dev_err(dwc->dev, "error mapping io at %#lx\n",
			(unsigned long)dwc->res.start);
		ret = -EIO;
		goto err_release;
	}

	dwc->snps_id = readl(hcd->regs + DWC_SNPS_ID_REG);
	if ((dwc->snps_id & DWC_SNPS_ID_MASK) != DWC_SNPS_ID_MATCH) {
		dev_err(dwc->dev, "incompatible device %08x\n", dwc->snps_id);
		ret = -ENODEV;
		goto err_unmap;
	}
	dwc->user_id = readl(hcd->regs + DWC_USER_ID_REG);
	dwc2xx_hcd_get_cfg(hcd);

	if (dwc->hw_cfg2.arch != DWC_INT_DMA_ARCH) {
		dev_err(dwc->dev, "architecture %d not supported\n",
			dwc->hw_cfg2.arch);
		ret = -ENODEV;
		goto err_unmap;
	}

	if (dwc->hw_cfg2.hs_phy == DWC_CFG2_HS_PHY_NOT_SUPPORTED) {
		dev_err(dwc->dev, "hs phy %d not supported\n",
			dwc->hw_cfg2.hs_phy);
		ret = -ENODEV;
		goto err_unmap;
	}

	dwc->irq = irq_of_parse_and_map(np, 0);
	if (dwc->irq <= 0) {
		dev_err(dwc->dev, "no IRQ\n");
		ret = -ENXIO;
		goto err_unmap;
	}

#ifdef CONFIG_BCM_VC_POWER
	/* We're going to need power now */
	if (bcm_vc_power_is_user(np)) {
		dwc->power = bcm_vc_power_get(np, NULL, NULL);
		if (IS_ERR(dwc->power)) {
			ret = PTR_ERR(dwc->power);
			dev_err(dwc->dev, "no power manager (%d)\n", ret);
			goto err_unmap;
		} else {
			ret = bcm_vc_power_on(dwc->power);
			if (ret) {
				dev_err(dwc->dev,
					"unable to power on (%d)\n", ret);
				goto err_power_off;
			}
		}
	} else {
		dwc->power = NULL;
	}
#endif

	dev_info(dwc->dev, "HCD v2.%03x (%08x) at MMIO %#lx (irq = %d)\n",
		dwc->snps_id & ~DWC_SNPS_ID_MASK, dwc->user_id,
		(unsigned long)dwc->res.start, dwc->irq);

	dwc2xx_hcd_debug(hcd);

	/* Ensure the device won't trigger any interrupts */
	dwc->ahb_cfg.int_enable = false;
	dwc2xx_hcd_set_ahb_cfg(hcd);
	writel(0, hcd->regs + DWC_CORE_INT_MASK_REG);

	spin_lock_init(&dwc->lock);

	ret = usb_add_hcd(hcd, dwc->irq, 0);
	if (ret) {
		dev_err(dwc->dev, "failed to add HCD (%d)\n", ret);
		goto err_power_off;
	}

	platform_set_drvdata(pdev, hcd);
	return 0;

err_power_off:
#ifdef CONFIG_BCM_VC_POWER
	if (dwc->power)
		bcm_vc_power_put(dwc->power);
#endif
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
#ifdef CONFIG_BCM_VC_POWER
	if (dwc->power)
		bcm_vc_power_put(dwc->power);
#endif
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
