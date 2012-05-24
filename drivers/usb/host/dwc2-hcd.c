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
 * Derived from the Synopsys, Inc. driver (see dwc2-hcd.h for licence notice)
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

#include "dwc2-hcd.h"

#define MODULE_NAME "dwc2-hcd"

/* TODO simplify this */
static u16 dwc_calc_frame_interval(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);
	u16 clock = 60;

	if (dwc->usb_cfg.physel) {
		if (dwc->hw_cfg2.fs_phy == DWC_CFG2_FS_PHY_3)
			clock = 48;
	} else if (dwc->usb_cfg.ulpi_utmi_sel && !dwc->usb_cfg.phyif) {
		clock = 60;
	}

	if (dwc->usb_cfg.phy_lpm_clk_sel) {
		if (!dwc->usb_cfg.ulpi_utmi_sel && dwc->usb_cfg.phyif)
			clock = dwc->usb_cfg.physel ? 30 : 48;
	} else {
		if (!dwc->usb_cfg.ulpi_utmi_sel && !dwc->usb_cfg.phyif
				&& !dwc->usb_cfg.physel)
			clock = 60;
	}

	if (dwc->usb_cfg.physel) {
		if (dwc->hw_cfg2.fs_phy == DWC_CFG2_FS_PHY_FSLS
				|| (dwc->hw_cfg2.fs_phy == DWC_CFG2_FS_PHY_2
					&& !dwc->usb_cfg.phyif))
			clock = 48;
	}

	if (dwc->hprt.speed == DWC_HPRT_SPEED_HIGH)
		return 125 * clock;
	else
		return 1000 * clock;
}

static irqreturn_t dwc2_hcd_host_chan_irq(struct usb_hcd *hcd, int chan)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);
	u32 status = readl(hcd->regs + DWC_HOST_CHAN_INT_STAT_REG(chan));
	u32 mask = readl(hcd->regs + DWC_HOST_CHAN_INT_MASK_REG(chan));
	u32 handled = 0;

	status &= mask;
	dev_dbg(dwc->dev, "%s: status = %08x\n", __func__, status);

	if (status & DWC_CHAN_XFER_COMP_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_XFER_COMP_INT\n", __func__);

	if (status & DWC_CHAN_HALT_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_HALT_INT\n", __func__);

	if (status & DWC_CHAN_AHB_ERR_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_AHB_ERR_INT\n", __func__);

	if (status & DWC_CHAN_STALL_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_STALL_INT\n", __func__);

	if (status & DWC_CHAN_NAK_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_NAK_INT\n", __func__);

	if (status & DWC_CHAN_ACK_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_ACK_INT\n", __func__);

	if (status & DWC_CHAN_NYET_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_NYET_INT\n", __func__);

	if (status & DWC_CHAN_XACT_ERR_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_XACT_ERR_INT\n", __func__);

	if (status & DWC_CHAN_BBL_ERR_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_BBL_ERR_INT\n", __func__);

	if (status & DWC_CHAN_FR_OVERRUN_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_FR_OVERRUN_INT\n", __func__);

	if (status & DWC_CHAN_DTGL_ERR_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_DTGL_ERR_INT\n", __func__);

	if (status & DWC_CHAN_BNA_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_BNA_INT\n", __func__);

	if (status & DWC_CHAN_XCES_ERR_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_XCES_ERR_INT\n", __func__);

	if (status & DWC_CHAN_FR_LIST_ROLL_INT)
		dev_warn(dwc->dev, "%s: DWC_CHAN_FR_LIST_ROLL_INT\n", __func__);

	writel(handled, hcd->regs + DWC_HOST_CHAN_INT_STAT_REG(chan));
	if (status & handled) {
		dev_warn(dwc->dev, "%s: IRQ_NONE %08x\n", __func__,
			status & handled);
		return IRQ_NONE;
	} else {
		dev_dbg(dwc->dev, "%s: IRQ_HANDLED %08x\n", __func__, status);
		return IRQ_HANDLED;
	}
}

static irqreturn_t dwc2_hcd_irq(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);
	u32 status = readl(hcd->regs + DWC_CORE_INT_STAT_REG);
	u32 mask = readl(hcd->regs + DWC_CORE_INT_MASK_REG);
	u32 handled = 0;

	status &= mask;
	dev_dbg(dwc->dev, "%s: status = %08x\n", __func__, status);

	if (status & DWC_MODE_MISMATCH_INT) {
		dev_emerg(dwc->dev, "%s: OTG USB mode mismatch\n", __func__);
		/* We could turn off the power? */
		handled |= DWC_MODE_MISMATCH_INT;
	}

	if (status & DWC_DMA_SOF_INT)
		dev_warn(dwc->dev, "%s: DWC_DMA_SOF_INT\n", __func__);

	if (status & DWC_RX_STAT_LEVEL_INT) {
		dev_dbg(dwc->dev, "%s: DWC_RX_STAT_LEVEL_INT\n", __func__);
		/* TODO */
		handled |= DWC_RX_STAT_LEVEL_INT;
	}

	if (status & DWC_NP_TX_FIFO_EMPTY_INT) {
		dev_dbg(dwc->dev, "%s: DWC_NP_TX_FIFO_EMPTY_INT\n", __func__);
		/* TODO */
		handled |= DWC_NP_TX_FIFO_EMPTY_INT;
	}

	if (status & DWC_GINNAKEFF_INT)
		dev_warn(dwc->dev, "%s: DWC_GINNAKEFF_INT\n", __func__);

	if (status & DWC_GOUTNAKEFF_INT)
		dev_warn(dwc->dev, "%s: DWC_GOUTNAKEFF_INT\n", __func__);

	if (status & DWC_ULPICK_INT)
		dev_warn(dwc->dev, "%s: DWC_ULPICK_INT\n", __func__);

	if (status & DWC_I2C_INT)
		dev_warn(dwc->dev, "%s: DWC_I2C_INT\n", __func__);

	if (status & DWC_EARLY_SUSPEND_INT)
		dev_warn(dwc->dev, "%s: DWC_EARLY_SUSPEND_INT\n", __func__);

	if (status & DWC_USB_RESET_INT)
		dev_warn(dwc->dev, "%s: DWC_USB_RESET_INT\n", __func__);

	if (status & DWC_ENUM_DONE_INT)
		dev_warn(dwc->dev, "%s: DWC_ENUM_DONE_INT\n", __func__);

	if (status & DWC_ISO_OUT_DROP_INT)
		dev_warn(dwc->dev, "%s: DWC_ISO_OUT_DROP_INT\n", __func__);

	if (status & DWC_EOP_FRAME_INT)
		dev_warn(dwc->dev, "%s: DWC_EOP_FRAME_INT\n", __func__);

	if (status & DWC_RESTORE_DONE_INT)
		dev_warn(dwc->dev, "%s: DWC_RESTORE_DONE_INT\n", __func__);

	if (status & DWC_EP_MISMATCH_INT)
		dev_warn(dwc->dev, "%s: DWC_EP_MISMATCH_INT\n", __func__);

	if (status & DWC_IN_EP_INT)
		dev_warn(dwc->dev, "%s: DWC_IN_EP_INT\n", __func__);

	if (status & DWC_OUT_EP_INT)
		dev_warn(dwc->dev, "%s: DWC_OUT_EP_INT\n", __func__);

	if (status & DWC_INCOMPLETE_ISO_IN_INT)
		dev_warn(dwc->dev, "%s: DWC_INCOMPLETE_ISO_IN_INT\n", __func__);

	if (status & DWC_INCOMPLETE_ISO_OUT_INT)
		dev_warn(dwc->dev, "%s: DWC_INCOMPLETE_ISO_OUT_INT\n", __func__);

	if (status & DWC_FETSUSP_INT)
		dev_warn(dwc->dev, "%s: DWC_FETSUSP_INT\n", __func__);

	if (status & DWC_RESET_DETECT_INT)
		dev_warn(dwc->dev, "%s: DWC_RESET_DETECT_INT\n", __func__);

	if (status & DWC_HOST_CHAN_INT) {
		bool ok = true;
		int i;

		dev_dbg(dwc->dev, "%s: DWC_HOST_CHAN_INT\n", __func__);

		for (i = 0; i < DWC_HOST_CHAN_COUNT; i++)
			ok &= dwc2_hcd_host_chan_irq(hcd, i) != IRQ_NONE;

		if (ok)
			handled |= DWC_HOST_CHAN_INT;
	}

	if (status & DWC_HP_TX_FIFO_EMPTY_INT) {
		dev_dbg(dwc->dev, "%s: DWC_HP_TX_FIFO_EMPTY_INT\n", __func__);
		/* TODO */
		handled |= DWC_HP_TX_FIFO_EMPTY_INT;
	}

	writel(handled, hcd->regs + DWC_CORE_INT_STAT_REG);
	if (status & handled) {
		dev_warn(dwc->dev, "%s: IRQ_NONE %08x\n", __func__,
			status & handled);
		return IRQ_NONE;
	} else {
		dev_dbg(dwc->dev, "%s: IRQ_HANDLED %08x\n", __func__, status);
		return IRQ_HANDLED;
	}
}

static int dwc2_hcd_do_soft_reset(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);
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
	dwc2_hcd_get_cfg(hcd);
	spin_unlock_irq	(&dwc->lock);
	return 0;
}

static int dwc2_hcd_do_flush_fifos(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);
	int i;

	/* Perform TX fifo flush */
	writel(BIT(DWC_TX_FIFO_FLUSH) | (16 << DWC_TX_FIFO_FLUSH_SHIFT),
		hcd->regs + DWC_CORE_RESET_REG);

	/* Wait for it to complete */
	for (i = 0; i < DWC_FIFO_FLUSH_TIMEOUT; i++) {
		if (!(readl(hcd->regs + DWC_CORE_RESET_REG)
				& BIT(DWC_TX_FIFO_FLUSH)))
			break;
		msleep(1);
	}
	dev_dbg(dwc->dev, "%s: tx fifo flush in %d\n", __func__, i+1);
	if (i == DWC_SOFT_RESET_TIMEOUT) {
		dev_err(dwc->dev, "%s: tx fifo flush did not complete", __func__);
		return -ETIMEDOUT;
	}

	/* Perform RX fifo flush */
	writel(BIT(DWC_RX_FIFO_FLUSH), hcd->regs + DWC_CORE_RESET_REG);

	/* Wait for it to complete */
	for (i = 0; i < DWC_FIFO_FLUSH_TIMEOUT; i++) {
		if (!(readl(hcd->regs + DWC_CORE_RESET_REG)
				& BIT(DWC_RX_FIFO_FLUSH)))
			break;
		msleep(1);
	}
	dev_dbg(dwc->dev, "%s: rx fifo flush in %d\n", __func__, i+1);
	if (i == DWC_SOFT_RESET_TIMEOUT) {
		dev_err(dwc->dev, "%s: rx fifo flush did not complete", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

static int dwc2_hcd_reset(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);
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

	ret = dwc2_hcd_do_soft_reset(hcd);
	if (ret)
		return ret;

	spin_lock_irq(&dwc->lock);

	/* Configure PHY */
	dwc2_hcd_get_usb_cfg(hcd);
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
		dwc2_hcd_set_usb_cfg(hcd);
		spin_unlock_irq(&dwc->lock);

		ret = dwc2_hcd_do_soft_reset(hcd);
		spin_lock_irq(&dwc->lock);
	}

	/* Turn the host port off */
	dwc2_hcd_get_hprt(hcd);
	dwc->hprt.power = false;
	dwc2_hcd_set_hprt(hcd);

	spin_unlock_irq(&dwc->lock);
	return ret;
}

static int dwc2_hcd_start(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);
	struct usb_bus *bus = hcd_to_bus(hcd);
	bool sg_dma;
	u32 ints = 0;
	int i;

	dev_dbg(dwc->dev, "%s\n", __func__);
	spin_lock_irq(&dwc->lock);

	/* Restart the PHY Clock */
	writel(0, hcd->regs + DWC_OTG_PWR_CLK_CTL_REG);

	/* Host configuration */
	dwc2_hcd_get_ahb_cfg(hcd);
	dwc2_hcd_get_usb_cfg(hcd);
	dwc2_hcd_get_lpm_cfg(hcd);

	dwc2_hcd_get_host_cfg(hcd);
	dwc->host_cfg.fsls_pclk = DWC_HOST_PCLK_30_60_MHZ;
	dwc2_hcd_set_host_cfg(hcd);

	dwc2_hcd_get_hfir_cfg(hcd);
	dwc->hfir_cfg.frame_interval = dwc_calc_frame_interval(hcd);
	dwc->hfir_cfg.dyn_frame_reload = false;
	dwc2_hcd_set_hfir_cfg(hcd);

	/* Configure DMA */
	dwc->host_cfg.sg_dma = dwc->hw_cfg4.desc_dma
		&& (dwc->snps_id & ~DWC_SNPS_ID_MASK) >= 0x90a;
	dwc->ahb_cfg.dma_enable = true;
	dwc->ahb_cfg.dma_single = false;
	dwc->ahb_cfg.dma_burst = DWC_AHB_DMA_BURST_INCR4;

	/* Configure FIFOs */
	BUILD_BUG_ON(DWC_RX_FIFO_SZ > 32678);
	writel(DWC_RX_FIFO_SZ, hcd->regs + DWC_RX_FIFO_SZ_REG);

	BUILD_BUG_ON(DWC_NP_TX_FIFO_SZ > 32678);
	BUILD_BUG_ON(DWC_HP_TX_FIFO_SZ > 32678);
	BUILD_BUG_ON(DWC_HP_TX_FIFO_SZ > 32678);
	BUILD_BUG_ON(
		(DWC_RX_FIFO_SZ + DWC_NP_TX_FIFO_SZ + DWC_HP_TX_FIFO_SZ)
		>= 65536);

	writel(DWC_NP_TX_FIFO_SZ | (DWC_RX_FIFO_SZ << 16),
		hcd->regs + DWC_RX_FIFO_SZ_REG);

	writel(DWC_HP_TX_FIFO_SZ | ((DWC_RX_FIFO_SZ + DWC_NP_TX_FIFO_SZ) << 16),
		hcd->regs + DWC_RX_FIFO_SZ_REG);

	writel(DWC_HP_TX_FIFO_SZ | ((DWC_RX_FIFO_SZ + DWC_NP_TX_FIFO_SZ) << 16),
		hcd->regs + DWC_RX_FIFO_SZ_REG);

	writel(DWC_RX_FIFO_SZ + DWC_NP_TX_FIFO_SZ + DWC_HP_TX_FIFO_SZ,
		hcd->regs + DWC_RX_FIFO_SZ_REG);

	/* Configure ULPI FSLS */
	if (dwc->hw_cfg2.hs_phy == DWC_CFG2_HS_PHY_ULPI
			&& dwc->hw_cfg2.fs_phy == DWC_CFG2_FS_PHY_FSLS) {
		dwc->usb_cfg.ulpi_fsls = true;
		dwc->usb_cfg.ulpi_clk_sus_m = true;
	} else {
		dwc->usb_cfg.ulpi_fsls = false;
		dwc->usb_cfg.ulpi_clk_sus_m = false;
	}

	/* Configure HNP/SRP capablitites */
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

	/* Low power mode not supported */
	dwc->lpm_cfg.lpm_cap_en = false;
	dwc->usb_cfg.phy_lpm_clk_sel = false;

	/* Configure Inter-Chip USB */
	dwc->usb_cfg.ic_usb_capable = dwc->hw_cfg2.otg_enable_ic_usb;

	/* Apply configuration */
	dwc2_hcd_set_ahb_cfg(hcd);
	dwc2_hcd_set_usb_cfg(hcd);
	dwc2_hcd_set_lpm_cfg(hcd);
	sg_dma = dwc->host_cfg.sg_dma;

	/* Unmask common interrupts */
	if (dwc->ahb_cfg.dma_enable)
		ints |= DWC_RX_STAT_LEVEL_INT;

	/* Unmask the OTG interrupts */
	ints |= DWC_MODE_MISMATCH_INT;

	/* Don't care about debounce done interrupt */
	/* ints |= DWC_OTG_INT; */

	/* This is a host, it won't change status */
	/* ints |= DWC_CON_ID_STAT_CHG_INT; */

	/* This is a host, we're always awake */
	/* ints |= DWC_WAKEUP_INT; */

	/*
	 * This is a host, so ignore disconnections
	 * (and assume the port status will change)
	 */
	/* ints |= DWC_DISCONNECT_INT; */

	/* This is a host, so it can't be suspended by a host */
	/* ints |= DWC_USB_SUSPEND_INT; */

	/*
	 * Ignore port session request as power will be enabled when
	 * the root hub receives a request to enable power on the port
	 */
	/* ints |= DWC_SESS_REQ_INT; */

	/* This is a host, it won't receive LPM transactions */
	/* if (dwc->lpm_cfg.lpm_cap_en)
		ints |= DWC_LPM_TXN_RCVD_INT; */

	/* Unmask host interrupts */
	/* ints |= DWC_PORT_INT; */
	ints |= DWC_HOST_CHAN_INT;
	spin_unlock_irq(&dwc->lock);

	/* Flush FIFOs */
	dwc2_hcd_do_flush_fifos(hcd);

	if (!sg_dma) {
		struct dwc2_hcd_cchar hcchar;
		int j;

		/* Flush queued requests */
		for (i = 0; i < DWC_HOST_CHAN_COUNT; i++) {
			hcchar = dwc2_hcd_get_cchar(hcd, i);
			hcchar.enable = false;
			hcchar.disable = true;
			hcchar.outep = false;
			dwc2_hcd_set_cchar(hcd, i, hcchar);
		}

		/* Halt all channels */
		for (i = 0; i < DWC_HOST_CHAN_COUNT; i++) {
			hcchar = dwc2_hcd_get_cchar(hcd, i);
			hcchar.enable = true;
			hcchar.disable = true;
			hcchar.outep = false;
			dwc2_hcd_set_cchar(hcd, i, hcchar);

			/* Wait for it to complete */
			for (j = 0; j < DWC_CHAN_HALT_TIMEOUT; j++) {
				hcchar = dwc2_hcd_get_cchar(hcd, i);
				if (!hcchar.enable)
					break;
				msleep(1);
			}
			dev_dbg(dwc->dev, "%s: chan %d halt in %d\n", __func__,
				i, j+1);
			if (j == DWC_CHAN_HALT_TIMEOUT) {
				dev_err(dwc->dev,
					"%s: chan %d halt did not complete",
					__func__, i);
				return -ETIMEDOUT;
			}
		}
	}

	/* Clear all pending interrupts */
	writel(~0, hcd->regs + DWC_OTG_INT_REG);
	writel(~0, hcd->regs + DWC_CORE_INT_STAT_REG);

	/* Apply interrupt configuration */
	writel(ints, hcd->regs + DWC_CORE_INT_MASK_REG);

	/* TODO Unmask all host channel interrutps */
	for (i = 0; i < DWC_HOST_CHAN_COUNT; i++)
		writel(0, hcd->regs + DWC_HOST_CHAN_INT_MASK_REG(i));

	/* Enable interrupts */
	spin_lock_irq(&dwc->lock);
	dwc2_hcd_get_ahb_cfg(hcd);
	dwc->ahb_cfg.int_enable = true;
	dwc2_hcd_set_ahb_cfg(hcd);
	spin_unlock_irq(&dwc->lock);

	if (bus->root_hub)
		usb_hcd_resume_root_hub(hcd);
	return 0;
}

static void dwc2_hcd_stop(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);

	spin_lock_irq(&dwc->lock);

	/* Disable interrupts */
	dwc->ahb_cfg.int_enable = false;
	dwc2_hcd_set_ahb_cfg(hcd);

	/* Mask all interrupts */
	writel(0, hcd->regs + DWC_CORE_INT_MASK_REG);

	spin_unlock_irq(&dwc->lock);

	/* Turn the host port off */
	dwc2_hcd_get_hprt(hcd);
	dwc->hprt.power = false;
	dwc2_hcd_set_hprt(hcd);
}

static void dwc2_hcd_shutdown(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);

#ifdef CONFIG_BCM_VC_POWER
	/* Turn our power off */
	if (dwc->power)
		bcm_vc_power_off(dwc->power);
#endif
}

static int dwc2_hcd_get_frame_number(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static int dwc2_hcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
	gfp_t mem_flags)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static int dwc2_hcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb,
	int status)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static void dwc2_hcd_endpoint_disable(struct usb_hcd *hcd,
	struct usb_host_endpoint *ep)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
}

static int dwc2_hcd_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	*buf = 0;
	spin_lock_irq(&dwc->lock);
	dwc2_hcd_get_hprt(hcd);
	if (dwc->hprt.connect_chg || dwc->hprt.enable_chg || dwc->hprt.reset
			|| dwc->hprt.overcurrent_chg)
		*buf |= BIT(1);
	spin_unlock_irq(&dwc->lock);
	return !!*buf;
}

static int dwc2_hcd_hub_control(struct usb_hcd *hcd,
	u16 typeReq, u16 wValue, u16 wIndex, char *buf, u16 wLength)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);
	int port = wIndex - 1;

	switch (typeReq) {
	case GetHubDescriptor: {
		struct usb_hub_descriptor *d = (struct usb_hub_descriptor *)buf;
		dev_dbg(dwc->dev, "GetHubDescriptor\n");
		d->bDescLength = 9;
		d->bDescriptorType = USB_DT_HUB;
		d->bNbrPorts = 1;
		d->wHubCharacteristics = HUB_CHAR_INDV_PORT_LPSM
			| HUB_CHAR_INDV_PORT_OCPM;
		d->bPwrOn2PwrGood = 1;
		d->bHubContrCurrent = 0;
		d->u.hs.DeviceRemovable[0] = 0;
		d->u.hs.PortPwrCtrlMask[0] = ~0;
		return d->bDescLength;
	}

	case GetHubStatus: {
		dev_dbg(dwc->dev, "GetHubStatus\n");
		*(__le32 *)buf = cpu_to_le32(0); /* OK */
		return 4;
	}

	case SetPortFeature: {
		int ret;

		if (port != 1)
			break;

		switch (wValue) {
		case USB_PORT_FEAT_POWER:
			dev_dbg(dwc->dev, "SetPortFeature USB_PORT_FEAT_POWER\n");
			spin_lock_irq(&dwc->lock);
			dwc2_hcd_get_hprt(hcd);
			dwc->hprt.power = true;
			dwc2_hcd_set_hprt(hcd);
			ret = dwc->hprt.power ? 0 : -EPIPE;
			spin_unlock_irq(&dwc->lock);
			return ret;

		case USB_PORT_FEAT_RESET:
			dev_dbg(dwc->dev, "SetPortFeature USB_PORT_FEAT_RESET\n");
			spin_lock_irq(&dwc->lock);
			dwc2_hcd_get_hprt(hcd);
			dwc->hprt.reset = true;
			dwc2_hcd_set_hprt(hcd);
			spin_unlock_irq(&dwc->lock);
			return 0;
		}
		break;
	}

	case ClearPortFeature: {
		int ret;

		if (port != 1)
			break;

		switch (wValue) {
		case USB_PORT_FEAT_POWER:
			dev_dbg(dwc->dev, "ClearPortFeature USB_PORT_FEAT_POWER\n");
			spin_lock_irq(&dwc->lock);
			dwc2_hcd_get_hprt(hcd);
			dwc->hprt.power = false;
			dwc2_hcd_set_hprt(hcd);
			ret = !dwc->hprt.power ? 0 : -EPIPE;
			spin_unlock_irq(&dwc->lock);
			return ret;

		case USB_PORT_FEAT_ENABLE:
			dev_dbg(dwc->dev, "ClearPortFeature USB_PORT_FEAT_ENABLE\n");
			spin_lock_irq(&dwc->lock);
			dwc2_hcd_get_hprt(hcd);
			dwc->hprt.enable = false;
			dwc2_hcd_set_hprt(hcd);
			spin_unlock_irq(&dwc->lock);
			return 0;

		case USB_PORT_FEAT_C_CONNECTION:
			dev_dbg(dwc->dev, "ClearPortFeature USB_PORT_FEAT_C_CONNECTION\n");
			dwc2_hcd_ack_hprt(hcd, wValue);
			return 0;

		case USB_PORT_FEAT_C_ENABLE:
			dev_dbg(dwc->dev, "ClearPortFeature USB_PORT_FEAT_C_ENABLE\n");
			dwc2_hcd_ack_hprt(hcd, wValue);
			return 0;

		case USB_PORT_FEAT_C_OVER_CURRENT:
			dev_dbg(dwc->dev, "ClearPortFeature USB_PORT_FEAT_C_OVER_CURRENT\n");
			dwc2_hcd_ack_hprt(hcd, wValue);
			return 0;

		case USB_PORT_FEAT_C_RESET:
			dev_dbg(dwc->dev, "ClearPortFeature USB_PORT_FEAT_C_RESET\n");
			spin_lock_irq(&dwc->lock);
			dwc2_hcd_get_hprt(hcd);
			dwc->hprt.reset = false;
			dwc2_hcd_set_hprt(hcd);
			spin_unlock_irq(&dwc->lock);
			return 0;
		}
		break;
	}

	case GetPortStatus: {
		u32 status = 0;

		if (port != 1)
			break;

		dev_dbg(dwc->dev, "GetPortStatus:\n");
		spin_lock_irq(&dwc->lock);
		dwc2_hcd_get_hprt(hcd);

		if (dwc->hprt.reset) {
			dwc->hprt.reset = false;
			dwc2_hcd_set_hprt(hcd);

			dwc2_hcd_get_hprt(hcd);
			if (dwc->hprt.reset)
				dev_dbg(dwc->dev, "USB_PORT_STAT_RESET\n");
		}

		if (dwc->hprt.overcurrent_chg) {
			dev_dbg(dwc->dev, "USB_PORT_STAT_C_OVERCURRENT\n");
			status |= USB_PORT_STAT_C_OVERCURRENT << 16;
		}

		if (dwc->hprt.overcurrent) {
			dev_dbg(dwc->dev, "USB_PORT_STAT_OVERCURRENT\n");
			status |= USB_PORT_STAT_OVERCURRENT;
		}

		if (dwc->hprt.connect_chg) {
			dev_dbg(dwc->dev, "USB_PORT_STAT_C_CONNECTION\n");
			status |= USB_PORT_STAT_C_CONNECTION << 16;
		}

		if (dwc->hprt.connect) {
			dev_dbg(dwc->dev, "USB_PORT_STAT_CONNECTION\n");
			status |= USB_PORT_STAT_CONNECTION;
		}

		if (dwc->hprt.enable_chg) {
			dev_dbg(dwc->dev, "USB_PORT_STAT_C_ENABLE\n");
			status |= USB_PORT_STAT_C_ENABLE << 16;
		}

		if (dwc->hprt.enable) {
			dev_dbg(dwc->dev, "USB_PORT_STAT_ENABLE\n");
			status |= USB_PORT_STAT_ENABLE;
		}

		if (dwc->hprt.power) {
			dev_dbg(dwc->dev, "USB_PORT_STAT_POWER\n");
			status |= USB_PORT_STAT_POWER;
		}

		switch (dwc->hprt.speed) {
		case DWC_HPRT_SPEED_HIGH:
			dev_dbg(dwc->dev, "USB_PORT_STAT_HIGH_SPEED\n");
			status |= USB_PORT_STAT_HIGH_SPEED;
			break;

		case DWC_HPRT_SPEED_LOW:
			dev_dbg(dwc->dev, "USB_PORT_STAT_LOW_SPEED\n");
			status |= USB_PORT_STAT_LOW_SPEED;
			break;
		}

		if (dwc->hprt.test_ctl) {
			dev_dbg(dwc->dev, "USB_PORT_STAT_TEST\n");
			status |= USB_PORT_STAT_TEST;
		}

		if (!status)
			dev_dbg(dwc->dev, "NONE\n");

		dwc2_hcd_set_hprt(hcd);
		spin_unlock_irq(&dwc->lock);

		*(__le32 *)buf = cpu_to_le32(status);
		return 0;
	}
	}

	dev_warn(dwc->dev,
		"%s typeReq=%04x wValue=%04x wIndex=%04x wLength=%04x\n",
		__func__, typeReq, wValue, wIndex, wLength);

	return -EPIPE;
}

static int dwc2_hcd_bus_suspend(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static int dwc2_hcd_bus_resume(struct usb_hcd *hcd)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return 0;
}

static int dwc2_hcd_start_port_reset(struct usb_hcd *hcd, unsigned port_num)
{
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "%s\n", __func__);
	return -ENOSYS;
}

static const struct hc_driver dwc2_hcd_hc_driver = {
	.description		= MODULE_NAME,
	.product_desc		= "Synopsys DesignWare USB 2.0 Host Controller",
	.hcd_priv_size		= sizeof(struct dwc2_hcd),
	
	.irq			= dwc2_hcd_irq,
	.flags			= HCD_MEMORY | HCD_USB2,

	.reset			= dwc2_hcd_reset,
	.start			= dwc2_hcd_start,
	.stop			= dwc2_hcd_stop,
	.shutdown		= dwc2_hcd_shutdown,

	.urb_enqueue		= dwc2_hcd_urb_enqueue,
	.urb_dequeue		= dwc2_hcd_urb_dequeue,
	.endpoint_disable	= dwc2_hcd_endpoint_disable,

	.get_frame_number	= dwc2_hcd_get_frame_number,

	.hub_status_data	= dwc2_hcd_hub_status_data,
	.hub_control		= dwc2_hcd_hub_control,

	.bus_suspend		= dwc2_hcd_bus_suspend,
	.bus_resume		= dwc2_hcd_bus_resume,

	.start_port_reset       = dwc2_hcd_start_port_reset
};

static int __devinit dwc2_hcd_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct usb_hcd *hcd;
	struct dwc2_hcd *dwc;
	int ret;

	BUILD_BUG_ON(sizeof(dwc->__ahb_cfg) != sizeof(dwc->ahb_cfg));
	BUILD_BUG_ON(sizeof(dwc->__host_cfg) != sizeof(dwc->host_cfg));
	BUILD_BUG_ON(sizeof(dwc->__usb_cfg) != sizeof(dwc->usb_cfg));
	BUILD_BUG_ON(sizeof(dwc->__hw_cfg1) != sizeof(dwc->hw_cfg1));
	BUILD_BUG_ON(sizeof(dwc->__hw_cfg2) != sizeof(dwc->hw_cfg2));
	BUILD_BUG_ON(sizeof(dwc->__hw_cfg3) != sizeof(dwc->hw_cfg3));
	BUILD_BUG_ON(sizeof(dwc->__hw_cfg4) != sizeof(dwc->hw_cfg4));
	BUILD_BUG_ON(sizeof(dwc->__lpm_cfg) != sizeof(dwc->lpm_cfg));
	BUILD_BUG_ON(sizeof(dwc->__hfir_cfg) != sizeof(dwc->hfir_cfg));
	BUILD_BUG_ON(sizeof(dwc->__hprt) != sizeof(dwc->hprt));

	hcd = usb_create_hcd(&dwc2_hcd_hc_driver,
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
	dwc2_hcd_get_cfg(hcd);

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

	dwc2_hcd_debug(hcd);

	/* Ensure the device won't trigger any interrupts */
	dwc->ahb_cfg.int_enable = false;
	dwc2_hcd_set_ahb_cfg(hcd);
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

static int dwc2_hcd_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct dwc2_hcd *dwc = hcd_to_dwc(hcd);

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

static struct of_device_id dwc2_hcd_match[] __devinitconst = {
	{ .compatible = "synopsys,designware-hs-otg2-host" },
	{ .compatible = "broadcom,bcm2708-usb" },
	{}
};
MODULE_DEVICE_TABLE(of, dwc2_hcd_match);

static struct platform_driver dwc2_hcd_driver = {
	.probe	= dwc2_hcd_probe,
	.remove	= dwc2_hcd_remove,
	.driver	= {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = dwc2_hcd_match
	}
};
module_platform_driver(dwc2_hcd_driver);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("Synopsys Designware HS OTG 2.xx driver (host mode)");
MODULE_LICENSE("GPL");
