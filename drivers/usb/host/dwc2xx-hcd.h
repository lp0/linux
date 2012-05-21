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
 *
 * Synopsys HS OTG Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 * 
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 * 
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef _DWC2XX_HCD_H
#define _DWC2XX_HCD_H

struct dwc2xx_hcd {
	struct device *dev;
	struct resource res;
	void __iomem *base;
	int irq;
	struct irqaction irqaction;
	struct spinlock lock;

	u32 id;		/* Synopsys ID */
};

/* 
 * The application interfaces with the HS OTG core by reading from and
 * writing to the Control and Status Register (CSR) space through the
 * AHB Slave interface. These registers are 32 bits wide, and the
 * addresses are 32-bit-block aligned.
 * CSRs are classified as follows:
 * - Core Global Registers
 * - Device Mode Registers
 * - Device Global Registers
 * - Device Endpoint Specific Registers
 * - Host Mode Registers
 * - Host Global Registers
 * - Host Port CSRs
 * - Host Channel Specific Registers
 *
 * Only the Core Global registers can be accessed in both Device and
 * Host modes. When the HS OTG core is operating in one mode, either
 * Device or Host, the application must not access registers from the
 * other mode. When the core switches from one mode to another, the
 * registers in the new mode of operation must be reprogrammed as they
 * would be after a power-on reset.
 */
#define DWC_OTG_CTL_REG		0x000	/* OTG Control and Status */
#define DWC_OTG_INT_REG		0x004	/* OTG Interrupt */
#define DWC_CORE_AHB_CFG_REG	0x008	/* Core AHB Configuration */
#define DWC_CORE_USB_CFG_REG	0x00c	/* Core USB Configuration */
#define DWC_CORE_RESET_REG	0x010	/* Core Reset */
#define DWC_CORE_INT_REG	0x014	/* Core Interrupt */
#define DWC_INT_MASK_REG	0x018	/* Core Interrupt Mask */
#define DWC_RX_STAT_PEEK_REG	0x01C	/* Receive Status Queue Read */
#define DWC_RX_STAT_POP_REG	0x020	/* Receive Status Queue Read & POP (Read Only) */
#define DWC_RX_FIFO_SZ_REG	0x024	/* Receive FIFO Size */
#define DWC_NP_TX_FIFO_SZ_REG	0x028	/* Non Periodic Transmit FIFO Size */
#define DWC_NP_TX_STAT_READ_REG	0x02c	/* Non Periodic Transmit FIFO/Queue Status */
#define DWC_I2C_CTL_REG		0x030	/* I2C Access */
#define DWC_PHY_VENDOR_CTRL_REG	0x034	/* PHY Vendor Control */
#define DWC_GPIO_REG		0x038	/* General Purpose Input/Output */
#define DWC_UID_REG		0x03c 	/* User ID */
#define DWC_SNPS_ID_REG		0x040	/* Synopsys ID (Read Only) */
#define DWC_SNPS_ID_MASK	0xfffff000
#define DWC_SNPS_ID_MATCH	0x4f542000 /* "OT2" */
#define DWC_USER_HW_CFG1_REG	0x044	/* User HW Config1 (Read Only) */
#define DWC_USER_HW_CFG2_REG	0x048	/* User HW Config2 (Read Only) */
enum dwc_hwcfg2_arch {
	DWC_SLAVE_ONLY_ARCH	= 0,
	DWC_EXT_DMA_ARCH	= 1,
	DWC_INT_DMA_ARCH	= 2
};
enum dwc_hwcfg2_mode {
	DWC_MODE_HNP_SRP_CAPABLE	= 0,
	DWC_MODE_SRP_ONLY_CAPABLE	= 1,
	DWC_MODE_NO_HNP_SRP_CAPABLE	= 2,
	DWC_MODE_SRP_CAPABLE_DEVICE	= 3,
	DWC_MODE_NO_SRP_CAPABLE_DEVICE	= 4,
	DWC_MODE_SRP_CAPABLE_HOST	= 5,
	DWC_MODE_NO_SRP_CAPABLE_HOST	= 6
};
#define DWC_USER_HW_CFG3_REG	0x04c	/* User HW Config3 (Read Only) */
#define DWC_USER_HW_CFG4_REG	0x050	/* User HW Config4 (Read Only) */
#define DWC_CORE_LPM_CFG_REG	0x054	/* Core LPM Configuration */
#define DWC_HP_TX_FIFO_SZ_REG	0x100	/* Host Periodic Transmit FIFO Size */
#define DWC_DV_TX_FIFO_SZ_BASE	0x104	/* Device Periodic Transmit FIFO#n if dedicated fifos are disabled, otherwise Device Transmit FIFO#n */
#define DWC_DV_TX_FIFO_SZ_REG(n) (DWC_DV_TX_FIFO_SZ_BASE + (n) * 4)
#define DWC_DV_TX_FIFO_SZ_COUNT	16

static void dwc2xx_hcd_dump_regs(struct dwc2xx_hcd *hcd)
{
	int i;

	WARN_ON(1);
	dev_dbg(hcd->dev, "%03x = %08x; DWC_OTG_CTL_REG\n", DWC_OTG_CTL_REG, readl(hcd->base + DWC_OTG_CTL_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_OTG_INT_REG\n", DWC_OTG_INT_REG, readl(hcd->base + DWC_OTG_INT_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_CORE_AHB_CFG_REG\n", DWC_CORE_AHB_CFG_REG, readl(hcd->base + DWC_CORE_AHB_CFG_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_CORE_USB_CFG_REG\n", DWC_CORE_USB_CFG_REG, readl(hcd->base + DWC_CORE_USB_CFG_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_CORE_RESET_REG\n", DWC_CORE_RESET_REG, readl(hcd->base + DWC_CORE_RESET_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_CORE_INT_REG\n", DWC_CORE_INT_REG, readl(hcd->base + DWC_CORE_INT_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_INT_MASK_REG\n", DWC_INT_MASK_REG, readl(hcd->base + DWC_INT_MASK_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_RX_STAT_PEEK_REG\n", DWC_RX_STAT_PEEK_REG, readl(hcd->base + DWC_RX_STAT_PEEK_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_RX_STAT_POP_REG\n", DWC_RX_STAT_POP_REG, readl(hcd->base + DWC_RX_STAT_POP_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_RX_FIFO_SZ_REG\n", DWC_RX_FIFO_SZ_REG, readl(hcd->base + DWC_RX_FIFO_SZ_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_NP_TX_FIFO_SZ_REG\n", DWC_NP_TX_FIFO_SZ_REG, readl(hcd->base + DWC_NP_TX_FIFO_SZ_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_NP_TX_STAT_READ_REG\n", DWC_NP_TX_STAT_READ_REG, readl(hcd->base + DWC_NP_TX_STAT_READ_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_I2C_CTL_REG\n", DWC_I2C_CTL_REG, readl(hcd->base + DWC_I2C_CTL_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_PHY_VENDOR_CTRL_REG\n", DWC_PHY_VENDOR_CTRL_REG, readl(hcd->base + DWC_PHY_VENDOR_CTRL_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_GPIO_REG\n", DWC_GPIO_REG, readl(hcd->base + DWC_GPIO_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_UID_REG\n", DWC_UID_REG, readl(hcd->base + DWC_UID_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_SNPS_ID_REG\n", DWC_SNPS_ID_REG, readl(hcd->base + DWC_SNPS_ID_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_USER_HW_CFG1_REG\n", DWC_USER_HW_CFG1_REG, readl(hcd->base + DWC_USER_HW_CFG1_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_USER_HW_CFG2_REG\n", DWC_USER_HW_CFG2_REG, readl(hcd->base + DWC_USER_HW_CFG2_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_USER_HW_CFG3_REG\n", DWC_USER_HW_CFG3_REG, readl(hcd->base + DWC_USER_HW_CFG3_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_USER_HW_CFG4_REG\n", DWC_USER_HW_CFG4_REG, readl(hcd->base + DWC_USER_HW_CFG4_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_CORE_LPM_CFG_REG\n", DWC_CORE_LPM_CFG_REG, readl(hcd->base + DWC_CORE_LPM_CFG_REG));
	dev_dbg(hcd->dev, "%03x = %08x; DWC_HP_TX_FIFO_SZ_REG\n", DWC_HP_TX_FIFO_SZ_REG, readl(hcd->base + DWC_HP_TX_FIFO_SZ_REG));
	for (i = 0; i < DWC_DV_TX_FIFO_SZ_COUNT; i++)
		dev_dbg(hcd->dev, "%03x = %08x; DWC_DV_TX_FIFO_SZ_REG(%d)\n", DWC_DV_TX_FIFO_SZ_REG(i), readl(hcd->base + DWC_DV_TX_FIFO_SZ_REG(i)), i);
}

#endif
