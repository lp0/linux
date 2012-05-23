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

#define DWC_SOFT_RESET_TIMEOUT		100
#define DWC_AHB_TIMEOUT			100

enum dwc_ahb_cfg_dma_burst {
	DWC_AHB_DMA_BURST_SINGLE,
	DWC_AHB_DMA_BURST_INCR,
	DWC_AHB_DMA_BURST_INCR4 = 3,
	DWC_AHB_DMA_BURST_INCR8 = 5,
	DWC_AHB_DMA_BURST_INCR16 = 7
};
struct dwc2xx_hcd_ahb_cfg {
	bool					int_enable:1;
	enum dwc_ahb_cfg_dma_burst		dma_burst:4;
	bool					dma_enable:1;
	unsigned				reserved:1;
	bool					nptxfemplvl_txfemplvl:1;
	bool					ptxfemplvl:1;
	unsigned				reserved9_20:12;
	bool					remmemsupp:1;
	bool					notialldmawrit:1;
	/* Remainder data in a DMA transfer:
	 * 0: The remainder data will be sent using INCR burst size (default)
	 * 1: The remainder data will be sent using SINGLE burst size
	 */
	bool					dma_single:1;
	unsigned				reserved24_31:8;
};

enum dwc_host_cfg_pclk {
	DWC_HOST_PCLK_30_60_MHZ,
	DWC_HOST_PCLK_48_MHZ,
	DWC_HOST_PCLK_6_MHZ
};
struct dwc2xx_hcd_host_cfg {
	enum dwc_host_cfg_pclk			fsls_pclk:2;
	bool					fsls_only:1;
	unsigned				reserved3_6:4;
	unsigned				en_32khz_susp:1;
	unsigned				res_val_period:8;
	unsigned				reserved16_22:7;
	bool					sg_dma:1;
	unsigned				fr_list_sz:2;
	unsigned				per_sched:1;
	unsigned				reserved27_30:4;
	bool					mode_chg_time:1;
};

struct dwc2xx_hcd_usb_cfg {
	unsigned				toutcal:3;
	bool					phyif:1;
	bool					ulpi_utmi_sel:1;
	bool					fsintf:1;
	bool					physel:1;
	bool					ddrsel:1;
	bool					srp_capable:1;
	bool					hnp_capable:1;
	unsigned				usbtrdtim:4;
	unsigned				reserved1:1;
	/* PHY lower power mode clock select */
	bool					phy_lpm_clk_sel:1;
	bool					otgutmifssel:1;
	bool					ulpi_fsls:1;
	bool					ulpi_auto_res:1;
	bool					ulpi_clk_sus_m:1;
	bool					ulpi_ext_vbus_drv:1;
	bool					ulpi_int_vbus_indicator:1;
	bool					term_sel_dl_pulse:1;
	bool					indicator_complement:1;
	bool					indicator_pass_through:1;
	bool					ulpi_int_prot_dis:1;
	bool					ic_usb_capable:1;
	bool					ic_traffic_pull_remove:1;
	bool					tx_end_delay:1;
	bool					force_host_mode:1;
	bool					force_dev_mode:1;
	unsigned				reserved31:1;
};

struct dwc2xx_hcd_hw_cfg1 {
	unsigned ep_dir0:2;
	unsigned ep_dir1:2;
	unsigned ep_dir2:2;
	unsigned ep_dir3:2;
	unsigned ep_dir4:2;
	unsigned ep_dir5:2;
	unsigned ep_dir6:2;
	unsigned ep_dir7:2;
	unsigned ep_dir8:2;
	unsigned ep_dir9:2;
	unsigned ep_dir10:2;
	unsigned ep_dir11:2;
	unsigned ep_dir12:2;
	unsigned ep_dir13:2;
	unsigned ep_dir14:2;
	unsigned ep_dir15:2;
};

enum dwc_cfg2_op_mode {
	DWC_MODE_HNP_SRP_CAPABLE,
	DWC_MODE_SRP_ONLY_CAPABLE,
	DWC_MODE_NO_HNP_SRP_CAPABLE,
	DWC_MODE_SRP_CAPABLE_DEVICE,
	DWC_MODE_NO_SRP_CAPABLE_DEVICE,
	DWC_MODE_SRP_CAPABLE_HOST,
	DWC_MODE_NO_SRP_CAPABLE_HOST
};
enum dwc_cfg2_arch {
	DWC_SLAVE_ONLY_ARCH,
	DWC_EXT_DMA_ARCH,
	DWC_INT_DMA_ARCH
};
enum dwc_cfg2_hs_phy_type {
	DWC_CFG2_HS_PHY_NOT_SUPPORTED,
	DWC_CFG2_HS_PHY_UTMI,
	DWC_CFG2_HS_PHY_ULPI,
	DWC_CFG2_HS_PHY_UTMI_ULPI
};
enum dwc_cfg2_fs_phy_type {
	DWC_CFG2_FS_PHY_0,
	DWC_CFG2_FS_PHY_FSLS,
	DWC_CFG2_FS_PHY_2,
	DWC_CFG2_FS_PHY_3
};
struct dwc2xx_hcd_hw_cfg2 {
	enum dwc_cfg2_op_mode			op_mode:3;
	enum dwc_cfg2_arch			arch:2;
	bool					point2point:1;
	enum dwc_cfg2_hs_phy_type		hs_phy:2;
	enum dwc_cfg2_fs_phy_type		fs_phy:2;
	unsigned				num_dev_ep:4;
	unsigned				num_host_chan:4;
	bool					perio_ep_supported:1;
	bool					dynamic_fifo:1;
	bool					multi_proc_int:1;
	unsigned				reserved21:1;
	unsigned				nonperio_tx_q_depth:2;
	unsigned				host_perio_tx_q_depth:2;
	unsigned				dev_token_q_depth:5;
	bool					otg_enable_ic_usb:1;
};

struct dwc2xx_hcd_hw_cfg3 {
	unsigned				xfer_size_cntr_width:4;
	unsigned				packet_size_cntr_width:3;
	bool					otg_func:1;
	bool					i2c:1;
	bool					vendor_ctrl_if:1;
	bool					optional_features:1;
	bool					synch_reset_type:1;
	bool					adp_supp:1;
	bool					otg_enable_hsic:1;
	bool					bc_support:1;
	bool					otg_lpm_en:1;
	unsigned				dfifo_depth:16;
};

enum dwc_cfg4_phy_width {
	DWC_CFG4_PHY_WIDTH_8BIT,
	DWC_CFG4_PHY_WIDTH_16BIT,
	DWC_CFG4_PHY_WIDTH_8_OR_16BIT
};
struct dwc2xx_hcd_hw_cfg4 {
	unsigned				num_dev_perio_in_ep:4;
	bool					power_optimiz:1;
	bool					min_ahb_freq:1;
	bool					part_power_down:1;
	unsigned				reserved:7;
	enum dwc_cfg4_phy_width			utmi_phy_data_width:2;
	unsigned				num_dev_mode_ctrl_ep:4;
	bool					iddig_filt_en:1;
	bool					vbus_valid_filt_en:1;
	bool					a_valid_filt_en:1;
	bool					b_valid_filt_en:1;
	bool					session_end_filt_en:1;
	bool					ded_fifo_en:1;
	unsigned				num_in_eps:4;
	bool					desc_dma:1;
	bool					desc_dma_dyn:1;
};

struct dwc2xx_hcd_lpm_cfg {
	/* LPM-Capable (LPMCap) (Device and Host) The application uses this bit
	 * to control the DWC_otg core LPM capabilities. */
	bool					lpm_cap_en:1;
	/* LPM response programmed by application (AppL1Res) (Device) Handshake
	 * response to LPM token pre-programmed by device application software.
	 */
	bool					appl_resp:1;
	/* Host Initiated Resume Duration (HIRD) (Device and Host) In Host mode
	 * this field indicates the value of HIRD to be sent in an LPM
	 * transaction. */
	unsigned				hird:4;
	/* RemoteWakeEnable (bRemoteWake) (Device and Host) In Host mode this
	 * bit indicates the value of remote wake up to be sent in wIndex field
	 * of LPM transaction. */
	bool					rem_wkup_en:1;
	/* Enable utmi_sleep_n (EnblSlpM) (Device and Host) The application uses
	 * this bit to control the utmi_sleep_n assertion to the PHY when in L1
	 * state. */
	bool					en_utmi_sleep:1;
	/* HIRD Threshold (HIRD_Thres) (Device and Host). */
	unsigned				hird_thres:5;
	/* LPM Response (CoreL1Res) (Device and Host) In Host mode this bit
	 * contains handsake response to LPM transaction. */
	unsigned				lpm_resp:2;
	/* Port Sleep Status (SlpSts) (Device and Host) This bit is set as long
	 * as a Sleep condition is present on the USB bus. */
	bool					prt_sleep_sts:1;
	/* Sleep State Resume OK (L1ResumeOK) (Device and Host) Indicates that
	 * the application or host can start resume from Sleep state. */
	bool					sleep_state_resumeok:1;
	/* LPM channel Index (LPM_Chnl_Indx) (Host) The channel number on which
	 * the LPM transaction has to be applied while sending an LPM
	 * transaction to the local device. */
	unsigned				lpm_chan_index:4;
	/* LPM Retry Count (LPM_Retry_Cnt) (Host) Number host retries that would
	 * be performed if the device response was not valid response. */
	unsigned				retry_count:3;
	/* Send LPM Transaction (SndLPM) (Host) When set by application
	 * software, an LPM transaction containing two tokens is sent. */
	bool					send_lpm:1;
	/* LPM Retry status (LPM_RetryCnt_Sts) (Host) Number of LPM Host Retries
	 * still remaining to be transmitted for the current LPM sequence. */
	unsigned				retry_count_sts:3;
	unsigned				reserved28_29:2;
	/* In host mode once this bit is set, the host configures to drive the
	 * HSIC Idle state on the bus. */
	bool					hsic_connect:1;
	/* This bit overrides and functionally inverts the if_select_hsic input
	 * port signal. */
	bool					inv_sel_hsic:1;
};

struct dwc2xx_hcd_hfir_cfg {
	u16				frame_interval:16;
	bool				dyn_frame_reload:1;
	unsigned			reserved:15;
};

enum dwc_hprt_speed {
	DWC_HPRT_SPEED_HIGH,
	DWC_HPRT_SPEED_FULL,
	DWC_HPRT_SPEED_LOW
};
struct dwc2xx_hcd_hprt {
	bool				connect:1;
	bool				connect_int:1;		/* intr */
	bool				enabled:1;
	bool				enabled_int:1;		/* intr */
	bool				overcurrent:1;
	bool				overcurrent_int:1;	/* intr */	
	bool				resume:1;
	bool				suspend:1;
	bool				reset:1;
	unsigned			reserved9:1;
	unsigned			prtlnsts:2;
	bool				power:1;
	unsigned			prttstctl:4;
	enum dwc_hprt_speed		speed:2;
	unsigned			reserved19_31:13;
};

struct dwc2xx_hcd {
	struct device *dev;
	struct resource res;
	int irq;
#ifdef CONFIG_BCM_VC_POWER
	struct bcm_vc_power_dev *power;
#endif
	struct spinlock lock;

	u32 snps_id;
	u32 user_id;
	union {
		u32 __ahb_cfg;
		struct dwc2xx_hcd_ahb_cfg ahb_cfg;
	};
	union {
		u32 __host_cfg;
		struct dwc2xx_hcd_host_cfg host_cfg;
	};
	union {
		u32 __usb_cfg;
		struct dwc2xx_hcd_usb_cfg usb_cfg;
	};
	union {
		u32 __hw_cfg1;
		struct dwc2xx_hcd_hw_cfg1 hw_cfg1;
	};
	union {
		u32 __hw_cfg2;
		struct dwc2xx_hcd_hw_cfg2 hw_cfg2;
	};
	union {
		u32 __hw_cfg3;
		struct dwc2xx_hcd_hw_cfg3 hw_cfg3;
	};
	union {
		u32 __hw_cfg4;
		struct dwc2xx_hcd_hw_cfg4 hw_cfg4;
	};
	union {
		u32 __lpm_cfg;
		struct dwc2xx_hcd_lpm_cfg lpm_cfg;
	};
	union {
		u32 __hfir_cfg;
		struct dwc2xx_hcd_hfir_cfg hfir_cfg;
	};
	union {
		u32 __hprt;
		struct dwc2xx_hcd_hprt hprt;
	};
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
#define DWC_OTG_CTL_REG			0x000	/* OTG Control and Status */
#define DWC_OTG_INT_REG			0x004	/* OTG Interrupt */
#define DWC_CORE_AHB_CFG_REG		0x008	/* Core AHB Configuration */
#define DWC_CORE_USB_CFG_REG		0x00c	/* Core USB Configuration */

#define DWC_CORE_RESET_REG		0x010	/* Core Reset */
enum dwc2xx_hcd_core_reset {
	DWC_CORE_SOFT_RESET,		/* Core Soft Reset (CSftRst) (Device and Host) */
	DWC_HCLK_SOFT_RESET,		/* Hclk Soft Reset */
	DWC_HOST_FC_RESET,		/* Host Frame Counter Reset (Host Only) */
	DWC_IN_TOKEN_QUEUE_FLUSH,	/* In Token Sequence Learning Queue Flush (INTknQFlsh) (Device Only) */
	DWC_RX_FIFO_FLUSH,		/* RxFIFO Flush (RxFFlsh) (Device and Host) */
	DWC_TX_FIFO_FLUSH,		/* TxFIFO Flush (TxFFlsh) (Device and Host) */
#define DWC_TX_FIFO_FLUSH_MASK		0x07C0	/* TxFIFO Number (TxFNum) (Device and Host) */
	DWC_DMA_REQ_SIGNAL = 30,	/* DMA Request Signal */
	DWC_AHB_MASTER_IDLE		/* AHB Master Idle */
};

#define DWC_CORE_INT_STAT_REG		0x014	/* Core Interrupt */
#define DWC_CORE_INT_MASK_REG		0x018	/* Core Interrupt Mask */
enum dwc2xx_hcd_core_int {
	DWC_CURRENT_MODE_INT		= BIT(0),
	DWC_MODE_MISMATCH_INT		= BIT(1),
	DWC_OTG_INT			= BIT(2),
	DWC_DMA_SOF_INT			= BIT(3),
	DWC_RX_STAT_LEVEL_INT		= BIT(4),
	DWC_NP_TX_FIFO_EMPTY_INT	= BIT(5),
	DWC_GINNAKEFF_INT		= BIT(6), /* TODO: rename */
	DWC_GOUTNAKEFF_INT		= BIT(7), /* TODO: rename */
	DWC_ULPICK_INT			= BIT(8), /* TODO: rename */
	DWC_I2C_INT			= BIT(9),
	DWC_EARLY_SUSPEND_INT		= BIT(10),
	DWC_USB_SUSPEND_INT		= BIT(11),
	DWC_USB_RESET_INT		= BIT(12),
	DWC_ENUM_DONE_INT		= BIT(13),
	DWC_ISO_OUT_DROP_INT		= BIT(14),
	DWC_EOP_FRAME_INT		= BIT(15),
	DWC_RESTORE_DONE_INT		= BIT(16),
	DWC_EP_MISMATCH_INT		= BIT(17),
	DWC_IN_EP_INT			= BIT(18),
	DWC_OUT_EP_INT			= BIT(19),
	DWC_INCOMPLETE_ISO_IN_INT	= BIT(20),
	DWC_INCOMPLETE_ISO_OUT_INT	= BIT(21),
	DWC_FETSUSP_INT			= BIT(22), /* TODO: rename */
	DWC_RESET_DETECT_INT		= BIT(23),
	DWC_PORT_INT			= BIT(24),
	DWC_HOST_CHAN_INT		= BIT(25),
	DWC_HP_TX_FIFO_EMPTY_INT	= BIT(26),
	DWC_LPM_TXN_RCVD_INT		= BIT(27),
	DWC_CON_ID_STAT_CHG_INT		= BIT(28),
	DWC_DISCONNECT_INT		= BIT(29),
	DWC_SESS_REQ_INT		= BIT(30),
	DWC_WAKEUP_INT			= BIT(31)
};

#define DWC_RX_STAT_PEEK_REG	0x01C	/* Receive Status Queue Read */
#define DWC_RX_STAT_POP_REG	0x020	/* Receive Status Queue Read & POP (Read Only) */
#define DWC_RX_FIFO_SZ_REG	0x024	/* Receive FIFO Size */
#define DWC_NP_TX_FIFO_SZ_REG	0x028	/* Non Periodic Transmit FIFO Size */
#define DWC_NP_TX_STAT_REG	0x02c	/* Non Periodic Transmit FIFO/Queue Status */
#define DWC_I2C_CTL_REG		0x030	/* I2C Access */
#define DWC_PHY_VENDOR_CTRL_REG	0x034	/* PHY Vendor Control */
#define DWC_GPIO_REG		0x038	/* General Purpose Input/Output */
#define DWC_USER_ID_REG		0x03c 	/* User ID */
#define DWC_SNPS_ID_REG		0x040	/* Synopsys ID (Read Only) */
#define DWC_SNPS_ID_MASK	0xfffff000
#define DWC_SNPS_ID_MATCH	0x4f542000 /* "OT2" */
#define DWC_USER_HW_CFG1_REG	0x044	/* User HW Config1 (Read Only) */
#define DWC_USER_HW_CFG2_REG	0x048	/* User HW Config2 (Read Only) */
#define DWC_USER_HW_CFG3_REG	0x04c	/* User HW Config3 (Read Only) */
#define DWC_USER_HW_CFG4_REG	0x050	/* User HW Config4 (Read Only) */
#define DWC_CORE_LPM_CFG_REG	0x054	/* Core LPM Configuration */
#define DWC_HP_TX_FIFO_SZ_REG	0x100	/* Host Periodic Transmit FIFO Size */
#define DWC_DV_TX_FIFO_SZ_BASE	0x104	/* Device Periodic Transmit FIFO#n if dedicated fifos are disabled, otherwise Device Transmit FIFO#n */
#define DWC_DV_TX_FIFO_SZ_REG(n) (DWC_DV_TX_FIFO_SZ_BASE + (n) * 4)
#define DWC_DV_TX_FIFO_SZ_COUNT	15

#define DWC_HOST_CFG_REG		0x400	/* Host Configuration Register */
#define DWC_HOST_FRAME_INTVL_REG	0x404	/* Host Frame Interval Register */
#define DWC_HOST_FRAME_NUM_REG		0x408	/* Host Frame Number / Frame Remaining */
#define DWC_HP_TX_FIFO_STAT_REG		0x410	/* Host Periodic Transmit FIFO/Queue Status */
#define DWC_HOST_ALL_CHANS_INT_STAT_REG	0x414	/* Host All Channels Interrupt */
#define DWC_HOST_ALL_CHANS_INT_MASK_REG	0x418	/* Host All Channels Interrupt Mask */
#define DWC_HOST_FRAME_LIST_REG		0x41c	/* Host Frame List Base Address */

#define DWC_HOST_PORT_REG		0x440	/* Host Port */

#define DWC_HOST_CHAN_BASE		0x500	/* Host Channel Registers */
#define DWC_HOST_CHAN_CHAR_REG(n)	(DWC_HOST_CHAN_BASE + (n) * 0x20 + 0x00)	/* Host Channel Characteristic */
#define DWC_HOST_CHAN_SPLIT_REG(n)	(DWC_HOST_CHAN_BASE + (n) * 0x20 + 0x04)	/* Host Channel Split Control */
#define DWC_HOST_CHAN_INT_STAT_REG(n)	(DWC_HOST_CHAN_BASE + (n) * 0x20 + 0x08)	/* Host Channel Interrupt */
#define DWC_HOST_CHAN_INT_MASK_REG(n)	(DWC_HOST_CHAN_BASE + (n) * 0x20 + 0x0c)	/* Host Channel Interrupt Mask */
enum dwc2xx_hcd_host_chan_int {
	DWC_CHAN_XFER_COMP_INT		= BIT(0),
	DWC_CHAN_HALT_INT		= BIT(1),
	DWC_CHAN_AHB_ERR_INT		= BIT(2),
	DWC_CHAN_STALL_INT		= BIT(3),
	DWC_CHAN_NAK_INT		= BIT(4),
	DWC_CHAN_ACK_INT		= BIT(5),
	DWC_CHAN_NYET_INT		= BIT(6),
	DWC_CHAN_XACT_ERR_INT		= BIT(7),
	DWC_CHAN_BBL_ERR_INT		= BIT(8), /* Babble error */
	DWC_CHAN_FR_OVERRUN_INT		= BIT(9),
	DWC_CHAN_DTGL_ERR_INT		= BIT(10), /* Data toggle error */
	DWC_CHAN_BNA_INT		= BIT(11), /* Buffer not available */
	DWC_CHAN_XCES_ERR_INT		= BIT(12), /* Excessive txn error */
	DWC_CHAN_FR_LIST_ROLL_INT	= BIT(13) /* Frame list rollover */
};

#define DWC_HOST_CHAN_TX_SZ_REG(n)	(DWC_HOST_CHAN_BASE + (n) * 0x20 + 0x10)	/* Host Channel Transfer Size */
#define DWC_HOST_CHAN_DMA_ADDR_REG(n)	(DWC_HOST_CHAN_BASE + (n) * 0x20 + 0x14)	/* Host Channel DMA Address */
#define DWC_HOST_CHAN_DMA_BUFA_REG(n)	(DWC_HOST_CHAN_BASE + (n) * 0x20 + 0x1c)	/* Host Channel DMA Buffer Address */
#define DWC_HOST_CHAN_COUNT		16	

#define DWC_OTG_PWR_CLK_CTL_REG		0xe00

static inline struct dwc2xx_hcd *hcd_to_dwc(struct usb_hcd *hcd)
{
	return (struct dwc2xx_hcd *)hcd->hcd_priv;
}

static void dwc2xx_hcd_get_cfg(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	dwc->__ahb_cfg = readl(hcd->regs + DWC_CORE_AHB_CFG_REG);
	dwc->__usb_cfg = readl(hcd->regs + DWC_CORE_USB_CFG_REG);
	dwc->__hw_cfg1 = readl(hcd->regs + DWC_USER_HW_CFG1_REG);
	dwc->__hw_cfg2 = readl(hcd->regs + DWC_USER_HW_CFG2_REG);
	dwc->__hw_cfg3 = readl(hcd->regs + DWC_USER_HW_CFG3_REG);
	dwc->__hw_cfg4 = readl(hcd->regs + DWC_USER_HW_CFG4_REG);
	dwc->__lpm_cfg = readl(hcd->regs + DWC_CORE_LPM_CFG_REG);
}

static void dwc2xx_hcd_set_ahb_cfg(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	u32 value = dwc->__ahb_cfg;
	writel(dwc->__ahb_cfg, hcd->regs + DWC_CORE_AHB_CFG_REG);
	dwc->__ahb_cfg = readl(hcd->regs + DWC_CORE_AHB_CFG_REG);
	WARN(dwc->__ahb_cfg != value, "%s: write %08x, read %08x\n",
		__func__, value, dwc->__ahb_cfg);
}

static void dwc2xx_hcd_get_host_cfg(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	dwc->__host_cfg = readl(hcd->regs + DWC_HOST_CFG_REG);
}

static void dwc2xx_hcd_set_host_cfg(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	u32 value = dwc->__host_cfg;
	writel(dwc->__host_cfg, hcd->regs + DWC_HOST_CFG_REG);
	dwc->__host_cfg = readl(hcd->regs + DWC_HOST_CFG_REG);
	WARN(dwc->__host_cfg != value, "%s: write %08x, read %08x\n",
		__func__, value, dwc->__host_cfg);
}

static void dwc2xx_hcd_set_usb_cfg(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	u32 value = dwc->__usb_cfg;
	writel(dwc->__usb_cfg, hcd->regs + DWC_CORE_USB_CFG_REG);
	dwc->__usb_cfg = readl(hcd->regs + DWC_CORE_USB_CFG_REG);
	WARN(dwc->__usb_cfg != value, "%s: write %08x, read %08x\n",
		__func__, value, dwc->__usb_cfg);
}

static void dwc2xx_hcd_set_lpm_cfg(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	u32 value = dwc->__lpm_cfg;
	writel(dwc->__lpm_cfg, hcd->regs + DWC_CORE_LPM_CFG_REG);
	dwc->__lpm_cfg = readl(hcd->regs + DWC_CORE_LPM_CFG_REG);
	WARN(dwc->__lpm_cfg != value, "%s: write %08x, read %08x\n",
		__func__, value, dwc->__lpm_cfg);
}

static void dwc2xx_hcd_get_hprt(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	dwc->__hprt = readl(hcd->regs + DWC_HOST_PORT_REG);
}

static void dwc2xx_hcd_set_hprt(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	u32 value = dwc->__hprt;
	writel(dwc->__hprt, hcd->regs + DWC_HOST_PORT_REG);
	dwc->__hprt = readl(hcd->regs + DWC_HOST_PORT_REG);
	value &= ~0x2A; /* Interrupts */
	WARN(dwc->__hprt != value, "%s: write %08x, read %08x\n",
		__func__, value, dwc->__hprt);
}

static void dwc2xx_hcd_get_hfir_cfg(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	dwc->__hfir_cfg = readl(hcd->regs + DWC_HOST_FRAME_INTVL_REG);
}

static void dwc2xx_hcd_set_hfir_cfg(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	u32 value = dwc->__hfir_cfg;
	writel(dwc->__hfir_cfg, hcd->regs + DWC_HOST_FRAME_INTVL_REG);
	dwc->__hfir_cfg = readl(hcd->regs + DWC_HOST_FRAME_INTVL_REG);
	WARN(dwc->__hfir_cfg != value, "%s: write %08x, read %08x\n",
		__func__, value, dwc->__hfir_cfg);
}

static void dwc2xx_hcd_dump_regs(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);
	int i;

	dev_dbg(dwc->dev, "%03x = %08x; DWC_OTG_CTL_REG\n", DWC_OTG_CTL_REG, readl(hcd->regs + DWC_OTG_CTL_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_OTG_INT_REG\n", DWC_OTG_INT_REG, readl(hcd->regs + DWC_OTG_INT_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_CORE_RESET_REG\n", DWC_CORE_RESET_REG, readl(hcd->regs + DWC_CORE_RESET_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_CORE_INT_STAT_REG\n", DWC_CORE_INT_STAT_REG, readl(hcd->regs + DWC_CORE_INT_STAT_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_CORE_INT_MASK_REG\n", DWC_CORE_INT_MASK_REG, readl(hcd->regs + DWC_CORE_INT_MASK_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_RX_STAT_PEEK_REG\n", DWC_RX_STAT_PEEK_REG, readl(hcd->regs + DWC_RX_STAT_PEEK_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_RX_STAT_POP_REG\n", DWC_RX_STAT_POP_REG, readl(hcd->regs + DWC_RX_STAT_POP_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_RX_FIFO_SZ_REG\n", DWC_RX_FIFO_SZ_REG, readl(hcd->regs + DWC_RX_FIFO_SZ_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_NP_TX_FIFO_SZ_REG\n", DWC_NP_TX_FIFO_SZ_REG, readl(hcd->regs + DWC_NP_TX_FIFO_SZ_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_NP_TX_STAT_REG\n", DWC_NP_TX_STAT_REG, readl(hcd->regs + DWC_NP_TX_STAT_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_I2C_CTL_REG\n", DWC_I2C_CTL_REG, readl(hcd->regs + DWC_I2C_CTL_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_PHY_VENDOR_CTRL_REG\n", DWC_PHY_VENDOR_CTRL_REG, readl(hcd->regs + DWC_PHY_VENDOR_CTRL_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_GPIO_REG\n", DWC_GPIO_REG, readl(hcd->regs + DWC_GPIO_REG));
	dev_dbg(dwc->dev, "%03x = %08x; DWC_HP_TX_FIFO_SZ_REG\n", DWC_HP_TX_FIFO_SZ_REG, readl(hcd->regs + DWC_HP_TX_FIFO_SZ_REG));
	for (i = 0; i < DWC_DV_TX_FIFO_SZ_COUNT; i++)
		dev_dbg(dwc->dev, "%03x = %08x; DWC_DV_TX_FIFO_SZ_REG(%d)\n", DWC_DV_TX_FIFO_SZ_REG(i), readl(hcd->regs + DWC_DV_TX_FIFO_SZ_REG(i)), i);
}

static void dwc2xx_hcd_dump_config(struct usb_hcd *hcd)
{
	struct dwc2xx_hcd *dwc = hcd_to_dwc(hcd);

	dev_dbg(dwc->dev, "ahb.int_enable = %u\n", dwc->ahb_cfg.int_enable);
	dev_dbg(dwc->dev, "ahb.dma_burst = %u\n", dwc->ahb_cfg.dma_burst);
	dev_dbg(dwc->dev, "ahb.dma_enable = %u\n", dwc->ahb_cfg.dma_enable);
	dev_dbg(dwc->dev, "ahb.reserved = %u\n", dwc->ahb_cfg.reserved);
	dev_dbg(dwc->dev, "ahb.nptxfemplvl_txfemplvl = %u\n", dwc->ahb_cfg.nptxfemplvl_txfemplvl);
	dev_dbg(dwc->dev, "ahb.ptxfemplvl = %u\n", dwc->ahb_cfg.ptxfemplvl);
	dev_dbg(dwc->dev, "ahb.reserved9_20 = %u\n", dwc->ahb_cfg.reserved9_20);
	dev_dbg(dwc->dev, "ahb.remmemsupp = %u\n", dwc->ahb_cfg.remmemsupp);
	dev_dbg(dwc->dev, "ahb.notialldmawrit = %u\n", dwc->ahb_cfg.notialldmawrit);
	dev_dbg(dwc->dev, "ahb.dma_single = %u\n", dwc->ahb_cfg.dma_single);
	dev_dbg(dwc->dev, "ahb.reserved24_31 = %u\n", dwc->ahb_cfg.reserved24_31);

	dev_dbg(dwc->dev, "usb.toutcal = %u\n", dwc->usb_cfg.toutcal);
	dev_dbg(dwc->dev, "usb.phyif = %u\n", dwc->usb_cfg.phyif);
	dev_dbg(dwc->dev, "usb.ulpi_utmi_sel = %u\n", dwc->usb_cfg.ulpi_utmi_sel);
	dev_dbg(dwc->dev, "usb.fsintf = %u\n", dwc->usb_cfg.fsintf);
	dev_dbg(dwc->dev, "usb.physel = %u\n", dwc->usb_cfg.physel);
	dev_dbg(dwc->dev, "usb.ddrsel = %u\n", dwc->usb_cfg.ddrsel);
	dev_dbg(dwc->dev, "usb.srp_capable = %u\n", dwc->usb_cfg.srp_capable);
	dev_dbg(dwc->dev, "usb.hnp_capable = %u\n", dwc->usb_cfg.hnp_capable);
	dev_dbg(dwc->dev, "usb.usbtrdtim = %u\n", dwc->usb_cfg.usbtrdtim);
	dev_dbg(dwc->dev, "usb.reserved1 = %u\n", dwc->usb_cfg.reserved1);
	dev_dbg(dwc->dev, "usb.phy_lpm_clk_sel = %u\n", dwc->usb_cfg.phy_lpm_clk_sel);
	dev_dbg(dwc->dev, "usb.otgutmifssel = %u\n", dwc->usb_cfg.otgutmifssel);
	dev_dbg(dwc->dev, "usb.ulpi_fsls = %u\n", dwc->usb_cfg.ulpi_fsls);
	dev_dbg(dwc->dev, "usb.ulpi_auto_res = %u\n", dwc->usb_cfg.ulpi_auto_res);
	dev_dbg(dwc->dev, "usb.ulpi_clk_sus_m = %u\n", dwc->usb_cfg.ulpi_clk_sus_m);
	dev_dbg(dwc->dev, "usb.ulpi_ext_vbus_drv = %u\n", dwc->usb_cfg.ulpi_ext_vbus_drv);
	dev_dbg(dwc->dev, "usb.ulpi_int_vbus_indicator = %u\n", dwc->usb_cfg.ulpi_int_vbus_indicator);
	dev_dbg(dwc->dev, "usb.term_sel_dl_pulse = %u\n", dwc->usb_cfg.term_sel_dl_pulse);
	dev_dbg(dwc->dev, "usb.indicator_complement = %u\n", dwc->usb_cfg.indicator_complement);
	dev_dbg(dwc->dev, "usb.indicator_pass_through = %u\n", dwc->usb_cfg.indicator_pass_through);
	dev_dbg(dwc->dev, "usb.ulpi_int_prot_dis = %u\n", dwc->usb_cfg.ulpi_int_prot_dis);
	dev_dbg(dwc->dev, "usb.ic_usb_capable = %u\n", dwc->usb_cfg.ic_usb_capable);
	dev_dbg(dwc->dev, "usb.ic_traffic_pull_remove = %u\n", dwc->usb_cfg.ic_traffic_pull_remove);
	dev_dbg(dwc->dev, "usb.tx_end_delay = %u\n", dwc->usb_cfg.tx_end_delay);
	dev_dbg(dwc->dev, "usb.force_host_mode = %u\n", dwc->usb_cfg.force_host_mode);
	dev_dbg(dwc->dev, "usb.force_dev_mode = %u\n", dwc->usb_cfg.force_dev_mode);
	dev_dbg(dwc->dev, "usb.reserved31 = %u\n", dwc->usb_cfg.reserved31);

	dev_dbg(dwc->dev, "hw1.ep_dir0 = %u\n", dwc->hw_cfg1.ep_dir0);
	dev_dbg(dwc->dev, "hw1.ep_dir1 = %u\n", dwc->hw_cfg1.ep_dir1);
	dev_dbg(dwc->dev, "hw1.ep_dir2 = %u\n", dwc->hw_cfg1.ep_dir2);
	dev_dbg(dwc->dev, "hw1.ep_dir3 = %u\n", dwc->hw_cfg1.ep_dir3);
	dev_dbg(dwc->dev, "hw1.ep_dir4 = %u\n", dwc->hw_cfg1.ep_dir4);
	dev_dbg(dwc->dev, "hw1.ep_dir5 = %u\n", dwc->hw_cfg1.ep_dir5);
	dev_dbg(dwc->dev, "hw1.ep_dir6 = %u\n", dwc->hw_cfg1.ep_dir6);
	dev_dbg(dwc->dev, "hw1.ep_dir7 = %u\n", dwc->hw_cfg1.ep_dir7);
	dev_dbg(dwc->dev, "hw1.ep_dir8 = %u\n", dwc->hw_cfg1.ep_dir8);
	dev_dbg(dwc->dev, "hw1.ep_dir9 = %u\n", dwc->hw_cfg1.ep_dir9);
	dev_dbg(dwc->dev, "hw1.ep_dir10 = %u\n", dwc->hw_cfg1.ep_dir10);
	dev_dbg(dwc->dev, "hw1.ep_dir11 = %u\n", dwc->hw_cfg1.ep_dir11);
	dev_dbg(dwc->dev, "hw1.ep_dir12 = %u\n", dwc->hw_cfg1.ep_dir12);
	dev_dbg(dwc->dev, "hw1.ep_dir13 = %u\n", dwc->hw_cfg1.ep_dir13);
	dev_dbg(dwc->dev, "hw1.ep_dir14 = %u\n", dwc->hw_cfg1.ep_dir14);
	dev_dbg(dwc->dev, "hw1.ep_dir15 = %u\n", dwc->hw_cfg1.ep_dir15);

	dev_dbg(dwc->dev, "hw2.op_mode = %u\n", dwc->hw_cfg2.op_mode);
	dev_dbg(dwc->dev, "hw2.arch = %u\n", dwc->hw_cfg2.arch);
	dev_dbg(dwc->dev, "hw2.point2point = %u\n", dwc->hw_cfg2.point2point);
	dev_dbg(dwc->dev, "hw2.hs_phy = %u\n", dwc->hw_cfg2.hs_phy);
	dev_dbg(dwc->dev, "hw2.fs_phy = %u\n", dwc->hw_cfg2.fs_phy);
	dev_dbg(dwc->dev, "hw2.num_dev_ep = %u\n", dwc->hw_cfg2.num_dev_ep);
	dev_dbg(dwc->dev, "hw2.num_host_chan = %u\n", dwc->hw_cfg2.num_host_chan);
	dev_dbg(dwc->dev, "hw2.perio_ep_supported = %u\n", dwc->hw_cfg2.perio_ep_supported);
	dev_dbg(dwc->dev, "hw2.dynamic_fifo = %u\n", dwc->hw_cfg2.dynamic_fifo);
	dev_dbg(dwc->dev, "hw2.multi_proc_int = %u\n", dwc->hw_cfg2.multi_proc_int);
	dev_dbg(dwc->dev, "hw2.reserved21 = %u\n", dwc->hw_cfg2.reserved21);
	dev_dbg(dwc->dev, "hw2.nonperio_tx_q_depth = %u\n", dwc->hw_cfg2.nonperio_tx_q_depth);
	dev_dbg(dwc->dev, "hw2.host_perio_tx_q_depth = %u\n", dwc->hw_cfg2.host_perio_tx_q_depth);
	dev_dbg(dwc->dev, "hw2.dev_token_q_depth = %u\n", dwc->hw_cfg2.dev_token_q_depth);
	dev_dbg(dwc->dev, "hw2.otg_enable_ic_usb = %u\n", dwc->hw_cfg2.otg_enable_ic_usb);

	dev_dbg(dwc->dev, "hw3.xfer_size_cntr_width = %u\n", dwc->hw_cfg3.xfer_size_cntr_width);
	dev_dbg(dwc->dev, "hw3.packet_size_cntr_width = %u\n", dwc->hw_cfg3.packet_size_cntr_width);
	dev_dbg(dwc->dev, "hw3.otg_func = %u\n", dwc->hw_cfg3.otg_func);
	dev_dbg(dwc->dev, "hw3.i2c = %u\n", dwc->hw_cfg3.i2c);
	dev_dbg(dwc->dev, "hw3.vendor_ctrl_if = %u\n", dwc->hw_cfg3.vendor_ctrl_if);
	dev_dbg(dwc->dev, "hw3.optional_features = %u\n", dwc->hw_cfg3.optional_features);
	dev_dbg(dwc->dev, "hw3.synch_reset_type = %u\n", dwc->hw_cfg3.synch_reset_type);
	dev_dbg(dwc->dev, "hw3.adp_supp = %u\n", dwc->hw_cfg3.adp_supp);
	dev_dbg(dwc->dev, "hw3.otg_enable_hsic = %u\n", dwc->hw_cfg3.otg_enable_hsic);
	dev_dbg(dwc->dev, "hw3.bc_support = %u\n", dwc->hw_cfg3.bc_support);
	dev_dbg(dwc->dev, "hw3.otg_lpm_en = %u\n", dwc->hw_cfg3.otg_lpm_en);
	dev_dbg(dwc->dev, "hw3.dfifo_depth = %u\n", dwc->hw_cfg3.dfifo_depth);

	dev_dbg(dwc->dev, "hw4.num_dev_perio_in_ep = %u\n", dwc->hw_cfg4.num_dev_perio_in_ep);
	dev_dbg(dwc->dev, "hw4.power_optimiz = %u\n", dwc->hw_cfg4.power_optimiz);
	dev_dbg(dwc->dev, "hw4.min_ahb_freq = %u\n", dwc->hw_cfg4.min_ahb_freq);
	dev_dbg(dwc->dev, "hw4.part_power_down = %u\n", dwc->hw_cfg4.part_power_down);
	dev_dbg(dwc->dev, "hw4.reserved = %u\n", dwc->hw_cfg4.reserved);
	dev_dbg(dwc->dev, "hw4.utmi_phy_data_width = %u\n", dwc->hw_cfg4.utmi_phy_data_width);
	dev_dbg(dwc->dev, "hw4.num_dev_mode_ctrl_ep = %u\n", dwc->hw_cfg4.num_dev_mode_ctrl_ep);
	dev_dbg(dwc->dev, "hw4.iddig_filt_en = %u\n", dwc->hw_cfg4.iddig_filt_en);
	dev_dbg(dwc->dev, "hw4.vbus_valid_filt_en = %u\n", dwc->hw_cfg4.vbus_valid_filt_en);
	dev_dbg(dwc->dev, "hw4.a_valid_filt_en = %u\n", dwc->hw_cfg4.a_valid_filt_en);
	dev_dbg(dwc->dev, "hw4.b_valid_filt_en = %u\n", dwc->hw_cfg4.b_valid_filt_en);
	dev_dbg(dwc->dev, "hw4.session_end_filt_en = %u\n", dwc->hw_cfg4.session_end_filt_en);
	dev_dbg(dwc->dev, "hw4.ded_fifo_en = %u\n", dwc->hw_cfg4.ded_fifo_en);
	dev_dbg(dwc->dev, "hw4.num_in_eps = %u\n", dwc->hw_cfg4.num_in_eps);
	dev_dbg(dwc->dev, "hw4.desc_dma = %u\n", dwc->hw_cfg4.desc_dma);
	dev_dbg(dwc->dev, "hw4.desc_dma_dyn = %u\n", dwc->hw_cfg4.desc_dma_dyn);

	dev_dbg(dwc->dev, "lpm.lpm_cap_en = %u\n", dwc->lpm_cfg.lpm_cap_en);
	dev_dbg(dwc->dev, "lpm.appl_resp = %u\n", dwc->lpm_cfg.appl_resp);
	dev_dbg(dwc->dev, "lpm.hird = %u\n", dwc->lpm_cfg.hird);
	dev_dbg(dwc->dev, "lpm.rem_wkup_en = %u\n", dwc->lpm_cfg.rem_wkup_en);
	dev_dbg(dwc->dev, "lpm.en_utmi_sleep = %u\n", dwc->lpm_cfg.en_utmi_sleep);
	dev_dbg(dwc->dev, "lpm.hird_thres = %u\n", dwc->lpm_cfg.hird_thres);
	dev_dbg(dwc->dev, "lpm.lpm_resp = %u\n", dwc->lpm_cfg.lpm_resp);
	dev_dbg(dwc->dev, "lpm.prt_sleep_sts = %u\n", dwc->lpm_cfg.prt_sleep_sts);
	dev_dbg(dwc->dev, "lpm.sleep_state_resumeok = %u\n", dwc->lpm_cfg.sleep_state_resumeok);
	dev_dbg(dwc->dev, "lpm.lpm_chan_index = %u\n", dwc->lpm_cfg.lpm_chan_index);
	dev_dbg(dwc->dev, "lpm.retry_count = %u\n", dwc->lpm_cfg.retry_count);
	dev_dbg(dwc->dev, "lpm.send_lpm = %u\n", dwc->lpm_cfg.send_lpm);
	dev_dbg(dwc->dev, "lpm.retry_count_sts = %u\n", dwc->lpm_cfg.retry_count_sts);
	dev_dbg(dwc->dev, "lpm.reserved28_29 = %u\n", dwc->lpm_cfg.reserved28_29);
	dev_dbg(dwc->dev, "lpm.hsic_connect = %u\n", dwc->lpm_cfg.hsic_connect);
	dev_dbg(dwc->dev, "lpm.inv_sel_hsic = %u\n", dwc->lpm_cfg.inv_sel_hsic);
}

static void dwc2xx_hcd_debug(struct usb_hcd *hcd)
{
	WARN_ON(1);
	dwc2xx_hcd_dump_regs(hcd);
	dwc2xx_hcd_dump_config(hcd);
}

#endif
