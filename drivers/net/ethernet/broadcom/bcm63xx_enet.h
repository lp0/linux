#ifndef BCM63XX_ENET_H_
#define BCM63XX_ENET_H_

#include <linux/bcm63xx_dev_enet.h>
#include <linux/bcm63xx_iudma.h>
#include <linux/types.h>
#include <linux/mii.h>
#include <linux/mutex.h>
#include <linux/phy.h>
#include <linux/platform_device.h>

/* default number of descriptor */
#define BCMENET_DEF_RX_DESC	16
#define BCMENET_DEF_TX_DESC	16

/* maximum number of descriptor */
#define BCMENET_MAX_RX_DESC	8192
#define BCMENET_MAX_TX_DESC	8192

/* maximum burst len for dma (4 bytes unit) */
#define BCMENET_DMA_MAXBURST	16
#define BCMENETSW_DMA_MAXBURST	8

/* tx transmit threshold (4 bytes unit), fifo is 256 bytes, the value
 * must be low enough so that a DMA transfer of above burst length can
 * not overflow the fifo  */
#define BCMENET_TX_FIFO_TRESH	32

/*
 * hardware maximum rx/tx packet size including FCS, max mtu is
 * actually 2047, but if we set max rx size register to 2047 we won't
 * get overflow information if packet size is 2048 or above
 */
#define BCMENET_MAX_MTU		2046


#define BCMENET_MAX_CLKS	3


/* Receiver Configuration register */
#define ENET_RXCFG_REG			0x0
#define ENET_RXCFG_ALLMCAST_SHIFT	1
#define ENET_RXCFG_ALLMCAST_MASK	(1 << ENET_RXCFG_ALLMCAST_SHIFT)
#define ENET_RXCFG_PROMISC_SHIFT	3
#define ENET_RXCFG_PROMISC_MASK		(1 << ENET_RXCFG_PROMISC_SHIFT)
#define ENET_RXCFG_LOOPBACK_SHIFT	4
#define ENET_RXCFG_LOOPBACK_MASK	(1 << ENET_RXCFG_LOOPBACK_SHIFT)
#define ENET_RXCFG_ENFLOW_SHIFT		5
#define ENET_RXCFG_ENFLOW_MASK		(1 << ENET_RXCFG_ENFLOW_SHIFT)

/* Receive Maximum Length register */
#define ENET_RXMAXLEN_REG		0x4
#define ENET_RXMAXLEN_SHIFT		0
#define ENET_RXMAXLEN_MASK		(0x7ff << ENET_RXMAXLEN_SHIFT)

/* Transmit Maximum Length register */
#define ENET_TXMAXLEN_REG		0x8
#define ENET_TXMAXLEN_SHIFT		0
#define ENET_TXMAXLEN_MASK		(0x7ff << ENET_TXMAXLEN_SHIFT)

/* MII Status/Control register */
#define ENET_MIISC_REG			0x10
#define ENET_MIISC_MDCFREQDIV_SHIFT	0
#define ENET_MIISC_MDCFREQDIV_MASK	(0x7f << ENET_MIISC_MDCFREQDIV_SHIFT)
#define ENET_MIISC_PREAMBLEEN_SHIFT	7
#define ENET_MIISC_PREAMBLEEN_MASK	(1 << ENET_MIISC_PREAMBLEEN_SHIFT)

/* MII Data register */
#define ENET_MIIDATA_REG		0x14
#define ENET_MIIDATA_DATA_SHIFT		0
#define ENET_MIIDATA_DATA_MASK		(0xffff << ENET_MIIDATA_DATA_SHIFT)
#define ENET_MIIDATA_TA_SHIFT		16
#define ENET_MIIDATA_TA_MASK		(0x3 << ENET_MIIDATA_TA_SHIFT)
#define ENET_MIIDATA_REG_SHIFT		18
#define ENET_MIIDATA_REG_MASK		(0x1f << ENET_MIIDATA_REG_SHIFT)
#define ENET_MIIDATA_PHYID_SHIFT	23
#define ENET_MIIDATA_PHYID_MASK		(0x1f << ENET_MIIDATA_PHYID_SHIFT)
#define ENET_MIIDATA_OP_READ_MASK	(0x6 << 28)
#define ENET_MIIDATA_OP_WRITE_MASK	(0x5 << 28)

/* Ethernet Interrupt Mask register */
#define ENET_IRMASK_REG			0x18

/* Ethernet Interrupt register */
#define ENET_IR_REG			0x1c
#define ENET_IR_MII			(1 << 0)
#define ENET_IR_MIB			(1 << 1)
#define ENET_IR_FLOWC			(1 << 2)

/* Ethernet Control register */
#define ENET_CTL_REG			0x2c
#define ENET_CTL_ENABLE_SHIFT		0
#define ENET_CTL_ENABLE_MASK		(1 << ENET_CTL_ENABLE_SHIFT)
#define ENET_CTL_DISABLE_SHIFT		1
#define ENET_CTL_DISABLE_MASK		(1 << ENET_CTL_DISABLE_SHIFT)
#define ENET_CTL_SRESET_SHIFT		2
#define ENET_CTL_SRESET_MASK		(1 << ENET_CTL_SRESET_SHIFT)
#define ENET_CTL_EPHYSEL_SHIFT		3
#define ENET_CTL_EPHYSEL_MASK		(1 << ENET_CTL_EPHYSEL_SHIFT)

/* Transmit Control register */
#define ENET_TXCTL_REG			0x30
#define ENET_TXCTL_FD_SHIFT		0
#define ENET_TXCTL_FD_MASK		(1 << ENET_TXCTL_FD_SHIFT)

/* Transmit Watermask register */
#define ENET_TXWMARK_REG		0x34
#define ENET_TXWMARK_WM_SHIFT		0
#define ENET_TXWMARK_WM_MASK		(0x3f << ENET_TXWMARK_WM_SHIFT)

/* MIB Control register */
#define ENET_MIBCTL_REG			0x38
#define ENET_MIBCTL_RDCLEAR_SHIFT	0
#define ENET_MIBCTL_RDCLEAR_MASK	(1 << ENET_MIBCTL_RDCLEAR_SHIFT)

/* Perfect Match Data Low register */
#define ENET_PML_REG(x)			(0x58 + (x) * 8)
#define ENET_PMH_REG(x)			(0x5c + (x) * 8)
#define ENET_PMH_DATAVALID_SHIFT	16
#define ENET_PMH_DATAVALID_MASK		(1 << ENET_PMH_DATAVALID_SHIFT)

/* MIB register */
#define ENET_MIB_REG(x)			(0x200 + (x) * 4)
#define ENET_MIB_REG_COUNT		55

/* Port traffic control */
#define ENETSW_PTCTRL_REG(x)		(0x0 + (x))
#define ENETSW_PTCTRL_RXDIS_MASK	(1 << 0)
#define ENETSW_PTCTRL_TXDIS_MASK	(1 << 1)
#define ENETSW_PTCTRL_NO_STP		0x00
#define ENETSW_PTCTRL_STP_DISABLED	0x20
#define ENETSW_PTCTRL_STP_BLOCKING	0x40
#define ENETSW_PTCTRL_STP_LISTENING	0x60
#define ENETSW_PTCTRL_STP_LEARNING	0x80
#define ENETSW_PTCTRL_STP_FORWARDING	0xA0

/* Switch mode register */
#define ENETSW_SWMODE_REG		(0xb)
#define ENETSW_SWMODE_FWD_EN_MASK	(1 << 1)
#define ENETSW_SWMODE_BCAST_MIPS_ONLY	(1 << 5)

/* IMP override Register */
#define ENETSW_IMPOV_REG		(0xe)
#define ENETSW_IMPOV_FORCE_MASK		(1 << 7)
#define ENETSW_IMPOV_TXFLOW_MASK	(1 << 5)
#define ENETSW_IMPOV_RXFLOW_MASK	(1 << 4)
#define ENETSW_IMPOV_1000_MASK		(1 << 3)
#define ENETSW_IMPOV_100_MASK		(1 << 2)
#define ENETSW_IMPOV_FDX_MASK		(1 << 1)
#define ENETSW_IMPOV_LINKUP_MASK	(1 << 0)

/* Port config registers */
#define ENETSW_PORT_FORWARD_MODE_REG	(0x21)
#define ENETSW_PORT_FORWARD_MCAST	(1 << 7)
#define ENETSW_PORT_FORWARD_UCAST	(1 << 6)
#define ENETSW_PORT_FORWARD_IP_MCAST	(1 << 1)
#define ENETSW_PORT_ENABLE_REG		(0x23)
#define ENETSW_PORT_PROTECTED_MAP_REG	(0x24)
#define ENETSW_PORT_WAN_MAP_REG		(0x26)
#define ENETSW_PORT_UCAST_LOOKUP_FAIL	(0x32)
#define ENETSW_PORT_MCAST_LOOKUP_FAIL	(0x34)
#define ENETSW_PORT_IP_MC_LOOKUP_FAIL	(0x36)
#define ENETSW_PORT_DISABLE_LEARNING	(0x3c) /* u16 bitmask */

/* Port override Register */
#define ENETSW_PORTOV_REG(x)		(0x58 + (x))
#define ENETSW_PORTOV_ENABLE_MASK	(1 << 6)
#define ENETSW_PORTOV_TXFLOW_MASK	(1 << 5)
#define ENETSW_PORTOV_RXFLOW_MASK	(1 << 4)
#define ENETSW_PORTOV_1000_MASK		(1 << 3)
#define ENETSW_PORTOV_100_MASK		(1 << 2)
#define ENETSW_PORTOV_FDX_MASK		(1 << 1)
#define ENETSW_PORTOV_LINKUP_MASK	(1 << 0)

/* MDIO control register */
#define ENETSW_MDIOC_REG		(0xb0)
#define ENETSW_MDIOC_EXT_MASK		(1 << 16)
#define ENETSW_MDIOC_REG_SHIFT		20
#define ENETSW_MDIOC_PHYID_SHIFT	25
#define ENETSW_MDIOC_RD_MASK		(1 << 30)
#define ENETSW_MDIOC_WR_MASK		(1 << 31)

/* MDIO data register */
#define ENETSW_MDIOD_REG		(0xb4)

/* Global Management Configuration Register */
#define ENETSW_GMCR_REG			(0x200)
#define ENETSW_GMCR_RST_MIB_MASK	(1 << 0)

/* MIB register */
#define ENETSW_MIB_REG(x)		(0x2800 + (x) * 4)
#define ENETSW_MIB_REG_COUNT		47

/* Port based VLAN */
#define ENETSW_PORT_BASED_VLAN(x)	(0x3100 + (x) * 2) /* u16 bitmask */

/* Jumbo control register port mask register */
#define ENETSW_JMBCTL_PORT_REG		(0x4004)

/* Jumbo control mib good frame register */
#define ENETSW_JMBCTL_MAXSIZE_REG	(0x4008)


/*
 * MIB Counters register definitions
*/
#define ETH_MIB_TX_GD_OCTETS			0
#define ETH_MIB_TX_GD_PKTS			1
#define ETH_MIB_TX_ALL_OCTETS			2
#define ETH_MIB_TX_ALL_PKTS			3
#define ETH_MIB_TX_BRDCAST			4
#define ETH_MIB_TX_MULT				5
#define ETH_MIB_TX_64				6
#define ETH_MIB_TX_65_127			7
#define ETH_MIB_TX_128_255			8
#define ETH_MIB_TX_256_511			9
#define ETH_MIB_TX_512_1023			10
#define ETH_MIB_TX_1024_MAX			11
#define ETH_MIB_TX_JAB				12
#define ETH_MIB_TX_OVR				13
#define ETH_MIB_TX_FRAG				14
#define ETH_MIB_TX_UNDERRUN			15
#define ETH_MIB_TX_COL				16
#define ETH_MIB_TX_1_COL			17
#define ETH_MIB_TX_M_COL			18
#define ETH_MIB_TX_EX_COL			19
#define ETH_MIB_TX_LATE				20
#define ETH_MIB_TX_DEF				21
#define ETH_MIB_TX_CRS				22
#define ETH_MIB_TX_PAUSE			23

#define ETH_MIB_RX_GD_OCTETS			32
#define ETH_MIB_RX_GD_PKTS			33
#define ETH_MIB_RX_ALL_OCTETS			34
#define ETH_MIB_RX_ALL_PKTS			35
#define ETH_MIB_RX_BRDCAST			36
#define ETH_MIB_RX_MULT				37
#define ETH_MIB_RX_64				38
#define ETH_MIB_RX_65_127			39
#define ETH_MIB_RX_128_255			40
#define ETH_MIB_RX_256_511			41
#define ETH_MIB_RX_512_1023			42
#define ETH_MIB_RX_1024_MAX			43
#define ETH_MIB_RX_JAB				44
#define ETH_MIB_RX_OVR				45
#define ETH_MIB_RX_FRAG				46
#define ETH_MIB_RX_DROP				47
#define ETH_MIB_RX_CRC_ALIGN			48
#define ETH_MIB_RX_UND				49
#define ETH_MIB_RX_CRC				50
#define ETH_MIB_RX_ALIGN			51
#define ETH_MIB_RX_SYM				52
#define ETH_MIB_RX_PAUSE			53
#define ETH_MIB_RX_CNTRL			54


/*
 * SW MIB Counters register definitions
*/
#define ETHSW_MIB_TX_ALL_OCT			0
#define ETHSW_MIB_TX_DROP_PKTS			2
#define ETHSW_MIB_TX_QOS_PKTS			3
#define ETHSW_MIB_TX_BRDCAST			4
#define ETHSW_MIB_TX_MULT			5
#define ETHSW_MIB_TX_UNI			6
#define ETHSW_MIB_TX_COL			7
#define ETHSW_MIB_TX_1_COL			8
#define ETHSW_MIB_TX_M_COL			9
#define ETHSW_MIB_TX_DEF			10
#define ETHSW_MIB_TX_LATE			11
#define ETHSW_MIB_TX_EX_COL			12
#define ETHSW_MIB_TX_PAUSE			14
#define ETHSW_MIB_TX_QOS_OCT			15

#define ETHSW_MIB_RX_ALL_OCT			17
#define ETHSW_MIB_RX_UND			19
#define ETHSW_MIB_RX_PAUSE			20
#define ETHSW_MIB_RX_64				21
#define ETHSW_MIB_RX_65_127			22
#define ETHSW_MIB_RX_128_255			23
#define ETHSW_MIB_RX_256_511			24
#define ETHSW_MIB_RX_512_1023			25
#define ETHSW_MIB_RX_1024_1522			26
#define ETHSW_MIB_RX_OVR			27
#define ETHSW_MIB_RX_JAB			28
#define ETHSW_MIB_RX_ALIGN			29
#define ETHSW_MIB_RX_CRC			30
#define ETHSW_MIB_RX_GD_OCT			31
#define ETHSW_MIB_RX_DROP			33
#define ETHSW_MIB_RX_UNI			34
#define ETHSW_MIB_RX_MULT			35
#define ETHSW_MIB_RX_BRDCAST			36
#define ETHSW_MIB_RX_SA_CHANGE			37
#define ETHSW_MIB_RX_FRAG			38
#define ETHSW_MIB_RX_OVR_DISC			39
#define ETHSW_MIB_RX_SYM			40
#define ETHSW_MIB_RX_QOS_PKTS			41
#define ETHSW_MIB_RX_QOS_OCT			42
#define ETHSW_MIB_RX_1523_2047			44
#define ETHSW_MIB_RX_2048_4095			45
#define ETHSW_MIB_RX_4096_8191			46
#define ETHSW_MIB_RX_8192_9728			47


struct bcm_enet_mib_counters {
	u64 tx_gd_octets;
	u32 tx_gd_pkts;
	u32 tx_all_octets;
	u32 tx_all_pkts;
	u32 tx_unicast;
	u32 tx_brdcast;
	u32 tx_mult;
	u32 tx_64;
	u32 tx_65_127;
	u32 tx_128_255;
	u32 tx_256_511;
	u32 tx_512_1023;
	u32 tx_1024_max;
	u32 tx_1523_2047;
	u32 tx_2048_4095;
	u32 tx_4096_8191;
	u32 tx_8192_9728;
	u32 tx_jab;
	u32 tx_drop;
	u32 tx_ovr;
	u32 tx_frag;
	u32 tx_underrun;
	u32 tx_col;
	u32 tx_1_col;
	u32 tx_m_col;
	u32 tx_ex_col;
	u32 tx_late;
	u32 tx_def;
	u32 tx_crs;
	u32 tx_pause;
	u64 rx_gd_octets;
	u32 rx_gd_pkts;
	u32 rx_all_octets;
	u32 rx_all_pkts;
	u32 rx_brdcast;
	u32 rx_unicast;
	u32 rx_mult;
	u32 rx_64;
	u32 rx_65_127;
	u32 rx_128_255;
	u32 rx_256_511;
	u32 rx_512_1023;
	u32 rx_1024_max;
	u32 rx_jab;
	u32 rx_ovr;
	u32 rx_frag;
	u32 rx_drop;
	u32 rx_crc_align;
	u32 rx_und;
	u32 rx_crc;
	u32 rx_align;
	u32 rx_sym;
	u32 rx_pause;
	u32 rx_cntrl;
};


struct bcm_enet_pkt {
	struct list_head node;
	struct sk_buff *skb;
	dma_addr_t buf;
	struct bcm63xx_iudma_context ctx;
};


struct bcm_enet_priv {

	/* mac id (from platform device id) */
	int mac_id;

	/* base remapped address of device */
	void __iomem *base;

	/* mac irq */
	int irq;

	/* rx and tx dma channel/packet queues */
	struct dma_chan *rx_dma;
	struct dma_chan *tx_dma;
	struct list_head rx_active;
	struct list_head rx_inactive;
	struct list_head tx_active;
	struct list_head tx_inactive;
	unsigned int rx_max_ring_size;
	unsigned int tx_max_ring_size;
	unsigned int rx_ring_size;
	unsigned int tx_ring_size;
	unsigned int rx_count;
	unsigned int rx_skb_size;
	unsigned int tx_count;
	spinlock_t rx_lock;
	spinlock_t tx_lock;
	bool rx_running;
	bool tx_running;

	/* maximum dma burst size */
	int dma_maxburst;

	/* platform dma config */
	struct bcm63xx_enet_platform_dma_data pd_rx_dma;
	struct bcm63xx_enet_platform_dma_data pd_tx_dma;

	/* used when rx skb allocation failed, so we defer rx queue
	 * refill */
	struct timer_list rx_timeout;

	/* set if internal phy is ignored and external mii interface
	 * is selected */
	int use_external_mii;

	/* set if a phy is connected, phy address must be known,
	 * probing is not possible */
	int has_phy;
	int phy_id;

	/* set if connected phy has an associated irq */
	int has_phy_interrupt;
	int phy_interrupt;

	/* used when a phy is connected (phylib used) */
	struct mii_bus *mii_bus;
	struct phy_device *phydev;
	int old_link;
	int old_duplex;
	int old_pause;

	/* used when no phy is connected */
	int force_speed_100;
	int force_duplex_full;

	/* pause parameters */
	int pause_auto;
	int pause_rx;
	int pause_tx;

	/* stats */
	struct bcm_enet_mib_counters mib;

	/* after mib interrupt, mib registers update is done in this
	 * work queue */
	struct work_struct mib_update_task;

	/* lock mib update between userspace request and workqueue */
	struct mutex mib_update_lock;

	/* mac clock */
	struct clk *mac_clks[BCMENET_MAX_CLKS];

	/* phy clock if internal phy is used */
	struct clk *phy_clk;

	struct reset_control *reset;
	struct regulator *regulator;

	/* network device reference */
	struct net_device *net_dev;

	/* platform device reference */
	struct platform_device *pdev;

	/* maximum hardware transmit/receive size */
	unsigned int hw_mtu;

	bool enet_is_sw;

	/* port mapping for switch devices */
	int num_ports;
	struct bcm63xx_enetsw_port used_ports[ENETSW_MAX_PORT];
	int sw_port_link[ENETSW_MAX_PORT];

	/* used to poll switch port state */
	struct timer_list swphy_poll;
	spinlock_t enetsw_mdio_lock;
};


#endif /* ! BCM63XX_ENET_H_ */
