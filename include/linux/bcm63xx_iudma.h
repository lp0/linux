#ifndef LINUX_BCM63XX_IUDMA_H_
#define LINUX_BCM63XX_IUDMA_H_

#include <linux/types.h>

struct bcm63xx_iudma_context {
	u16 length;
	union {
		u16 control;
		u16 status;
/* control: */
	/* usbd: send zero length packet */
#define IUDMA_D_CTL_USBD_ZERO		BIT(0)
	/* enetsw: tx port 0-7 */
#define IUDMA_D_CTL_ENETSW_PORT(x)	BIT(x)
	/* enet: append CRC */
#define IUDMA_D_CTL_ENET_APPEND_CRC	BIT(8)
	/* enet: append BRCM tag */
#define IUDMA_D_CTL_ENET_APPEND_TAG	BIT(9)
	/* enet: tx priority 0-3 */
#define IUDMA_D_CTL_ENET_PRIO(x)	((x) << 10)

/* control/status: */
	/* first buffer in packet */
#define IUDMA_D_CS_SOP			BIT(13)
	/* last buffer in packet */
#define IUDMA_D_CS_EOP			BIT(14)
#define IUDMA_D_CS_ESOP_MASK		(IUDMA_D_CS_EOP | IUDMA_D_CS_SOP)

/* status: */
	/* enetsw: rx port 0-7 */
#define IUDMA_D_STA_ENETSW_PORT_SHIFT	8
#define IUDMA_D_STA_ENETSW_PORT_MASK	(0xf << IUDMA_D_STA_ENETSW_PORT_SHIFT)
#define IUDMA_D_STA_ENETSW_PORT(x)	(((x) & IUDMA_D_STA_ENETSW_PORT_MASK) >> IUDMA_D_STA_ENETSW_PORT_SHIFT)

	/* enet: packet errors */
#define	IUDMA_D_STA_ENET_OV		BIT(0)
#define	IUDMA_D_STA_ENET_CRC		BIT(1)
#define	IUDMA_D_STA_ENET_RXER		BIT(2)
#define	IUDMA_D_STA_ENET_OVSIZE		BIT(4)
#define	IUDMA_D_STA_ENET_UNDER		BIT(9)
#define	IUDMA_D_STA_ENET_ERR_MASK	(IUDMA_D_STA_ENET_OV | \
					IUDMA_D_STA_ENET_CRC | \
					IUDMA_D_STA_ENET_RXER | \
					IUDMA_D_STA_ENET_OVSIZE | \
					IUDMA_D_STA_ENET_UNDER)
	};
};

struct bcm63xx_iudma_platform_data {
	unsigned int n_channels;
	unsigned int n_requests;
	unsigned int dmac_offset;
	unsigned int dmas_offset;
};

#endif
