/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 * Copyright 2015 Simon Arlott
 */

#include <linux/types.h>
#include <linux/serial_bcm63xx.h>

#define BCM_REGS_VA(x)		((void __iomem *)(x))

#define bcm_uart_readl(a)	(*(volatile unsigned int *) BCM_REGS_VA(CONFIG_BMIPS_EP_BCM63XX_UART_BASE + (a)))
#define bcm_uart_writel(v, a)	(*(volatile unsigned int *) BCM_REGS_VA(CONFIG_BMIPS_EP_BCM63XX_UART_BASE + (a)) = (v))

static void wait_xfered(void)
{
	unsigned int val;

	/* wait for any previous char to be transmitted */
	do {
		val = bcm_uart_readl(UART_IR_REG);
		if (val & UART_IR_STAT(UART_IR_TXEMPTY))
			break;
	} while (1);
}

void prom_putchar(char c)
{
	wait_xfered();
	bcm_uart_writel(c, UART_FIFO_REG);
	wait_xfered();
}
