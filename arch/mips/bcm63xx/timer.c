/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <bcm63xx_cpu.h>
#include <bcm63xx_io.h>
#include <bcm63xx_regs.h>

static int bcm63xx_timer_init(void)
{
	u32 reg;

	/* Disable all timers so that they won't interfere with use of the
	 * timer interrupt by the watchdog.
	 */
	reg = bcm_timer_readl(TIMER_IRQSTAT_REG);
	reg &= ~TIMER_IRQSTAT_TIMER0_IR_EN;
	reg &= ~TIMER_IRQSTAT_TIMER1_IR_EN;
	reg &= ~TIMER_IRQSTAT_TIMER2_IR_EN;
	bcm_timer_writel(reg, TIMER_IRQSTAT_REG);

	return 0;
}

arch_initcall(bcm63xx_timer_init);
