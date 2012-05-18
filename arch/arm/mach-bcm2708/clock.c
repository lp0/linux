/*
 *  Copyright (C) 2010 Broadcom
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/clk.h>
#include <linux/clk-private.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>

#include "clock.h"

/*
 * These are fixed clocks (and device tree doesn't support clk!).
 *
 * They're probably not all root clocks and it may be possible to
 * turn them on and off but until this is mapped out better it's
 * the only way they can be used.
 */
DEFINE_CLK_FIXED_RATE(sys_pclk,   CLK_IS_ROOT, 250000000, 0);
DEFINE_CLK_FIXED_RATE(apb_pclk,   CLK_IS_ROOT, 126000000, 0);
DEFINE_CLK_FIXED_RATE(uart0_pclk, CLK_IS_ROOT,   3000000, 0);
DEFINE_CLK_FIXED_RATE(uart1_pclk, CLK_IS_ROOT, 125000000, 0);

static struct clk_lookup lookups[] = {
	{ .con_id = "sys_pclk", .clk = &sys_pclk },
	{ .con_id = "apb_pclk", .clk = &apb_pclk },
	{ .dev_id = "20201000.uart0", .clk = &uart0_pclk },
	{ .dev_id = "20215000.uart1", .clk = &uart1_pclk }
};

void __init bcm2708_init_clocks(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lookups); i++) {
		__clk_init(NULL, lookups[i].clk);
		clkdev_add(&lookups[i]);
	}
}
