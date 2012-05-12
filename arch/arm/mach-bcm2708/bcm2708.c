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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/syscore_ops.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>

#include <linux/version.h>
#include <linux/clkdev.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/exception.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <asm/hardware/timer-sp.h>

#include "clock.h"
#include "irq.h"

/* command line parameters */
static unsigned boardrev, serial;

static struct map_desc bcm2708_io_desc[] __initdata = {
	{
		.virtual = IO_ADDRESS(BCM2708_PERI_BASE),
		.pfn = __phys_to_pfn(BCM2708_PERI_BASE),
		.length = SZ_16M,
		.type = MT_DEVICE
	}
};

void __init bcm2708_map_io(void)
{
	iotable_init(bcm2708_io_desc, ARRAY_SIZE(bcm2708_io_desc));
}

static struct of_device_id irq_of_match[] __initconst = {
	{ .compatible = "broadcom,bcm2708-armctrl-ic", .data = armctrl_of_init }
};

void __init bcm2708_init_irq(void)
{
	of_irq_init(irq_of_match);
}

/*
 * These are fixed clocks (and device tree doesn't support clk!).
 */
static struct clk uart0_pclk = {
	.rate = UART0_CLOCK
};
static struct clk uart1_pclk = {
	.rate = UART1_CLOCK
};

static struct clk_lookup lookups[] = {
	{
		.dev_id = "20201000.uart0",
		.clk = &uart0_pclk
	},
	{
		.dev_id = "20215000.uart1",
		.clk = &uart1_pclk
	}
};

void __init bcm2708_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);

	system_rev = boardrev;
	system_serial_low = serial;

	ret = of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	if (ret) {
		printk(KERN_ERR "BCM2708 devicetree: %d\n", ret);
		BUG();
	}
}

static struct of_device_id sp804_match[] __initconst = {
	{
		.compatible = "arm,sp804"
	},
	{}
};

static void __init bcm2708_timer_init(void)
{
	struct device_node *node;

	/* Disable the SP804 */
	for_each_matching_node(node, sp804_match) {
		sp804_disable(of_iomap(node, 0));
	}

	clockevent_mmio_dt_init();
}

struct sys_timer bcm2708_timer = {
	.init = bcm2708_timer_init
};

static const char *bcm2708_compat[] = {
	"broadcom,bcm2708",
	NULL
};

MACHINE_START(BCM2708, "BCM2708")
	/* Maintainer: Broadcom Europe Ltd. */
	.init_machine = bcm2708_init,
	.map_io = bcm2708_map_io,
	.init_irq = bcm2708_init_irq,
	.handle_irq = armctrl_handle_irq,
	.timer = &bcm2708_timer,
	.dt_compat = bcm2708_compat
MACHINE_END
module_param(boardrev, uint, 0644);
module_param(serial, uint, 0644);
