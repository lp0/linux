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
#include <linux/cnt32_to_63.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>

#include <linux/version.h>
#include <linux/clkdev.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/sched_clock.h>

#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include "bcm2708.h"
#include "armctrl.h"
#include "clock.h"

/* command line parameters */
static unsigned boardrev, serial;

static struct map_desc bcm2708_io_desc[] __initdata = {
	{
		.virtual = IO_ADDRESS(BCM2708_PERI_BASE),
		.pfn = __phys_to_pfn(BCM2708_PERI_BASE),
		.length = SZ_16M,
		.type = MT_DEVICE
	},
	{
		.virtual = IO_ADDRESS(UART0_BASE),
		.pfn = __phys_to_pfn(UART0_BASE),
		.length = SZ_4K,
		.type = MT_DEVICE
	}
};

void __init bcm2708_map_io(void)
{
	iotable_init(bcm2708_io_desc, ARRAY_SIZE(bcm2708_io_desc));
}

int __init bcm_of_irq_init(struct device_node *node,
		struct device_node *parent)
{
	void __iomem *base = of_iomap(node, 0);
	if (!base)
		panic("unable to map intc cpu registers\n");

	irq_domain_add_legacy(node, NR_IRQS, 0, 0, &irq_domain_simple_ops, NULL);
	armctrl_init(base, 0, 0, 0);
   return 0;
}

static const struct of_device_id irq_of_match[] __initconst = {
	{ .compatible = "broadcom,bcm2708-armctrl-ic", .data = bcm_of_irq_init }
};

void __init bcm2708_init_irq(void)
{
	of_irq_init(irq_of_match);
}

/*
 * These are fixed clocks (and device tree doesn't support clk!).
 */
static struct clk apb_pclk = {
	.rate = APB_CLOCK
};

static struct clk_lookup lookups[] = {
	{
		.con_id = "apb_pclk",
		.clk = &apb_pclk,
	},
	{
		.dev_id = "20201000.uart",
		.clk = &apb_pclk,
	}
};


// The STC is a free running counter that increments at the rate of 1MHz
#define STC_FREQ_HZ 1000000

static cycle_t stc_read_cycles(struct clocksource *cs)
{
	/* STC: a free running counter that increments at the rate of 1MHz */
	return (cycle_t)readl(__io_address(ST_BASE+0x04));
}

static struct clocksource clocksource_stc = {
	.name	= "stc",
	.rating	= 300,
	.read	= stc_read_cycles,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

unsigned long frc_clock_ticks32(void)
{
	return (unsigned long)stc_read_cycles(&clocksource_stc);
}

static void __init bcm2708_clocksource_init(void)
{
	// calculate .shift and .mult values and register clocksource
	if (clocksource_register_hz(&clocksource_stc, STC_FREQ_HZ)) {
		printk(KERN_ERR "timer: failed to initialize clock "
			"source %s\n", clocksource_stc.name);
	}
}

u32 notrace bcm2708_read_sched_clock(void)
{
	return clocksource_stc.read(&clocksource_stc);
}

void __init bcm2708_init(void)
{
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);

	system_rev = boardrev;
	system_serial_low = serial;

	ret = of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	printk("BCM2708 devicetree: %d\n", ret);
	BUG_ON(ret != 0);
}

#define TIMER_PERIOD 10000	/* HZ in microsecs */

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	unsigned long stc;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		stc = readl(__io_address(ST_BASE + 0x04));
		writel(stc + TIMER_PERIOD, __io_address(ST_BASE + 0x18)); /* stc3 */
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;
	default:
		printk(KERN_ERR "timer_set_mode: unhandled mode: %d\n",
			(int)mode);
		break;
	}
}

static int timer_set_next_event(unsigned long cycles,
				struct clock_event_device *unused)
{
	unsigned long stc;

	stc = readl(__io_address(ST_BASE + 0x04));
	writel(stc + cycles, __io_address(ST_BASE + 0x18)); /* stc3 */
	return 0;
}

static struct clock_event_device timer0_clockevent = {
	.name = "timer0",
	.shift = 32,
	.features = CLOCK_EVT_FEAT_ONESHOT,
	.set_mode = timer_set_mode,
	.set_next_event = timer_set_next_event,
};

/*
 * IRQ handler for the timer
 */
static irqreturn_t bcm2708_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &timer0_clockevent;

	writel(1 << 3, __io_address(ST_BASE + 0x00));	/* stcs clear timer int */

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction bcm2708_timer_irq = {
	.name = "BCM2708 Timer Tick",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = bcm2708_timer_interrupt,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init bcm2708_timer_init(void)
{
	/* init high res timer */
	bcm2708_clocksource_init();

	/*
	 * Initialise to a known state (all timers off)
	 */
	writel(0, __io_address(ARM_T_CONTROL));
	/*
	 * Make irqs happen for the system timer
	 */
	setup_irq(IRQ_TIMER3, &bcm2708_timer_irq);

	setup_sched_clock(bcm2708_read_sched_clock, 32, STC_FREQ_HZ);

	timer0_clockevent.mult =
		div_sc(STC_FREQ_HZ, NSEC_PER_SEC, timer0_clockevent.shift);
	timer0_clockevent.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &timer0_clockevent);
	timer0_clockevent.min_delta_ns =
		clockevent_delta2ns(0xf, &timer0_clockevent);

	timer0_clockevent.cpumask = cpumask_of(0);
	clockevents_register_device(&timer0_clockevent);
}

struct sys_timer bcm2708_timer = {
	.init = bcm2708_timer_init
};

static const char *bcm2708_compat[] __initdata = {
	"broadcom,bcm2708",
	NULL
};

MACHINE_START(BCM2708, "BCM2708")
	/* Maintainer: Broadcom Europe Ltd. */
	.init_machine = bcm2708_init,
	.map_io = bcm2708_map_io,
	.init_irq = bcm2708_init_irq,
	.timer = &bcm2708_timer,
	.dt_compat = bcm2708_compat
MACHINE_END
module_param(boardrev, uint, 0644);
module_param(serial, uint, 0644);
