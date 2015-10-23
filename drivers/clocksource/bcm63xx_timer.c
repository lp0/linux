/*
 * Copyright 2015 Simon Arlott
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
 *
 * Based on arch/mips/bcm63xx/timer.c:
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 *
 * Derived from cevt-r4k-bcm-pwr.c:
 * Copyright (c) 2009 Broadcom Corporation
 *
 * Based on bcm2835_timer.c:
 * Copyright 2012 Simon Arlott
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>

#include <asm/irq.h>

#define REG_IRQ_ENABLE		0x02
#define REG_IRQ_STATUS		0x03
#define REG_CONTROL(n)		(0x04 + (n) * 4)
#define REG_COUNT(n)		(0x10 + (n) * 4)
#define CONTROL_COUNTDOWN_MASK	(0x3fffffff)
#define CONTROL_RSTCNTCLR_MASK	(1 << 30)
#define CONTROL_ENABLE_MASK	(1 << 31)
#define VALID_BITS		30

#define NR_TIMER_IRQS	4
#define NR_TIMERS	3

/* Use two timers for clockevent (per cpu) */
#define EVT_TIMERS	2

/* Use one of the timers for clocksource */
#define CS_TIMER_ID	2

/* The watchdog timer has seperate control/remaining registers */
#define WDT_TIMER_ID	3

struct bcm63xx_timer_event {
	struct bcm63xx_timer *timer;
	int id;
	struct clock_event_device dev;
	struct irqaction act;
	bool oneshot;
};

struct bcm63xx_timer {
	raw_spinlock_t lock;
	void __iomem *base;
	unsigned int irq;
	struct irq_domain *domain;
	struct clk *clk;
	unsigned long ticks_per_jiffy;
	struct bcm63xx_timer_event event[EVT_TIMERS];
};

static void __iomem *system_clock __read_mostly;

static u64 notrace bcm63xx_sched_read(void)
{
	return __raw_readl(system_clock) & CONTROL_COUNTDOWN_MASK;
}

static inline void bcm63xx_timer_stop(struct bcm63xx_timer *timer, int id)
{
	__raw_writel(0, timer->base + REG_CONTROL(id));
	__raw_writel(0, timer->base + REG_COUNT(id));
	__raw_writeb(BIT(id), timer->base + REG_IRQ_STATUS);
}

static inline void bcm63xx_timer_start(struct bcm63xx_timer *timer, int id,
					unsigned long delay)
{
	u32 val;

	bcm63xx_timer_stop(timer, id);
	val = CONTROL_ENABLE_MASK | (delay & CONTROL_COUNTDOWN_MASK);
	__raw_writel(val, timer->base + REG_CONTROL(id));
}

static int bcm63xx_timer_shutdown(struct clock_event_device *evt_dev)
{
	struct bcm63xx_timer_event *event = container_of(evt_dev,
		struct bcm63xx_timer_event, dev);
	struct bcm63xx_timer *timer = event->timer;
	unsigned long flags;

	raw_spin_lock_irqsave(&timer->lock, flags);
	bcm63xx_timer_stop(timer, event->id);
	raw_spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}

static int bcm63xx_timer_periodic(struct clock_event_device *evt_dev)
{
	struct bcm63xx_timer_event *event = container_of(evt_dev,
		struct bcm63xx_timer_event, dev);
	struct bcm63xx_timer *timer = event->timer;
	unsigned long flags;

	raw_spin_lock_irqsave(&timer->lock, flags);
	bcm63xx_timer_start(timer, event->id, timer->ticks_per_jiffy);
	event->oneshot = false;
	raw_spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}

static int bcm63xx_timer_oneshot(unsigned long next_event,
				struct clock_event_device *evt_dev)
{
	struct bcm63xx_timer_event *event = container_of(evt_dev,
		struct bcm63xx_timer_event, dev);
	struct bcm63xx_timer *timer = event->timer;
	unsigned long flags;

//	pr_info("%s: %d %lu\n", __func__, event->id, next_event);

	if (next_event & CONTROL_COUNTDOWN_MASK)
		return -EINVAL;

	raw_spin_lock_irqsave(&timer->lock, flags);
	bcm63xx_timer_start(timer, event->id, next_event);
	event->oneshot = true;
	raw_spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}

static irqreturn_t bcm63xx_timer_event_interrupt(int irq, void *dev_id)
{
	struct bcm63xx_timer_event *event = dev_id;
	struct bcm63xx_timer *timer = event->timer;
	void (*event_handler)(struct clock_event_device *);
	unsigned long flags;

	raw_spin_lock_irqsave(&timer->lock, flags);
	if (event->oneshot) {
		/* Oneshot mode, stop the timer */
		__raw_writel(0, timer->base + REG_CONTROL(event->id));
	}
	raw_spin_unlock_irqrestore(&timer->lock, flags);

	event_handler = ACCESS_ONCE(event->dev.event_handler);
	if (event_handler)
		event_handler(&event->dev);

	return IRQ_HANDLED;
}

static void bcm63xx_timer_interrupt(struct irq_desc *desc)
{
	struct bcm63xx_timer *timer = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long pending;
	irq_hw_number_t hwirq;
	unsigned int irq;

	chained_irq_enter(chip, desc);

	pending = __raw_readb(timer->base + REG_IRQ_STATUS);
	pending &= __raw_readb(timer->base + REG_IRQ_ENABLE)
			| BIT(WDT_TIMER_ID); /* Watchdog can't be masked */

	for_each_set_bit(hwirq, &pending, NR_TIMER_IRQS) {
		irq = irq_linear_revmap(timer->domain, hwirq);
		if (irq)
			do_IRQ(irq);
		else
			spurious_interrupt();
	}

	chained_irq_exit(chip, desc);
}

static void bcm63xx_timer_unmask(struct irq_data *d)
{
	struct bcm63xx_timer *timer = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	u8 val;

	if (d->hwirq < NR_TIMERS) {
		raw_spin_lock_irqsave(&timer->lock, flags);
		val = __raw_readb(timer->base + REG_IRQ_ENABLE);
		val |= BIT(d->hwirq);
		__raw_writeb(val, timer->base + REG_IRQ_ENABLE);
		raw_spin_unlock_irqrestore(&timer->lock, flags);
	}
}

static void bcm63xx_timer_mask(struct irq_data *d)
{
	struct bcm63xx_timer *timer = irq_data_get_irq_chip_data(d);
	unsigned long flags;
	u32 val;

	if (d->hwirq < NR_TIMERS) {
		raw_spin_lock_irqsave(&timer->lock, flags);
		val = __raw_readb(timer->base + REG_IRQ_ENABLE);
		val &= ~BIT(d->hwirq);
		__raw_writeb(val, timer->base + REG_IRQ_ENABLE);
		raw_spin_unlock_irqrestore(&timer->lock, flags);
	}
}

static void bcm63xx_timer_eoi(struct irq_data *d)
{
	struct bcm63xx_timer *timer = irq_data_get_irq_chip_data(d);

	if (d->hwirq < NR_TIMERS)
		__raw_writeb(BIT(d->hwirq), timer->base + REG_IRQ_STATUS);
}

static struct irq_chip bcm63xx_timer_chip = {
	.name			= "bcm63xx-timer",
	.irq_mask		= bcm63xx_timer_mask,
	.irq_unmask		= bcm63xx_timer_unmask,
	.irq_eoi		= bcm63xx_timer_eoi,
};

static int bcm63xx_timer_map(struct irq_domain *d, unsigned int virq,
			     irq_hw_number_t hwirq)
{

	irq_set_chip_and_handler(virq, &bcm63xx_timer_chip,
		hwirq == WDT_TIMER_ID ? handle_simple_irq : handle_fasteoi_irq);
	irq_set_chip_data(virq, d->host_data);
	return 0;
}

static const struct irq_domain_ops bcm63xx_timer_domain_ops = {
	.xlate			= irq_domain_xlate_onecell,
	.map			= bcm63xx_timer_map,
};

static irqreturn_t ping(int irq, void *data)
{
	pr_info("ping\n");
	return IRQ_HANDLED;
}

static int __init bcm63xx_timer_init_irqchip(struct device_node *node,
				      struct device_node *parent)
{
	struct bcm63xx_timer *timer;
	int ret, i;

	timer = kzalloc(sizeof(*timer), GFP_KERNEL);
	if (!timer) {
		pr_err("unable to allocate timer struct\n");
		return -ENOMEM;
	}

	raw_spin_lock_init(&timer->lock);

	timer->irq = irq_of_parse_and_map(node, 0);
	if (!timer->irq) {
		pr_err("unable to map parent IRQ\n");
		ret = -EINVAL;
		goto free_timer;
	}

	timer->base = of_iomap(node, 0);
	if (!timer->base) {
		pr_err("unable to remap registers\n");
		ret = -ENOMEM;
		goto free_timer;
	}

	timer->domain = irq_domain_add_linear(node, NR_TIMER_IRQS,
					&bcm63xx_timer_domain_ops, timer);
	if (!timer->domain) {
		pr_err("unable to add IRQ domain");
		ret = -ENOMEM;
		goto unmap_io;
	}

	/* Mask all interrupts and stop all timers */
	__raw_writeb(0, timer->base + REG_IRQ_ENABLE);
	for (i = 0; i < NR_TIMERS; i++)
		bcm63xx_timer_stop(timer, i);

	irq_set_chained_handler_and_data(timer->irq,
					bcm63xx_timer_interrupt, timer);

	pr_info("registered BCM63xx-timer L2 intc at MMIO 0x%p (irq = %d, IRQs: %d)\n",
			timer->base, timer->irq, NR_TIMER_IRQS);
//	bcm63xx_timer_start(timer, CS_TIMER_ID, 50*1000*1000 - 1);
//	request_irq(irq_create_mapping(timer->domain, CS_TIMER_ID), ping, 0, "ping", NULL);
	return 0;

unmap_io:
	iounmap(timer->base);
free_timer:
	kfree(timer);
	return ret;
}

static void __init bcm63xx_timer_init_clocksource(struct device_node *node)
{
	struct bcm63xx_timer *timer;
	struct irq_domain *domain;
	u32 freq;
	int i;

	domain = irq_find_host(node);
	if (!domain) {
		pr_err("unable to find IRQ domain");
		return;
	}

	timer = domain->host_data;

	// TODO timer->clk

	if (of_property_read_u32(node, "clock-frequency", &freq)) {
		pr_err("unable to get clock frequency\n");
		return;
	}

	timer->ticks_per_jiffy = DIV_ROUND_UP(freq, HZ);

	/* Start a timer for the clocksource */
	bcm63xx_timer_start(timer, CS_TIMER_ID, CONTROL_COUNTDOWN_MASK);
	system_clock = timer->base + REG_COUNT(CS_TIMER_ID);
	sched_clock_register(bcm63xx_sched_read, VALID_BITS, freq);
	clocksource_mmio_init(system_clock, "BCM63xx", freq, 400, VALID_BITS,
			clocksource_mmio_raw_readl_up);

	/* Use the other timers for clockevents */
	for (i = 0; i < EVT_TIMERS; i++) {
		struct bcm63xx_timer_event *event = &timer->event[i];
		unsigned int irq = irq_create_mapping(timer->domain, i);

		event->id = i;
		event->timer = timer;

		event->dev.name = "BCM63xx";
		event->dev.rating = 400;
		event->dev.features = CLOCK_EVT_FEAT_ONESHOT
					| CLOCK_EVT_FEAT_PERIODIC;
		event->dev.set_next_event = bcm63xx_timer_oneshot;
		event->dev.set_state_periodic = bcm63xx_timer_periodic;
		event->dev.set_state_shutdown = bcm63xx_timer_shutdown;
		event->dev.cpumask = cpu_all_mask;
		event->dev.cpumask = cpumask_of(0); // FIXME
		event->dev.irq = timer->irq;

		event->act.name = node->name;
		event->act.flags = IRQF_TIMER;
		event->act.dev_id = event;
		event->act.handler = bcm63xx_timer_event_interrupt;

		if (!irq || setup_irq(irq, &event->act)) {
			pr_err("unable to set up timer IRQ %d\n", i);
			continue;
		}

		clockevents_config_and_register(&event->dev, freq, 0x300,
			CONTROL_COUNTDOWN_MASK);
		pr_info("registered timer %d (irq = %d)\n", event->id, irq);
	}
}

IRQCHIP_DECLARE(bcm63xx_irqchip, "brcm,bcm63xx-timer", bcm63xx_timer_init_irqchip);
CLOCKSOURCE_OF_DECLARE(bcm63xx_clocksource, "brcm,bcm63xx-timer", bcm63xx_timer_init_clocksource);
