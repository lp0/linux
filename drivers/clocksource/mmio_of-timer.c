/*
 * Generic MMIO clockevent support (Device Tree)
 *
 * Copyright 2012 Simon Arlott
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

#include <linux/bitops.h>
#include <linux/clockchips.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/irq.h>

#include "mmio_of.h"

extern void mmio_of_clock_free(struct mmio_of_clock *data);
extern struct mmio_of_clock *mmio_of_clock_read(struct device_node *node);

static void mmio_of_timer_free(struct mmio_of_timer *data)
{
	if (data->clock)
		mmio_of_clock_free(data->clock);
	if (data->compare)
		iounmap(data->compare);
	kfree(data);
}

static bool mmio_of_timer_getw(struct mmio_of_timer *dev);
static int mmio_of_timer_setw(struct mmio_of_timer *dev, unsigned int value);
static void mmio_of_timer_clearw(struct mmio_of_timer *dev);

static bool mmio_of_timer_getl(struct mmio_of_timer *dev);
static int mmio_of_timer_setl(struct mmio_of_timer *dev, unsigned int value);
static void mmio_of_timer_clearl(struct mmio_of_timer *dev);

static void clockevent_mmio_of_set_mode(enum clock_event_mode mode,
	struct clock_event_device *evt_dev);
static int clockevent_mmio_of_set_next_event(unsigned long event,
	struct clock_event_device *evt_dev);

static struct mmio_of_timer __devinit *mmio_of_timer_read(
	struct device_node *node)
{
	struct mmio_of_timer *data;
	int compare_sz, control_sz, ret;

	if (node == NULL)
		return ERR_PTR(-EINVAL);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return ERR_PTR(-ENOMEM);

	if (of_address_to_resource(node, 0, &data->rcompare)) {
		ret = -ENXIO;
		goto err;
	}

	/* we need to access the clock to set oneshot events */
	data->clock = mmio_of_clock_read(of_get_parent(node));
	if (IS_ERR(data->clock)) {
		ret = PTR_ERR(data->clock);
		data->clock = NULL;
		goto err;
	}
	data->cs.reg = data->clock->value;

	data->compare = ioremap(data->rcompare.start,
		resource_size(&data->rcompare));
	compare_sz = resource_size(&data->rcompare) * 8;
	if (data->clock->control == NULL)
		control_sz = 0;
	else
		control_sz = resource_size(&data->clock->rcontrol) * 8;

	if (control_sz != 16 && control_sz != 32) {
		ret = -EINVAL;
		goto err;
	}

	data->irq = irq_of_parse_and_map(node, 0);
	data->cpu = data->index = 0;
	data->rating = data->clock->rating;
	of_property_read_u32(node, "cpu", &data->cpu);
	of_property_read_u32(node, "index", &data->index);
	of_property_read_u32(node, "rating", &data->rating);

	if (data->index >= control_sz) {
		ret = -EINVAL;
		goto err;
	}

	data->min_delta = 1;
	if (compare_sz == 16) {
		data->max_delta = 0xffff;
	} else {
		data->max_delta = 0xffffffff;
	}
	of_property_read_u32(node, "min-delta", &data->min_delta);
	of_property_read_u32(node, "max-delta", &data->max_delta);

	if (control_sz == 16) {
		data->get = mmio_of_timer_getw;
		data->set = mmio_of_timer_setw;
		data->clear = mmio_of_timer_clearw;
	} else {
		data->get = mmio_of_timer_getl;
		data->set = mmio_of_timer_setl;
		data->clear = mmio_of_timer_clearl;
	}

	data->ce.name = node->name;
	data->ce.rating = data->rating;
	data->ce.features = CLOCK_EVT_FEAT_ONESHOT;
	data->ce.set_mode = clockevent_mmio_of_set_mode;
	data->ce.set_next_event = clockevent_mmio_of_set_next_event;
	data->ce.cpumask = cpumask_of(data->cpu);

	return data;

err:
	mmio_of_timer_free(data);
	return ERR_PTR(ret);
}

static struct of_device_id mmio_of_timer_match[] __initconst = {
	{ .compatible = "mmio-timer" },
	{}
};

static bool mmio_of_timer_getw(struct mmio_of_timer *dev)
{
	return readw_relaxed(dev->clock->control) & BIT(dev->index);
}

static int mmio_of_timer_setw(struct mmio_of_timer *dev, unsigned int value)
{
	writew_relaxed((u16)value, dev->compare);
	return 0;
}

static void mmio_of_timer_clearw(struct mmio_of_timer *dev)
{
	writew_relaxed(BIT(dev->index), dev->clock->control);
}

static bool mmio_of_timer_getl(struct mmio_of_timer *dev)
{
	return readl_relaxed(dev->clock->control) & BIT(dev->index);
}

static int mmio_of_timer_setl(struct mmio_of_timer *dev, unsigned int value)
{
	writel_relaxed((u32)value, dev->compare);
	return 0;
}

static void mmio_of_timer_clearl(struct mmio_of_timer *dev)
{
	writel_relaxed(BIT(dev->index), dev->clock->control);
}

static void clockevent_mmio_of_set_mode(enum clock_event_mode mode,
	struct clock_event_device *evt_dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	default:
		WARN(1, "%s: unhandled event mode %d\n", __func__, mode);
		break;
	}
}

static int clockevent_mmio_of_set_next_event(unsigned long event,
	struct clock_event_device *evt_dev)
{
	struct mmio_of_timer *dev = container_of(evt_dev, struct mmio_of_timer, ce);
	unsigned int value = dev->clock->read(&dev->cs.clksrc);
	value = dev->clock->invert ? (value - event) : (value + event);
	return dev->set(dev, value);
}

static irqreturn_t clockevent_mmio_of_interrupt(int irq, void *dev_id)
{
	struct mmio_of_timer *dev = dev_id;
	void (*event_handler)(struct clock_event_device *);
	if (dev->get(dev)) {
		dev->clear(dev);

		event_handler = ACCESS_ONCE(dev->ce.event_handler);
		if (event_handler)
			event_handler(&dev->ce);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

void __init clockevent_mmio_of_init(void)
{
	struct device_node *node;
	bool found = false;

	for_each_matching_node(node, mmio_of_timer_match) {
		struct mmio_of_timer *data;
		struct irqaction *timer_irq;
		int ret;

		data = mmio_of_timer_read(node);
		if (IS_ERR(data))
			continue;

		if (data->rating == 0) {
			mmio_of_timer_free(data);
			continue;
		}

		if (!request_region(data->rcompare.start,
				resource_size(&data->rcompare),
				node->full_name)) {
			mmio_of_timer_free(data);
			continue;
		}

		timer_irq = kzalloc(sizeof(*timer_irq), GFP_KERNEL);
		BUG_ON(timer_irq == NULL);
		timer_irq->name = node->name;
		timer_irq->flags = IRQF_TIMER | IRQF_SHARED;
		timer_irq->dev_id = data;
		timer_irq->handler = clockevent_mmio_of_interrupt;

		clockevents_config_and_register(&data->ce, data->clock->freq,
			data->min_delta, data->max_delta);
		ret = setup_irq(data->irq, timer_irq);
		if (ret) {
			kfree(timer_irq);
			mmio_of_timer_free(data);
			continue;
		}

		printk(KERN_INFO "%s: timer at MMIO %#lx (irq = %d)\n",
			node->name, (unsigned long)data->rcompare.start,
			data->irq);

		if (!found)
			found = true;
	}

	if (!found)
		panic("can't find clockevent timer");
}
