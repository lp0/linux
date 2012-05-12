/*
 * Generic MMIO clocksource support (Device Tree)
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

#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/irq.h>

enum mmio_dt_type {
	MMIO_CLOCK,
	MMIO_TIMER
};

cycle_t (*read_16[])(struct clocksource *) __devinitconst = {
	clocksource_mmio_readw_up,
	clocksource_mmio_readw_down
};

cycle_t (*read_32[])(struct clocksource *) __devinitconst = {
	clocksource_mmio_readl_up,
	clocksource_mmio_readl_down
};

static const struct of_device_id clocksource_mmio_dt_match[] = {
	{
		/* clocksource */
		.compatible = "mmio-clock",
		.data = (void *)MMIO_CLOCK
	},
	{
		/* clockevent */
		.compatible = "mmio-timer",
		.data = (void *)MMIO_TIMER
	},
	{}
};
MODULE_DEVICE_TABLE(of, clocksource_mmio_dt_match);

struct of_mmio_dt {
	char *name;
	enum mmio_dt_type type;
	u32 freq;
	u32 invert;

	unsigned long base;
	void __iomem *value;
	int value_sz;
	void __iomem *control;
	int control_sz;

	cycle_t (*read)(struct clocksource *);

	union {
		struct of_mmio_dt_clock {
			u32 rating;

			struct clocksource_mmio *cs;
		} clock;

		struct of_mmio_dt_timer {
			int irq;

			u32 cpu;
			u32 index;
			u32 min_delta;
			u32 max_delta;

			bool (*get)(struct of_mmio_dt *);
			int (*set)(struct of_mmio_dt *, unsigned int);
			void (*clear)(struct of_mmio_dt *);
			/* this is a fake clocksource_mmio that is
			 * compatible with its read functions
			 */
			struct clocksource_mmio cs;
			struct clock_event_device ce;
		} timer;
	};
};

static void mmio_dt_free(struct of_mmio_dt *data)
{
	kfree(data->name);
	kfree(data);
}

static bool clockevent_mmio_dt_getw(struct of_mmio_dt *dev)
{
	return readw_relaxed(dev->control) & BIT(dev->timer.index);
}

static int clockevent_mmio_dt_setw(struct of_mmio_dt *dev, unsigned int value)
{
	writew_relaxed((u16)value, dev->value);
	return 0;
}

static void clockevent_mmio_dt_clearw(struct of_mmio_dt *dev)
{
	writew_relaxed(BIT(dev->timer.index), dev->control);
}

static bool clockevent_mmio_dt_getl(struct of_mmio_dt *dev)
{
	return readl_relaxed(dev->control) & BIT(dev->timer.index);
}

static int clockevent_mmio_dt_setl(struct of_mmio_dt *dev, unsigned int value)
{
	printk(KERN_DEBUG "%s: %d -> %p\n", __func__, (u32)value, dev->control);
	writel_relaxed((u32)value, dev->value);
	return 0;
}

static void clockevent_mmio_dt_clearl(struct of_mmio_dt *dev)
{
	printk(KERN_DEBUG "%s: %d -> %p\n", __func__, dev->timer.index, dev->control);
	writel_relaxed(BIT(dev->timer.index), dev->control);
}

static void mmio_dt_timer_set_mode(enum clock_event_mode mode,
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

static int mmio_dt_timer_set_next_event(unsigned long event,
	struct clock_event_device *evt_dev)
{
	struct of_mmio_dt *dev = container_of(evt_dev, struct of_mmio_dt, timer.ce);
	unsigned int value = dev->read(&dev->timer.cs.clksrc);
	value = dev->invert ? (value - event) : (value + event);
	return dev->timer.set(dev, value);
}

static irqreturn_t mmio_dt_timer_interrupt(int irq, void *dev_id)
{
	struct of_mmio_dt *dev = dev_id;
	if (dev->timer.get(dev)) {
		dev->timer.clear(dev);
		if (dev->timer.ce.event_handler)
			dev->timer.ce.event_handler(&dev->timer.ce);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int __devinit of_clocksource_mmio_dt(struct of_mmio_dt *data,
	enum mmio_dt_type type, struct device_node *node)
{
	struct resource res[2];
	int ret;

	printk(KERN_DEBUG "%s(%p, %d, %p)\n", __func__, data, type, node);

	if (node == NULL)
		return -EINVAL;

	data->name = kstrdup(node->name, GFP_KERNEL);
	data->type = type;

	if (of_address_to_resource(node, 0, &res[0])) {
		ret = -EFAULT;
		goto err;
	}

	switch (data->type) {
	case MMIO_CLOCK: {
		struct of_mmio_dt_clock *clock = &data->clock;

		data->base = (unsigned long)res[0].start;
		data->value = ioremap(res[0].start, resource_size(&res[0]));
		data->value_sz = resource_size(&res[0]) * 8;
		if (of_address_to_resource(node, 1, &res[1])) {
			data->control = NULL;
			data->control_sz = 0;
		} else {
			data->control = ioremap(res[1].start, resource_size(&res[1]));
			data->control_sz = resource_size(&res[1]) * 8;
		}

		data->freq = clock->rating = data->invert = 0;
		of_property_read_u32(node, "clock-frequency", &data->freq);
		of_property_read_u32(node, "clock-rating", &clock->rating);
		of_property_read_u32(node, "clock-invert", &data->invert);

		if (!data->base || !data->freq || data->invert & ~1) {
			ret = -EINVAL;
			goto err;
		}

		if (data->value_sz > 32) {
			ret = -EOVERFLOW;
			goto err;
		}

		if (data->value_sz <= 16) {
			data->read = read_16[data->invert];
		} else {
			data->read = read_32[data->invert];
		}
		break;
	}

	case MMIO_TIMER: {
		struct of_mmio_dt cdata;
		struct of_mmio_dt_timer *timer = &data->timer;

		ret = of_clocksource_mmio_dt(&cdata, MMIO_CLOCK, of_get_parent(node));
		printk(KERN_DEBUG "%s: parent=%d\n", __func__, ret);
		if (ret)
			goto err;

		/* we need to know about the clock to set oneshot events */
		timer->cs.reg = cdata.value;
		data->read = cdata.read;
		data->freq = cdata.freq;
		data->invert = cdata.invert;

		data->base = (unsigned long)res[0].start;
		data->value = ioremap(res[0].start, resource_size(&res[0]));
		data->value_sz = resource_size(&res[0]) * 8;
		data->control = cdata.control;
		data->control_sz = cdata.control_sz;

		if (data->control_sz != 16 && data->control_sz != 32) {
			ret = -EINVAL;
			goto err;
		}

		timer->irq = irq_of_parse_and_map(node, 0);
		timer->cpu = timer->index = 0;
		of_property_read_u32(node, "cpu", &timer->cpu);
		of_property_read_u32(node, "index", &timer->index);

		if (timer->index >= data->control_sz) {
			ret = -EINVAL;
			goto err;
		}

		timer->min_delta = 1;
		if (data->value_sz == 16) {
			timer->max_delta = 0xffff;
		} else {
			timer->max_delta = 0xffffffff;
		}
		of_property_read_u32(node, "min-delta", &timer->min_delta);
		of_property_read_u32(node, "max-delta", &timer->max_delta);

		if (data->control_sz == 16) {
			timer->get = clockevent_mmio_dt_getw;
			timer->set = clockevent_mmio_dt_setw;
			timer->clear = clockevent_mmio_dt_clearw;
		} else {
			timer->get = clockevent_mmio_dt_getl;
			timer->set = clockevent_mmio_dt_setl;
			timer->clear = clockevent_mmio_dt_clearl;
		}

		timer->ce.name = data->name;
		timer->ce.features = CLOCK_EVT_FEAT_ONESHOT;
		timer->ce.set_mode = mmio_dt_timer_set_mode;
		timer->ce.set_next_event = mmio_dt_timer_set_next_event;
		timer->ce.cpumask = cpumask_of(timer->cpu);
		break;
	}

	}

	return 0;

err:
	mmio_dt_free(data);
	return ret;
}

static int __devinit clocksource_mmio_dt_probe(struct platform_device *of_dev)
{
	const struct of_device_id *match;
	struct device_node *node = of_dev->dev.of_node;
	struct of_mmio_dt *data = kzalloc(sizeof(*data), GFP_KERNEL);
	enum mmio_dt_type type;
	int ret;

	match = of_match_device(clocksource_mmio_dt_match, &of_dev->dev);
	if (!match) {
		ret = -EINVAL;
		goto err;
	}
	type = (enum mmio_dt_type)match->data;

	ret = of_clocksource_mmio_dt(data, type, node);
	if (ret)
		goto err;

	switch (data->type) {
	case MMIO_CLOCK: {
		struct of_mmio_dt_clock *clock = &data->clock;

		clock->cs = clocksource_mmio_init(data->value, data->name,
			data->freq, clock->rating, data->value_sz, data->read);
		if (IS_ERR(clock->cs)) {
			ret = PTR_ERR(clock->cs);
			goto err;
		}

		printk(KERN_INFO "%s: %d-bit clock at MMIO %#lx, %u Hz\n",
			data->name, data->value_sz, data->base, data->freq);
		break;
	}

	case MMIO_TIMER: {
		struct of_mmio_dt_timer *timer = &data->timer;
		static struct irqaction *timer_irq;

		timer_irq = kzalloc(sizeof(*timer_irq), GFP_KERNEL);
		timer_irq->name = data->name;
		timer_irq->flags = IRQF_TIMER;
		timer_irq->dev_id = data;
		timer_irq->handler = mmio_dt_timer_interrupt;

		printk(KERN_INFO "%s: timer at MMIO %#lx (irq = %d)\n",
			data->name, data->base, timer->irq);

		clockevents_config_and_register(&timer->ce, data->freq,
			timer->min_delta, timer->max_delta);
		setup_irq(timer->irq, timer_irq);

		/* clockevents can't be unregistered */
		__module_get(THIS_MODULE);
		break;
	}

	default:
		ret = -ENODEV;
		goto err;
	}

	platform_set_drvdata(of_dev, data);
	return 0;

err:
	mmio_dt_free(data);
	return ret;
}

static int __devexit clocksource_mmio_dt_remove(struct platform_device *of_dev)
{
	struct of_mmio_dt *data = platform_get_drvdata(of_dev);

	switch (data->type) {
	case MMIO_CLOCK:
		clocksource_mmio_remove(data->clock.cs);
		break;

	case MMIO_TIMER:
		return -EBUSY;
	}

	mmio_dt_free(data);
	return 0;
}

static struct platform_driver clocksource_mmio_dt_driver = {
	.probe = clocksource_mmio_dt_probe,
	.remove = __devexit_p(clocksource_mmio_dt_remove),
	.driver = {
		.name = "clocksource_mmio_dt",
		.owner = THIS_MODULE,
		.of_match_table = clocksource_mmio_dt_match
	}
};

static int __init clocksource_mmio_dt_init(void)
{
	return platform_driver_register(&clocksource_mmio_dt_driver);
}
arch_initcall(clocksource_mmio_dt_init);

static void __exit clocksource_mmio_dt_exit(void)
{
	return platform_driver_unregister(&clocksource_mmio_dt_driver);
}
module_exit(clocksource_mmio_dt_exit);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("Driver for MMIO clock source (Device Tree)");
MODULE_LICENSE("GPL");
