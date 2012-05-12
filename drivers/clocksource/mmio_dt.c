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
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>

enum mmio_dt_type {
	MMIO_CLOCK,
	MMIO_TIMER
};

cycle_t (*func_16[])(struct clocksource *) __devinitconst = {
	clocksource_mmio_readw_up,
	clocksource_mmio_readw_down
};

cycle_t (*func_32[])(struct clocksource *) __devinitconst = {
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
	enum mmio_dt_type type;
	unsigned long base;
	void __iomem *value;
	int value_sz;
	void __iomem *control;
	int control_sz;

	union {
		struct of_mmio_dt_clock {
			u32 freq;
			u32 rating;
			u32 invert;

			cycle_t (*func)(struct clocksource *);
			struct clocksource *cp;
		} clock;

		struct of_mmio_dt_timer {
			u32 index;

			bool (*get)(struct of_mmio_dt *);
			void (*set)(struct of_mmio_dt *);
			struct clock_event_device *evt;
		} timer;
	};
};

static bool clockevent_mmio_dt_readw(struct of_mmio_dt *dev)
{
	return readw_relaxed(dev->control) & BIT(dev->timer.index);
}

static void clockevent_mmio_dt_writew(struct of_mmio_dt *dev)
{
	writew_relaxed(BIT(dev->timer.index), dev->control);
}

static bool clockevent_mmio_dt_readl(struct of_mmio_dt *dev)
{
	return readl_relaxed(dev->control) & BIT(dev->timer.index);
}

static void clockevent_mmio_dt_writel(struct of_mmio_dt *dev)
{
	writel_relaxed(BIT(dev->timer.index), dev->control);
}

static int __devinit of_clocksource_mmio_dt(struct of_mmio_dt *data,
	enum mmio_dt_type type, struct device_node *node)
{
	struct resource res[2];

	printk(KERN_DEBUG "%s(%p, %d, %p)\n", __func__, data, type, node);

	if (node == NULL)
		return -EINVAL;

	data->type = type;
	if (of_address_to_resource(node, 0, &res[0]))
		return -EFAULT;

	switch (data->type) {
	case MMIO_CLOCK: {
		struct of_mmio_dt_clock *clock = &data->clock;

		if (of_address_to_resource(node, 1, &res[1]))
			return -EFAULT;

		data->base = (unsigned long)res[0].start;
		data->value = ioremap(res[0].start, resource_size(&res[0]));
		data->value_sz = resource_size(&res[0]) * 8;
		data->control = ioremap(res[1].start, resource_size(&res[1]));
		data->control_sz = resource_size(&res[1]) * 8;

		clock->freq = clock->rating = clock->invert = 0;
		of_property_read_u32(node, "clock-frequency", &clock->freq);
		of_property_read_u32(node, "clock-rating", &clock->rating);
		of_property_read_u32(node, "clock-invert", &clock->invert);

		if (!data->base || !clock->freq || clock->invert & ~1)
			return -EINVAL;

		if (data->value_sz > 32)
			return -EOVERFLOW;

		if (data->value_sz <= 16) {
			clock->func = func_16[clock->invert];
		} else {
			clock->func = func_32[clock->invert];
		}
		break;
	}

	case MMIO_TIMER: {
		struct of_mmio_dt clock;
		struct of_mmio_dt_timer *timer = &data->timer;
		int ret;

		ret = of_clocksource_mmio_dt(&clock, MMIO_CLOCK, of_get_parent(node));
		printk(KERN_DEBUG "%s: parent=%d\n", __func__, ret);
		if (ret)
			return ret;

		if (clock.control_sz != 16 && clock.control_sz != 32)
			return -EINVAL;

		data->base = (unsigned long)res[0].start;
		data->value = ioremap(res[0].start, resource_size(&res[0]));
		data->value_sz = resource_size(&res[0]) * 8;
		data->control = clock.control;
		data->control_sz = clock.control_sz;

		timer->index = 0;
		of_property_read_u32(node, "index", &timer->index);

		if (timer->index >= data->control_sz)
			return -EINVAL;
		break;
	}

	}

	return 0;
}

static int __devinit clocksource_mmio_dt_probe(struct platform_device *of_dev)
{
	const struct of_device_id *match;
	struct device_node *node = of_dev->dev.of_node;
	struct of_mmio_dt *data = kzalloc(sizeof(*data), GFP_KERNEL);
	enum mmio_dt_type type;
	int ret;

	match = of_match_device(clocksource_mmio_dt_match, &of_dev->dev);
	if (!match)
		return -EINVAL;
	type = (enum mmio_dt_type)match->data;

	ret = of_clocksource_mmio_dt(data, type, node);
	if (ret)
		return ret;

	switch (data->type) {
	case MMIO_CLOCK: {
		struct of_mmio_dt_clock *clock = &data->clock;

		ret = clocksource_mmio_init(data->value, node->name,
			clock->freq, clock->rating, data->value_sz,
			&clock->cp, clock->func);
		if (ret)
			return ret;

		printk(KERN_INFO "%s: %d-bit clock at MMIO %#lx, %u Hz\n",
			node->name, data->value_sz, data->base, clock->freq);
		break;
	}

	case MMIO_TIMER: {
		struct of_mmio_dt_timer *timer = &data->timer;

		(void)timer;

		printk(KERN_INFO "%s: timer at MMIO %#lx\n",
			node->name, data->base);
		break;
	}

	default:
		return -ENODEV;
	}

	platform_set_drvdata(of_dev, data);
	return 0;
}

static int __devexit clocksource_mmio_dt_remove(struct platform_device *of_dev)
{
	struct of_mmio_dt *data = platform_get_drvdata(of_dev);

	switch (data->type) {
	case MMIO_CLOCK:
		clocksource_mmio_remove(data->clock.cp);
		break;

	case MMIO_TIMER:
		break;
	}

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
