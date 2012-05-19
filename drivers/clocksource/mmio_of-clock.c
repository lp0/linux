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
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/string.h>

#if defined(CONFIG_ARM) && !defined(CONFIG_MMIO_OF_MODULE)
# include <asm/sched_clock.h>
#endif

#include "mmio_of.h"

void mmio_of_clock_free(struct mmio_of_clock *data)
{
	if (data->value)
		iounmap(data->value);
	if (data->control)
		iounmap(data->control);
	kfree(data);
}

static cycle_t (*read_16[])(struct clocksource *) __devinitconst = {
	clocksource_mmio_readw_up,
	clocksource_mmio_readw_down
};

static cycle_t (*read_32[])(struct clocksource *) __devinitconst = {
	clocksource_mmio_readl_up,
	clocksource_mmio_readl_down
};

struct mmio_of_clock __devinit *mmio_of_clock_read(
	struct device_node *node)
{
	struct mmio_of_clock *data;
	int ret;

	if (node == NULL)
		return ERR_PTR(-EINVAL);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return ERR_PTR(-ENOMEM);

	if (of_address_to_resource(node, 0, &data->rvalue)) {
		ret = -ENXIO;
		goto err;
	}


	data->value = ioremap(data->rvalue.start, resource_size(&data->rvalue));
	if (!data->value) {
		ret = -EIO;
		goto err;
	}

	data->size = resource_size(&data->rvalue) * 8;
	if (!of_address_to_resource(node, 1, &data->rcontrol)) {
		data->control = ioremap(data->rcontrol.start,
			resource_size(&data->rcontrol));
	}

	data->system = false;
	if (of_property_match_string(node, "clock-outputs", "sys") >= 0)
		data->system = true;

	data->freq = data->invert = data->rating = 0;
	of_property_read_u32(node, "clock-frequency", &data->freq);
	of_property_read_u32(node, "clock-invert", &data->invert);
	of_property_read_u32(node, "rating", &data->rating);

	if (!data->value || !data->freq || data->invert & ~1) {
		ret = -EINVAL;
		goto err;
	}

	if (data->size > 32) {
		ret = -EOVERFLOW;
		goto err;
	}

	if (data->size <= 16) {
		data->read = read_16[data->invert];
	} else {
		data->read = read_32[data->invert];
	}

	return data;

err:
	mmio_of_clock_free(data);
	return ERR_PTR(ret);
}

static struct of_device_id mmio_of_clock_match[] __devinitconst = {
	{ .compatible = "mmio-clock" },
	{}
};
MODULE_DEVICE_TABLE(of, mmio_of_clock_match);

#if defined(CONFIG_ARM) && !defined(CONFIG_MMIO_OF_MODULE)
static struct clocksource_mmio_of_clock {
	cycle_t (*read)(struct clocksource *);

	/* this is a fake clocksource_mmio that is
	 * compatible with its read functions
	 */
	struct clocksource_mmio cs;
} system_clock __read_mostly;

static u32 notrace clocksource_mmio_of_read(void)
{
	return system_clock.read(&system_clock.cs.clksrc);
}

void __init clocksource_mmio_of_init(void)
{
	struct device_node *node;
	bool found = false;

	for_each_matching_node(node, mmio_of_clock_match) {
		struct mmio_of_clock *data = mmio_of_clock_read(node);

		if (IS_ERR(data))
			continue;

		if (data->system) {
			system_clock.read = data->read;
			system_clock.cs.reg = data->value;

			setup_sched_clock(clocksource_mmio_of_read,
				data->size, data->freq);
			found = true;
		}
		mmio_of_clock_free(data);

		if (found)
			break;
	}

	if (!found)
		panic("can't find scheduler clocksource");
}
#endif

static int __devinit mmio_of_clock_probe(struct platform_device *of_dev)
{
	struct device_node *node = of_dev->dev.of_node;
	struct mmio_of_clock *data = mmio_of_clock_read(node);
	int ret;
	if (IS_ERR(data)) {
		ret = PTR_ERR(data);
		goto err;
	}

	if (!request_region(data->rvalue.start, resource_size(&data->rvalue),
			node->full_name)) {
		ret = -EBUSY;
		goto err_free;
	}

	if (data->control && !request_region(data->rcontrol.start,
			resource_size(&data->rcontrol), node->full_name)) {
		ret = -EBUSY;
		goto err_release;
	}

	data->cs = clocksource_mmio_init(data->value, node->name,
		data->freq, data->rating, data->size, data->read);
	if (IS_ERR(data->cs)) {
		ret = PTR_ERR(data->cs);
		goto err_free;
	}

	dev_info(&of_dev->dev, "%d-bit clock at MMIO %#lx, %u Hz\n",
		data->size, (unsigned long)data->rvalue.start, data->freq);

	platform_set_drvdata(of_dev, data);
	return 0;

err_release:
	release_region(data->rvalue.start, resource_size(&data->rvalue));
err_free:
	mmio_of_clock_free(data);
err:
	return ret;
}

static int __devexit mmio_of_clock_remove(struct platform_device *of_dev)
{
	struct mmio_of_clock *data = platform_get_drvdata(of_dev);

	clocksource_mmio_remove(data->cs);
	if (data->control)
		release_region(data->rcontrol.start,
			resource_size(&data->rcontrol));
	release_region(data->rvalue.start, resource_size(&data->rvalue));
	mmio_of_clock_free(data);
	platform_set_drvdata(of_dev, NULL);
	return 0;
}

static struct platform_driver mmio_of_clock_driver = {
	.probe = mmio_of_clock_probe,
	.remove = __devexit_p(mmio_of_clock_remove),
	.driver = {
		.name = "mmio_of_clock",
		.owner = THIS_MODULE,
		.of_match_table = mmio_of_clock_match
	}
};

static int __init mmio_of_clock_init(void)
{
	return platform_driver_register(&mmio_of_clock_driver);
}
arch_initcall(mmio_of_clock_init);

static void __exit mmio_of_clock_exit(void)
{
	platform_driver_unregister(&mmio_of_clock_driver);
}
module_exit(mmio_of_clock_exit);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("Driver for MMIO clock source (Device Tree)");
MODULE_LICENSE("GPL");
