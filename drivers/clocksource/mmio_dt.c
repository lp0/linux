/*
 * Generic MMIO clocksource support (Device Tree)
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

static const struct of_device_id clocksource_mmio_dt_match[] = {
	{
		.compatible = "mmio-clock"
	},
	{}
};
MODULE_DEVICE_TABLE(of, clocksource_mmio_dt_match);

static int __devinit clocksource_mmio_dt_probe(struct platform_device *of_dev)
{
	struct device_node *node = of_dev->dev.of_node;
	struct resource res;
	void __iomem *base;
	int size;
	u32 freq = 0;
	u32 rating = 0;
	u32 invert = 0;
	cycle_t (*func)(struct clocksource *);
	struct clocksource *cp;
	int ret;

	if (of_address_to_resource(node, 0, &res)) // FIXME get it from of_dev
		return -EFAULT;
	
	base = ioremap(res.start, resource_size(&res));
	size = resource_size(&res) * 8;

	of_property_read_u32(node, "clock-frequency", &freq);
	of_property_read_u32(node, "clock-rating", &rating);
	of_property_read_u32(node, "clock-invert", &invert);

	if (!base || !freq)
		return -EINVAL;

	if (size > 32)
		return -EOVERFLOW;

	if (size <= 16) {
		func = invert ? clocksource_mmio_readw_down : clocksource_mmio_readw_up;
	} else {
		func = invert ? clocksource_mmio_readl_down : clocksource_mmio_readl_up;
	}

	ret = clocksource_mmio_init(base, node->name, freq, rating, resource_size(&res) * 8, &cp, func);
	if (ret)
		return ret;

	platform_set_drvdata(of_dev, cp);

	printk(KERN_INFO "%s: %d-bit clock at MMIO %#lx, %u Hz\n",
		node->name, size, (unsigned long)res.start, freq);
	return ret;
}

static int __devexit clocksource_mmio_dt_remove(struct platform_device *of_dev)
{
	struct clocksource *cp = platform_get_drvdata(of_dev);
	clocksource_mmio_remove(cp);
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

