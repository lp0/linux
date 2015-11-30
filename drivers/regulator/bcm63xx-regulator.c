/*
 * Copyright 2015 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * Derived from fixed.c:
 * Copyright 2008 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * Copyright (c) 2009 Nokia Corporation
 * Roger Quadros <ext-roger.quadros@nokia.com>
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/syscon.h>

static struct regulator_ops bcm63xx_regulator_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static int __init bcm63xx_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regulator_desc *desc;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = { };
	struct regulator_dev *reg_dev;
	u32 val;
	int ret;

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	init_data = of_get_regulator_init_data(dev, np, desc);
	if (!init_data) {
		dev_err(dev, "Failed to get regulator init data\n");
		return -EINVAL;
	}

	/* Only status change is supported, regardless of DT init data */
	init_data->constraints.valid_ops_mask &= REGULATOR_CHANGE_STATUS;

	desc->name = devm_kstrdup(dev, init_data->constraints.name, GFP_KERNEL);
	if (desc->name == NULL)
		return -ENOMEM;
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;
	desc->ops = &bcm63xx_regulator_ops;

	/* Clearing a bit in the register enables power */
	desc->enable_is_inverted = true;

	cfg.dev = dev;
	cfg.init_data = init_data;
	cfg.of_node = np;

	cfg.regmap = syscon_regmap_lookup_by_phandle(np, "regmap");
	if (IS_ERR(cfg.regmap)) {
		ret = PTR_ERR(cfg.regmap);
		dev_err(dev, "Failed to get regmap for %s: %d\n",
			desc->name, ret);
		return ret;
	}

	if (of_property_read_u32(np, "offset", &val)) {
		dev_err(dev, "Missing register offset for %s\n", desc->name);
		return -EINVAL;
	}
	desc->enable_reg = val;

	if (of_property_read_u32(np, "mask", &val)) {
		dev_err(dev, "Missing register mask for %s\n", desc->name);
		return -EINVAL;
	}
	desc->enable_mask = val;

	if (!of_property_read_u32(np, "startup-delay-us", &val))
		desc->enable_time = val;

	reg_dev = devm_regulator_register(dev, desc, &cfg);
	if (IS_ERR(reg_dev)) {
		ret = PTR_ERR(reg_dev);
		dev_err(dev, "Failed to register regulator %s: %d\n",
			desc->name, ret);
		return ret;
	}

	return 0;
}

static const struct of_device_id bcm63xx_regulator_of_match[] __initconst = {
	{ .compatible = "brcm,bcm6345-regulator", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm63xx_regulator_of_match);

static struct platform_driver bcm63xx_regulator_driver __refdata = {
	.probe		= bcm63xx_regulator_probe,
	.driver		= {
		.name		= "bcm63xx-regulator",
		.of_match_table = bcm63xx_regulator_of_match,
	},
};

module_platform_driver(bcm63xx_regulator_driver);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("BCM63xx regulator");
MODULE_LICENSE("GPL");
