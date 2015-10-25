/*
 * Copyright 2015 Simon Arlott
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * Derived from reset-berlin.c:
 * Copyright (C) 2014 Marvell Technology Group Ltd.
 *
 * Antoine Tenart <antoine.tenart@free-electrons.com>
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/types.h>

#define to_bcm6345_reset_priv(p) \
		container_of((p), struct bcm6345_reset_priv, rcdev)

struct bcm6345_reset_priv {
	struct regmap *map;
	u32 offset;
	struct reset_controller_dev rcdev;
	struct mutex mutex;
};

static int bcm6345_reset_reset(struct reset_controller_dev *rcdev,
	unsigned long id)
{
	struct bcm6345_reset_priv *priv = to_bcm6345_reset_priv(rcdev);

	mutex_lock(&priv->mutex);
	regmap_write_bits(priv->map, priv->offset, BIT(id), 0);
	usleep_range(10000, 20000);
	regmap_write_bits(priv->map, priv->offset, BIT(id), BIT(id));
	usleep_range(10000, 20000);
	mutex_unlock(&priv->mutex);

	return 0;
}

static struct reset_control_ops bcm6345_reset_ops = {
	.reset = bcm6345_reset_reset,
};

static int __init bcm6345_reset_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct bcm6345_reset_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->mutex);

	priv->map = syscon_regmap_lookup_by_phandle(np, "regmap");
	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);

	if (of_property_read_u32(np, "offset", &priv->offset))
		return -EINVAL;

	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.ops = &bcm6345_reset_ops;
	priv->rcdev.nr_resets = 32;
	priv->rcdev.of_node = pdev->dev.of_node;

	ret = reset_controller_register(&priv->rcdev);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register reset controller: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "registered reset controller\n");
	return 0;
}

static const struct of_device_id bcm6345_reset_dt_match[] __initconst = {
	{ .compatible = "brcm,bcm6345-reset" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm6345_reset_dt_match);

static struct platform_driver bcm6345_reset_driver __refdata = {
	.probe	= bcm6345_reset_probe,
	.driver	= {
		.name = "bcm6345-reset",
		.of_match_table = bcm6345_reset_dt_match,
	},
};
module_platform_driver(bcm6345_reset_driver);

MODULE_DESCRIPTION("BCM6345 reset driver");
MODULE_AUTHOR("Simon Arlott");
MODULE_LICENSE("GPL");
