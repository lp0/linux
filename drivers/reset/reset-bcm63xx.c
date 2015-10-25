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

#define to_bcm63xx_reset_priv(p) \
		container_of((p), struct bcm63xx_reset_priv, rcdev)

struct bcm63xx_reset_priv {
	struct resource res;
	void __iomem *reg;
	struct reset_controller_dev rcdev;
	struct mutex mutex;
};

static int bcm63xx_reset_reset(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct bcm63xx_reset_priv *priv = to_bcm63xx_reset_priv(rcdev);
	u32 val;

	mutex_lock(&priv->mutex);
	val = __raw_readl(priv->reg);
	val &= ~BIT(id);
	__raw_writel(val, priv->reg);
	msleep(10);

	val |= BIT(id);
	__raw_writel(val, priv->reg);
	msleep(10);
	mutex_unlock(&priv->mutex);

	return 0;
}

static struct reset_control_ops bcm63xx_reset_ops = {
	.reset = bcm63xx_reset_reset,
};

static int bcm63xx_reset_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct bcm63xx_reset_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->mutex);

	ret = of_address_to_resource(np, 0, &priv->res);
	if (ret)
		return ret;

	priv->reg = devm_ioremap_resource(&pdev->dev, &priv->res);
	if (IS_ERR(priv->reg))
		return PTR_ERR(priv->reg);

	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.ops = &bcm63xx_reset_ops;
	priv->rcdev.nr_resets = 32;
	priv->rcdev.of_node = pdev->dev.of_node;

	ret = reset_controller_register(&priv->rcdev);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register reset controller: %d\n", ret);
		return ret;
	}

	dev_info(&pdev->dev, "registered reset controller at MMIO 0x%p\n",
			priv->reg);
	return 0;
}

static const struct of_device_id bcm63xx_reset_dt_match[] = {
	{ .compatible = "brcm,bcm63xx-reset" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm63xx_reset_dt_match);

static struct platform_driver bcm63xx_reset_driver = {
	.probe	= bcm63xx_reset_probe,
	.driver	= {
		.name = "bcm63xx-reset",
		.of_match_table = bcm63xx_reset_dt_match,
	},
};
module_platform_driver(bcm63xx_reset_driver);

MODULE_DESCRIPTION("BCM63xx reset driver");
MODULE_LICENSE("GPL");
