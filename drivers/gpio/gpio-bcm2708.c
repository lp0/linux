/*
 * Copyright (C) 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * TODO: IRQ support
 */

#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>

#define MODULE_NAME "gpio-bcm2708"

#define BANK_REG(p)	((p / 32) * 4)
#define BANK_BIT(p)	(p - 32 * (p / 32))
#define BANK_IRQ(p)	(p / 32)

#define BANKS		3
#define MAX_GPIOS	(32 * BANKS)
#define REG_SZ		(BANKS * 4)
#define ALL_REG_SZ	(REG_SZ * 10)
#define ALL_BANKS_IRQ	BANKS

struct bcm2708_gpio {
	struct device *dev;
        struct gpio_chip gc;

	struct resource res;
	void __iomem *base;
	void __iomem *get;
	void __iomem *set;
	void __iomem *clear;
	void __iomem *evt_stat;
	void __iomem *rise_det;
	void __iomem *fall_det;
	void __iomem *high_det;
	void __iomem *low_det;
	void __iomem *arise_det;
	void __iomem *afall_det;
};

static bool inline bcm2708_gpiolib_valid(const char *func,
	struct gpio_chip *chip, unsigned offset)
{
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, gc);

	if (unlikely(offset >= chip->ngpio)) {
		dev_warn(gpio->dev,
			"%s: invalid gpio %u requested (not in range %u-%u)\n",
			func,
			chip->base + offset,
			chip->base, chip->base + chip->ngpio);
		return false;
	}
	return true;
}

static int bcm2708_gpiolib_request(struct gpio_chip *chip, unsigned offset)
{
	if (unlikely(!bcm2708_gpiolib_valid(__func__, chip, offset)))
		return -EINVAL;

	return pinctrl_request_gpio(chip->base + offset);
}

static void bcm2708_gpiolib_free(struct gpio_chip *chip, unsigned offset)
{
	if (unlikely(!bcm2708_gpiolib_valid(__func__, chip, offset)))
		return;

	pinctrl_free_gpio(chip->base + offset);
}


static int bcm2708_gpiolib_direction_input(struct gpio_chip *chip,
	unsigned offset)
{
	if (unlikely(!bcm2708_gpiolib_valid(__func__, chip, offset)))
		return -EINVAL;

	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int bcm2708_gpiolib_direction_output(struct gpio_chip *chip,
	unsigned offset, int value)
{
	if (unlikely(!bcm2708_gpiolib_valid(__func__, chip, offset)))
		return -EINVAL;

	return pinctrl_gpio_direction_output(chip->base + offset);
}

static int bcm2708_gpiolib_get(struct gpio_chip *chip, unsigned offset)
{
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, gc);

	return (readl(gpio->get + BANK_REG(offset)) >> BANK_BIT(offset)) & 1;
}

static void bcm2708_gpiolib_set(struct gpio_chip *chip, unsigned offset,
	int value)
{
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, gc);

	writel(BIT(BANK_BIT(offset)),
		(value ? gpio->set : gpio->clear) + BANK_REG(offset));
}

static void bcm2708_gpio_free(struct bcm2708_gpio *gpio)
{
	if (gpio->base) {
		release_region(gpio->res.start, resource_size(&gpio->res));
		iounmap(gpio->base);
	}
	kfree(gpio);
}

static int bcm2708_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *pctl_np;
	struct bcm2708_gpio *gpio = kzalloc(sizeof(*gpio), GFP_KERNEL);
	u32 gpio_base, nr_gpios;
	void __iomem *base;
	int ret;

	if (gpio == NULL)
		return -ENOMEM;

	gpio->dev = &pdev->dev;

	if (of_address_to_resource(np, 0, &gpio->res)) {
		ret = -EINVAL;
		goto err;
	}

	if (resource_size(&gpio->res) < ALL_REG_SZ) {
		dev_err(gpio->dev, "resource size too small (%#x)\n",
			resource_size(&gpio->res));
		ret = -EINVAL;
		goto err;
	}

	if (!request_region(gpio->res.start, resource_size(&gpio->res),
			np->full_name)) {
		dev_err(gpio->dev, "resource %#lx unavailable\n",
			(unsigned long)gpio->res.start);
		ret = -EBUSY;
		goto err;
	}

	base = ioremap(gpio->res.start, resource_size(&gpio->res));
	if (!base) {
		dev_err(gpio->dev, "error mapping io at %#lx\n",
			(unsigned long)gpio->res.start);
		release_region(gpio->res.start, resource_size(&gpio->res));
		ret = -EIO;
		goto err;
	}

	gpio->base = base;

	gpio->set = base;
	base += REG_SZ;

	gpio->clear = base;
	base += REG_SZ;

	gpio->get = base;
	base += REG_SZ;

	gpio->evt_stat = base;
	base += REG_SZ;

	gpio->rise_det = base;
	base += REG_SZ;

	gpio->fall_det = base;
	base += REG_SZ;

	gpio->high_det = base;
	base += REG_SZ;

	gpio->low_det = base;
	base += REG_SZ;

	gpio->arise_det = base;
	base += REG_SZ;

	gpio->afall_det = base;

	pctl_np = of_parse_phandle(np, "pinctrl", 0);
	if (pctl_np == NULL) {
		dev_err(gpio->dev, "pinctrl node missing\n");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32(pctl_np, "count", &nr_gpios);
	if (ret) {
		dev_err(gpio->dev, "gpio count missing (%d)\n", ret);
		goto err_put;
	}
	if (nr_gpios > MAX_GPIOS) {
		dev_err(gpio->dev, "unhandled gpio count %d\n", nr_gpios);
		goto err_put;
	}

	gpio_base = 0;
	of_property_read_u32(pctl_np, "base", &gpio_base);

	of_node_put(pctl_np);

	gpio->gc.label = MODULE_NAME;
	gpio->gc.owner = THIS_MODULE;
	gpio->gc.base = gpio_base;
	gpio->gc.ngpio = nr_gpios;
	gpio->gc.request = bcm2708_gpiolib_request;
	gpio->gc.free = bcm2708_gpiolib_free;
	gpio->gc.direction_input = bcm2708_gpiolib_direction_input;
	gpio->gc.direction_output = bcm2708_gpiolib_direction_output;
	gpio->gc.get = bcm2708_gpiolib_get;
	gpio->gc.set = bcm2708_gpiolib_set;
	gpio->gc.can_sleep = 0;

	ret = gpiochip_add(&gpio->gc);
	if (ret)
		goto err;

	dev_info(gpio->dev, "%d pins at MMIO %#lx\n", nr_gpios,
		(unsigned long)gpio->res.start);

	platform_set_drvdata(pdev, gpio);
	return 0;

err_put:
	of_node_put(pctl_np);
err:
	bcm2708_gpio_free(gpio);
	return ret;
}

static int bcm2708_gpio_remove(struct platform_device *pdev)
{
	struct bcm2708_gpio *gpio = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(&gpio->gc);
	if (ret)
		return ret;

	bcm2708_gpio_free(gpio);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct of_device_id bcm2708_gpio_match[] __devinitconst = {
	{ .compatible = "broadcom,bcm2708-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm2708_gpio_match);

static struct platform_driver bcm2708_gpio_driver = {
	.probe	= bcm2708_gpio_probe,
	.remove	= bcm2708_gpio_remove,
	.driver	= {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm2708_gpio_match
	}
};
module_platform_driver(bcm2708_gpio_driver);

MODULE_DESCRIPTION("Broadcom BCM2708 GPIO driver");
MODULE_LICENSE("GPL");
