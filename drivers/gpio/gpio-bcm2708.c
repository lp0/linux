/*
 * Copyright (C) 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * There's another IRQ at the end of the list of IRQs for "all banks" but this
 * isn't used as it's not required.
 *
 * The support for async rise/fall interrupts isn't used, all interrupts are
 * configured to use synchronous detection to suppress glitches.
 */

#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/consumer.h>

#define MODULE_NAME "gpio-bcm2708"

#define GPIOS_PER_BANK	32
#define BANKS		3
#define MAX_GPIOS	(GPIOS_PER_BANK * BANKS)

#define BANK_REG(b)	(b * 4)
#define BANK_OFFSET(b)	(b * GPIOS_PER_BANK)
#define GPIO_BANK(p)	(p / GPIOS_PER_BANK)
#define GPIO_REG(p)	BANK_REG(GPIO_BANK(p))
#define GPIO_BIT(p)	(p - GPIOS_PER_BANK * GPIO_BANK(p))

#define REG_SZ		BANK_REG(BANKS)
#define ALL_REG_SZ	(REG_SZ * 10)

struct bcm2708_gpio_irqdata {
	struct bcm2708_gpio *gpio;
	int bank;
};

struct bcm2708_gpio {
	struct device *dev;
        struct gpio_chip gc;
	struct irq_chip ic;

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

	u32 irq[BANKS];
	struct spinlock irq_lock[BANKS];
	struct bcm2708_gpio_irqdata irq_data[BANKS];
	unsigned int irq_type[MAX_GPIOS];
	bool irq_masked[MAX_GPIOS];
	struct irq_domain *irq_domain;
};

static struct lock_class_key gpio_lock_class;

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

	return (readl(gpio->get + GPIO_REG(offset)) >> GPIO_BIT(offset)) & 1;
}

static void bcm2708_gpiolib_set(struct gpio_chip *chip, unsigned offset,
	int value)
{
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, gc);

	writel(BIT(GPIO_BIT(offset)),
		(value ? gpio->set : gpio->clear) + GPIO_REG(offset));
}

static int bcm2708_gpiolib_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, gc);

	if (unlikely(!bcm2708_gpiolib_valid(__func__, chip, offset)))
		return -EINVAL;

	return irq_linear_revmap(gpio->irq_domain, offset);
}

static void bcm2708_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct bcm2708_gpio_irqdata *irqdata = irq_desc_get_handler_data(desc);
	struct bcm2708_gpio *gpio = irqdata->gpio;
	int bank = irqdata->bank;
	unsigned long events;
	unsigned offset;

	events = readl(gpio->evt_stat + BANK_REG(bank));
	for_each_set_bit(offset, &events, GPIOS_PER_BANK)
		generic_handle_irq(
			irq_linear_revmap(gpio->irq_domain,
				BANK_OFFSET(bank) + offset));
}

static void bcm2708_gpio_irq_ack(struct irq_data *data)
{
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, ic);
	unsigned offset = data->hwirq;

	/*
	 * This has no effect on level triggered events if the level hasn't
	 * changed.
	 */
	writel(BIT(GPIO_BIT(offset)), gpio->evt_stat + GPIO_REG(offset));
}

static void __bcm2708_gpio_irq_config(void __iomem *base,
	unsigned offset, bool enable)
{
	u32 value = readl(base + GPIO_REG(offset));
	if (enable)
		value |= BIT(GPIO_BIT(offset));
	else
		value &= ~(BIT(GPIO_BIT(offset)));
	writel(value, base + GPIO_REG(offset));
}

/* fast path for IRQ handler */
static void bcm2708_gpio_irq_config(struct bcm2708_gpio *gpio,
	unsigned offset, bool enable)
{
	switch (gpio->irq_type[offset]) {
	case IRQ_TYPE_EDGE_RISING:
		__bcm2708_gpio_irq_config(gpio->rise_det, offset, enable);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		__bcm2708_gpio_irq_config(gpio->fall_det, offset, enable);
		break;

	case IRQ_TYPE_EDGE_BOTH:
		__bcm2708_gpio_irq_config(gpio->rise_det, offset, enable);
		__bcm2708_gpio_irq_config(gpio->fall_det, offset, enable);
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		__bcm2708_gpio_irq_config(gpio->high_det, offset, enable);
		break;

	case IRQ_TYPE_LEVEL_LOW:
		__bcm2708_gpio_irq_config(gpio->low_det, offset, enable);
		break;
	}
}

static void bcm2708_gpio_irq_mask(struct irq_data *data, bool ack)
{
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, ic);
	unsigned offset = data->hwirq;
	int bank = GPIO_BANK(offset);
	unsigned long flags;

	spin_lock_irqsave(&gpio->irq_lock[bank], flags);
	bcm2708_gpio_irq_config(gpio, offset, false);
	gpio->irq_masked[offset] = true;

	if (ack)
		bcm2708_gpio_irq_ack(data);
	spin_unlock_irqrestore(&gpio->irq_lock[bank], flags);
}

static void bcm2708_gpio_irq_mask_ack(struct irq_data *data)
{
	bcm2708_gpio_irq_mask(data, true);
}

static void bcm2708_gpio_irq_mask_no_ack(struct irq_data *data)
{
	bcm2708_gpio_irq_mask(data, false);
}

static void bcm2708_gpio_irq_unmask(struct irq_data *data)
{
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, ic);
	unsigned offset = data->hwirq;
	int bank = GPIO_BANK(offset);
	unsigned long flags;

	spin_lock_irqsave(&gpio->irq_lock[bank], flags);
	gpio->irq_masked[offset] = false;
	bcm2708_gpio_irq_config(gpio, offset, true);
	spin_unlock_irqrestore(&gpio->irq_lock[bank], flags);
}

static int __bcm2708_gpio_irq_set_type_masked(struct bcm2708_gpio *gpio,
	unsigned offset, unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_NONE:
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
	case IRQ_TYPE_EDGE_BOTH:
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		gpio->irq_type[offset] = type;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/* slower path for reconfiguring IRQ type */
static int __bcm2708_gpio_irq_set_type_unmasked(struct bcm2708_gpio *gpio,
	unsigned offset, unsigned int type)
{
	switch (type) {
	case IRQ_TYPE_NONE:
		if (gpio->irq_type[offset] != type) {
			bcm2708_gpio_irq_config(gpio, offset, false);
			gpio->irq_type[offset] = type;
		}
		break;

	case IRQ_TYPE_EDGE_RISING:
		if (gpio->irq_type[offset] == IRQ_TYPE_EDGE_BOTH) {
			/* RISING already enabled, disable FALLING */
			gpio->irq_type[offset] = IRQ_TYPE_EDGE_FALLING;
			bcm2708_gpio_irq_config(gpio, offset, false);
			gpio->irq_type[offset] = type;
		} else if (gpio->irq_type[offset] != type) {
			bcm2708_gpio_irq_config(gpio, offset, false);
			gpio->irq_type[offset] = type;
			bcm2708_gpio_irq_config(gpio, offset, true);
		}
		break;

	case IRQ_TYPE_EDGE_FALLING:
		if (gpio->irq_type[offset] == IRQ_TYPE_EDGE_BOTH) {
			/* FALLING already enabled, disable RISING */
			gpio->irq_type[offset] = IRQ_TYPE_EDGE_RISING;
			bcm2708_gpio_irq_config(gpio, offset, false);
			gpio->irq_type[offset] = type;
		} else if (gpio->irq_type[offset] != type) {
			bcm2708_gpio_irq_config(gpio, offset, false);
			gpio->irq_type[offset] = type;
			bcm2708_gpio_irq_config(gpio, offset, true);
		}
		break;

	case IRQ_TYPE_EDGE_BOTH:
		if (gpio->irq_type[offset] == IRQ_TYPE_EDGE_RISING) {
			/* RISING already enabled, enable FALLING too */
			gpio->irq_type[offset] = IRQ_TYPE_EDGE_FALLING;
			bcm2708_gpio_irq_config(gpio, offset, true);
			gpio->irq_type[offset] = type;
		} else if (gpio->irq_type[offset] == IRQ_TYPE_EDGE_FALLING) {
			/* FALLING already enabled, enable RISING too */
			gpio->irq_type[offset] = IRQ_TYPE_EDGE_RISING;
			bcm2708_gpio_irq_config(gpio, offset, true);
			gpio->irq_type[offset] = type;
		} else if (gpio->irq_type[offset] != type) {
			bcm2708_gpio_irq_config(gpio, offset, false);
			gpio->irq_type[offset] = type;
			bcm2708_gpio_irq_config(gpio, offset, true);
		}
		break;

	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		if (gpio->irq_type[offset] != type) {
			bcm2708_gpio_irq_config(gpio, offset, false);
			gpio->irq_type[offset] = type;
			bcm2708_gpio_irq_config(gpio, offset, true);
		}
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int bcm2708_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	struct bcm2708_gpio *gpio = container_of(chip, struct bcm2708_gpio, ic);
	unsigned offset = data->hwirq;
	int bank = GPIO_BANK(offset);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&gpio->irq_lock[bank], flags);
	if (gpio->irq_masked)
		ret = __bcm2708_gpio_irq_set_type_masked(gpio, offset, type);
	else
		ret = __bcm2708_gpio_irq_set_type_unmasked(gpio, offset, type);
	spin_unlock_irqrestore(&gpio->irq_lock[bank], flags);
	return ret;
}

static void bcm2708_gpio_free(struct bcm2708_gpio *gpio)
{
	if (gpio->irq_domain) {
		/* can't remove an IRQ domain! */
		WARN_ON(1);
	}
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
	int ret, i;

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
	gpio->set = base + REG_SZ * 0;
	gpio->clear = base + REG_SZ * 1;
	gpio->get = base + REG_SZ * 2;
	gpio->evt_stat = base + REG_SZ * 3;
	gpio->rise_det = base + REG_SZ * 4;
	gpio->fall_det = base + REG_SZ * 5;
	gpio->high_det = base + REG_SZ * 6;
	gpio->low_det = base + REG_SZ * 7;
	gpio->arise_det = base + REG_SZ * 8;
	gpio->afall_det = base + REG_SZ * 9;

	pctl_np = of_parse_phandle(np, "pinctrl", 0);
	if (pctl_np == NULL) {
		dev_err(gpio->dev, "pinctrl node missing\n");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32(pctl_np, "count", &nr_gpios);
	if (ret) {
		dev_err(gpio->dev, "gpio count missing (%d)\n", ret);
		of_node_put(pctl_np);
		goto err;
	}
	if (nr_gpios > MAX_GPIOS) {
		dev_err(gpio->dev, "unhandled gpio count %d\n", nr_gpios);
		of_node_put(pctl_np);
		goto err;
	}

	gpio_base = 0;
	of_property_read_u32(pctl_np, "base", &gpio_base);
	of_node_put(pctl_np);

	gpio->gc.label = dev_name(gpio->dev);
	gpio->gc.dev = gpio->dev;
	gpio->gc.owner = THIS_MODULE;
	gpio->gc.base = gpio_base;
	gpio->gc.ngpio = nr_gpios;
	gpio->gc.request = bcm2708_gpiolib_request;
	gpio->gc.free = bcm2708_gpiolib_free;
	gpio->gc.direction_input = bcm2708_gpiolib_direction_input;
	gpio->gc.direction_output = bcm2708_gpiolib_direction_output;
	gpio->gc.get = bcm2708_gpiolib_get;
	gpio->gc.set = bcm2708_gpiolib_set;
	gpio->gc.to_irq = bcm2708_gpiolib_to_irq;
	gpio->gc.can_sleep = 0;

	gpio->ic.name = "GPIO-event";
	gpio->ic.irq_ack = bcm2708_gpio_irq_ack;
	gpio->ic.irq_mask = bcm2708_gpio_irq_mask_no_ack;
	gpio->ic.irq_mask_ack = bcm2708_gpio_irq_mask_ack;
	gpio->ic.irq_unmask = bcm2708_gpio_irq_unmask;
	gpio->ic.irq_set_type = bcm2708_gpio_irq_set_type;

	gpio->irq_domain = irq_domain_add_linear(np, nr_gpios,
		&irq_domain_simple_ops, NULL);
	if (gpio->irq_domain == NULL) {
		dev_err(gpio->dev, "error creating gpio IRQ domain\n");
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < nr_gpios; i++) {
		int irq = irq_create_mapping(gpio->irq_domain, i);
		gpio->irq_masked[i] = true;
		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_and_handler(irq, &gpio->ic, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	for (i = 0; i <= GPIO_BANK(nr_gpios); i++) {
		/* reset detect flags */
		unsigned long events;
		unsigned offset;
		writel(0, gpio->rise_det + BANK_REG(i));
		writel(0, gpio->fall_det + BANK_REG(i));
		writel(0, gpio->high_det + BANK_REG(i));
		writel(0, gpio->low_det + BANK_REG(i));
		writel(0, gpio->arise_det + BANK_REG(i));
		writel(0, gpio->afall_det + BANK_REG(i));

		/* clear all the events */
		events = readl(gpio->evt_stat + BANK_REG(i));
		for_each_set_bit(offset, &events, GPIOS_PER_BANK)
			writel(BIT(offset), gpio->evt_stat + BANK_REG(i));

		gpio->irq[i] = irq_of_parse_and_map(np, i);
		spin_lock_init(&gpio->irq_lock[i]);
		gpio->irq_data[i].gpio = gpio;
		gpio->irq_data[i].bank = i;

		irq_set_chained_handler(gpio->irq[i], bcm2708_gpio_irq_handler);
		irq_set_handler_data(gpio->irq[i], &gpio->irq_data[i]);
	}

	/* all IRQ handling must be set up before adding the gpio chip */
	ret = gpiochip_add(&gpio->gc);
	if (ret)
		goto err_domain;

	dev_info(gpio->dev, "%d pins at MMIO %#lx\n", nr_gpios,
		(unsigned long)gpio->res.start);

	for (i = 0; i <= GPIO_BANK(nr_gpios); i++)
		dev_info(gpio->dev, "event detection on pins %d-%d (irq = %d)\n",
			i * GPIOS_PER_BANK,
			min((int)nr_gpios, (i + 1) * GPIOS_PER_BANK) - 1,
			gpio->irq[i]);

	platform_set_drvdata(pdev, gpio);
	return 0;

err_domain:
	/* can't remove an IRQ domain! */
	WARN_ON(1);
err:
	bcm2708_gpio_free(gpio);
	return ret;
}

static int bcm2708_gpio_remove(struct platform_device *pdev)
{
	struct bcm2708_gpio *gpio = platform_get_drvdata(pdev);
	int ret;

	/* can't remove an IRQ domain! */
	if (gpio->irq_domain)
		return -EBUSY;

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
