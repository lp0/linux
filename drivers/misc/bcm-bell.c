/*
 * Copyright (C) 2010 Broadcom
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This device provides a shared mechanism for using the doorbells
 * shared between the ARM and the VideoCore processor.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "bcm-bell.h"

#define MODULE_NAME "bcm-bell"

#define BELL_RING 0x4

struct bcm_bell {
	struct device *dev;
	struct resource res;
	void __iomem *base;

	/* used to lock open, handler and data */
	spinlock_t lock;
	bool open;
	void (*handler)(struct bcm_bell *, void *);
	void *data;

	u32 irq;
};

static irqreturn_t bcm_bell_irq_handler(int irq, void *dev_id)
{
	struct bcm_bell *bell = dev_id;
	int status = readl(bell->base);

	if (status & BELL_RING) {
		void (*handler)(struct bcm_bell *, void *);
		void *data;

		spin_lock(&bell->lock);
		handler = bell->handler;
		data = bell->data;
		spin_unlock(&bell->lock);

		if (handler)
			handler(bell, data);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static int __devinit bcm_bell_probe(struct platform_device *of_dev)
{
	struct device_node *node = of_dev->dev.of_node;
	struct bcm_bell *bell = devm_kzalloc(&of_dev->dev,
		sizeof(*bell), GFP_KERNEL);
	int ret;

	if (bell == NULL)
		return -ENOMEM;

	bell->dev = &of_dev->dev;
	spin_lock_init(&bell->lock);

	if (of_address_to_resource(node, 0, &bell->res))
		return -EINVAL;

	if (resource_size(&bell->res) < sizeof(u32)) {
		dev_err(bell->dev, "resource too small (%#x)\n",
			resource_size(&bell->res));
		return -EINVAL;
	}

	bell->base = devm_request_and_ioremap(bell->dev, &bell->res);
	if (!bell->base) {
		dev_err(bell->dev, "error mapping io at %#lx\n",
			(unsigned long)bell->res.start);
		return -EIO;
	}

	/* register the interrupt handler */
	bell->irq = irq_of_parse_and_map(node, 0);
	if (bell->irq) {
		bell->handler = NULL;

		ret = devm_request_irq(bell->dev, bell->irq,
			bcm_bell_irq_handler, IRQF_SHARED,
			dev_name(bell->dev), bell);
		if (ret) {
			dev_err(bell->dev, "unable to request irq %d", bell->irq);
			spin_unlock_irq(&bell->lock);
			return ret;
		}

		dev_info(bell->dev, "doorbell at MMIO %#lx (irq = %d)\n",
			(unsigned long)bell->res.start, bell->irq);
	} else {
		dev_info(bell->dev, "doorbell at MMIO %#lx\n",
			(unsigned long)bell->res.start);
	}

	return 0;
}

static int bcm_bell_remove(struct platform_device *of_dev)
{
	return 0;
}

struct bcm_bell *bcm_bell_get(struct device_node *node, const char *pbell)
{
	struct device_node *bell_node;
	struct platform_device *pdev;
	struct bcm_bell *bell;
	int ret;
	unsigned long flags;

	if (node == NULL || pbell == NULL)
		return ERR_PTR(-EINVAL);

	bell_node = of_parse_phandle(node, pbell, 0);
	if (bell_node == NULL)
		return ERR_PTR(-ENOENT);

	pdev = of_find_device_by_node(bell_node);
	if (pdev == NULL) {
		of_node_put(bell_node);
		return ERR_PTR(-ENODEV);
	}

	/* swap our node ref for a device ref */
	get_device(&pdev->dev);
	of_node_put(bell_node);

	if (pdev->dev.driver == NULL
			|| pdev->dev.driver->owner != THIS_MODULE) {
		ret = -ENODEV;
		goto put_dev;
	}

	bell = platform_get_drvdata(pdev);
	if (bell == NULL || bell->dev != &pdev->dev) {
		ret = -EINVAL;
		goto put_dev;
	}

	spin_lock_irqsave(&bell->lock, flags);
	if (bell->open) {
		spin_unlock_irqrestore(&bell->lock, flags);
		ret = -EBUSY;
		goto put_dev;
	} else {
		bell->open = true;
	}
	spin_unlock_irqrestore(&bell->lock, flags);
	return bell;

put_dev:
	put_device(&pdev->dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(bcm_bell_get);

int bcm_bell_read(struct bcm_bell *bell,
	void (*handler)(struct bcm_bell *bell, void *data), void *data)
{
	unsigned long flags;

	if (bell == NULL || handler == NULL)
		return -EINVAL;

	if (!bell->irq)
		return -EPERM;

	spin_lock_irqsave(&bell->lock, flags);
	bell->handler = handler;
	bell->data = data;
	spin_unlock_irqrestore(&bell->lock, flags);

	if (handler == NULL)
		synchronize_irq(bell->irq);
	return 0;
}
EXPORT_SYMBOL_GPL(bcm_bell_read);

void bcm_bell_put(struct bcm_bell *bell)
{
	unsigned long flags;

	if (bell != NULL) {
		spin_lock_irqsave(&bell->lock, flags);
		if (bell->irq)
			bell->handler = NULL;
		bell->open = false;
		spin_unlock_irqrestore(&bell->lock, flags);

		if (bell->irq)
			synchronize_irq(bell->irq);
		put_device(bell->dev);
	} else {
		WARN_ON(1);
	}
}
EXPORT_SYMBOL_GPL(bcm_bell_put);

int bcm_bell_write(struct bcm_bell *bell)
{
	if (bell == NULL)
		return -EINVAL;

	if (bell->irq)
		return -EPERM;

	writel(0, bell->base);
	return 0;
}
EXPORT_SYMBOL_GPL(bcm_bell_write);

static struct of_device_id bcm_bell_dt_match[] __devinitconst = {
	{ .compatible = "broadcom,bcm2708-bell" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm_bell_dt_match);

static struct platform_driver bcm_bell_driver = {
	.probe = bcm_bell_probe,
	.remove = bcm_bell_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm_bell_dt_match
	}
};

static int __init bcm_bell_init(void)
{
	int ret;
	printk(KERN_INFO MODULE_NAME ": Broadcom VideoCore Doorbell driver\n");

	ret = platform_driver_register(&bcm_bell_driver);
	if (ret)
		printk(KERN_ERR MODULE_NAME ": registration failed (%d)\n", ret);

	return ret;
}

static void __exit bcm_bell_exit(void)
{
	platform_driver_unregister(&bcm_bell_driver);
}

arch_initcall(bcm_bell_init);
module_exit(bcm_bell_exit);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("ARM doorbell I/O to VideoCore processor");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm-bell");
