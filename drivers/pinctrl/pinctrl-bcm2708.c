/*
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * publishhed by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>

#define MODULE_NAME "bcm2708_pinctrl"

/*
static struct pinctrl_ops bcm2708_pctrl_ops = {
};

static struct pinmux_ops bcm2708_pmx_ops = {
};
*/

#define REG_SZ 0x1c

#define FSEL_MASK 0x7
#define FSEL_REG(p) ((p / 10) * 4)
#define FSEL_SHIFT(p) ((p - 10 * (p / 10)) * 3)

#define GPIO_IN_STR "GPIO_IN"
#define GPIO_OUT_STR "GPIO_OUT"

#define BUF_LEN 64

enum pin_fsel {
	FSEL_NONE = -1,
	FSEL_GPIO_IN,	/* 000 */
	FSEL_GPIO_OUT,	/* 001 */
	FSEL_ALT_BASE,
	FSEL_ALT5 = 2,	/* 010 */
	FSEL_ALT4,	/* 011 */
	FSEL_ALT0,	/* 100 */
	FSEL_ALT1,	/* 101 */
	FSEL_ALT2,	/* 110 */
	FSEL_ALT3	/* 111 */
};

#define ALTS 6
#define PINS 54
#define FSELS (FSEL_ALT_BASE+6) /* +2 for GPIO */
#define NAME_SPLIT "|"

enum def_pull { PULL_NONE, PULL_LOW, PULL_HIGH };

struct bcm2708_pinctrl;

struct bcm2708_pinctrl_attr {
	struct bcm2708_pinctrl *pc;
	int pin;
	struct device_attribute dev;
};

struct bcm2708_pinpair {
	int pin;
	enum pin_fsel fsel;
};

struct bcm2708_pinmux {
	struct list_head list;
	char *name;
	int count;
	struct bcm2708_pinpair pins[0];
};

struct bcm2708_pinctrl {
	struct device *dev;
	struct spinlock lock;
	bool active;

	struct resource res;
	void __iomem *base;

	const char *gpio[PINS][ALTS];
	const char *pins[PINS];
	struct list_head groups;
	u32 pull[PINS];

	struct bcm2708_pinctrl_attr attr_gpio[PINS];
	struct bcm2708_pinctrl_attr attr_pins[PINS];
};

static inline int to_alt_index(enum pin_fsel value)
{
	switch (value) {
	case FSEL_ALT0: return 0;
	case FSEL_ALT1: return 1;
	case FSEL_ALT2: return 2;
	case FSEL_ALT3: return 3;
	case FSEL_ALT4: return 4;
	case FSEL_ALT5: return 5;
	default:
		WARN_ON(1);
		break;
	}
	return 0;
}

static inline enum pin_fsel to_fsel_value(int index)
{
	switch (index) {
	case 0: return FSEL_ALT0;
	case 1: return FSEL_ALT1;
	case 2: return FSEL_ALT2;
	case 3: return FSEL_ALT3;
	case 4: return FSEL_ALT4;
	case 5: return FSEL_ALT5;
	default:
		WARN_ON(1);
		break;
	}
	return FSEL_ALT0;
}

static enum pin_fsel bcm2708_fsel_get(struct bcm2708_pinctrl *pc, int p)
{
	enum pin_fsel status;
	void __iomem *reg = pc->base + FSEL_REG(p);
	u32 val = readl(reg);

	status = (val >> FSEL_SHIFT(p)) & FSEL_MASK;
	dev_dbg(pc->dev, "get %08x@%p (%d = %d)\n", val, reg, p, status);
	return status;
}

static void bcm2708_fsel_set(struct bcm2708_pinctrl *pc, int p,
	enum pin_fsel set)
{
	enum pin_fsel cur;
	void __iomem *reg = pc->base + FSEL_REG(p);
	u32 val = readl(reg);

	cur = (val >> FSEL_SHIFT(p)) & FSEL_MASK;
	dev_dbg(pc->dev, "read %08x@%p (%d = %d)\n", val, reg, p, cur);

	/* all GPIOs need to go via GPIO_IN first or it'll lock up */
	if (cur != FSEL_GPIO_IN && set != FSEL_GPIO_IN) {
		val &= ~(FSEL_MASK << FSEL_SHIFT(p));
		val |= (FSEL_GPIO_IN & FSEL_MASK) << FSEL_SHIFT(p);
		dev_dbg(pc->dev, "write %08x@%p (%d = %d)\n", val, reg, p,
			FSEL_GPIO_IN);
	}

	val &= ~(FSEL_MASK << FSEL_SHIFT(p));
	val |= (set & FSEL_MASK) << FSEL_SHIFT(p);
	dev_dbg(pc->dev, "write %08x@%p (%d = %d)\n", val, reg, p, set);
	writel(val, reg);
}

/* split the name up by NAME_SPLIT as some gpios have more than one name */
static void bcm2708_pinctrl_sysfs_show_gpio_split(char *out, int *len,
	const char *name, bool selected)
{
	char buf[BUF_LEN+1];
	char *tmp = buf;
	char *cur;
	int ret;

	strncpy(buf, name, BUF_LEN);
	while (tmp) {
		cur = strsep(&tmp, NAME_SPLIT);
		if (!strcmp("", cur))
			continue;

		ret = snprintf(out + *len, PAGE_SIZE - *len,
			selected ? " [%s]" : " %s", cur);
		if (ret < 0) {
			*len = -1;
			return;
		}
		*len += ret;
	}
}

static int bcm2708_pinctrl_sysfs_show_gpio(struct device *dev,
	struct device_attribute *dattr, char *buf)
{
	struct bcm2708_pinctrl_attr *attr = container_of(dattr,
		struct bcm2708_pinctrl_attr, dev);
	struct bcm2708_pinctrl *pc;
	enum pin_fsel status;
	int p, a, i;
	int len = 0;
	int ret = 0;

	if (attr == NULL)
		return -ENODEV;

	pc = attr->pc;
	if (pc == NULL)
		return -ENODEV;

	spin_lock_irq(&pc->lock);
	if (!pc->active) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	p = attr->pin;
	status = bcm2708_fsel_get(pc, p);

	if (status == FSEL_GPIO_IN) {
		ret = snprintf(buf + len, PAGE_SIZE - len, "[%s] %s",
			GPIO_IN_STR, GPIO_OUT_STR);
		i = -1;
	} else if (status == FSEL_GPIO_OUT) {
		ret = snprintf(buf + len, PAGE_SIZE - len, "%s [%s]",
			GPIO_IN_STR, GPIO_OUT_STR);
		i = -1;
	} else {
		ret = snprintf(buf + len, PAGE_SIZE - len, "%s %s",
			GPIO_IN_STR, GPIO_OUT_STR);
		i = to_alt_index(status);
	}
	if (ret < 0)
		goto err;
	len += ret;

	for (a = 0; a < ALTS; a++) {
		if (!strcmp("", pc->gpio[p][a]))
			continue;

		bcm2708_pinctrl_sysfs_show_gpio_split(buf, &len,
			pc->gpio[p][a], a == i);
		if (len < 0)
			goto err;
	}
	strcat(buf, "\n");
	len++;

	spin_unlock_irq(&pc->lock);
	return len;

err:
	return -EIO;
}

/* split the name up by NAME_SPLIT as some gpios have more than one name */
static bool bcm2708_pinctrl_sysfs_store_gpio_match(const char *value,
	const char *name)
{
	char buf[BUF_LEN+1];
	char *tmp = buf;
	char *cur;

	strncpy(buf, name, BUF_LEN);
	while (tmp) {
		cur = strsep(&tmp, NAME_SPLIT);
		if (strcmp("", cur) && sysfs_streq(cur, value))
			return true;
	}
	return false;
}

static ssize_t bcm2708_pinctrl_sysfs_store_gpio(struct device *dev,
	struct device_attribute *dattr, const char *buf, size_t count)
{
	struct bcm2708_pinctrl_attr *attr = container_of(dattr,
		struct bcm2708_pinctrl_attr, dev);
	struct bcm2708_pinctrl *pc;
	enum pin_fsel value = FSEL_NONE;
	int len = strlen(buf);
	int p, a;

	if (attr == NULL)
		return -ENODEV;

	pc = attr->pc;
	if (pc == NULL)
		return -ENODEV;

	spin_lock_irq(&pc->lock);
	if (!pc->active) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	p = attr->pin;
	if (sysfs_streq(buf, GPIO_IN_STR)) {
		value = FSEL_GPIO_IN;
	} else if (sysfs_streq(buf, GPIO_OUT_STR)) {
		value = FSEL_GPIO_OUT;
	} else {
		for (a = 0; a < ALTS; a++) {
			if (bcm2708_pinctrl_sysfs_store_gpio_match(buf,
					pc->gpio[p][a])) {
				value = to_fsel_value(a);
				break;
			}
		}
	}

	if (value == FSEL_NONE) {
		spin_unlock_irq(&pc->lock);
		return -EINVAL;
	}

	bcm2708_fsel_set(pc, p, value);
	spin_unlock_irq(&pc->lock);
	return len;
}

static int bcm2708_pinctrl_sysfs_show_pins(struct device *dev,
	struct device_attribute *dattr, char *buf)
{
	struct bcm2708_pinctrl_attr *attr = container_of(dattr,
		struct bcm2708_pinctrl_attr, dev);
	struct bcm2708_pinctrl *pc;

	if (attr == NULL)
		return -ENODEV;

	pc = attr->pc;
	if (pc == NULL)
		return -ENODEV;

	return bcm2708_pinctrl_sysfs_show_gpio(dev,
		&pc->attr_pins[attr->pin].dev, buf);
}

static ssize_t bcm2708_pinctrl_sysfs_store_pins(struct device *dev,
	struct device_attribute *dattr, const char *buf, size_t count)
{
	struct bcm2708_pinctrl_attr *attr = container_of(dattr,
		struct bcm2708_pinctrl_attr, dev);
	struct bcm2708_pinctrl *pc;

	if (attr == NULL)
		return -ENODEV;

	pc = attr->pc;
	if (pc == NULL)
		return -ENODEV;

	return bcm2708_pinctrl_sysfs_store_gpio(dev,
		&pc->attr_pins[attr->pin].dev, buf, count);
}

static int bcm2708_pinctrl_sysfs_register(struct bcm2708_pinctrl *pc,
	int p)
{
	int ret;

	pc->attr_gpio[p].pc = pc;
	pc->attr_gpio[p].pin = p;
	pc->attr_gpio[p].dev.attr.name = kasprintf(GFP_KERNEL, "gpio_%02d", p);
	pc->attr_gpio[p].dev.attr.mode = S_IWUSR | S_IRUGO;
	pc->attr_gpio[p].dev.show = bcm2708_pinctrl_sysfs_show_gpio;
	pc->attr_gpio[p].dev.store = bcm2708_pinctrl_sysfs_store_gpio;

	if (pc->attr_gpio[p].dev.attr.name == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	ret = device_create_file(pc->dev, &pc->attr_gpio[p].dev);
	if (ret)
		goto err_free_gpio;

	if (!strcmp("", pc->pins[p]))
		return 0;

	pc->attr_pins[p].pc = pc;
	pc->attr_pins[p].pin = p;
	pc->attr_pins[p].dev.attr.name = kasprintf(GFP_KERNEL, "pin_%s",
		pc->pins[p]);
	pc->attr_pins[p].dev.attr.mode = S_IWUSR | S_IRUGO;
	pc->attr_pins[p].dev.show = bcm2708_pinctrl_sysfs_show_pins;
	pc->attr_pins[p].dev.store = bcm2708_pinctrl_sysfs_store_pins;

	if (pc->attr_pins[p].dev.attr.name == NULL) {
		ret = -ENOMEM;
		goto err_remove_gpio;
	}

	ret = device_create_file(pc->dev, &pc->attr_pins[p].dev);
	if (ret)
		goto err_free_pins;

	return 0;

err_free_pins:
	kfree(pc->attr_pins[p].dev.attr.name);
err_remove_gpio:
	device_remove_file(pc->dev, &pc->attr_gpio[p].dev);
err_free_gpio:
	kfree(pc->attr_gpio[p].dev.attr.name);
err:
	return ret;
}

static void bcm2708_pinctrl_sysfs_unregister(struct bcm2708_pinctrl *pc,
	int p)
{
	device_remove_file(pc->dev, &pc->attr_gpio[p].dev);
	kfree(pc->attr_gpio[p].dev.attr.name);

	if (!strcmp("", pc->pins[p]))
		return;

	device_remove_file(pc->dev, &pc->attr_pins[p].dev);
	kfree(pc->attr_pins[p].dev.attr.name);
}

static int bcm2708_pinmux_group_init_one(struct list_head *groups, int p,
	enum pin_fsel f, const char *name)
{
	struct bcm2708_pinmux *group = NULL;
	struct bcm2708_pinmux *tmp;

	if (!strcmp("", name))
		return 0;

	list_for_each_entry(tmp, groups, list) {
		if (!strcmp(tmp->name, name)) {
			group = tmp;
			break;
		}
	};

	if (group == NULL) {
		group = kmalloc(sizeof(*group)+sizeof(group->pins), GFP_KERNEL);
		if (group == NULL)
			return -ENOMEM;

		group->name = kstrdup(name, GFP_KERNEL);
		group->count = 0;
		list_add_tail(&group->list, groups);
	} else {
		tmp = krealloc(group, sizeof(*group)
			+ sizeof(group->pins)*(group->count + 1), GFP_KERNEL);
		if (tmp == NULL)
			return -ENOMEM;
		group = tmp;
	}

	group->pins[group->count].pin = p;
	group->pins[group->count].fsel = p;
	group->count++;
	return 0;
}

/* split the name up by NAME_SPLIT as some function
 * selections belong to more than one group
 */
static int bcm2708_pinmux_group_init(struct list_head *groups, int p,
	enum pin_fsel f, const char *name)
{
	char buf[BUF_LEN+1];
	char *tmp = buf;
	char *cur;
	int ret;

	strncpy(buf, name, BUF_LEN);
	while (tmp) {
		cur = strsep(&tmp, NAME_SPLIT);
		ret = bcm2708_pinmux_group_init_one(groups, p, f, cur);
		if (ret)
			return ret;
	}
	return 0;
}

static int __devinit bcm2708_pinmux_init(struct platform_device *pdev,
	struct bcm2708_pinctrl *pc)
{
	struct device_node *np = pdev->dev.of_node;
	int ret, p, f;

	for (p = 0; p < PINS; p++) {
		for (f = 0; f < FSELS; f++) {
			const char *name;

			ret = of_property_read_string_index(np, "groups",
				(p * FSELS) + f, &name);
			if (ret) {
				dev_err(pc->dev,
					"error reading group info %d %d (%d)\n",
					p, f, ret);
				goto err;
			}

			/* need to map the alt to an fsel
			 * (gpio_i and gpio_o are already 0 and 1)
			 */
			if (f >= FSEL_ALT_BASE)
				f = to_fsel_value(f - FSEL_ALT_BASE);

			bcm2708_pinmux_group_init(&pc->groups, p, f, name);
		}
	}

err:
	return ret;
}

static int __devinit bcm2708_pinctrl_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct bcm2708_pinctrl *pc = kzalloc(sizeof(*pc), GFP_KERNEL);
	int ret, p, a;

	if (pc == NULL)
		return -ENOMEM;

	pc->dev = &pdev->dev;
	spin_lock_init(&pc->lock);
	pc->active = false;
	INIT_LIST_HEAD(&pc->groups);

	if (of_address_to_resource(np, 0, &pc->res)) {
		ret = -EINVAL;
		goto err;
	}

	if (resource_size(&pc->res) < REG_SZ) {
		dev_err(pc->dev, "resource size too small (%#x)\n",
			resource_size(&pc->res));
		ret = -EINVAL;
		goto err;
	}

	if (!request_region(pc->res.start, resource_size(&pc->res),
			np->full_name)) {
		dev_err(pc->dev, "resource %#lx unavailable\n",
			(unsigned long)pc->res.start);
		ret = -EBUSY;
		goto err;
	}

	pc->base = ioremap(pc->res.start, resource_size(&pc->res));
	if (!pc->base) {
		dev_err(pc->dev, "error mapping io at %#lx\n",
			(unsigned long)pc->res.start);
		ret = -EIO;
		goto err;
	}

	for (p = 0; p < PINS; p++) {
		for (a = 0; a < ALTS; a++) {
			ret = of_property_read_string_index(np, "gpio",
				(p * ALTS) + a, &pc->gpio[p][a]);
			if (ret) {
				dev_err(pc->dev,
					"error reading gpio info %d %d (%d)\n",
					p, a, ret);
				goto err;
			}
		}

		ret = of_property_read_string_index(np, "pins", p, &pc->pins[p]);
		if (ret) {
			dev_err(pc->dev,
				"error reading pin info %d (%d)\n", p, ret);
			goto err;
		}

		ret = of_property_read_u32_array(np, "pull", pc->pull, PINS);
		if (ret) {
			dev_err(pc->dev,
				"error reading pull info (%d)\n", ret);
			goto err;
		}
	}

	for (p = 0; p < PINS; p++) {
		ret = bcm2708_pinctrl_sysfs_register(pc, p);
		if (ret) {
			dev_err(pc->dev,
				"sysfs register failure for pin %d (%d)\n",
				p, ret);
			while (--p >= 0)
				bcm2708_pinctrl_sysfs_unregister(pc, p);
			goto err;
		}
	}

	spin_lock_irq(&pc->lock);
	pc->active = true;
	spin_unlock_irq(&pc->lock);

	dev_info(pc->dev, "%d pins at MMIO %#lx\n", PINS,
		(unsigned long)pc->res.start);
	
	platform_set_drvdata(pdev, pc);
	return 0;

err:
	kfree(pc);
	return ret;
}

static int __devexit bcm2708_pinctrl_remove(struct platform_device *pdev)
{
	struct bcm2708_pinctrl *pc = platform_get_drvdata(pdev);
	int p;

	/* make sure no sysfs activity can occur */
	spin_lock_irq(&pc->lock);
	pc->active = false;
	spin_unlock_irq(&pc->lock);

	for (p = 0; p < PINS; p++)
		bcm2708_pinctrl_sysfs_unregister(pc, p);

	kfree(pc);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct of_device_id bcm2708_pinctrl_match[] __devinitconst = {
	{ .compatible = "broadcom,bcm2708-pinctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm2708_pinctrl_match);

static struct platform_driver bcm2708_pinctrl_driver = {
	.probe = bcm2708_pinctrl_probe,
	.remove = bcm2708_pinctrl_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm2708_pinctrl_match
	}
};
module_platform_driver(bcm2708_pinctrl_driver);

MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("BCM2708 Pin control driver");
MODULE_LICENSE("GPL");
