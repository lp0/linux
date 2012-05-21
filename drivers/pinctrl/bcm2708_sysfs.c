/*
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "bcm2708.h"

#define GPIO_IN_STR	"GPIO_IN"
#define GPIO_OUT_STR	"GPIO_OUT"

/* split the name up by NAME_SPLIT as some gpios have more than one name */
static void bcm2708_pinctrl_sysfs_show_gpio_split(char *out, int *len,
	const char *name, bool selected)
{
	char buf[NAME_LEN+1];
	char *tmp = buf;
	char *cur;
	int ret;

	strncpy(buf, name, NAME_LEN);
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

static int to_lockint(struct bcm2708_pinctrl *pc, int p)
{
	if (pc->pm_locked[p])
		return 2;
	else if (pc->usr_locked[p])
		return 1;
	else
		return 0;
}

static const char *to_lockstr(struct bcm2708_pinctrl *pc, int p)
{
	if (pc->pm_locked[p])
		return "*";
	else if (pc->usr_locked[p])
		return "+";
	else
		return "-";
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
	status = bcm2708_pinctrl_fsel_get(pc, p);

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

	ret = snprintf(buf + len, PAGE_SIZE - len, " (locked=%d)\n",
		to_lockint(pc, p));
	if (ret < 0)
		goto err;
	len += ret;

	spin_unlock_irq(&pc->lock);
	return len;

err:
	return -EIO;
}

/* split the name up by NAME_SPLIT as some gpios have more than one name */
static bool bcm2708_pinctrl_sysfs_store_gpio_match(const char *value,
	const char *name)
{
	char buf[NAME_LEN+1];
	char *tmp = buf;
	char *cur;

	strncpy(buf, name, NAME_LEN);
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
	} else if (sysfs_streq(buf, "lock")) {
		if (!pc->pm_locked[p])
			pc->usr_locked[p] = true;
		else
			len = -EBUSY;
	} else if (sysfs_streq(buf, "unlock")) {
		pc->usr_locked[p] = false;
	} else {
		for (a = 0; a < ALTS; a++) {
			if (bcm2708_pinctrl_sysfs_store_gpio_match(buf,
					pc->gpio[p][a])) {
				value = to_fsel_value(a);
				break;
			}
		}

		if (value != FSEL_NONE)
			len = -EINVAL;
	}

	if (value != FSEL_NONE) {
		if (pc->pm_locked[p] || pc->usr_locked[p])
			len = -EBUSY;
		else
			bcm2708_pinctrl_fsel_set(pc, p, value);
	}

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

static int bcm2708_pinctrl_sysfs_register_pin(struct bcm2708_pinctrl *pc,
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

	sysfs_attr_init(&pc->attr_gpio[p].dev.attr);
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

	sysfs_attr_init(&pc->attr_pins[p].dev.attr);
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

static void bcm2708_pinctrl_sysfs_unregister_pin(struct bcm2708_pinctrl *pc,
	int p)
{
	device_remove_file(pc->dev, &pc->attr_gpio[p].dev);
	kfree(pc->attr_gpio[p].dev.attr.name);

	if (!strcmp("", pc->pins[p]))
		return;

	device_remove_file(pc->dev, &pc->attr_pins[p].dev);
	kfree(pc->attr_pins[p].dev.attr.name);
}

static int bcm2708_pinmux_sysfs_show_group(struct device *dev,
	struct device_attribute *dattr, char *buf)
{
	struct bcm2708_pinmux_attr *attr = container_of(dattr,
		struct bcm2708_pinmux_attr, dev);
	struct bcm2708_pinmux *pm;
	struct bcm2708_pinctrl *pc;
	bool selected[MAX_PINS];
	int count = 0;
	int len = 0;
	int ret = 0;
	int p, i;

	if (attr == NULL)
		return -ENODEV;

	pm = attr->pm;
	if (pm == NULL)
		return -ENODEV;

	pc = attr->pc;
	if (pc == NULL)
		return -ENODEV;

	spin_lock_irq(&pc->lock);
	if (!pc->active) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	for (i = 0; i < pm->count; i++) {
		p = pm->pins[i];
		selected[p] = bcm2708_pinctrl_fsel_get(pc, p) == pm->fsel[i];
		if (selected[p])
			count++;
	}

	ret = snprintf(buf + len, PAGE_SIZE - len,
		"%02d/%02d:", count, pm->count);
	if (ret < 0)
		goto err;
	len += ret;

	for (i = 0; i < pm->count; i++) {
		p = pm->pins[i];
		if (!strcmp("", pc->pins[p])) {
			ret = snprintf(buf + len, PAGE_SIZE - len, selected[p]
				? " [GPIO_%02d](%s)"
					: " GPIO_%02d(%s)",
				p, to_lockstr(pc, p));
		} else {
			ret = snprintf(buf + len, PAGE_SIZE - len, selected[p]
				? " [GPIO_%02d/PIN_%s](%s)"
					: " GPIO_%02d/PIN_%s(%s)",
				p, pc->pins[p], to_lockstr(pc, p));
		}
		if (ret < 0)
			goto err;
		len += ret;
	}

	ret = snprintf(buf + len, PAGE_SIZE - len, "\n");
	if (ret < 0)
		goto err;
	len += ret;

	spin_unlock_irq(&pc->lock);
	return len;

err:
	return -EIO;
}

static ssize_t bcm2708_pinmux_sysfs_store_group(struct device *dev,
	struct device_attribute *dattr, const char *buf, size_t count)
{
	struct bcm2708_pinmux_attr *attr = container_of(dattr,
		struct bcm2708_pinmux_attr, dev);
	struct bcm2708_pinmux *pm;
	struct bcm2708_pinctrl *pc;
	int len = strlen(buf);
	int p, i;

	if (attr == NULL)
		return -ENODEV;

	pm = attr->pm;
	if (pm == NULL)
		return -ENODEV;

	pc = attr->pc;
	if (pc == NULL)
		return -ENODEV;

	spin_lock_irq(&pc->lock);
	if (!pc->active) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	if (sysfs_streq(buf, "select")) {
		for (i = 0; i < pm->count; i++) {
			p = pm->pins[i];
			if (!(pc->pm_locked[p] || pc->usr_locked[p]))
				bcm2708_pinctrl_fsel_set(pc, p, pm->fsel[i]);
		}
	} else if (sysfs_streq(buf, "deselect")) {
		for (i = 0; i < pm->count; i++) {
			p = pm->pins[i];
			if (!(pc->pm_locked[p] || pc->usr_locked[p]))
				bcm2708_pinctrl_fsel_set(pc, p, FSEL_GPIO_IN);
		}
	} else if (sysfs_streq(buf, "lock")) {
		for (i = 0; i < pm->count; i++) {
			p = pm->pins[i];
			if (!pc->pm_locked[p])
				pc->usr_locked[p] = true;
		}
	} else if (sysfs_streq(buf, "unlock")) {
		for (i = 0; i < pm->count; i++) {
			p = pm->pins[i];
			pc->usr_locked[p] = false;
		}
	} else {
		len = -EINVAL;
	}

	spin_unlock_irq(&pc->lock);
	return len;
}

static int bcm2708_pinmux_sysfs_register_group(struct bcm2708_pinctrl *pc,
	struct bcm2708_pinmux *pm)
{
	int ret;

	pm->attr.pm = pm;
	pm->attr.pc = pc;
	pm->attr.dev.attr.name = kasprintf(GFP_KERNEL, "group_%s", pm->name);
	pm->attr.dev.attr.mode = S_IWUSR | S_IRUGO;
	pm->attr.dev.show = bcm2708_pinmux_sysfs_show_group;
	pm->attr.dev.store = bcm2708_pinmux_sysfs_store_group;

	if (pm->attr.dev.attr.name == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	sysfs_attr_init(&pm->attr.dev.attr);
	ret = device_create_file(pc->dev, &pm->attr.dev);
	if (ret)
		goto err_free;

	return 0;

err_free:
	kfree(pm->attr.dev.attr.name);
err:
	return ret;
}

static void bcm2708_pinmux_sysfs_unregister_group(struct bcm2708_pinctrl *pc,
	struct bcm2708_pinmux *pm)
{
	device_remove_file(pc->dev, &pm->attr.dev);
	kfree(pm->attr.dev.attr.name);
}

int bcm2708_pinctrl_sysfs_register(struct bcm2708_pinctrl *pc)
{
	struct bcm2708_pinmux *group;
	int ret, p;

	for (p = 0; p < pc->nr_pins; p++) {
		ret = bcm2708_pinctrl_sysfs_register_pin(pc, p);
		if (ret) {
			dev_err(pc->dev,
				"sysfs register failure for pin %d (%d)\n",
				p, ret);
			while (--p >= 0)
				bcm2708_pinctrl_sysfs_unregister_pin(pc, p);
			return ret;
		}
	}

	list_for_each_entry(group, &pc->groups, list) {
		ret = bcm2708_pinmux_sysfs_register_group(pc, group);
		if (!ret)
			continue;

		dev_err(pc->dev, "sysfs register failure for group %s (%d)\n",
			group->name, ret);
		list_for_each_entry_continue_reverse(group, &pc->groups, list)
			bcm2708_pinmux_sysfs_unregister_group(pc, group);
		goto err_group;
	}

	return 0;

err_group:
	for (p = 0; p < pc->nr_pins; p++)
		bcm2708_pinctrl_sysfs_unregister_pin(pc, p);
	return ret;
}

void bcm2708_pinctrl_sysfs_unregister(struct bcm2708_pinctrl *pc)
{
	struct bcm2708_pinmux *group;
	int p;

	for (p = 0; p < pc->nr_pins; p++)
		bcm2708_pinctrl_sysfs_unregister_pin(pc, p);

	list_for_each_entry(group, &pc->groups, list)
		bcm2708_pinmux_sysfs_unregister_group(pc, group);
}
