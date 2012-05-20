/*
 * Copyright 2012 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * publishhed by the Free Software Foundation.
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>

#include "bcm2708.h"

#define MODULE_NAME "pinctrl-bcm2708"

static const char *fsel_names[FSELS] = {
	"IN",
	"OUT",
	"ALT5",
	"ALT4",
	"ALT0",
	"ALT1",
	"ALT2",
	"ALT3"
};

static int bcm2708_pinctrl_list_groups(struct pinctrl_dev *pctl,
	unsigned selector)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	return selector < pc->nr_groups ? 0 : -EINVAL;
}

static const char *bcm2708_pinctrl_get_group_name(struct pinctrl_dev *pctl,
	unsigned selector)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	if (selector >= pc->nr_groups)
		return NULL;

	return pc->grpidx[selector]->name;
}

static int bcm2708_pinctrl_get_group_pins(struct pinctrl_dev *pctl,
	unsigned selector, const unsigned **pins, unsigned *num_pins)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	if (selector >= pc->nr_groups)
		return -EINVAL;

	*pins = pc->grpidx[selector]->pins;
	*num_pins = pc->grpidx[selector]->count;
	return 0;
}

static int bcm2708_pinmux_request(struct pinctrl_dev *pctl, unsigned offset)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	int ret = 0;

	dev_dbg(pc->dev, "pinmux_request %d\n", offset);

	spin_lock_irq(&pc->lock);
	if (!pc->active) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	if (pc->usr_locked[offset]) {
		ret = -EBUSY;
	} else {
		WARN_ON(pc->pm_locked[offset]);
		pc->pm_locked[offset] = true;
	}

	spin_unlock_irq(&pc->lock);
	return ret;
}

static int bcm2708_pinmux_free(struct pinctrl_dev *pctl, unsigned offset)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	dev_dbg(pc->dev, "pinmux_free %d\n", offset);

	spin_lock_irq(&pc->lock);
	if (!pc->active) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	WARN_ON(!pc->pm_locked[offset]);
	pc->pm_locked[offset] = false;

	spin_unlock_irq(&pc->lock);
	return 0;
}

static int bcm2708_pinmux_list_functions(struct pinctrl_dev *pctl,
	unsigned selector)
{
	return selector < FSELS ? 0 : -EINVAL;
}

static const char *bcm2708_pinmux_get_function_name(struct pinctrl_dev *pctl,
	unsigned selector)
{
	if (selector >= FSELS)
		return NULL;

	return fsel_names[selector];
}

static int bcm2708_pinmux_get_function_groups(struct pinctrl_dev *pctl,
	unsigned selector, const char * const **groups,
	unsigned * const num_groups)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	if (selector >= FSELS)
		return -EINVAL;

	*groups = pc->grpnam;
	*num_groups = pc->nr_groups;
	return 0;
}

static int bcm2708_pinmux_enable(struct pinctrl_dev *pctl,
	unsigned func_selector, unsigned group_selector)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	struct bcm2708_pinmux *pm = pc->grpidx[group_selector];
	bool busy = false;
	int ret = 0;
	int p, i;

	if (func_selector >= FSELS || group_selector >= pc->nr_groups)
		return -EINVAL;

	dev_dbg(pc->dev, "pinmux_enable func=%s group=%s\n",
		fsel_names[func_selector], pc->grpidx[group_selector]->name);

	spin_lock_irq(&pc->lock);
	if (!pc->active) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	for (i = 0; i < pm->count; i++)
		busy |= pc->usr_locked[pm->pins[i]];

	if (busy) {
		ret = -EBUSY;
	} else {
		for (i = 0; i < pm->count; i++) {
			WARN_ON(pc->pm_locked[pm->pins[i]]);
			pc->pm_locked[pm->pins[i]] = true;
		}
	}

	spin_unlock_irq(&pc->lock);
	return ret;
}

static void bcm2708_pinmux_disable(struct pinctrl_dev *pctl,
	unsigned func_selector, unsigned group_selector)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	struct bcm2708_pinmux *pm = pc->grpidx[group_selector];

	if (func_selector >= FSELS || group_selector >= pc->nr_groups)
		return;

	dev_dbg(pc->dev, "pinmux_disable func=%s group=%s\n",
		fsel_names[func_selector], pc->grpidx[group_selector]->name);

	spin_lock_irq(&pc->lock);
	if (!pc->active) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	for (i = 0; i < pm->count; i++) {
		WARN_ON(!pc->pm_locked[pm->pins[i]]);
		pc->pm_locked[pm->pins[i]] = false;
	}

	spin_unlock_irq(&pc->lock);
}

static int bcm2708_pinmux_gpio_request_enable(struct pinctrl_dev *pctl,
	struct pinctrl_gpio_range *range, unsigned offset)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	dev_dbg(pc->dev,
		"pinmux_gpio_request_enable offset=%u base=%u pin_base=%u npins=%u\n",
		offset, range->base, range->pin_base, range->npins);

	return 0;
}

static void bcm2708_pinmux_gpio_disable_free(struct pinctrl_dev *pctl,
	struct pinctrl_gpio_range *range, unsigned offset)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	dev_dbg(pc->dev,
		"pinmux_gpio_disable_free offset=%u base=%u pin_base=%u npins=%u\n",
		offset, range->base, range->pin_base, range->npins);
}

static int bcm2708_pinmux_gpio_set_direction(struct pinctrl_dev *pctl,
	struct pinctrl_gpio_range *range, unsigned offset, bool input)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	dev_dbg(pc->dev,
		"pinmux_gpio_set_direction offset=%u base=%u pin_base=%u npins=%u input=%d\n",
		offset, range->base, range->pin_base, range->npins, input);

	return 0;
}

static int __devinit bcm2708_pinctrl_register(struct bcm2708_pinctrl *pc)
{
	struct pinctrl_desc *desc = &pc->desc;

	desc->name = dev_name(pc->dev);
	desc->pins = pc->pindesc;
	desc->npins = PINS;
	desc->pctlops = &pc->pctlops;
	desc->pmxops = &pc->pmxops;
	//desc->confops = &pc->confops;
	desc->owner = THIS_MODULE;

	pc->pctlops.list_groups = bcm2708_pinctrl_list_groups;
	pc->pctlops.get_group_name = bcm2708_pinctrl_get_group_name;
	pc->pctlops.get_group_pins = bcm2708_pinctrl_get_group_pins;

	pc->pmxops.request = bcm2708_pinmux_request;
	pc->pmxops.free = bcm2708_pinmux_free;
	pc->pmxops.list_functions = bcm2708_pinmux_list_functions;
	pc->pmxops.get_function_name = bcm2708_pinmux_get_function_name;
	pc->pmxops.get_function_groups = bcm2708_pinmux_get_function_groups;
	pc->pmxops.enable = bcm2708_pinmux_enable;
	pc->pmxops.disable = bcm2708_pinmux_disable;
	pc->pmxops.gpio_request_enable = bcm2708_pinmux_gpio_request_enable;
	pc->pmxops.gpio_disable_free = bcm2708_pinmux_gpio_disable_free;
	pc->pmxops.gpio_set_direction = bcm2708_pinmux_gpio_set_direction;

	pc->pctl = pinctrl_register(desc, pc->dev, pc);
	if (IS_ERR(pc->pctl))
		return PTR_ERR(pc->pctl);

	return 0;
}

static void __devexit bcm2708_pinctrl_unregister(struct bcm2708_pinctrl *pc)
{
	pinctrl_unregister(pc->pctl);
}

static int __devinit bcm2708_pinctrl_probe(struct platform_device *pdev)
{
	struct bcm2708_pinctrl *pc = bcm2708_pinctrl_of_init(pdev);
	int ret;

	if (IS_ERR(pc))
		return PTR_ERR(pc);

	ret = bcm2708_pinctrl_sysfs_register(pc);
	if (ret)
		goto err_free;

	ret = bcm2708_pinctrl_register(pc);
	if (ret)
		goto err_sysfs;

	spin_lock_irq(&pc->lock);
	pc->active = true;
	spin_unlock_irq(&pc->lock);

	dev_info(pc->dev, "%d pins at MMIO %#lx\n", PINS,
		(unsigned long)pc->res.start);
	
	platform_set_drvdata(pdev, pc);
	return 0;

err_sysfs:
	bcm2708_pinctrl_sysfs_unregister(pc);
err_free:
	bcm2708_pinctrl_of_free(pc);
	return ret;
}

static int __devexit bcm2708_pinctrl_remove(struct platform_device *pdev)
{
	struct bcm2708_pinctrl *pc = platform_get_drvdata(pdev);

	/* make sure no sysfs activity can occur */
	spin_lock_irq(&pc->lock);
	pc->active = false;
	spin_unlock_irq(&pc->lock);

	bcm2708_pinctrl_unregister(pc);
	bcm2708_pinctrl_sysfs_unregister(pc);
	bcm2708_pinctrl_of_free(pc);
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
