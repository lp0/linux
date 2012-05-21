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
#include <linux/spinlock.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>

#include "bcm2708.h"

int bcm2708_pinctrl_list_groups(struct pinctrl_dev *pctl, unsigned selector)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	return selector < pc->nr_groups ? 0 : -EINVAL;
}

const char *bcm2708_pinctrl_get_group_name(struct pinctrl_dev *pctl,
	unsigned selector)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	if (unlikely(selector >= pc->nr_groups))
		return NULL;

	return pc->grpidx[selector]->name;
}

int bcm2708_pinctrl_get_group_pins(struct pinctrl_dev *pctl, unsigned selector,
	const unsigned **pins, unsigned *num_pins)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	if (unlikely(selector >= pc->nr_groups))
		return -EINVAL;

	*pins = pc->grpidx[selector]->pins;
	*num_pins = pc->grpidx[selector]->count;
	return 0;
}

int bcm2708_pinmux_request(struct pinctrl_dev *pctl, unsigned offset)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	int ret = 0;

	dev_dbg(pc->dev, "pinmux_request %d\n", offset);

	spin_lock_irq(&pc->lock);
	if (unlikely(!pc->active)) {
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

int bcm2708_pinmux_free(struct pinctrl_dev *pctl, unsigned offset)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	dev_dbg(pc->dev, "pinmux_free %d\n", offset);

	spin_lock_irq(&pc->lock);
	WARN_ON(!pc->pm_locked[offset]);
	pc->pm_locked[offset] = false;
	spin_unlock_irq(&pc->lock);
	return 0;
}

int bcm2708_pinmux_list_functions(struct pinctrl_dev *pctl,
	unsigned selector)
{
	/* There is only one function because pinmux has no way of specifying
	 * that a device or group requires each pin with a specific function
	 * and some devices require multiple pins with different function
	 * selections. This is ok because we map each device to a group and so
	 * we know which function selection to use.
	 */
	return selector == 0 ? 0 : -EINVAL;
}

const char *bcm2708_pinmux_get_function_name(struct pinctrl_dev *pctl,
	unsigned selector)
{
	if (unlikely(selector != 0))
		return NULL;

	return MUX_FUNCTION;
}

int bcm2708_pinmux_get_function_groups(struct pinctrl_dev *pctl,
	unsigned selector, const char * const **groups,
	unsigned * const num_groups)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	if (unlikely(selector != 0))
		return -EINVAL;

	*groups = pc->grpnam;
	*num_groups = pc->nr_groups;
	return 0;
}

int bcm2708_pinmux_enable(struct pinctrl_dev *pctl,
	unsigned func_selector, unsigned group_selector)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	struct bcm2708_pinmux *pm = pc->grpidx[group_selector];
	int ret = 0;
	int i;

	if (unlikely(func_selector != 0 || group_selector >= pc->nr_groups))
		return -EINVAL;

	dev_dbg(pc->dev, "pinmux_enable group=%s\n",
		pc->grpidx[group_selector]->name);

	spin_lock_irq(&pc->lock);
	if (unlikely(!pc->active)) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	for (i = 0; i < pm->count; i++)
		bcm2708_pinctrl_fsel_set(pc, pm->pins[i], pm->fsel[i]);

	spin_unlock_irq(&pc->lock);
	return ret;
}


void bcm2708_pinmux_disable(struct pinctrl_dev *pctl,
	unsigned func_selector, unsigned group_selector)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	struct bcm2708_pinmux *pm = pc->grpidx[group_selector];
	int i;

	if (unlikely(func_selector != 0 || group_selector >= pc->nr_groups))
		return;

	dev_dbg(pc->dev, "pinmux_disable group=%s\n",
		pc->grpidx[group_selector]->name);

	spin_lock_irq(&pc->lock);
	if (unlikely(!pc->active)) {
		spin_unlock_irq(&pc->lock);
		return;
	}

	for (i = 0; i < pm->count; i++)
		bcm2708_pinctrl_fsel_set(pc, pm->pins[i], FSEL_GPIO_IN);

	spin_unlock_irq(&pc->lock);
}

int bcm2708_pinmux_gpio_request_enable(struct pinctrl_dev *pctl,
	struct pinctrl_gpio_range *range, unsigned offset)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);
	int ret;

	dev_dbg(pc->dev, "pinmux_gpio_request_enable %d\n", offset);

	if (unlikely(offset >= pc->nr_pins))
		return -EINVAL;

	spin_lock_irq(&pc->lock);
	if (unlikely(!pc->active)) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	if (pc->usr_locked[offset]) {
		ret = -EBUSY;
	} else {
		WARN_ON(pc->pm_locked[offset]);
		pc->pm_locked[offset] = true;
	}

	bcm2708_pinctrl_fsel_set(pc, offset, FSEL_GPIO_IN);
	spin_unlock_irq(&pc->lock);
	return 0;
}

void bcm2708_pinmux_gpio_disable_free(struct pinctrl_dev *pctl,
	struct pinctrl_gpio_range *range, unsigned offset)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	dev_dbg(pc->dev, "pinmux_gpio_disable_free %d\n", offset);

	if (unlikely(offset >= pc->nr_pins))
		return;

	spin_lock_irq(&pc->lock);
	if (unlikely(!pc->active)) {
		spin_unlock_irq(&pc->lock);
		return;
	}

	WARN_ON(!pc->pm_locked[offset]);
	pc->pm_locked[offset] = false;

	bcm2708_pinctrl_fsel_set(pc, offset, FSEL_GPIO_IN);
	spin_unlock_irq(&pc->lock);
}

int bcm2708_pinmux_gpio_set_direction(struct pinctrl_dev *pctl,
	struct pinctrl_gpio_range *range, unsigned offset, bool input)
{
	struct bcm2708_pinctrl *pc = pinctrl_dev_get_drvdata(pctl);

	dev_dbg(pc->dev, "pinmux_gpio_set_direction %d %d\n", offset, input);

	if (unlikely(offset >= pc->nr_pins))
		return -EINVAL;

	spin_lock_irq(&pc->lock);
	if (unlikely(!pc->active)) {
		spin_unlock_irq(&pc->lock);
		return -ENODEV;
	}

	bcm2708_pinctrl_fsel_set(pc, offset,
		input ? FSEL_GPIO_IN : FSEL_GPIO_OUT);
	spin_unlock_irq(&pc->lock);
	return 0;
}
