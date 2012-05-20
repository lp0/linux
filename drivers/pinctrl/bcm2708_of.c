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
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "bcm2708.h"

#define REG_SZ 0x1c

static int bcm2708_pinmux_of_group_init_one(struct bcm2708_pinctrl *pc, int p,
	enum pin_fsel f, const char *name)
{
	struct bcm2708_pinmux *group = NULL;
	struct bcm2708_pinmux *tmp;

	if (!strcmp("", name))
		return 0;

	list_for_each_entry(tmp, &pc->groups, list) {
		if (!strcmp(tmp->name, name)) {
			group = tmp;
			break;
		}
	}

	if (group == NULL) {
		group = kzalloc(sizeof(*group), GFP_KERNEL);
		if (group == NULL)
			goto nomem;

		group->name = kstrdup(name, GFP_KERNEL);
		group->count = 0;
		group->pins = kzalloc(sizeof(group->pins[0]), GFP_KERNEL);
		group->fsel = kzalloc(sizeof(group->fsel[0]), GFP_KERNEL);
		if (group->name == NULL
				|| group->pins == NULL || group->fsel == NULL)
			goto nomem;
	} else {
		unsigned *tmp_pins;
		unsigned *tmp_fsel;

		list_del(&group->list);
		pc->nr_groups--;

		tmp_pins = krealloc(group->pins,
			sizeof(group->pins[0]) * (group->count + 1),
			GFP_KERNEL);
		tmp_fsel = krealloc(group->fsel,
			sizeof(group->fsel[0]) * (group->count + 1),
			GFP_KERNEL);
		if (tmp_pins != NULL)
			group->pins = tmp_pins;
		if (tmp_fsel != NULL)
			group->fsel = tmp_fsel;
		if (tmp_pins == NULL || tmp_fsel == NULL)
			goto nomem;
	}

	/* need to map the alt index to fsel
	 * (GPIO_IN and GPIO_OUT are already 0 and 1)
	 */
	if (f >= FSEL_ALT_BASE)
		f = to_fsel_value(f - FSEL_ALT_BASE);

	group->pins[group->count] = p;
	group->fsel[group->count] = f;
	group->count++;
	list_add_tail(&group->list, &pc->groups);
	pc->nr_groups++;
	return 0;

nomem:
	kfree(group->name);
	kfree(group->pins);
	kfree(group->fsel);
	kfree(group);
	return -ENOMEM;
}

/* split the name up by NAME_SPLIT as some function
 * selections belong to more than one group
 */
static int bcm2708_pinmux_of_group_init(struct bcm2708_pinctrl *pc, int p,
	enum pin_fsel f, const char *name)
{
	char buf[NAME_LEN+1];
	char *tmp = buf;
	char *cur;
	int ret;

	strncpy(buf, name, NAME_LEN);
	while (tmp) {
		cur = strsep(&tmp, NAME_SPLIT);
		ret = bcm2708_pinmux_of_group_init_one(pc, p, f, cur);
		if (ret)
			return ret;
	}
	return 0;
}

static void bcm2708_pinmux_of_free(struct bcm2708_pinctrl *pc)
{
	struct bcm2708_pinmux *group;
	struct bcm2708_pinmux *tmp;

	kfree(pc->grpidx);
	kfree(pc->grpnam);
	kfree(pc->pindesc);

	list_for_each_entry_safe(group, tmp, &pc->groups, list) {
		list_del(&group->list);
		pc->nr_groups--;

		kfree(group->name);
		kfree(group->pins);
		kfree(group->fsel);
		kfree(group);
	}
}

static int __devinit bcm2708_pinmux_of_init(struct device_node *np,
	struct bcm2708_pinctrl *pc)
{
	struct bcm2708_pinmux *tmp;
	int ret, p, f, i;

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

			ret = bcm2708_pinmux_of_group_init(pc, p, f, name);
			if (ret) {
				dev_err(pc->dev,
					"error adding group info %d %d (%d)\n",
					p, f, ret);
				goto err;
			}
		}
	}

	pc->grpidx = kzalloc(sizeof(pc->grpidx[0]) * pc->nr_groups, GFP_KERNEL);
	pc->grpnam = kzalloc(sizeof(pc->grpnam[0]) * pc->nr_groups, GFP_KERNEL);
	pc->pindesc = kzalloc(sizeof(pc->pindesc[0]) * PINS, GFP_KERNEL);
	if (pc->grpidx == NULL || pc->grpnam == NULL || pc->pindesc == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	i = 0;
	list_for_each_entry(tmp, &pc->groups, list) {
		if (i >= pc->nr_groups) {
			WARN_ON(1);
			ret = -EINVAL;
			goto err;
		}
		pc->grpidx[i] = tmp;
		pc->grpnam[i] = tmp->name;
		i++;
	}

	for (p = 0; p < PINS; p++)
		pc->pindesc[p].number = p;

	return 0;

err:
	bcm2708_pinmux_of_free(pc);
	return ret;
}

struct bcm2708_pinctrl __devinit *bcm2708_pinctrl_of_init(
	struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct bcm2708_pinctrl *pc = kzalloc(sizeof(*pc), GFP_KERNEL);
	int ret, p, a;

	if (pc == NULL)
		return ERR_PTR(-ENOMEM);

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
		release_region(pc->res.start, resource_size(&pc->res));
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

	ret = bcm2708_pinmux_of_init(np, pc);
	if (ret)
		goto err;

	return pc;

err:
	bcm2708_pinctrl_of_free(pc);
	return ERR_PTR(ret);
}

int bcm2708_pinctrl_of_free(struct bcm2708_pinctrl *pc)
{
	bcm2708_pinmux_of_free(pc);
	if (pc->base) {
		release_region(pc->res.start, resource_size(&pc->res));
		iounmap(pc->base);
	}
	kfree(pc);
	return 0;
}
