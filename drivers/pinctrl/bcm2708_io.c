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

#include <linux/io.h>

#include "bcm2708.h"

#define FSEL_MASK 0x7
#define FSEL_REG(p) ((p / 10) * 4)
#define FSEL_SHIFT(p) ((p - 10 * (p / 10)) * 3)

static const char *fsel_names[FSELS] = {
	"GPIO_IN",
	"GPIO_OUT",
	"ALT5",
	"ALT4",
	"ALT0",
	"ALT1",
	"ALT2",
	"ALT3"
};

enum pin_fsel bcm2708_pinctrl_fsel_get(struct bcm2708_pinctrl *pc, unsigned p)
{
	enum pin_fsel status;
	void __iomem *reg = pc->base + FSEL_REG(p);
	u32 val = readl(reg);

	status = (val >> FSEL_SHIFT(p)) & FSEL_MASK;
	dev_dbg(pc->dev, "get %08x@%p (%u = %s)\n", val, reg, p,
		fsel_names[status]);
	return status;
}

void bcm2708_pinctrl_fsel_set(struct bcm2708_pinctrl *pc, unsigned p,
	enum pin_fsel set)
{
	enum pin_fsel cur;
	void __iomem *reg = pc->base + FSEL_REG(p);
	u32 val = readl(reg);

	cur = (val >> FSEL_SHIFT(p)) & FSEL_MASK;
	dev_dbg(pc->dev, "read %08x@%p (%u = %s)\n", val, reg, p,
		fsel_names[cur]);

	/* all GPIOs need to go via GPIO_IN first or it'll lock up */
	if (cur != FSEL_GPIO_IN && set != FSEL_GPIO_IN) {
		val &= ~(FSEL_MASK << FSEL_SHIFT(p));
		val |= (FSEL_GPIO_IN & FSEL_MASK) << FSEL_SHIFT(p);
		dev_dbg(pc->dev, "write %08x@%p (%u = %s)\n", val, reg, p,
			fsel_names[FSEL_GPIO_IN]);
	}

	val &= ~(FSEL_MASK << FSEL_SHIFT(p));
	val |= (set & FSEL_MASK) << FSEL_SHIFT(p);
	dev_dbg(pc->dev, "write %08x@%p (%u = %s)\n", val, reg, p,
		fsel_names[set]);
	writel(val, reg);
}
