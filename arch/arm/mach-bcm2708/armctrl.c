/*
 *  Copyright (C) 2010 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/irqdomain.h>

#include <asm/mach/irq.h>
#include <asm/exception.h>
#include <mach/hardware.h>
#include "armctrl.h"

#define IS_VALID_BANK(x) ((x > 0) && (x < 32))
#define IS_VALID_IRQ(x) (x < 32)

struct armctrl_ic {
	void __iomem *pending;
	void __iomem *enable;
	void __iomem *disable;
	struct irq_domain *domain;

	u32 valid_mask;
	u32 shortcut_mask;
	u32 bank_mask;

	struct armctrl_ic_shortcut {
		u32 bank;
		u32 irq;
		struct irq_domain *domain;
	} shortcuts[32];
	struct armctrl_ic *banks[32];
};

struct of_armctrl_ic {
	u32 base_irq;
	u32 bank_id;
	u32 source_mask;

	struct armctrl_ic *ic;
};

static struct armctrl_ic *intc = NULL;

static void armctrl_mask_irq(struct irq_data *d)
{
	struct armctrl_ic *data = irq_get_chip_data(d->irq);
	writel(BIT(d->hwirq), data->disable);
}

static void armctrl_unmask_irq(struct irq_data *d)
{
	struct armctrl_ic *data = irq_get_chip_data(d->irq);
	writel(BIT(d->hwirq), data->enable);
}

static struct irq_chip armctrl_chip = {
	.name = "ARMCTRL-level",
	.irq_ack = armctrl_mask_irq,
	.irq_mask = armctrl_mask_irq,
	.irq_mask_ack = armctrl_mask_irq,
	.irq_unmask = armctrl_unmask_irq
};

static u32 of_read_u32(const struct device_node *np, const char *name, u32 def)
{
	const __be32 *prop = of_get_property(np, name, NULL);
	if (prop)
		return be32_to_cpup(prop);
	return def;
}

void of_read_armctrl_shortcuts(struct device_node *node, int count)
{
	struct of_armctrl_ic *data = node->data;
	u32 smap[count * 2];
	int ret, i, j;

	/* smap is multiple pairs of {bank_id, irq} */
	ret = of_property_read_u32_array(node, "shortcut-map", smap, count * 2);
	if (ret != 0)
		panic("%s: invalid shortcut map (%d)\n", node->full_name, ret);

	for (i = 0, j = 0; i < 32; i++) {
		if (!(data->ic->shortcut_mask & BIT(i)))
			continue;

		if (!IS_VALID_BANK(smap[j]) || !IS_VALID_IRQ(smap[j + 1]))
			panic("%s: invalid vic shortcut %u: %u->%u\n",
				node->full_name, i, smap[j], smap[j + 1]);

		data->ic->shortcuts[i].bank = smap[j];
		data->ic->shortcuts[i].irq = smap[j + 1];
		j += 2;
	}
}

struct of_armctrl_ic __init *of_read_armctrl_ic(struct device_node *node)
{
	struct of_armctrl_ic *data = kmalloc(sizeof(*data), GFP_ATOMIC);
	int nr_shortcuts, i;

	BUG_ON(data == NULL);
	node->data = data;

	data->ic = kzalloc(sizeof(*data->ic), GFP_ATOMIC);
	BUG_ON(data->ic == NULL);

	data->ic->pending = of_iomap(node, 0);
	data->ic->enable = of_iomap(node, 1);
	data->ic->disable = of_iomap(node, 2);

	if (!data->ic->pending || !data->ic->enable || !data->ic->disable)
		panic("%s: unable to map all vic cpu registers\n",
			node->full_name);

	data->base_irq = of_read_u32(node, "interrupt-base", 0);
	data->bank_id = of_read_u32(node, "bank-interrupt", 0);

	data->source_mask = of_read_u32(node, "source-mask", ~0);
	data->ic->bank_mask = of_read_u32(node, "bank-mask", 0);
	data->ic->shortcut_mask = of_read_u32(node, "shortcut-mask", 0);
	data->ic->valid_mask = data->source_mask;

	if ((data->source_mask & data->ic->bank_mask)
			|| (data->source_mask & data->ic->shortcut_mask)
			|| (data->ic->bank_mask & data->ic->shortcut_mask))
		panic("%s: vic mask overlap\n", node->full_name);

	nr_shortcuts = 0;
	for (i = 0; i < 32; i++) {
		if (data->ic->shortcut_mask & BIT(i))
			nr_shortcuts++;
	}

	if (nr_shortcuts > 0) {
		of_read_armctrl_shortcuts(node, nr_shortcuts);
	}

	return data;
}

void __init armctrl_of_link(const char *name, struct of_armctrl_ic *c,
	struct of_armctrl_ic *p)
{
	BUG_ON(c == NULL);
	BUG_ON(p == NULL);

	if (!c->bank_id)
		panic("%s: missing bank id for child vic\n", name);

	if (c->bank_id > 32 || !(p->ic->bank_mask & BIT(c->bank_id)))
		panic("%s: invalid vic bank %d\n", name, c->bank_id);

	if (p->ic->banks[c->bank_id])
		panic("%s: duplicate vic bank %d\n", name, c->bank_id);

	p->ic->banks[c->bank_id] = c->ic;
	p->ic->valid_mask |= BIT(c->bank_id);
}

void __init armctrl_of_link_shortcuts(struct of_armctrl_ic *c,
	struct of_armctrl_ic *p)
{
	int i;

	for (i = 0; i < 32; i++) {
		if (p->ic->shortcuts[i].bank == c->bank_id) {
			p->ic->valid_mask |= BIT(i);
			p->ic->shortcuts[i].domain = c->ic->domain;
		}
	}
}

int __init armctrl_of_init(struct device_node *node,
		struct device_node *parent)
{
	struct of_armctrl_ic *data = of_read_armctrl_ic(node);
	int nr_irqs, irq, i;

	if (parent != NULL) {
		armctrl_of_link(node->full_name, node->data, parent->data);
	} else if (intc != NULL) {
		panic("%s: multiple top level vics\n", node->full_name);
	} else {
		intc = data->ic;
	}

	nr_irqs = 0;
	for (i = 0, irq = data->base_irq; i < 32; i++, irq++) {
		if (!(data->source_mask & BIT(i)))
			continue;

		irq_set_chip_and_handler(irq, &armctrl_chip, handle_level_irq);
		irq_set_chip_data(irq, data->ic);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE | IRQF_DISABLED);
		nr_irqs++;
	}

	data->ic->domain = irq_domain_add_legacy(node, nr_irqs, data->base_irq,
		0, &irq_domain_simple_ops, NULL);
	if (!data->ic->domain)
		panic("%s: unable to create IRQ domain\n",
			node->full_name);

	if (parent != NULL) {
		armctrl_of_link_shortcuts(node->data, parent->data);
	}

	return 0;
}

/*
 * Handle each interrupt across the entire interrupt controller.  Returns
 * non-zero if we've handled at least one interrupt.  This reads the status
 * register before handling each interrupt, which is necessary given that
 * handle_IRQ may briefly re-enable interrupts for soft IRQ handling.
 */
static int handle_one_irq(struct armctrl_ic *dev, struct pt_regs *regs)
{
	u32 stat, irq;
	int handled = 0;

	while (likely(stat = readl_relaxed(dev->pending) & dev->valid_mask)) {
		irq = ffs(stat) - 1;
		if (dev->shortcut_mask & BIT(irq)) {
			handle_IRQ(irq_find_mapping(dev->shortcuts[irq].domain,
				dev->shortcuts[irq].irq), regs);
		} else if (dev->bank_mask & BIT(irq)) {
			handle_one_irq(dev->banks[irq], regs);
		} else {
			handle_IRQ(irq_find_mapping(dev->domain, irq), regs);
		}
		handled = 1;
	}

	return handled;
}

asmlinkage void __exception_irq_entry armctrl_handle_irq(struct pt_regs *regs)
{
	int handled;

	do {
		handled = handle_one_irq(intc, regs);
	} while (handled);
}
