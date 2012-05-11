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
#include <linux/syscore_ops.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/irqdomain.h>

#include <asm/mach/irq.h>
#include <asm/exception.h>
#include <mach/hardware.h>
#include "armctrl.h"

struct armctrl_irq {
	void __iomem *pending;
	void __iomem *enable;
	void __iomem *disable;

	u32 valid_mask;
	u32 source_mask;
	u32 bank_mask;
	u32 shortcut_mask;
	u32 shortcut_bank[32];
	u32 shortcut_irq[32];

	struct armctrl_irq *bank[32];
	struct armctrl_irq *parent;
	struct irq_domain *domain;
};

static struct armctrl_irq *intc = NULL;

static void armctrl_mask_irq(struct irq_data *d)
{
	struct armctrl_irq *data = irq_get_chip_data(d->irq);
	writel(BIT(d->hwirq), data->disable);
}

static void armctrl_unmask_irq(struct irq_data *d)
{
	struct armctrl_irq *data = irq_get_chip_data(d->irq);
	writel(BIT(d->hwirq), data->enable);
}

#if defined(CONFIG_PM)

/* Static defines
 * @irq: The IRQ number for the base of the VIC.
 * @base: The register base for the VIC.
 * @resume_sources: A bitmask of interrupts for resume.
 * @resume_irqs: The IRQs enabled for resume.
 * @int_select: Save for VIC_INT_SELECT.
 * @int_enable: Save for VIC_INT_ENABLE.
 * @soft_int: Save for VIC_INT_SOFT.
 * @protect: Save for VIC_PROTECT.
 */
struct armctrl_info {
	void __iomem *base;
	int irq;
	u32 resume_sources;
	u32 resume_irqs;
	u32 int_select;
	u32 int_enable;
	u32 soft_int;
	u32 protect;
} armctrl;

static int armctrl_suspend(void)
{
	return 0;
}

static void armctrl_resume(void)
{
	return;
}


/**
 * armctrl_pm_register - Register a VIC for later power management control
 * @base: The base address of the VIC.
 * @irq: The base IRQ for the VIC.
 * @resume_sources: bitmask of interrupts allowed for resume sources.
 */
static void __init armctrl_pm_register(void __iomem * base, unsigned int irq,
				       u32 resume_sources)
{
	armctrl.base = base;
	armctrl.resume_sources = resume_sources;
	armctrl.irq = irq;
}

static int armctrl_set_wake(struct irq_data *d, unsigned int on)
{
	unsigned int off = d->irq & 31;
	u32 bit = BIT(off);

	if (!(bit & armctrl.resume_sources))
		return -EINVAL;

	if (on)
		armctrl.resume_irqs |= bit;
	else
		armctrl.resume_irqs &= ~bit;

	return 0;
}

#else
static inline void armctrl_pm_register(void __iomem *base, unsigned int irq,
				       u32 arg1)
{
}
#define armctrl_suspend NULL
#define armctrl_resume NULL
#define armctrl_set_wake NULL
#endif /* CONFIG_PM */

static struct syscore_ops armctrl_syscore_ops = {
	.suspend = armctrl_suspend,
	.resume = armctrl_resume,
};

/**
 * armctrl_syscore_init - initicall to register VIC pm functions
 *
 * This is called via late_initcall() to register
 * the resources for the VICs due to the early
 * nature of the VIC's registration.
*/
static int __init armctrl_syscore_init(void)
{
	register_syscore_ops(&armctrl_syscore_ops);
	return 0;
}

late_initcall(armctrl_syscore_init);

static struct irq_chip armctrl_chip = {
	.name = "ARMCTRL-level",
	.irq_ack = armctrl_mask_irq,
	.irq_mask = armctrl_mask_irq,
	.irq_mask_ack = armctrl_mask_irq,
	.irq_unmask = armctrl_unmask_irq,
	.irq_set_wake = armctrl_set_wake,
};

static u32 of_read_u32(const struct device_node *np, const char *name, u32 def)
{
	const __be32 *prop = of_get_property(np, name, NULL);
	if (prop)
		return be32_to_cpup(prop);
	return def;
}

int __init armctrl_of_init(struct device_node *node,
		struct device_node *parent)
{
	struct armctrl_irq **ref = kmalloc(sizeof(**ref), GFP_KERNEL);
	struct armctrl_irq *data = kzalloc(sizeof(*data), GFP_KERNEL);
	u32 base_irq, nr_irqs, bank_id;
	int i, nr_smap;

	BUG_ON(ref == NULL);
	BUG_ON(data == NULL);
	node->data = ref;
	*ref = data;

	data->pending = of_iomap(node, 0);
	data->enable = of_iomap(node, 1);
	data->disable = of_iomap(node, 2);

	if (!data->pending)
		panic("%s: unable to map vic pending cpu register\n",
			node->full_name);
	if (!data->enable)
		panic("%s: unable to map vic enable cpu register\n",
			node->full_name);
	if (!data->disable)
		panic("%s: unable to map vic disable cpu register\n",
			node->full_name);

	base_irq = of_read_u32(node, "interrupt-base", 0);
	bank_id = of_read_u32(node, "bank-interrupt", 0);

	data->source_mask = of_read_u32(node, "source-mask", ~0);
	data->bank_mask = of_read_u32(node, "bank-mask", 0);
	data->shortcut_mask = of_read_u32(node, "shortcut-mask", 0);
	data->valid_mask = data->source_mask | data->bank_mask | data->shortcut_mask;

	if (data->source_mask & data->bank_mask)
		panic("%s: vic source mask %08x overlap with bank mask %08x: %08x\n",
			node->full_name, data->source_mask, data->bank_mask,
			data->source_mask & data->bank_mask);
	if (data->source_mask & data->shortcut_mask)
		panic("%s: vic source mask %08x overlap with shortcut mask %08x: %08x\n",
			node->full_name, data->source_mask, data->shortcut_mask,
			data->source_mask & data->shortcut_mask);
	if (data->bank_mask & data->shortcut_mask)
		panic("%s: vic bank mask %08x overlap with shortcut mask %08x: %08x\n",
			node->full_name, data->bank_mask, data->shortcut_mask,
			data->bank_mask & data->shortcut_mask);

	if (parent == NULL) {
		if (intc != NULL)
			panic("%s: attempted to register more than one top level vic\n",
				node->full_name);

		intc = data;
		data->parent = NULL;
	} else {
		if (!bank_id)
			panic("%s: missing bank id for child vic\n",
				node->full_name);

		data->parent = *(struct armctrl_irq **)parent->data;
		BUG_ON(data->parent == NULL);

		if (bank_id == 0 || bank_id > 32
				|| !(data->parent->bank_mask & BIT(bank_id)))
			panic("%s: attempted to register invalid vic bank %d\n",
				node->full_name, bank_id);

		if (data->parent->bank[bank_id])
			panic("%s: attempted to register vic bank %d twice\n",
				node->full_name, bank_id);

		data->parent->bank[bank_id] = data;
	}

	nr_smap = 0;
	for (i = 0; i < 32; i++) {
		if (data->shortcut_mask & BIT(i))
			nr_smap += 2;
	}

	if (nr_smap > 0) {
		u32 shortcuts[nr_smap];
		int ret = of_property_read_u32_array(node, "shortcut-map",
			shortcuts, nr_smap);

		if (ret != 0)
			panic("%s: invalid shortcut map (%d)\n", node->full_name, ret);

		nr_smap = 0;
		for (i = 0; i < 32; i++) {
			if (data->shortcut_mask & BIT(i)) {
				data->shortcut_bank[i] = shortcuts[nr_smap];
				data->shortcut_irq[i] = shortcuts[nr_smap + 1];

				if (data->shortcut_bank[i] == 0
						|| data->shortcut_bank[i] > 32)
					panic("%s: attempted to map shortcut %d to invalid vic bank %d\n",
						node->full_name,
						data->shortcut_irq[i],
						data->shortcut_bank[i]);

				if (data->shortcut_irq[i] > 32)
					panic("%s: attempted to map invalid shortcut %d to vic bank %d\n",
						node->full_name,
						data->shortcut_irq[i],
						data->shortcut_bank[i]);

				nr_smap += 2;
			}
		}
	}

	nr_irqs = 0;
	for (i = 0; i < 32; i++) {
		int irq;

		if (!(data->source_mask & BIT(i)))
			continue;

		irq = base_irq + i;
		irq_set_chip_and_handler(irq, &armctrl_chip, handle_level_irq);
		irq_set_chip_data(irq, data);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE | IRQF_DISABLED);
		nr_irqs++;
	}

	data->domain = irq_domain_add_legacy(node, nr_irqs, base_irq, 0,
		&irq_domain_simple_ops, NULL);
	if (!data->domain)
		panic("unable to create IRQ domain\n");

	return 0;
}

/*
 * Handle each interrupt across the entire interrupt controller.  Returns
 * non-zero if we've handled at least one interrupt.  This reads the status
 * register before handling each interrupt, which is necessary given that
 * handle_IRQ may briefly re-enable interrupts for soft IRQ handling.
 */
static int handle_one_irq(struct armctrl_irq *dev, struct pt_regs *regs)
{
	u32 stat, irq;
	int handled = 0;

	while (likely(stat = readl_relaxed(dev->pending) & dev->valid_mask)) {
		irq = ffs(stat) - 1;
		if (dev->shortcut_mask & BIT(irq)) {
			int bank = dev->shortcut_bank[irq];
			irq = dev->shortcut_irq[irq];

			if (dev->bank[bank]) {
				handle_IRQ(irq_find_mapping(
					dev->bank[bank]->domain, irq), regs);
			}
		} else if (dev->bank_mask & BIT(irq)) {
			if (dev->bank[irq]) {
				handle_one_irq(dev->bank[irq], regs);
			}
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
