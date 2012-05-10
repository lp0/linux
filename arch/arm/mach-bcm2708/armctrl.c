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
	u32 source_mask;
	struct irq_domain *domain;
};

static struct armctrl_irq irq_devices[3];
static int irq_id;

static void armctrl_mask_irq(struct irq_data *d)
{
	struct armctrl_irq *data = irq_get_chip_data(d->irq);
	if ((1 << d->hwirq) & data->source_mask)
		writel(1 << d->hwirq, data->disable);
}

static void armctrl_unmask_irq(struct irq_data *d)
{
	struct armctrl_irq *data = irq_get_chip_data(d->irq);
	if ((1 << d->hwirq) & data->source_mask)
		writel(1 << d->hwirq, data->enable);
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
	u32 bit = 1 << off;

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
	.name = "ARMCTRL",
	.irq_ack = armctrl_mask_irq,
	.irq_mask = armctrl_mask_irq,
	.irq_mask_ack = armctrl_mask_irq,
	.irq_unmask = armctrl_unmask_irq,
	.irq_set_wake = armctrl_set_wake,
};

int __init armctrl_of_init(struct device_node *node,
		struct device_node *parent)
{
	struct armctrl_irq *data;
	unsigned int i, irq;

	int base_irq;
	int nr_irqs;

	if (irq_id >= ARRAY_SIZE(irq_devices)) {
		printk(KERN_ERR "%s: too few IRQ devices\n", __func__);
		return -EBUSY;
	}

	data = &irq_devices[irq_id];
	data->pending = of_iomap(node, 0);
	data->enable = of_iomap(node, 1);
	data->disable = of_iomap(node, 2);
	data->source_mask = ~0;

	/* FIXME: provide an interrupt map from dt */
	switch ((unsigned int)data->pending) {
	case 0xf200b200:
		base_irq = 64; nr_irqs = 10;
		data->source_mask = 0xff;
		break;
	case 0xf200b204: base_irq = 0; nr_irqs = 32; break;
	case 0xf200b208: base_irq = 32; nr_irqs = 32; break;
	default: BUG();
	}

	if (!data->pending)
		panic("unable to map vic pending cpu register\n");
	if (!data->enable)
		panic("unable to map vic enable cpu register\n");
	if (!data->disable)
		panic("unable to map vic disable cpu register\n");

	for (i = 0; i < nr_irqs; i++) {
		if (!(data->source_mask & (1 << i)))
			continue;

		irq = base_irq + i;
		irq_set_chip_and_handler(irq, &armctrl_chip, handle_level_irq);
		irq_set_chip_data(irq, data);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE | IRQF_DISABLED);
	}

	data->domain = irq_domain_add_legacy(node, nr_irqs, base_irq, 0,
			&irq_domain_simple_ops, NULL);
	if (!data->domain)
		panic("unable to create IRQ domain\n");

	irq_id++;
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

	while (likely(stat = readl_relaxed(dev->pending) & dev->source_mask)) {
		irq = ffs(stat) - 1;
		handle_IRQ(irq_find_mapping(dev->domain, irq), regs);
		handled = 1;
	}

	return handled;
}

asmlinkage void __exception_irq_entry armctrl_handle_irq(struct pt_regs *regs)
{
	int i, handled;

	do {
		for (i = 0, handled = 0; i < irq_id; i++)
			handled |= handle_one_irq(&irq_devices[i], regs);
	} while (handled);
}

