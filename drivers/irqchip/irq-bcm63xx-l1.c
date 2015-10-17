/*
 * Broadcom BCM63xx style Level 1 interrupt controller driver
 *
 * Copyright (C) 2014 Broadcom Corporation
 * Copyright 2015 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This is based on the BCM7038 (which supports SMP) but with a single
 * enable register instead of separate mask/set/clear registers.
 *
 * The BCM3380 has a similar mask/status register layout, but each pair
 * of words is at separate locations (and SMP is not supported).
 *
 * ENABLE/STATUS words are packed next to each other for each CPU:
 *
 * 6368:
 *   0x1000_0020: CPU0_W0_ENABLE
 *   0x1000_0024: CPU0_W1_ENABLE
 *   0x1000_0028: CPU0_W0_STATUS		IRQs 31-63
 *   0x1000_002c: CPU0_W1_STATUS		IRQs 0-31
 *   0x1000_0030: CPU1_W0_ENABLE
 *   0x1000_0034: CPU1_W1_ENABLE
 *   0x1000_0038: CPU1_W0_STATUS		IRQs 31-63
 *   0x1000_003c: CPU1_W1_STATUS		IRQs 0-31
 *
 * 63168:
 *   0x1000_0020: CPU0_W0_ENABLE
 *   0x1000_0024: CPU0_W1_ENABLE
 *   0x1000_0028: CPU0_W2_ENABLE
 *   0x1000_002c: CPU0_W3_ENABLE
 *   0x1000_0030: CPU0_W0_STATUS	IRQs 96-127
 *   0x1000_0034: CPU0_W1_STATUS	IRQs 64-95
 *   0x1000_0038: CPU0_W2_STATUS	IRQs 32-63
 *   0x1000_003c: CPU0_W3_STATUS	IRQs 0-31
 *   0x1000_0040: CPU1_W0_ENABLE
 *   0x1000_0044: CPU1_W1_ENABLE
 *   0x1000_0048: CPU1_W2_ENABLE
 *   0x1000_004c: CPU1_W3_ENABLE
 *   0x1000_0050: CPU1_W0_STATUS	IRQs 96-127
 *   0x1000_0054: CPU1_W1_STATUS	IRQs 64-95
 *   0x1000_0058: CPU1_W2_STATUS	IRQs 32-63
 *   0x1000_005c: CPU1_W3_STATUS	IRQs 0-31
 *
 * IRQs are numbered in big-endian order
 */

#define pr_fmt(fmt)	KBUILD_MODNAME	": " fmt

#include <linux/bitops.h>
#include <linux/kconfig.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/types.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>

#define IRQS_PER_WORD		32
#define REG_BYTES_PER_IRQ_WORD	(sizeof(u32) * 2)
#define MAX_WORDS		4

struct bcm63xx_l1_cpu;

struct bcm63xx_l1_chip {
	raw_spinlock_t		lock[MAX_WORDS];
	unsigned int		n_words;
	struct irq_domain	*domain;
	struct bcm63xx_l1_cpu	*cpus[NR_CPUS];
};

struct bcm63xx_l1_cpu {
	void __iomem		*map_base;
	unsigned int		parent_irq;
	u32			enable_cache[];
};

static inline unsigned int reg_enable(struct bcm63xx_l1_chip *intc,
					   unsigned int word)
{
	return (1 * intc->n_words - word - 1) * sizeof(u32);
}

static inline unsigned int reg_status(struct bcm63xx_l1_chip *intc,
				      unsigned int word)
{
	return (2 * intc->n_words - word - 1) * sizeof(u32);
}

static void bcm63xx_l1_irq_handle(struct irq_desc *desc)
{
	struct bcm63xx_l1_chip *intc = irq_desc_get_handler_data(desc);
	struct bcm63xx_l1_cpu *cpu;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int idx;

#ifdef CONFIG_SMP
	cpu = intc->cpus[cpu_logical_map(smp_processor_id())];
#else
	cpu = intc->cpus[0];
#endif

	chained_irq_enter(chip, desc);

	for (idx = 0; idx < intc->n_words; idx++) {
		int base = idx * IRQS_PER_WORD;
		unsigned long pending;
		irq_hw_number_t hwirq;
		unsigned int irq;

		pending = __raw_readl(cpu->map_base + reg_status(intc, idx));
		pending &= __raw_readl(cpu->map_base + reg_enable(intc, idx));

		for_each_set_bit(hwirq, &pending, IRQS_PER_WORD) {
			irq = irq_linear_revmap(intc->domain, base + hwirq);
			if (irq)
				do_IRQ(irq);
			else
				spurious_interrupt();
		}
	}

	chained_irq_exit(chip, desc);
}

static inline void __bcm63xx_l1_unmask(struct irq_data *d)
{
	struct bcm63xx_l1_chip *intc = irq_data_get_irq_chip_data(d);
	u32 word = d->hwirq / IRQS_PER_WORD;
	u32 mask = BIT(d->hwirq % IRQS_PER_WORD);
	unsigned int cpu_idx = cpumask_first(irq_data_get_affinity_mask(d));

	intc->cpus[cpu_idx]->enable_cache[word] |= mask;
	__raw_writel(intc->cpus[cpu_idx]->enable_cache[word],
		intc->cpus[cpu_idx]->map_base + reg_enable(intc, word));
}

static inline void __bcm63xx_l1_mask(struct irq_data *d)
{
	struct bcm63xx_l1_chip *intc = irq_data_get_irq_chip_data(d);
	u32 word = d->hwirq / IRQS_PER_WORD;
	u32 mask = BIT(d->hwirq % IRQS_PER_WORD);
	unsigned int cpu_idx = cpumask_first(irq_data_get_affinity_mask(d));

	intc->cpus[cpu_idx]->enable_cache[word] &= ~mask;
	__raw_writel(intc->cpus[cpu_idx]->enable_cache[word],
		intc->cpus[cpu_idx]->map_base + reg_enable(intc, word));
}

static void bcm63xx_l1_unmask(struct irq_data *d)
{
	struct bcm63xx_l1_chip *intc = irq_data_get_irq_chip_data(d);
	u32 word = d->hwirq / IRQS_PER_WORD;
	unsigned long flags;

	raw_spin_lock_irqsave(&intc->lock[word], flags);
	__bcm63xx_l1_unmask(d);
	raw_spin_unlock_irqrestore(&intc->lock[word], flags);
}

static void bcm63xx_l1_mask(struct irq_data *d)
{
	struct bcm63xx_l1_chip *intc = irq_data_get_irq_chip_data(d);
	u32 word = d->hwirq / IRQS_PER_WORD;
	unsigned long flags;

	raw_spin_lock_irqsave(&intc->lock[word], flags);
	__bcm63xx_l1_mask(d);
	raw_spin_unlock_irqrestore(&intc->lock[word], flags);
}

static int bcm63xx_l1_set_affinity(struct irq_data *d,
				   const struct cpumask *dest,
				   bool force)
{
	struct bcm63xx_l1_chip *intc = irq_data_get_irq_chip_data(d);
	u32 word = d->hwirq / IRQS_PER_WORD;
	u32 mask = BIT(d->hwirq % IRQS_PER_WORD);
	unsigned int cpu_idx = cpumask_first(irq_data_get_affinity_mask(d));
	unsigned long flags;
	bool enabled;

	if (!force)
		dest = cpumask_of(cpumask_any_and(dest, cpu_online_mask));
	else
		dest = cpumask_of(cpumask_first(dest));

	raw_spin_lock_irqsave(&intc->lock[word], flags);
	enabled = intc->cpus[cpu_idx]->enable_cache[word] & mask;
	if (enabled)
		__bcm63xx_l1_mask(d);
	cpumask_copy(irq_data_get_affinity_mask(d), dest);
	if (enabled)
		__bcm63xx_l1_unmask(d);
	raw_spin_unlock_irqrestore(&intc->lock[word], flags);

	return IRQ_SET_MASK_OK_NOCOPY;
}

static int __init bcm63xx_l1_init_one(struct device_node *dn,
				      unsigned int idx,
				      struct bcm63xx_l1_chip *intc)
{
	struct resource res;
	resource_size_t sz;
	struct bcm63xx_l1_cpu *cpu;
	unsigned int i, n_words;

	if (of_address_to_resource(dn, idx, &res))
		return -EINVAL;
	sz = resource_size(&res);
	n_words = sz / REG_BYTES_PER_IRQ_WORD;

	if (n_words > MAX_WORDS)
		return -EINVAL;
	else if (!intc->n_words)
		intc->n_words = n_words;
	else if (intc->n_words != n_words)
		return -EINVAL;

	cpu = intc->cpus[idx] = kzalloc(sizeof(*cpu) + n_words * sizeof(u32),
					GFP_KERNEL);
	if (!cpu)
		return -ENOMEM;

	cpu->map_base = ioremap(res.start, sz);
	if (!cpu->map_base)
		return -ENOMEM;

	for (i = 0; i < n_words; i++) {
		cpu->enable_cache[i] = 0;
		__raw_writel(0, cpu->map_base + reg_enable(intc, i));
	}

	cpu->parent_irq = irq_of_parse_and_map(dn, idx);
	if (!cpu->parent_irq) {
		pr_err("failed to map parent interrupt %d\n", cpu->parent_irq);
		return -EINVAL;
	}
	irq_set_chained_handler_and_data(cpu->parent_irq,
						bcm63xx_l1_irq_handle, intc);

	return 0;
}

static struct irq_chip bcm63xx_l1_irq_chip = {
	.name			= "bcm63xx-l1",
	.irq_mask		= bcm63xx_l1_mask,
	.irq_unmask		= bcm63xx_l1_unmask,
	.irq_set_affinity	= bcm63xx_l1_set_affinity,
};

static int bcm63xx_l1_map(struct irq_domain *d, unsigned int virq,
			  irq_hw_number_t hw_irq)
{
	irq_set_chip_and_handler(virq, &bcm63xx_l1_irq_chip, handle_level_irq);
	irq_set_chip_data(virq, d->host_data);
	return 0;
}

static const struct irq_domain_ops bcm63xx_l1_domain_ops = {
	.xlate			= irq_domain_xlate_onecell,
	.map			= bcm63xx_l1_map,
};

static int __init bcm63xx_l1_of_init(struct device_node *dn,
			      struct device_node *parent)
{
	struct bcm63xx_l1_chip *intc;
	unsigned int idx;
	int ret;

	intc = kzalloc(sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;

	for_each_possible_cpu(idx) {
		ret = bcm63xx_l1_init_one(dn, idx, intc);
		if (ret < 0) {
			if (idx)
				break;
			pr_err("failed to remap intc L1 registers\n");
			goto out_free;
		}
	}
	// TODO n_cpus

	for (idx = 0; idx < intc->n_words; idx++)
		raw_spin_lock_init(&intc->lock[idx]);

	intc->domain = irq_domain_add_linear(dn, IRQS_PER_WORD * intc->n_words,
					     &bcm63xx_l1_domain_ops,
					     intc);
	if (!intc->domain) {
		ret = -ENOMEM;
		goto out_unmap;
	}

	pr_info("registered BCM63xx L1 intc (IRQs: %d)\n",
			IRQS_PER_WORD * intc->n_words);
	for_each_possible_cpu(idx) {
		struct bcm63xx_l1_cpu *cpu = intc->cpus[idx];

		pr_info("  CPU%u at MMIO 0x%p (irq = %d)\n", idx,
				cpu->map_base, cpu->parent_irq);
	}

	return 0;

out_unmap:
	for_each_possible_cpu(idx) {
		struct bcm63xx_l1_cpu *cpu = intc->cpus[idx];

		if (cpu) {
			if (cpu->map_base)
				iounmap(cpu->map_base);
			kfree(cpu);
		}
	}
out_free:
	kfree(intc);
	return ret;
}

IRQCHIP_DECLARE(bcm63xx_l1, "brcm,bcm63xx-l1-intc", bcm63xx_l1_of_init);
