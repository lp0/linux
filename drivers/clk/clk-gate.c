/*
 * Copyright (C) 2010-2011 Canonical Ltd <jeremy.kerr@canonical.com>
 * Copyright (C) 2011-2012 Mike Turquette, Linaro Ltd <mturquette@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Gated clock implementation
 */

#define pr_fmt(fmt)     KBUILD_MODNAME  ": " fmt

#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/resource.h>

/**
 * DOC: basic gatable clock which can gate and ungate it's ouput
 *
 * Traits of this clock:
 * prepare - clk_(un)prepare only ensures parent is (un)prepared
 * enable - clk_enable and clk_disable are functional & control gating
 * rate - inherits rate from parent.  No clk_set_rate support
 * parent - fixed parent.  No clk_set_parent support
 */

#define to_clk_gate(_hw) container_of(_hw, struct clk_gate, hw)

/*
 * It works on following logic:
 *
 * For enabling clock, enable = 1
 *	set2dis = 1	-> clear bit	-> set = 0
 *	set2dis = 0	-> set bit	-> set = 1
 *
 * For disabling clock, enable = 0
 *	set2dis = 1	-> set bit	-> set = 1
 *	set2dis = 0	-> clear bit	-> set = 0
 *
 * So, result is always: enable xor set2dis.
 */
static void clk_gate_endisable(struct clk_hw *hw, int enable)
{
	struct clk_gate *gate = to_clk_gate(hw);
	int set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	unsigned long uninitialized_var(flags);
	u32 reg;

	set ^= enable;

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);
	else
		__acquire(gate->lock);

	if (gate->flags & CLK_GATE_HIWORD_MASK) {
		reg = BIT(gate->bit_idx + 16);
		if (set)
			reg |= BIT(gate->bit_idx);
	} else {
		reg = clk_readl(gate->reg);
		if (set)
			reg |= BIT(gate->bit_idx);
		else
			reg &= ~BIT(gate->bit_idx);
	}

	clk_writel(reg, gate->reg);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
	else
		__release(gate->lock);
}

static int clk_gate_enable(struct clk_hw *hw)
{
	clk_gate_endisable(hw, 1);

	return 0;
}

static void clk_gate_disable(struct clk_hw *hw)
{
	clk_gate_endisable(hw, 0);
}

static int clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct clk_gate *gate = to_clk_gate(hw);

	reg = clk_readl(gate->reg);

	/* if a set bit disables this clk, flip it before masking */
	if (gate->flags & CLK_GATE_SET_TO_DISABLE)
		reg ^= BIT(gate->bit_idx);

	reg &= BIT(gate->bit_idx);

	return reg ? 1 : 0;
}

const struct clk_ops clk_gate_ops = {
	.enable = clk_gate_enable,
	.disable = clk_gate_disable,
	.is_enabled = clk_gate_is_enabled,
};
EXPORT_SYMBOL_GPL(clk_gate_ops);

/**
 * clk_register_gate - register a gate clock with the clock framework
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of this clock's parent
 * @flags: framework-specific flags for this clock
 * @reg: register address to control gating of this clock
 * @bit_idx: which bit in the register controls gating of this clock
 * @clk_gate_flags: gate-specific flags for this clock
 * @lock: shared register lock for this clock
 */
struct clk *clk_register_gate(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 bit_idx,
		u8 clk_gate_flags, spinlock_t *lock)
{
	struct clk_gate *gate;
	struct clk *clk;
	struct clk_init_data init;

	if (clk_gate_flags & CLK_GATE_HIWORD_MASK) {
		if (bit_idx > 15) {
			pr_err("gate bit exceeds LOWORD field\n");
			return ERR_PTR(-EINVAL);
		}
	}

	/* allocate the gate */
	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_gate_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = (parent_name ? &parent_name: NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct clk_gate assignments */
	gate->reg = reg;
	gate->bit_idx = bit_idx;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->hw.init = &init;

	clk = clk_register(dev, &gate->hw);

	if (IS_ERR(clk))
		kfree(gate);

	return clk;
}
EXPORT_SYMBOL_GPL(clk_register_gate);

void clk_unregister_gate(struct clk *clk)
{
	struct clk_gate *gate;
	struct clk_hw *hw;

	hw = __clk_get_hw(clk);
	if (!hw)
		return;

	gate = to_clk_gate(hw);

	clk_unregister(clk);
	kfree(gate);
}
EXPORT_SYMBOL_GPL(clk_unregister_gate);

#ifdef CONFIG_OF
static void __init of_mmio_gate_clk_setup(struct device_node *node)
{
	struct clk_onecell_data *clk_data;
	const char *parent_name = NULL;
	struct resource res;
	void __iomem *reg = NULL;
	spinlock_t *lock = NULL;
	unsigned int clocks = 0;
	int i;

	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		goto out;

	lock = kzalloc(sizeof(*lock), GFP_KERNEL);
	if (!lock)
		goto out;
	spin_lock_init(lock);

	if (of_clk_get_parent_count(node) > 0)
		parent_name = of_clk_get_parent_name(node, 0);

	if (of_address_to_resource(node, 0, &res))
		goto out;

	reg = ioremap(res.start, resource_size(&res));
	if (!reg)
		goto out;

	clk_data->clk_num = (resource_size(&res) / 4) * 32;
	if (!clk_data->clk_num)
		goto out;

	clk_data->clks = kzalloc(sizeof(*clk_data->clks) * clk_data->clk_num,
				GFP_KERNEL);

	for (i = 0; i < clk_data->clk_num; i++)
		clk_data->clks[i] = ERR_PTR(-ENODEV);

	/* clks[] is sparse, indexed by bit, maximum clocks checked using i */
	for (i = 0; i < clk_data->clk_num; i++) {
		u32 bit;
		const char *clk_name;

		if (of_property_read_u32_index(node, "clock-indices",
				i, &bit))
			goto out;

		if (of_property_read_string_index(node, "clock-output-names",
				i, &clk_name))
			goto out;

		if (bit >= clk_data->clk_num) {
			pr_err("%s: clock bit %u out of range\n",
				node->full_name, bit);
			continue;
		}

		if (!IS_ERR(clk_data->clks[bit])) {
			pr_err("%s: clock bit %u already exists\n",
				node->full_name, bit);
			continue;
		}

		clk_data->clks[bit] = clk_register_gate(NULL, clk_name,
				parent_name, 0, reg + (bit / 32), bit,
				bit == 0 ? CLK_GATE_SET_TO_DISABLE : 0, lock);
		if (!IS_ERR(clk_data->clks[bit]))
			clocks++;
	}

out:
	if (clocks) {
		of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
		pr_info("registered %u clocks at MMIO 0x%p\n", clocks, reg);
	} else {
		if (clk_data)
			kfree(clk_data->clks);
		kfree(clk_data);
		kfree(lock);
		if (reg)
			iounmap(reg);
	}
}

CLK_OF_DECLARE(mmio_gate_clk, "mmio-gate-clock", of_mmio_gate_clk_setup);
#endif
