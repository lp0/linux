/*
 * Copyright 2015 Simon Arlott
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on clk-gate.c:
 * Copyright (C) 2010-2011 Canonical Ltd <jeremy.kerr@canonical.com>
 * Copyright (C) 2011-2012 Mike Turquette, Linaro Ltd <mturquette@linaro.org>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/slab.h>
#include <linux/string.h>

/**
 * DOC: Basic clock which uses a bit in a regmap to gate and ungate the output
 *
 * Traits of this clock:
 * prepare - clk_(un)prepare only ensures parent is (un)prepared
 * enable - clk_enable and clk_disable are functional & control gating
 * rate - inherits rate from parent.  No clk_set_rate support
 * parent - fixed parent.  No clk_set_parent support
 */

struct clk_bcm63xx {
	struct clk_hw hw;
	struct regmap *map;
	u32 offset;
	u32 mask;
};

#define to_clk_bcm63xx(_hw) container_of(_hw, struct clk_bcm63xx, hw)

static int clk_bcm63xx_enable(struct clk_hw *hw)
{
	struct clk_bcm63xx *gate = to_clk_bcm63xx(hw);

	return regmap_write_bits(gate->map, gate->offset,
					gate->mask, gate->mask);
}

static void clk_bcm63xx_disable(struct clk_hw *hw)
{
	struct clk_bcm63xx *gate = to_clk_bcm63xx(hw);

	regmap_write_bits(gate->map, gate->offset,
					gate->mask, 0);
}

static int clk_bcm63xx_is_enabled(struct clk_hw *hw)
{
	struct clk_bcm63xx *gate = to_clk_bcm63xx(hw);
	unsigned int val;
	int ret;

	ret = regmap_read(gate->map, gate->offset, &val);
	if (ret)
		return ret;

	val &= gate->mask;

	return val ? 1 : 0;
}

const struct clk_ops clk_bcm63xx_ops = {
	.enable = clk_bcm63xx_enable,
	.disable = clk_bcm63xx_disable,
	.is_enabled = clk_bcm63xx_is_enabled,
};

static struct clk * __init of_bcm63xx_clk_register(const char *parent_name,
	const char *clk_name, struct regmap *map, u32 offset, u32 mask)
{
	struct clk_bcm63xx *gate;
	struct clk_init_data init;
	struct clk *ret;

	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = clk_name;
	init.ops = &clk_bcm63xx_ops;
	init.flags = CLK_IS_BASIC;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	gate->hw.init = &init;
	gate->map = map;
	gate->offset = offset;
	gate->mask = mask;

	ret = clk_register(NULL, &gate->hw);
	if (IS_ERR(ret))
		kfree(gate);

	return ret;
}

static void __init of_bcm63xx_clk_setup(struct device_node *node)
{
	struct clk_onecell_data *clk_data;
	const char *parent_name = NULL;
	struct regmap *map;
	u32 offset;
	unsigned int clocks = 0;
	int i;

	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		return;

	clk_data->clk_num = 32;
	clk_data->clks = kmalloc_array(clk_data->clk_num,
		sizeof(*clk_data->clks), GFP_KERNEL);

	for (i = 0; i < clk_data->clk_num; i++)
		clk_data->clks[i] = ERR_PTR(-ENODEV);

	if (of_clk_get_parent_count(node) > 0)
		parent_name = of_clk_get_parent_name(node, 0);

	map = syscon_regmap_lookup_by_phandle(node, "regmap");
	if (IS_ERR(map)) {
		pr_err("%s: regmap lookup error %ld\n",
			node->full_name, PTR_ERR(map));
		goto out;
	}

	if (of_property_read_u32(node, "offset", &offset)) {
		pr_err("%s: offset not specified\n", node->full_name);
		goto out;
	}

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

		clk_data->clks[bit] = of_bcm63xx_clk_register(parent_name,
					clk_name, map, offset, BIT(bit));
		if (!IS_ERR(clk_data->clks[bit]))
			clocks++;
	}

out:
	if (clocks) {
		of_clk_add_provider(node, of_clk_src_onecell_get, clk_data);
		pr_info("%s: registered %u clocks\n", node->name, clocks);
	} else {
		kfree(clk_data->clks);
		kfree(clk_data);
	}
}

CLK_OF_DECLARE(bcm63xx_clk, "brcm,bcm6345-gate-clk", of_bcm63xx_clk_setup);
