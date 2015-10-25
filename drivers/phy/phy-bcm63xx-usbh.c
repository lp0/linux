/*
 * Copyright 2015 Simon Arlott
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
 * Derived from arch/mips/bcm963xx/setup.c:
 * Copyright (c) 2002 Broadcom Corporation
 *
 * Derived from OpenWrt patches:
 * Copyright 2013 Maxime Bizon <mbizon@freebox.fr>
 * Copyright 2013 Florian Fainelli <florian@openwrt.org>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>

/* USBH control register offsets */
#define USBH_BRT_CONTROL1		0
#define USBH_BRT_CONTROL2		1
#define USBH_BRT_STATUS1		2
#define USBH_BRT_STATUS2		3
#define USBH_UTMI_CONTROL1		4
#define USBH_TEST_PORT_CONTROL		5
#define USBH_PLL_CONTROL1		6
#define   USBH_PLLC_REFCLKSEL_MASK		0x00000003
#define   USBH_PLLC_REFCLKSEL_SHIFT		0
#define   USBH_PLLC_CLKSEL_MASK			0x0000000c
#define   USBH_PLLC_CLKSEL_SHIFT		2
#define   USBH_PLLC_XTAL_PWRDWNB		BIT(4)
#define   USBH_PLLC_PLL_PWRDWNB			BIT(5)
#define   USBH_PLLC_PLL_CALEN			BIT(6)
#define   USBH_PLLC_PHYPLL_BYP			BIT(7)
#define   USBH_PLLC_PLL_RESET			BIT(8)
#define   USBH_PLLC_PLL_IDDQ_PWRDN		BIT(9)
#define   USBH_PLLC_PLL_PWRDN_DELAY		BIT(10)
#define   USBH_6318_PLLC_PLL_SUSPEND_EN		BIT(27)
#define   USBH_6318_PLLC_PHYPLL_BYP		BIT(29)
#define   USBH_6318_PLLC_PLL_RESET		BIT(30)
#define   USBH_6318_PLLC_PLL_IDDQ_PWRDN		BIT(31)
#define USBH_SWAP_CONTROL		7
#define   USBH_SC_OHCI_DATA_SWAP		BIT(0)
#define   USBH_SC_OHCI_ENDIAN_SWAP		BIT(1)
#define   USBH_SC_OHCI_LOGICAL_ADDRESS_EN	BIT(2)
#define   USBH_SC_EHCI_DATA_SWAP		BIT(3)
#define   USBH_SC_EHCI_ENDIAN_SWAP		BIT(4)
#define   USBH_SC_EHCI_LOGICAL_ADDRESS_EN	BIT(5)
#define   USBH_SC_USB_DEVICE_SEL		BIT(6)
#define USBH_GENERIC_CONTROL		8
#define   USBH_GC_PLL_SUSPEND_EN		BIT(1)
#define USBH_FRAME_ADJUST_VALUE		9
#define USBH_SETUP			10
#define   USBH_S_IOC				BIT(4)
#define   USBH_S_IPP				BIT(5)
#define USBH_MDIO			11
#define USBH_MDIO32			12
#define USBH_USB_SIM_CONTROL		13
#define   USBH_USC_LADDR_SEL			BIT(5)

static const s16 usbh_regs_6318[] = {
	[USBH_BRT_CONTROL1]	= -1,
	[USBH_BRT_CONTROL2]	= -1,
	[USBH_BRT_STATUS1]	 = -1,
	[USBH_BRT_STATUS2]	 = -1,
	[USBH_UTMI_CONTROL1]       = 0x2c,
	[USBH_TEST_PORT_CONTROL]   = 0x1c,
	[USBH_PLL_CONTROL1]	= 0x04,
	[USBH_SWAP_CONTROL]	= 0x0c,
	[USBH_GENERIC_CONTROL]     = -1,
	[USBH_FRAME_ADJUST_VALUE]  = 0x08,
	[USBH_SETUP]	       = 0x00,
	[USBH_MDIO]		= 0x14,
	[USBH_MDIO32]	      = 0x18,
	[USBH_USB_SIM_CONTROL]     = 0x20,
};

static const s16 usbh_regs_6328[] = {
	[USBH_BRT_CONTROL1]	= 0x00,
	[USBH_BRT_CONTROL2]	= 0x04,
	[USBH_BRT_STATUS1]	 = 0x08,
	[USBH_BRT_STATUS2]	 = 0x0c,
	[USBH_UTMI_CONTROL1]       = 0x10,
	[USBH_TEST_PORT_CONTROL]   = 0x14,
	[USBH_PLL_CONTROL1]	= 0x18,
	[USBH_SWAP_CONTROL]	= 0x1c,
	[USBH_GENERIC_CONTROL]     = 0x20,
	[USBH_FRAME_ADJUST_VALUE]  = 0x24,
	[USBH_SETUP]	       = 0x28,
	[USBH_MDIO]		= 0x2c,
	[USBH_MDIO32]	      = 0x30,
	[USBH_USB_SIM_CONTROL]     = 0x34,
};

static const s16 usbh_regs_6362[] = {
	[USBH_BRT_CONTROL1]	= 0x00,
	[USBH_BRT_CONTROL2]	= 0x04,
	[USBH_BRT_STATUS1]	 = 0x08,
	[USBH_BRT_STATUS2]	 = 0x0c,
	[USBH_UTMI_CONTROL1]       = 0x10,
	[USBH_TEST_PORT_CONTROL]   = 0x14,
	[USBH_PLL_CONTROL1]	= 0x18,
	[USBH_SWAP_CONTROL]	= 0x1c,
	[USBH_GENERIC_CONTROL]     = -1,
	[USBH_FRAME_ADJUST_VALUE]  = 0x24,
	[USBH_SETUP]	       = 0x28,
	[USBH_MDIO]		= 0x2c,
	[USBH_MDIO32]	      = 0x30,
	[USBH_USB_SIM_CONTROL]     = 0x34,
};

#define USBH_MAX_CLKS 2

struct bcm63xx_usbh_phy {
	void __iomem		*base;
	const s16		*regs;
	bool			is_6318;
	struct clk		*clks[USBH_MAX_CLKS];
	struct reset_control	*reset;
	struct regulator	*regulator;
};

static inline bool usbh_has_reg(struct bcm63xx_usbh_phy *usbh,
		int reg)
{
	return usbh->regs[reg] >= 0;
}

static inline u32 usbh_readl(struct bcm63xx_usbh_phy *usbh,
		int reg)
{
	BUG_ON(!usbh_has_reg(usbh, reg));
	return __raw_readl(usbh->base + usbh->regs[reg]);
}

static inline void usbh_writel(struct bcm63xx_usbh_phy *usbh,
		int reg, u32 value)
{
	BUG_ON(!usbh_has_reg(usbh, reg));
	__raw_writel(value, usbh->base + usbh->regs[reg]);
}

static int bcm63xx_usbh_phy_init(struct phy *phy)
{
	struct bcm63xx_usbh_phy *usbh = phy_get_drvdata(phy);
	int clk, ret;

	for (clk = 0; clk < USBH_MAX_CLKS && usbh->clks[clk]; clk++) {
		ret = clk_prepare_enable(usbh->clks[clk]);
		if (ret) {
			dev_err(&phy->dev, "unable to enable clock %s: %d\n",
				__clk_get_name(usbh->clks[clk]), ret);
			goto err_disable_clks;
		}
	}

	ret = reset_control_reset(usbh->reset);
	if (ret) {
		dev_err(&phy->dev, "unable to reset device: %d\n", ret);
		goto err_disable_clks;
	}

	/* Configure to work in native CPU endian */
	if (usbh_has_reg(usbh, USBH_SWAP_CONTROL)) {
		u32 val = usbh_readl(usbh, USBH_SWAP_CONTROL);

		val |= USBH_SC_EHCI_DATA_SWAP;
		val &= ~USBH_SC_EHCI_ENDIAN_SWAP;

		val |= USBH_SC_OHCI_DATA_SWAP;
		val &= ~USBH_SC_OHCI_ENDIAN_SWAP;

		usbh_writel(usbh, USBH_SWAP_CONTROL, val);
	}

	if (usbh_has_reg(usbh, USBH_SETUP)) {
		u32 val = usbh_readl(usbh, USBH_SETUP);

		val |= USBH_S_IOC;
		val &= ~USBH_S_IPP;

		usbh_writel(usbh, USBH_SETUP, val);
	}

	if (usbh->is_6318) {
		if (usbh_has_reg(usbh, USBH_USB_SIM_CONTROL)) {
			u32 val = usbh_readl(usbh, USBH_USB_SIM_CONTROL);

			val |= USBH_USC_LADDR_SEL;

			usbh_writel(usbh, USBH_USB_SIM_CONTROL, val);
		}
	}

	return 0;

err_disable_clks:
	while (--clk >= 0)
		clk_disable_unprepare(usbh->clks[clk]);
	return ret;
}

static int bcm63xx_usbh_phy_power_on(struct phy *phy)
{
	struct bcm63xx_usbh_phy *usbh = phy_get_drvdata(phy);
	int ret;

	ret = regulator_enable(usbh->regulator);
	if (ret) {
		dev_err(&phy->dev, "unable to power on device: %d\n", ret);
		return ret;
	}

	if (usbh_has_reg(usbh, USBH_PLL_CONTROL1)) {
		u32 val = usbh_readl(usbh, USBH_PLL_CONTROL1);

		if (usbh->is_6318) {
			val |= USBH_6318_PLLC_PLL_SUSPEND_EN;
			val &= ~USBH_6318_PLLC_PLL_IDDQ_PWRDN;
		} else {
			val &= ~USBH_PLLC_PLL_IDDQ_PWRDN;
			val &= ~USBH_PLLC_PLL_PWRDN_DELAY;
		}

		usbh_writel(usbh, USBH_PLL_CONTROL1, val);
	}

	return 0;
}

static int bcm63xx_usbh_phy_power_off(struct phy *phy)
{
	struct bcm63xx_usbh_phy *usbh = phy_get_drvdata(phy);

	if (usbh_has_reg(usbh, USBH_PLL_CONTROL1)) {
		u32 val = usbh_readl(usbh, USBH_PLL_CONTROL1);

		if (usbh->is_6318) {
			val &= ~USBH_6318_PLLC_PLL_SUSPEND_EN;
			val |= USBH_6318_PLLC_PLL_IDDQ_PWRDN;
		} else {
			val |= USBH_PLLC_PLL_IDDQ_PWRDN;
			val |= USBH_PLLC_PLL_PWRDN_DELAY;
		}

		usbh_writel(usbh, USBH_PLL_CONTROL1, val);
	}

	return regulator_disable(usbh->regulator);
}

static int bcm63xx_usbh_phy_exit(struct phy *phy)
{
	struct bcm63xx_usbh_phy *usbh = phy_get_drvdata(phy);
	int clk;

	for (clk = USBH_MAX_CLKS - 1; clk >= 0; clk--)
		if (usbh->clks[clk])
			clk_disable_unprepare(usbh->clks[clk]);

	return 0;
}

static const struct phy_ops bcm63xx_usbh_phy_ops = {
	.init		= bcm63xx_usbh_phy_init,
	.power_on	= bcm63xx_usbh_phy_power_on,
	.power_off	= bcm63xx_usbh_phy_power_off,
	.exit		= bcm63xx_usbh_phy_exit,
	.owner		= THIS_MODULE,
};

static int bcm63xx_usbh_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bcm63xx_usbh_phy	*usbh;
	struct resource *res;
	struct phy *phy;
	struct phy_provider *phy_provider;
	int ret, clk;

	usbh = devm_kzalloc(dev, sizeof(*usbh), GFP_KERNEL);
	if (!usbh)
		return -ENOMEM;

	usbh->regs = of_device_get_match_data(dev);
	if (!usbh->regs)
		return -EINVAL;

	usbh->is_6318 = of_device_is_compatible(dev->of_node,
				"brcm,bcm6318-usbh");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	usbh->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(usbh->base))
		return PTR_ERR(usbh->base);

	usbh->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(usbh->regulator)) {
		ret = PTR_ERR(usbh->regulator);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get regulator: %d\n", ret);
		return ret;
	}

	usbh->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(usbh->reset)) {
		ret = PTR_ERR(usbh->reset);
		if (ret != -EPROBE_DEFER)
			dev_err(dev,
				"failed to get reset controller: %d\n", ret);
		return ret;
	}

	for (clk = 0; clk < USBH_MAX_CLKS; clk++) {
		usbh->clks[clk] = of_clk_get(dev->of_node, clk);
		if (IS_ERR(usbh->clks[clk])) {
			ret = PTR_ERR(usbh->clks[clk]);
			if (ret == -EPROBE_DEFER)
				goto err_put_clks;
			usbh->clks[clk] = NULL;
			break;
		}
	}

	phy = devm_phy_create(dev, NULL, &bcm63xx_usbh_phy_ops);
	if (IS_ERR(phy)) {
		ret = PTR_ERR(phy);
		dev_err(dev, "failed to create PHY\n");
		goto err_put_clks;
	}

	platform_set_drvdata(pdev, usbh);
	phy_set_drvdata(phy, usbh);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	ret = PTR_ERR_OR_ZERO(phy_provider);
	if (ret) {
		dev_err(dev, "failed to register PHY provider: %d\n", ret);
		goto err_put_clks;
	}

	dev_info(dev, "registered USBH PHY at MMIO 0x%p\n", usbh->base);
	return 0;

err_put_clks:
	while (--clk >= 0)
		clk_put(usbh->clks[clk]);
	return ret;
}

static int bcm63xx_usbh_phy_remove(struct platform_device *pdev)
{
	struct bcm63xx_usbh_phy *usbh = platform_get_drvdata(pdev);
	int clk;

	for (clk = 0; clk < USBH_MAX_CLKS && usbh->clks[clk]; clk++)
		clk_put(usbh->clks[clk]);

	return 0;
}

static const struct of_device_id bcm63xx_usbh_phy_ids[] = {
	{ .compatible = "brcm,bcm6318-usbh", .data = usbh_regs_6318 },
	{ .compatible = "brcm,bcm6362-usbh", .data = usbh_regs_6362 },
	{ .compatible = "brcm,bcm6816-usbh", .data = usbh_regs_6362 },
	{ .compatible = "brcm,bcm63168-usbh", .data = usbh_regs_6328 },
	{ .compatible = "brcm,bcm63268-usbh", .data = usbh_regs_6328 },
	{ .compatible = "brcm,bcm68220-usbh", .data = usbh_regs_6328 },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm63xx_usbh_phy_ids);

static struct platform_driver bcm63xx_usbh_phy_driver = {
	.probe	  = bcm63xx_usbh_phy_probe,
	.remove	 = bcm63xx_usbh_phy_remove,
	.driver	 = {
		.name   = "bcm63xx-usbh",
		.of_match_table = bcm63xx_usbh_phy_ids,
	},
};

module_platform_driver(bcm63xx_usbh_phy_driver);

MODULE_ALIAS("platform:bcm63xx-usbh");
MODULE_DESCRIPTION("BCM63xx USBH PHY driver");
MODULE_AUTHOR("Simon Arlott");
MODULE_LICENSE("GPL");
