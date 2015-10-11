/*
 * Broadcom BCM63xx flash registration
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 * Copyright (C) 2008 Florian Fainelli <florian@openwrt.org>
 * Copyright (C) 2012 Jonas Gorski <jonas.gorski@gmail.com>
 * Copyright 2015 Simon Arlott
 *
 * NAND flash support derived from code:
 * Copyright 2000-2010 Broadcom Corporation
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>

#include <bcm63xx_cpu.h>
#include <bcm63xx_dev_flash.h>
#include <bcm63xx_regs.h>
#include <bcm63xx_io.h>

#define NAND_PFX "bcm63xx_flash: NAND "

static struct mtd_partition mtd_partitions[] = {
	{
		.name		= "cfe",
		.offset		= 0x0,
		.size		= 0x40000,
	}
};

static const char *bcm63xx_part_types[] = { "bcm63xxpart", NULL };

static struct physmap_flash_data flash_data = {
	.width			= 2,
	.parts			= mtd_partitions,
	.part_probe_types	= bcm63xx_part_types,
};

static struct resource mtd_resources[] = {
	{
		.start		= 0,	/* filled at runtime */
		.end		= 0,	/* filled at runtime */
		.flags		= IORESOURCE_MEM,
	}
};

static struct platform_device mtd_dev = {
	.name			= "physmap-flash",
	.resource		= mtd_resources,
	.num_resources		= ARRAY_SIZE(mtd_resources),
	.dev			= {
		.platform_data	= &flash_data,
	},
};

static int __init bcm63xx_detect_flash_type(void)
{
	u32 val;

	switch (bcm63xx_get_cpu_id()) {
	case BCM6328_CPU_ID:
		val = bcm_misc_readl(MISC_STRAPBUS_6328_REG);
		if (val & STRAPBUS_6328_BOOT_SEL_SERIAL)
			return BCM63XX_FLASH_TYPE_SERIAL;
		else
			return BCM63XX_FLASH_TYPE_NAND;
	case BCM6338_CPU_ID:
	case BCM6345_CPU_ID:
	case BCM6348_CPU_ID:
		/* no way to auto detect so assume parallel */
		return BCM63XX_FLASH_TYPE_PARALLEL;
	case BCM3368_CPU_ID:
	case BCM6358_CPU_ID:
		val = bcm_gpio_readl(GPIO_STRAPBUS_REG);
		if (val & STRAPBUS_6358_BOOT_SEL_PARALLEL)
			return BCM63XX_FLASH_TYPE_PARALLEL;
		else
			return BCM63XX_FLASH_TYPE_SERIAL;
	case BCM6362_CPU_ID:
		val = bcm_misc_readl(MISC_STRAPBUS_6362_REG);
		if (val & STRAPBUS_6362_BOOT_SEL_SERIAL)
			return BCM63XX_FLASH_TYPE_SERIAL;
		else
			return BCM63XX_FLASH_TYPE_NAND;
	case BCM6368_CPU_ID:
		val = bcm_gpio_readl(GPIO_STRAPBUS_REG);
		switch (val & STRAPBUS_6368_BOOT_SEL_MASK) {
		case STRAPBUS_6368_BOOT_SEL_NAND:
			return BCM63XX_FLASH_TYPE_NAND;
		case STRAPBUS_6368_BOOT_SEL_SERIAL:
			return BCM63XX_FLASH_TYPE_SERIAL;
		case STRAPBUS_6368_BOOT_SEL_PARALLEL:
			return BCM63XX_FLASH_TYPE_PARALLEL;
		}
	case BCM63168_CPU_ID:
		val = bcm_misc_readl(MISC_STRAPBUS_63168_REG);
		if (val & STRAPBUS_63168_BOOT_SEL_SERIAL)
			return BCM63XX_FLASH_TYPE_SERIAL;
		else
			return BCM63XX_FLASH_TYPE_NAND;
	default:
		return -EINVAL;
	}
}

static int __init bcm63xx_flash_nand_wait(u32 mask)
{
	u32 data;
	int poll_count = 0;

	do {
		data = bcm_nand_readl(NAND_INTF_STAT_REG);
	} while (!(data & mask) && (++poll_count < 2000000));

	if (!(data & mask)) {
		printk(KERN_ERR NAND_PFX "timeout after poll_count=%d\n", poll_count);
		return -EIO;
	}

	return 0;
}

static int __init bcm63xx_flash_nand_init(void)
{
	u16 revision;
	u16 chip_id;
	int ret;

	if (!BCMCPU_IS_6328()) {
		u32 perf_en;
		u32 nand_en;

		if (BCMCPU_IS_6362())
			nand_en = CKCTL_6362_NAND_EN;
		else if (BCMCPU_IS_6368())
			nand_en = CKCTL_6368_NAND_EN;
		else if (BCMCPU_IS_63168())
			nand_en = CKCTL_63168_NAND_EN;
		else
			return -ENODEV;

		/* Enable NAND data on MII ports. */
		perf_en = bcm_perf_readl(PERF_CKCTL_REG);
		perf_en |= nand_en;
		bcm_perf_writel(perf_en, PERF_CKCTL_REG);
	}

	if (BCMCPU_IS_6362() || BCMCPU_IS_63168()) {
		u32 gpio_mode;

		gpio_mode = bcm_gpio_readl(GPIO_BASEMODE_REG);
		gpio_mode |= GPIO_BASEMODE_NAND;
		bcm_gpio_writel(gpio_mode, GPIO_BASEMODE_REG);
	}

	revision = bcm_nand_readl(NAND_REVISION_REG);
	printk(KERN_INFO NAND_PFX "device: revision %#04x\n", revision);
	if (revision < 0x0400 || revision >= 0x0600) {
		/* < 4.0 untested, 6.0+ moves some registers */
		printk(KERN_WARNING NAND_PFX "revision not supported\n");
		return -ENODEV;
	}

	bcm_nand_writel(NAND_BOOT_CFG_AUTO_DEV_ID
			| NAND_BOOT_CFG_EBC_CS0_SEL, NAND_BOOT_CFG_REG);
	bcm_nand_writel(0, NAND_CMD_ADDR_REG);
	bcm_nand_writel(0, NAND_CMD_EXT_ADDR_REG);

	bcm_nand_writel(NAND_CMD_START_DEV_ID_READ, NAND_CMD_START_REG);
	ret = bcm63xx_flash_nand_wait(NAND_INTF_STAT_FLASH_READY);
	if (ret)
		return ret;

	chip_id = bcm_nand_readl(NAND_FLASH_DEV_ID_REG) >> 16;
	printk(KERN_INFO NAND_PFX "chip: id %#04x\n", chip_id);

	bcm_nand_writel(0, NAND_CS_XOR_REG);
	bcm_nand_writel(0, NAND_SEMAPHORE_REG);

	/* Don't attempt to reconfigure the NAND controller
	 * for the flash chip, assume CFE has already done this.
	 */
	return 0;
}

int __init bcm63xx_flash_register(void)
{
	int flash_type;
	u32 val;
	int ret;

	flash_type = bcm63xx_detect_flash_type();

	switch (flash_type) {
	case BCM63XX_FLASH_TYPE_PARALLEL:
		/* read base address of boot chip select (0) */
		val = bcm_mpi_readl(MPI_CSBASE_REG(0));
		val &= MPI_CSBASE_BASE_MASK;

		mtd_resources[0].start = val;
		mtd_resources[0].end = 0x1FFFFFFF;

		return platform_device_register(&mtd_dev);
	case BCM63XX_FLASH_TYPE_SERIAL:
		pr_warn("unsupported serial flash detected\n");
		return -ENODEV;
	case BCM63XX_FLASH_TYPE_NAND:
		ret = bcm63xx_flash_nand_init();
		if (ret == -ENODEV)
			pr_warn("unsupported nand flash detected\n");

		/* don't register a device, leave that to brcmnand */
		return ret;
	default:
		pr_err("flash detection failed for BCM%x: %d\n",
		       bcm63xx_get_cpu_id(), flash_type);
		return -ENODEV;
	}
}
