/*
 * BCM63XX CFE image tag parser
 *
 * Copyright © 2006-2008  Florian Fainelli <florian@openwrt.org>
 *			  Mike Albon <malbon@openwrt.org>
 * Copyright © 2009-2010  Daniel Dickinson <openwrt@cshore.neomailbox.net>
 * Copyright © 2011-2013  Jonas Gorski <jonas.gorski@gmail.com>
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
 * NAND flash layout derived from bcm963xx_4.12L.06B_consumer/bcmdrivers/opensource/char/board/bcm963xx/impl1/board.c,
 *	bcm963xx_4.12L.06B_consumer/shared/opensource/include/bcm963xx/bcm_hwdefs.h:
 * Copyright (c) 2002 Broadcom Corporation
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bcm963xx_nvram.h>
#include <linux/bcm963xx_tag.h>
#include <linux/crc32.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#define BCM963XX_CFE_BLOCK_SIZE		SZ_64K	/* always at least 64KiB */

#define BCM963XX_CFE_MAGIC_OFFSET	0x4e0
#define BCM963XX_CFE_VERSION_OFFSET	0x570
#define BCM963XX_NVRAM_OFFSET		0x580

/* Ensure strings read from flash structs are null terminated */
#define STR_NULL_TERMINATE(x) \
	do { char *_str = (x); _str[sizeof(x) - 1] = 0; } while (0)

static int bcm63xx_detect_cfe(struct mtd_info *master)
{
	char buf[9];
	int ret;
	size_t retlen;

	ret = mtd_read(master, BCM963XX_CFE_VERSION_OFFSET, 5, &retlen,
		       (void *)buf);
	buf[retlen] = 0;

	if (ret)
		return ret;

	if (strncmp("cfe-v", buf, 5) == 0)
		return 0;

	/* very old CFE's do not have the cfe-v string, so check for magic */
	ret = mtd_read(master, BCM963XX_CFE_MAGIC_OFFSET, 8, &retlen,
		       (void *)buf);
	buf[retlen] = 0;

	return strncmp("CFE1CFE1", buf, 8);
}

static int bcm63xx_read_nvram(struct mtd_info *master,
	struct bcm963xx_nvram *nvram)
{
	u32 actual_crc, expected_crc;
	size_t retlen;
	int ret;

	/* extract nvram data */
	ret = mtd_read(master, BCM963XX_NVRAM_OFFSET, BCM963XX_NVRAM_V5_SIZE,
			&retlen, (void *)nvram);
	if (ret)
		return ret;

	ret = bcm963xx_nvram_checksum(nvram, &expected_crc, &actual_crc);
	if (ret)
		pr_warn("nvram checksum failed, contents may be invalid (expected %08x, got %08x)\n",
			expected_crc, actual_crc);

	if (!nvram->psi_size)
		nvram->psi_size = BCM963XX_DEFAULT_PSI_SIZE;

	return 0;
}

static int bcm63xx_read_image_tag(struct mtd_info *master, const char *name,
	loff_t tag_offset, struct bcm_tag *buf)
{
	int ret;
	size_t retlen;
	u32 computed_crc;

	ret = mtd_read(master, tag_offset, sizeof(*buf), &retlen, (void *)buf);
	if (ret)
		return ret;

	if (retlen != sizeof(*buf))
		return -EIO;

	computed_crc = crc32_le(IMAGETAG_CRC_START, (u8 *)buf,
				offsetof(struct bcm_tag, header_crc));
	if (computed_crc == buf->header_crc) {
		STR_NULL_TERMINATE(buf->board_id);
		STR_NULL_TERMINATE(buf->tag_version);

		pr_info("%s: CFE image tag found at 0x%llx with version %s, board type %s\n",
			name, tag_offset, buf->tag_version, buf->board_id);

		return 0;
	}

	pr_warn("%s: CFE image tag at 0x%llx CRC invalid (expected %08x, actual %08x)\n",
		name, tag_offset, buf->header_crc, computed_crc);
	return 1;
}

static bool bcm63xx_boot_latest(struct bcm963xx_nvram *nvram)
{
	char *p;

	STR_NULL_TERMINATE(nvram->bootline);

	/* Find previous image parameter "p" */
	if (!strncmp(nvram->bootline, "p=", 2))
		p = nvram->bootline;
	else
		p = strstr(nvram->bootline, " p=");

	if (p == NULL)
		return true;

	p += 2;
	return *p != '0';
}

static int bcm63xx_parse_cfe_nor_partitions(struct mtd_info *master,
	const struct mtd_partition **pparts, struct bcm963xx_nvram *nvram)
{
	/* CFE, NVRAM and global Linux are always present */
	int nrparts = 3, curpart = 0;
	struct bcm_tag *buf = NULL;
	struct mtd_partition *parts;
	int ret;
	unsigned int rootfsaddr, kerneladdr, spareaddr;
	unsigned int rootfslen, kernellen, sparelen, totallen;
	unsigned int cfelen, nvramlen;
	unsigned int cfe_erasesize;
	int i;
	bool rootfs_first = false;

	cfe_erasesize = max_t(uint32_t, master->erasesize,
			      BCM963XX_CFE_BLOCK_SIZE);

	cfelen = cfe_erasesize;
	nvramlen = nvram->psi_size * SZ_1K;
	nvramlen = roundup(nvramlen, cfe_erasesize);

	buf = vmalloc(sizeof(struct bcm_tag));
	if (!buf)
		return -ENOMEM;

	/* Get the tag */
	ret = bcm63xx_read_image_tag(master, "rootfs", cfelen, buf);
	if (!ret) {
		STR_NULL_TERMINATE(buf->flash_image_start);
		if (kstrtouint(buf->flash_image_start, 10, &rootfsaddr) ||
				rootfsaddr < BCM963XX_EXTENDED_SIZE) {
			pr_err("invalid rootfs address: %*ph\n",
				sizeof(buf->flash_image_start),
				buf->flash_image_start);
			goto invalid_tag;
		}

		STR_NULL_TERMINATE(buf->kernel_address);
		if (kstrtouint(buf->kernel_address, 10, &kerneladdr) ||
				kerneladdr < BCM963XX_EXTENDED_SIZE) {
			pr_err("invalid kernel address: %*ph\n",
				sizeof(buf->kernel_address),
				buf->kernel_address);
			goto invalid_tag;
		}

		STR_NULL_TERMINATE(buf->kernel_length);
		if (kstrtouint(buf->kernel_length, 10, &kernellen)) {
			pr_err("invalid kernel length: %*ph\n",
				sizeof(buf->kernel_length),
				buf->kernel_length);
			goto invalid_tag;
		}

		STR_NULL_TERMINATE(buf->total_length);
		if (kstrtouint(buf->total_length, 10, &totallen)) {
			pr_err("invalid total length: %*ph\n",
				sizeof(buf->total_length),
				buf->total_length);
			goto invalid_tag;
		}

		kerneladdr = kerneladdr - BCM963XX_EXTENDED_SIZE;
		rootfsaddr = rootfsaddr - BCM963XX_EXTENDED_SIZE;
		spareaddr = roundup(totallen, master->erasesize) + cfelen;

		if (rootfsaddr < kerneladdr) {
			/* default Broadcom layout */
			rootfslen = kerneladdr - rootfsaddr;
			rootfs_first = true;
		} else {
			/* OpenWrt layout */
			rootfsaddr = kerneladdr + kernellen;
			rootfslen = spareaddr - rootfsaddr;
		}
	} else if (ret > 0) {
invalid_tag:
		kernellen = 0;
		rootfslen = 0;
		rootfsaddr = 0;
		spareaddr = cfelen;
	} else {
		goto out;
	}
	sparelen = master->size - spareaddr - nvramlen;

	/* Determine number of partitions */
	if (rootfslen > 0)
		nrparts++;

	if (kernellen > 0)
		nrparts++;

	parts = kzalloc(sizeof(*parts) * nrparts + 10 * nrparts, GFP_KERNEL);
	if (!parts) {
		ret = -ENOMEM;
		goto out;
	}

	/* Start building partition list */
	parts[curpart].name = "CFE";
	parts[curpart].offset = 0;
	parts[curpart].size = cfelen;
	curpart++;

	if (kernellen > 0) {
		int kernelpart = curpart;

		if (rootfslen > 0 && rootfs_first)
			kernelpart++;
		parts[kernelpart].name = "kernel";
		parts[kernelpart].offset = kerneladdr;
		parts[kernelpart].size = kernellen;
		curpart++;
	}

	if (rootfslen > 0) {
		int rootfspart = curpart;

		if (kernellen > 0 && rootfs_first)
			rootfspart--;
		parts[rootfspart].name = "rootfs";
		parts[rootfspart].offset = rootfsaddr;
		parts[rootfspart].size = rootfslen;
		if (sparelen > 0  && !rootfs_first)
			parts[rootfspart].size += sparelen;
		curpart++;
	}

	parts[curpart].name = "nvram";
	parts[curpart].offset = master->size - nvramlen;
	parts[curpart].size = nvramlen;
	curpart++;

	/* Global partition "linux" to make easy firmware upgrade */
	parts[curpart].name = "linux";
	parts[curpart].offset = cfelen;
	parts[curpart].size = master->size - cfelen - nvramlen;

	for (i = 0; i < nrparts; i++)
		pr_info("Partition %d is %s offset %llx and length %llx\n", i,
			parts[i].name, parts[i].offset,	parts[i].size);

	pr_info("Spare partition is offset %x and length %x\n",	spareaddr,
		sparelen);

	*pparts = parts;
	ret = 0;

out:
	vfree(buf);

	if (ret)
		return ret;

	return nrparts;
}

static bool bcm63xx_parse_nand_image_tag(struct mtd_info *master,
	const char *name, loff_t tag_offset, u64 *rootfs_offset,
	u64 *rootfs_size, unsigned int *rootfs_sequence)
{
	struct bcm_tag *buf;
	int ret;
	bool rootfs_ok = false;

	*rootfs_offset = 0;
	*rootfs_size = 0;
	*rootfs_sequence = 0;

	buf = vmalloc(sizeof(struct bcm_tag));
	if (!buf)
		goto out;

	ret = bcm63xx_read_image_tag(master, name, tag_offset, buf);
	if (!ret) {
		/* Get rootfs offset and size from tag data */
		STR_NULL_TERMINATE(buf->flash_image_start);
		if (kstrtou64(buf->flash_image_start, 10, rootfs_offset) ||
				*rootfs_offset < BCM963XX_EXTENDED_SIZE) {
			pr_err("%s: invalid rootfs offset: %*ph\n", name,
				sizeof(buf->flash_image_start),
				buf->flash_image_start);
			goto out;
		}

		STR_NULL_TERMINATE(buf->root_length);
		if (kstrtou64(buf->root_length, 10, rootfs_size) ||
				rootfs_size == 0) {
			pr_err("%s: invalid rootfs size: %*ph\n", name,
				sizeof(buf->root_length), buf->root_length);
			goto out;
		}

		/* Adjust for flash offset */
		*rootfs_offset -= BCM963XX_EXTENDED_SIZE;

		/* Remove bcm_tag data from length */
		*rootfs_size -= *rootfs_offset - tag_offset;

		/* Get image sequence number to determine which one is newer */
		STR_NULL_TERMINATE(buf->image_sequence);
		if (kstrtouint(buf->image_sequence, 10, rootfs_sequence)) {
			pr_err("%s: invalid rootfs sequence: %*ph\n", name,
				sizeof(buf->image_sequence),
				buf->image_sequence);
			goto out;
		}

		rootfs_ok = true;
	}

out:
	vfree(buf);
	return rootfs_ok;
}

static int bcm63xx_parse_cfe_nand_partitions(struct mtd_info *master,
	const struct mtd_partition **pparts,
	struct bcm963xx_nvram *nvram)
{
	int nrparts, i;
	struct mtd_partition *parts;
	u64 rootfs1_off, rootfs1_size;
	unsigned int rootfs1_seq;
	u64 rootfs2_off, rootfs2_size;
	unsigned int rootfs2_seq;
	bool rootfs1, rootfs2;
	bool use_first;

	if (nvram->version < 6) {
		pr_warn("nvram version %u not supported\n", nvram->version);
		return -EINVAL;
	}

	/* We've just read the nvram from offset 0,
	 * so it must be located there.
	 */
	if (BCM963XX_NVRAM_NAND_PART_OFFSET(nvram, BOOT) != 0)
		return -EINVAL;

	/* Get the rootfs partition locations */
	rootfs1 = bcm63xx_parse_nand_image_tag(master, "rootfs1",
		BCM963XX_NVRAM_NAND_PART_OFFSET(nvram, ROOTFS_1),
		&rootfs1_off, &rootfs1_size, &rootfs1_seq);
	rootfs2 = bcm63xx_parse_nand_image_tag(master, "rootfs2",
		BCM963XX_NVRAM_NAND_PART_OFFSET(nvram, ROOTFS_2),
		&rootfs2_off, &rootfs2_size, &rootfs2_seq);

	/* Determine primary rootfs partition */
	if (rootfs1 && rootfs2) {
		bool use_latest = bcm63xx_boot_latest(nvram);

		/* Compare sequence numbers */
		if (use_latest)
			use_first = rootfs1_seq > rootfs2_seq;
		else
			use_first = rootfs1_seq < rootfs2_seq;

		pr_info("CFE bootline selected %s image rootfs%u (rootfs1_seq=%u, rootfs2_seq=%u)\n",
			use_latest ? "latest" : "previous",
			use_first ? 1 : 2,
			rootfs1_seq, rootfs2_seq);
	} else {
		use_first = rootfs1;
	}

	/* Partitions:
	 * 1 boot
	 * 2 rootfs
	 * 3 data
	 * 4 rootfs1_update
	 * 5 rootfs2_update
	 * 6 rootfs_other
	 */
	nrparts = 6;
	i = 0;

	parts = kcalloc(nrparts, sizeof(*parts), GFP_KERNEL);
	if (!parts)
		return -ENOMEM;

	parts[i].name = "boot";
	parts[i].offset = BCM963XX_NVRAM_NAND_PART_OFFSET(nvram, BOOT);
	parts[i].size = BCM963XX_NVRAM_NAND_PART_SIZE(nvram, BOOT);
	i++;

	/* Primary rootfs if either is available */
	if (rootfs1 || rootfs2) {
		parts[i].name = "rootfs";
		parts[i].offset = use_first ? rootfs1_off : rootfs2_off;
		parts[i].size = use_first ? rootfs1_size : rootfs2_size;
		i++;
	}

	parts[i].name = "data";
	parts[i].offset = BCM963XX_NVRAM_NAND_PART_OFFSET(nvram, DATA);
	parts[i].size = BCM963XX_NVRAM_NAND_PART_SIZE(nvram, DATA);
	i++;

	/* Full rootfs partitions for updates */
	parts[i].name = "rootfs1_update";
	parts[i].offset = BCM963XX_NVRAM_NAND_PART_OFFSET(nvram, ROOTFS_1);
	parts[i].size = BCM963XX_NVRAM_NAND_PART_SIZE(nvram, ROOTFS_1);
	i++;

	parts[i].name = "rootfs2_update";
	parts[i].offset = BCM963XX_NVRAM_NAND_PART_OFFSET(nvram, ROOTFS_2);
	parts[i].size = BCM963XX_NVRAM_NAND_PART_SIZE(nvram, ROOTFS_2);
	i++;

	/* Other rootfs if both are available */
	if (rootfs1 && rootfs2) {
		parts[i].name = "rootfs_other";
		parts[i].offset = use_first ? rootfs2_off : rootfs1_off;
		parts[i].size = use_first ? rootfs2_size : rootfs1_size;
		i++;
	}

	*pparts = parts;
	return nrparts;
}

static int bcm63xx_parse_cfe_partitions(struct mtd_info *master,
					const struct mtd_partition **pparts,
					struct mtd_part_parser_data *data)
{
	struct bcm963xx_nvram *nvram = NULL;
	int ret;

	if (bcm63xx_detect_cfe(master))
		return -EINVAL;

	nvram = vzalloc(sizeof(*nvram));
	if (!nvram)
		return -ENOMEM;

	ret = bcm63xx_read_nvram(master, nvram);
	if (ret)
		goto out;

	if (!mtd_type_is_nand(master))
		ret = bcm63xx_parse_cfe_nor_partitions(master, pparts, nvram);
	else
		ret = bcm63xx_parse_cfe_nand_partitions(master, pparts, nvram);

out:
	vfree(nvram);
	return ret;
};

static struct mtd_part_parser bcm63xx_cfe_parser = {
	.parse_fn = bcm63xx_parse_cfe_partitions,
	.name = "bcm63xxpart",
};
module_mtd_part_parser(bcm63xx_cfe_parser);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Dickinson <openwrt@cshore.neomailbox.net>");
MODULE_AUTHOR("Florian Fainelli <florian@openwrt.org>");
MODULE_AUTHOR("Mike Albon <malbon@openwrt.org>");
MODULE_AUTHOR("Jonas Gorski <jonas.gorski@gmail.com>");
MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("MTD partitioning for BCM63XX CFE bootloaders");
