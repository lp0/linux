/*
 * BCM963268 CFE image tag parser
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
 * Derived from drivers/mtd/bcm63xxpart.c:
 * Copyright © 2006-2008  Florian Fainelli <florian@openwrt.org>
 *			  Mike Albon <malbon@openwrt.org>
 * Copyright © 2009-2010  Daniel Dickinson <openwrt@cshore.neomailbox.net>
 * Copyright © 2011-2013  Jonas Gorski <jonas.gorski@gmail.com>
 *
 * Derived from bcm963xx_4.12L.06B_consumer/bcmdrivers/opensource/char/board/bcm963xx/impl1/board.c,
 *	bcm963xx_4.12L.06B_consumer/shared/opensource/include/bcm963xx/bcm_hwdefs.h:
 * Copyright (c) 2002 Broadcom Corporation
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/crc32.h>
#include <linux/if_ether.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/mach-bcm63xx/bcm963xx_tag.h>

#define BCM63XX_EXTENDED_SIZE		0xBFC00000	/* Extended flash address */

#define BCM63XX_CFE_MAGIC_OFFSET	0x4e0
#define BCM963XX_CFE_VERSION_OFFSET	0x570
#define BCM963XX_NVRAM_OFFSET		0x580

enum bcm963268_nvram_part {
	PART_BOOT = 0,
	PART_ROOTFS_1,
	PART_ROOTFS_2,
	PART_DATA,
	PART_BBT,
	NR_PARTS
};

/*
 * nvram structure
 */
struct bcm963268_nvram {
	u32	version;
	char	bootline[256];
	u8	name[16];
	u32	main_tp_number;
	u32	psi_size;
	u32	mac_addr_count;
	u8	mac_addr_base[ETH_ALEN];
	u8	reserved1[2];
	u32	checksum_old;
	u8	reserved2[292];
	u32	part_offset[NR_PARTS];
	u32	part_size[NR_PARTS];
	u8	reserved3[254];
	char	partition_number;
	u8	reserved4[133];
	u32	checksum_high;
} __attribute__((packed));

static int bcm963268_detect_cfe(struct mtd_info *master)
{
	char buf[9];
	int ret;
	size_t retlen;

	ret = mtd_read(master, BCM963XX_CFE_VERSION_OFFSET, 5, &retlen,
		       (void *)buf);
	buf[retlen] = 0;

	if (ret < 0)
		return ret;

	if (strncmp("cfe-v", buf, 5) == 0)
		return 0;

	return -EINVAL;
}

static int bcm963268_read_nvram(struct mtd_info *master,
	struct bcm963268_nvram *nvram)
{
	unsigned int check_len;
	u32 crc, expected_crc;
	size_t retlen;
	int ret;

	/* extract nvram data */
	ret = mtd_read(master, BCM963XX_NVRAM_OFFSET, sizeof(*nvram), &retlen,
		       (void *)nvram);

	if (ret < 0)
		return ret;

	if (nvram->version < 6) {
		pr_warn("nvram version %d not supported\n", nvram->version);
		return -EINVAL;
	}

	/* check checksum before using data */
	check_len = sizeof(*nvram);
	expected_crc = nvram->checksum_high;
	nvram->checksum_high = 0;

	crc = crc32_le(~0, (u8 *)nvram, check_len);

	if (crc != expected_crc)
		pr_warn("nvram checksum failed, contents may be invalid (expected %08x, got %08x)\n",
			expected_crc, crc);

	return 0;
}

static bool bcm963268_boot_latest(struct bcm963268_nvram *nvram)
{
	char *p;

	/* Ensure bootline is null terminated */
	nvram->bootline[sizeof(nvram->bootline) - 1] = 0;

	/* Find previous image parameter "p" */
	if (!strncmp(nvram->bootline, "p=", 2))
		p = nvram->bootline;
	else
		p = strstr(nvram->bootline, " p=");

	if (p == NULL)
		return true;

	p += 2;
	if (*p == '\0')
		return true;

	return *p != '0';
}

static bool bcm963268_parse_rootfs_tag(struct mtd_info *master,
	const char *name, unsigned int rootfs_part, unsigned int *rootfs_offset,
	unsigned int *rootfs_size, unsigned int *rootfs_sequence)
{
	struct bcm_tag *buf;
	int ret;
	size_t retlen;
	u32 computed_crc;
	bool rootfs_ok = false;

	*rootfs_offset = 0;
	*rootfs_size = 0;
	*rootfs_sequence = 0;

	buf = vmalloc(sizeof(struct bcm_tag));
	if (!buf)
		goto out;

	ret = mtd_read(master, rootfs_part, sizeof(*buf), &retlen, (void *)buf);
	if (ret < 0)
		goto out;

	if (retlen != sizeof(*buf))
		goto out;

	computed_crc = crc32_le(IMAGETAG_CRC_START, (u8 *)buf,
				offsetof(struct bcm_tag, header_crc));
	if (computed_crc == buf->header_crc) {
		char *board_id = &buf->board_id[0];
		char *tag_version = &buf->tag_version[0];

		/* Get rootfs offset and size from tag data */
		sscanf(buf->flash_image_start, "%u", rootfs_offset);
		sscanf(buf->root_length, "%u", rootfs_size);
		sscanf(buf->dual_image, "%u", rootfs_sequence);

		pr_info("%s: CFE boot tag found at %#x with version %s, board type %s and sequence number %u\n",
			name, rootfs_part, tag_version, board_id, *rootfs_sequence);

		/* Adjust for flash offset */
		*rootfs_offset -= BCM63XX_EXTENDED_SIZE;

		/* Remove bcm_tag data from length */
		*rootfs_size -= *rootfs_offset - rootfs_part;

		rootfs_ok = true;
	} else {
		pr_warn("%s: CFE boot tag at %#x CRC invalid (expected %08x, actual %08x)\n",
			name, rootfs_part, buf->header_crc, computed_crc);
		goto out;
	}

out:
	vfree(buf);
	return rootfs_ok;
}

static int bcm963268_parse_cfe_partitions(struct mtd_info *master,
					struct mtd_partition **pparts,
					struct mtd_part_parser_data *data)
{
	int nrparts, curpart;
	struct bcm963268_nvram *nvram = NULL;
	struct mtd_partition *parts;
	unsigned int rootfs1_off, rootfs1_size, rootfs1_seq;
	unsigned int rootfs2_off, rootfs2_size, rootfs2_seq;
	bool rootfs1, rootfs2;
	bool use_first;
	int ret;

	if (bcm963268_detect_cfe(master)) {
		ret = -EINVAL;
		goto out;
	}

	nvram = vmalloc(sizeof(*nvram));
	if (!nvram) {
		ret = -ENOMEM;
		goto out;
	}

	if (bcm963268_read_nvram(master, nvram)) {
		ret = -EINVAL;
		goto out;
	}

	/* We've just read the nvram from offset 0,
	 * so it must be located there.
	 */
	if (nvram->part_offset[PART_BOOT] != 0) {
		ret = -EINVAL;
		goto out;
	}

	/* Get the rootfs partition locations */
	rootfs1 = bcm963268_parse_rootfs_tag(master, "rootfs1",
		nvram->part_offset[PART_ROOTFS_1] * SZ_1K,
		&rootfs1_off, &rootfs1_size, &rootfs1_seq);
	rootfs2 = bcm963268_parse_rootfs_tag(master, "rootfs2",
		nvram->part_offset[PART_ROOTFS_2] * SZ_1K,
		&rootfs2_off, &rootfs2_size, &rootfs2_seq);
	if (!rootfs1 && !rootfs2) {
		ret = -EINVAL;
		goto out;
	}

	/* Determine primary rootfs partition */
	if (rootfs1 && rootfs2) {
		if (nvram->partition_number == '0') {
			use_first = true;
			pr_info("NVRAM partition number selected rootfs1\n");
		} else if (nvram->partition_number == '1') {
			use_first = false;
			pr_info("NVRAM partition number selected rootfs2\n");
		} else {
			bool use_latest = bcm963268_boot_latest(nvram);

			/* Compare sequence numbers */
			if (use_latest)
				use_first = rootfs1_seq > rootfs2_seq;
			else
				use_first = rootfs1_seq < rootfs2_seq;

			pr_info("CFE bootline selected %s image rootfs%u\n",
				use_latest ? "latest" : "previous",
				use_first ? 1 : 2);
		}
	} else {
		use_first = rootfs1;
	}

	/* Partitions:
	 * 1 nvram
	 * 2 rootfs
	 * 3 data
	 * 4 rootfs1_update
	 * 5 rootfs2_update
	 * 6 rootfs_other
	 */
	nrparts = 6;
	curpart = 0;

	parts = kcalloc(nrparts, sizeof(*parts), GFP_KERNEL);
	if (!parts) {
		ret = -ENOMEM;
		goto out;
	}

	parts[curpart].name = "nvram";
	parts[curpart].offset = nvram->part_offset[PART_BOOT] * SZ_1K;
	parts[curpart].size = nvram->part_size[PART_BOOT] * SZ_1K;
	curpart++;

	parts[curpart].name = "rootfs";
	parts[curpart].offset = use_first ? rootfs1_off : rootfs2_off;
	parts[curpart].size = use_first ? rootfs1_size : rootfs2_size;
	curpart++;

	parts[curpart].name = "data";
	parts[curpart].offset = nvram->part_offset[PART_DATA] * SZ_1K;
	parts[curpart].size = nvram->part_size[PART_DATA] * SZ_1K;
	curpart++;

	/* Full rootfs partitions for updates */
	parts[curpart].name = "rootfs1_update";
	parts[curpart].offset = nvram->part_offset[PART_ROOTFS_1] * SZ_1K;
	parts[curpart].size = nvram->part_size[PART_ROOTFS_1] * SZ_1K;
	curpart++;

	parts[curpart].name = "rootfs2_update";
	parts[curpart].offset = nvram->part_offset[PART_ROOTFS_2] * SZ_1K;
	parts[curpart].size = nvram->part_size[PART_ROOTFS_2] * SZ_1K;
	curpart++;

	/* Other rootfs if both are available */
	if (rootfs1 && rootfs2) {
		parts[curpart].name = "rootfs_other";
		parts[curpart].offset = use_first ? rootfs2_off : rootfs1_off;
		parts[curpart].size = use_first ? rootfs2_size : rootfs1_size;
		curpart++;
	}

	*pparts = parts;
	ret = 0;

out:
	vfree(nvram);

	if (ret < 0)
		return ret;

	return nrparts;
};

static struct mtd_part_parser bcm963268_cfe_parser = {
	.owner = THIS_MODULE,
	.parse_fn = bcm963268_parse_cfe_partitions,
	.name = "bcm963268part",
};

static int __init bcm963268_cfe_parser_init(void)
{
	register_mtd_parser(&bcm963268_cfe_parser);
	return 0;
}

static void __exit bcm963268_cfe_parser_exit(void)
{
	deregister_mtd_parser(&bcm963268_cfe_parser);
}

module_init(bcm963268_cfe_parser_init);
module_exit(bcm963268_cfe_parser_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Simon Arlott");
MODULE_DESCRIPTION("MTD partitioning for BCM963268 CFE bootloaders");
