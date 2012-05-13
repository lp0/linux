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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* Memory overview:
 * [ARMcore] <--virtual addr--> [ARMmmu] <--physical addr--> [GERTmap] <--bus add--> [VCperiph]
 */

/* Physical DRAM offset.
 */
#define PLAT_PHYS_OFFSET	UL(0x00000000)
#define VC_ARMMEM_OFFSET	UL(0x00000000)	/* offset in VC of ARM memory */

/* VC memory aliases:
 */
#define VC_BUS_DIRECT_UNCACHED	UL(0xC0000000)	/* direct uncached */
#define VC_BUS_L2_CACHE_ONLY	UL(0x80000000)	/* L2 cached (only) */
#define VC_BUS_L2_CACHE		UL(0x40000000)	/* L2 cache coherent (non allocating) */
#define VC_BUS_L1_AND_L2_CACHED	UL(0x00000000)	/* L1 and L2 cached */

#ifdef CONFIG_BCM2708_NOL2CACHE
 /* don't use L1 or L2 caches */
 #define VC_BUS_OFFSET		VC_BUS_DIRECT_UNCACHED
#else
 /* use L2 cache */
 #define VC_BUS_OFFSET		VC_BUS_L2_CACHE
#endif

/* We're using the memory at 64M in the VideoCore for Linux - this adjustment
 * will provide the offset into this area as well as setting the bits that
 * stop the L1 and L2 cache from being used
 *
 * WARNING: this only works because the ARM is given memory at a fixed location
 *          (VC_ARMMEM_OFFSET)
 */
#define BUS_OFFSET		(VC_ARMMEM_OFFSET + VC_BUS_OFFSET)
#define __virt_to_bus(x)	((x) + (BUS_OFFSET - PAGE_OFFSET))
#define __bus_to_virt(x)	((x) - (BUS_OFFSET - PAGE_OFFSET))
#define __pfn_to_bus(x)		(__pfn_to_phys(x) + (BUS_OFFSET - PLAT_PHYS_OFFSET))
#define __bus_to_pfn(x)		__phys_to_pfn((x) - (BUS_OFFSET - PLAT_PHYS_OFFSET))

#endif
