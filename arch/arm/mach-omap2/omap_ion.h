/*
 * arch/arm/mach-omap2/omap_ion.h
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _OMAP_ION_H
#define _OMAP_ION_H

#define OMAP4_ION_HEAP_SECURE_INPUT_SIZE	(SZ_1M * 90)
#define OMAP4_ION_HEAP_TILER_SIZE		(SZ_128M - SZ_32M)
#define OMAP4_ION_HEAP_NONSECURE_TILER_SIZE	SZ_32M

#define OMAP3_ION_HEAP_CARVEOUT_INPUT_SIZE       (SZ_1M * 30)

#define PHYS_ADDR_SMC_SIZE	(SZ_1M * 3)
#define PHYS_ADDR_SMC_MEM	(0x80000000 + SZ_512M - PHYS_ADDR_SMC_SIZE)
#define PHYS_ADDR_DUCATI_SIZE	(SZ_1M * 105)
#define PHYS_ADDR_DUCATI_MEM	(PHYS_ADDR_SMC_MEM - PHYS_ADDR_DUCATI_SIZE - \
				OMAP4_ION_HEAP_SECURE_INPUT_SIZE)

/* Restricting the Memory size to 512 MB*/
#define OMAP3_PHYS_ADDR_SMC_MEM	(0x80000000 + SZ_512M - PHYS_ADDR_SMC_SIZE)

#ifdef CONFIG_ION_OMAP
void omap_ion_init(void);
void omap_register_ion(void);
#else
static inline void omap_ion_init(void) { return; }
static inline void omap_register_ion(void) { return; }
#endif

#endif
