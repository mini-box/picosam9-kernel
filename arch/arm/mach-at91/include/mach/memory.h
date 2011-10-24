/*
 * arch/arm/mach-at91/include/mach/memory.h
 *
 *  Copyright (C) 2004 SAN People
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

#include <mach/hardware.h>

#define PHYS_OFFSET	0x70000000     /* DDRSDRC0 */

#if defined(CONFIG_ARCH_AT91SAM9G45) || defined(CONFIG_ARCH_AT91SAM9M10)
/*
 * Non-linear mapping like so:
 * phys       => virt
 * 0x70000000 => 0xc0000000
 * 0x20000000 => 0xc8000000
 */

#define __phys_to_virt(p)   \
	(((p) & 0x07ffffff) + (((p) & 0x40000000) ? 0xc0000000 : 0xc8000000))

#define __virt_to_phys(v)   \
	(((v) & 0x07ffffff) + (((v) & 0x08000000) ? 0x20000000 : 0x70000000 ))

#define NODE_MEM_SIZE_BITS	27
#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	27 /*128 Mb */
#define HIGH_MEMORY_VIRT	0xd0000000
#endif

#endif
