#ifndef _LINUX_MEMBLOCK_H
#define _LINUX_MEMBLOCK_H
#ifdef __KERNEL__

/*
 * Logical memory blocks.
 *
 * Copyright (C) 2001 Peter Bergner, IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/mm.h>

#define MAX_MEMBLOCK_REGIONS 128

struct memblock_region {
	u64 base;
	u64 size;
};

struct memblock_type {
	unsigned long cnt;
	u64 size;
	struct memblock_region regions[MAX_MEMBLOCK_REGIONS+1];
};

struct memblock {
	unsigned long debug;
	u64 current_limit;
	struct memblock_type memory;
	struct memblock_type reserved;
};

extern struct memblock memblock;

extern void __init memblock_init(void);
extern void __init memblock_analyze(void);
extern long memblock_add(u64 base, u64 size);
extern long memblock_remove(u64 base, u64 size);
extern long __init memblock_free(u64 base, u64 size);
extern long __init memblock_reserve(u64 base, u64 size);

extern u64 __init memblock_alloc_nid(u64 size, u64 align, int nid);
extern u64 __init memblock_alloc(u64 size, u64 align);

/* Flags for memblock_alloc_base() amd __memblock_alloc_base() */
#define MEMBLOCK_ALLOC_ANYWHERE	(~(u64)0)
#define MEMBLOCK_ALLOC_ACCESSIBLE	0

extern u64 __init memblock_alloc_base(u64 size,
		u64, u64 max_addr);
extern u64 __init __memblock_alloc_base(u64 size,
		u64 align, u64 max_addr);
extern u64 __init memblock_phys_mem_size(void);
extern u64 memblock_end_of_DRAM(void);
extern void __init memblock_enforce_memory_limit(u64 memory_limit);
extern int memblock_is_memory(u64 addr);
extern int memblock_is_region_memory(u64 base, u64 size);
extern int __init memblock_is_reserved(u64 addr);
extern int memblock_is_region_reserved(u64 base, u64 size);

extern void memblock_dump_all(void);

/* Provided by the architecture */
extern u64 memblock_nid_range(u64 start, u64 end, int *nid);

/**
 * memblock_set_current_limit - Set the current allocation limit to allow
 *                         limiting allocations to what is currently
 *                         accessible during boot
 * @limit: New limit value (physical address)
 */
extern void memblock_set_current_limit(u64 limit);


/*
 * pfn conversion functions
 *
 * While the memory MEMBLOCKs should always be page aligned, the reserved
 * MEMBLOCKs may not be. This accessor attempt to provide a very clear
 * idea of what they return for such non aligned MEMBLOCKs.
 */

/**
 * memblock_region_memory_base_pfn - Return the lowest pfn intersecting with the memory region
 * @reg: memblock_region structure
 */
static inline unsigned long memblock_region_memory_base_pfn(const struct memblock_region *reg)
{
	return PFN_UP(reg->base);
}

/**
 * memblock_region_memory_end_pfn - Return the end_pfn this region
 * @reg: memblock_region structure
 */
static inline unsigned long memblock_region_memory_end_pfn(const struct memblock_region *reg)
{
	return PFN_DOWN(reg->base + reg->size);
}

/**
 * memblock_region_reserved_base_pfn - Return the lowest pfn intersecting with the reserved region
 * @reg: memblock_region structure
 */
static inline unsigned long memblock_region_reserved_base_pfn(const struct memblock_region *reg)
{
	return PFN_DOWN(reg->base);
}

/**
 * memblock_region_reserved_end_pfn - Return the end_pfn this region
 * @reg: memblock_region structure
 */
static inline unsigned long memblock_region_reserved_end_pfn(const struct memblock_region *reg)
{
	return PFN_UP(reg->base + reg->size);
}

#define for_each_memblock(memblock_type, region)					\
	for (region = memblock.memblock_type.regions;				\
	     region < (memblock.memblock_type.regions + memblock.memblock_type.cnt);	\
	     region++)


#endif /* __KERNEL__ */

#endif /* _LINUX_MEMBLOCK_H */
