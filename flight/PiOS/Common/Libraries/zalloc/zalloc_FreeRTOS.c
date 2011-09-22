/**
 * @file	zalloc_FreeRTOS.c
 * @brief	Implementation of pvPortMalloc and friends that uses the Dillon zalloc/malloc allocator.
 *
 * (c) 2011 Michael Smith, All Rights Reserved
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "malloc_private.h"

/*
 * Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file.
 * */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
# include "FreeRTOS.h"
# include "task.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/*
 * Symbols exported by the linker script telling us where the heap is.
 */
extern char	_sheap;
extern char	_eheap;

/*
 * Optional callback for allocation failures.
 */
extern void vApplicationMallocFailedHook(void) __attribute__((weak));

/*
 * Mutual exclusion support for the allocator.
 */
void
zlock(void)
{
	vPortEnterCritical();
}

void
zunlock(void)
{
	vPortExitCritical();
}

void *
pvPortMalloc(size_t s)
{
	void *p;

	p = malloc(s);

	if (p == NULL && &vApplicationMallocFailedHook != NULL)
		vApplicationMallocFailedHook();
	
	return p;
}

void
vPortFree(void *p)
{
	free(p);
}

size_t
xPortGetFreeHeapSize(void)
{
	return MallocPool.mp_Size - MallocPool.mp_Used;
}

void
vPortInitialiseBlocks(void)
{
	uint32_t	heap_size;

	heap_size = &_eheap - &_sheap;

	/* assign all of the heap space to the zalloc pool */
	/* XXX might be nice to have better panic hooks here */
	zinitPool(&MallocPool, "Malloc", znop, znot, (void *)&_sheap, heap_size);
	zclearPool(&MallocPool);
}

void
xPortIncreaseHeapSize(size_t bytes)
{
	/* can't implement this as-is due to zalloc supporting non-contiguous extensions */
	/* might want to implement an extended version to deal with that at some point */
}

