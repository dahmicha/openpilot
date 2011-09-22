/**
 * @file	zalloc.h
 * @brief	Public interfaces to the self contained low-overhead memory pool/allocation subsystem.
 *
 * This is a lightly modified version of the Dillon zalloc as documented in zalloc.c
 *
 * Modifications (c) 2011 Michael Smith.
 */

#ifndef __ZALLOC_H
#define __ZALLOC_H

#include <stdint.h>
#include <stdlib.h>

typedef uintptr_t	iaddr_t;	/**< unsigned int same size as pointer	*/
typedef intptr_t	saddr_t;	/**< signed int same size as pointer	*/

/* standard stdlib malloc functions */
extern void *malloc(size_t bytes);
extern void free(void *ptr);
extern void *calloc(size_t n1, size_t n2);
extern void *realloc(void *ptr, size_t size);
extern void *reallocf(void *ptr, size_t size);

/* malloc heap checker */
extern void	malloc_heap_check(void);

/**
 * Memory pool, from which memory can be allocated and to which it can be freed.
 */
struct MemPool;

/**
 * Allocate unzeroed memory from the pool.  Calls the pool reclaim function
 * if the pool is exhausted.
 * This is the workhorse allocator for zalloc.
 *
 * @param mpool		Pool from which to allocate.
 * @param bytes		Number of bytes to allocate.
 */
extern void	*znalloc(struct MemPool *mpool, iaddr_t bytes);

/**
 * Allocate zeroed memory from the pool by way of znalloc.
 *
 * @param mpool		Pool from which to allocate.
 * @param bytes		Number of bytes to allocate.
 */
extern void *zalloc(struct MemPool *mpool, iaddr_t bytes);

/**
 * Allocate zeroed, aligned memory from the pool by way of znalloc.
 *
 * @param mpool		Pool from which to allocate.
 * @param bytes		Number of bytes to allocate.
 * @param align		Minumum alignment for the returned pointer.
 */
extern void *zallocAlign(struct MemPool *mpool, iaddr_t bytes, iaddr_t align);

/**
 * Allocate memory from within a specific subrange of a memory pool.
 * The entire allocation must lie within the addr1 - addr2 range.
 *
 * @param mp		Pool from which to allocate.
 * @param addr1		Lowest permitted value of the return address.
 * @param addr2		Highest permitted value of the return address + bytes
 * @param bytes		Number of bytes to allocate.
 */
extern void *zxalloc(struct MemPool *mp, void *addr1, void *addr2, iaddr_t bytes);

/**
 * Allocate zeroed memory from within a specific subrange of a memory pool
 * by way of zxalloc.
 *
 * @param mp		Pool from which to allocate.
 * @param addr1		Lowest permitted value of the return address.
 * @param addr2		Highest permitted value of the return address + bytes
 * @param bytes		Number of bytes to allocate.
 */
extern void *znxalloc(struct MemPool *mp, void *addr1, void *addr2, iaddr_t bytes);

/**
 * Allocate memory for a string and copy it to the allocation.
 * This is fundamentally similar to strdup().
 *
 * @param mpool		Pool from which to allocate.
 * @param s			String to be copied.
 * @param slen		Length of the string to copy, or -1 if the length should be
 * 					determined by reading the string.
 * @note If -1 is pased for slen, the caller will have to use zfreeStre rather
 * than zfree to free the string.
 */
extern char *zallocStr(struct MemPool *mpool, const char *s, int slen);

/**
 * Free memory back to the pool.
 *
 * @param mpool		The pool into which the memory should be freed.
 * @param ptr		Pointer to the memory being freed.
 * @param bytes		Size of the memory in bytes.
 */
extern void zfree(struct MemPool *mpool, void *ptr, iaddr_t bytes);

/**
 * Free memory allocated by zallocStr
 *
 * @param mpool		The pool into which the memory should be freed.
 * @param s			Pointer to the memory that was allocated with zallocStr.
 */
extern void zfreeStr(struct MemPool *mpool, char *s);

/**
 * Initialise a memory pool.
 *
 * @note After initialisation, a pool is empty and zfree should be used to
 * add memory to it.
 *
 * @param mp		The pool to initialise.
 * @param id		A string used to identify the pool.
 * @param fpanic	A function to call when an unrecoverable error is detected.
 * @param freclaim	A function to call when allocation fails; this function can
 * 					free other memory back to the pool, for example.  The bytes
 * 					argument indicates the number of bytes the allocator needs.
 * 					The function should return zero if memory was reclaimed.
 * @param pBase		Address less than this are considered outside the pool.
 * @param pSize		The initial size of the pool.
 */
extern void zinitPool(struct MemPool *mp, const char *id, void (*fpanic)(const char *ctl, ...), int (*freclaim)(struct MemPool *memPool, iaddr_t bytes), void *pBase, iaddr_t pSize);

/**
 * Frees all memory associated with a pool, destroying any previous allocations.
 *
 * @param mp		The pool to clear.
 */
extern void zclearPool(struct MemPool *mp);

/**
 * Default do-nothing panic function.
 *
 * @param ctl		Ignored.
 */
extern void	znop(const char *ctl, ...);

/**
 * Default do-nothing reclaim function.
 *
 * @param memPool	Ignored.
 * @param bytes		Ignored.
 */
extern int	znot(struct MemPool *memPool, iaddr_t bytes);

/**
 * Stats gather/report function.
 * @todo make this useful
 *
 * @param mp		The pool for which stats are to be gathered/reported.
 */
extern void zallocstats(struct MemPool *mp);

/**
 * External locking primitive used to protect the allocator.
 *
 * If not defined, the allocator is not threadsafe.
 */
extern void zlock(void) __attribute__((weak));

/**
 * External locking primitive used to protect the allocator.
 *
 * If not defined, the allocator is not threadsafe.
 */
extern void zunlock(void) __attribute((weak));

#endif // __ZALLOC_H
