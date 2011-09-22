/**
 * @file	malloc.c
 * @brief	malloc equivalents, runs on top of zalloc
 *
 * This is a lightly modified version of the Dillon malloc/zalloc as documented
 * in zalloc.c
 *
 * Modifications (c) 2011 Michael Smith
 */

#include "malloc_private.h"
#include <string.h>

MemPool		MallocPool;

static size_t
_malloc_size(void *ptr)
{
	Guard *res = (Guard *)ptr - 1;

	return res->ga_Bytes;
}

static void
_malloc_check(void *ptr, int clear)
{
	Guard *res = (Guard *)ptr - 1;

	if (res->ga_Magic != GAMAGIC)
		MallocPool.mp_Panic("malloc: guard1 fail @ %p", ptr);
	if (clear)
		res->ga_Magic = NOMAGIC;
#ifdef USEENDGUARD
	unsigned char *eg = ((unsigned char *)res + res->ga_Bytes - 1);
	if (*eg != EGAMAGIC) {
		MallocPool.mp_Panic("malloc: guard2 fail @ %p", ptr);
	}
	if (clear)
		*eg = NOMAGIC;
#endif
}

void *
malloc(size_t bytes)
{
	Guard *res;

	bytes += sizeof(Guard);
#ifdef USEENDGUARD
	bytes += 1;
#endif

	res = znalloc(&MallocPool, bytes);
	if (res == NULL)
		return(NULL);

	res->ga_Magic = GAMAGIC;
	res->ga_Bytes = bytes;

#ifdef USEENDGUARD
	*((unsigned char *)res + bytes - 1) = EGAMAGIC;
#endif
	return res + 1;
}

void
free(void *ptr)
{
	if (ptr != NULL) {
		Guard *res = (Guard *)ptr - 1;

		/* check and clear validity */
		_malloc_check(ptr, 1);

		zfree(&MallocPool, res, _malloc_size(ptr));
	}
}

void *
calloc(size_t n1, size_t n2)
{
	iaddr_t bytes = (iaddr_t)n1 * (iaddr_t)n2;
	void *res;

	if ((res = malloc(bytes)) != NULL) {
		memset(res, 0, bytes);
	}
	return(res);
}

void *
realloc(void *ptr, size_t size)
{
	void *res;
	size_t old;

	if ((res = malloc(size)) != NULL) {
		if (ptr) {
			old = _malloc_size(ptr);
			if (old < size)
				memcpy(res, ptr, old);
			else
				memcpy(res, ptr, size);
			free(ptr);
		}
	}
	return(res);
}

void *
reallocf(void *ptr, size_t size)
{
	void *res;

	if ((res = realloc(ptr, size)) == NULL)
		free(ptr);
	return(res);
}

#include "zalloc_private.h"

void
malloc_heap_check(void)
{
	uint8_t	*alloc;
	MemNode	*hole;
	Guard	*g;
	iaddr_t	round;

	round = sizeof(struct MemNode) - 1;

	if ((MallocPool.mp_Base + MallocPool.mp_Size) != MallocPool.mp_End)
		MallocPool.mp_Panic("malloc: pool base/end/size corrupt");
	if (((void *)MallocPool.mp_First < MallocPool.mp_Base) || ((void *)MallocPool.mp_First > MallocPool.mp_End))
		MallocPool.mp_Panic("malloc: pool freelist corrupt");

	alloc = MallocPool.mp_Base;
	hole = MallocPool.mp_First;

	while ((void *)alloc < MallocPool.mp_End) {
		if (alloc == (uint8_t *)hole) {
			alloc += hole->mr_Bytes;
			if (((void *)alloc < MallocPool.mp_Base) || ((void *)alloc > MallocPool.mp_End) ||
				((void *)hole->mr_Next > MallocPool.mp_End))
				MallocPool.mp_Panic("malloc: pool free node %p corrupt", hole);
			hole = hole->mr_Next;
		} else {
			g = (Guard *)alloc;
			_malloc_check(g + 1, 0);

			alloc += (g->ga_Bytes + round) & ~round;
			if (((void *)alloc < MallocPool.mp_Base) || ((void *)alloc > MallocPool.mp_End))
				MallocPool.mp_Panic("malloc: allocation %p corrupt", g);
		}
	}
}
