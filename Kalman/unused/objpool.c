#include <string.h>
#include <stdlib.h>

#include "objpool.h"

#ifndef max
#define max(a,b) ((a)<(b)?(b):(a))
#endif

// obj_size - size of individual object
void objpool_init(objpool* p, const void* mem_ptr, const uint32_t obj_size, const uint32_t n_blocks)
{
	uint32_t i;

	p->obj_size = obj_size;
	p->n_blocks = n_blocks;
	
	poolFreeAll(p);

	p->blocksUsed = POOL_BLOCKS_INITIAL;
	p->blocks = malloc(sizeof(uint8_t*)* p->blocksUsed);

	for(i = 0; i < p->blocksUsed; ++i)
		p->blocks[i] = NULL;
}

void *objpool_new_obj(objpool *p)
{
	if(p->freed != NULL) {
		void *recycle = p->freed;
		p->freed = p->freed->nextFree;
		return recycle;
	}

	//if(++p->used == p->blockSize) {
	//	p->used = 0;
	//	if(++p->block == (int32_t)p->blocksUsed) {
	//		uint32_t i;

	//		p->blocksUsed <<= 1;
	//		p->blocks = realloc(p->blocks, sizeof(uint8_t*)* p->blocksUsed);

	//		for(i = p->blocksUsed >> 1; i < p->blocksUsed; ++i)
	//			p->blocks[i] = NULL;
	//	}

	//	if(p->blocks[p->block] == NULL)
	//		p->blocks[p->block] = malloc(p->elementSize * p->blockSize);
	//}
	//
	//return p->blocks[p->block] + p->used * p->elementSize;
}

void objpoll_free(pool *p, void *ptr)
{
	poolFreed *pFreed = p->freed;

	p->freed = ptr;
	p->freed->nextFree = pFreed;
}


//void poolFreeAll(pool *p)
//{
//	p->used = p->blockSize - 1;
//	p->block = -1;
//	p->freed = NULL;
//}
