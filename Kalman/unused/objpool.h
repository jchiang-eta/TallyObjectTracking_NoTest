#pragma once

#include <stdint.h>

#define POOL_BLOCKS_INITIAL 1

typedef struct free_linked_list {
	struct free_linked_list* next_free;
} free_linked_list;

typedef struct {
	uint32_t elementSize;
	uint32_t blockSize;
	uint32_t used;
	int32_t block;
	free_linked_list* freed;
	uint32_t blocksUsed;
	uint8_t **blocks;
} objpool;

void poolInitialize(pool *p, const uint32_t elementSize, const uint32_t blockSize);
void poolFreePool(pool *p);

#ifndef DISABLE_MEMORY_POOLING
void *poolMalloc(pool *p);
void poolFree(pool *p, void *ptr);
#else
#include <stdlib.h>
#define poolMalloc(p) malloc((p)->blockSize)
#define poolFree(p, d) free(d)
#endif
//void poolFreeAll(pool *p);

#include <string.h>
#include <stdlib.h>

#include "objpool.h"

#ifndef max
#define max(a,b) ((a)<(b)?(b):(a))
#endif

// obj_size - size of individual object
void objpool_init(objpool* p, const void* mem_ptr, const uint32_t obj_size, const uint32_t n_blocks);

// allocates object from the pool and releases the pointer
// returns NULL if no more free objects are available
void* objpool_new_obj(objpool* p);

// returns object to the pool
void objpoll_free(objpool* p, void* ptr);


//void poolFreeAll(pool *p)
//{
//	p->used = p->blockSize - 1;
//	p->block = -1;
//	p->freed = NULL;
//}
