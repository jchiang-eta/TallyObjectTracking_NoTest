#ifndef _VEC_TYPES_H
#define _VEC_TYPES_H

#include <stdint.h>

typedef struct {
	union {
		struct {
			float x;
			float y;
		};
		float v[2];
	};
} vec2f;

typedef struct {
	union {
		struct {
			int x;
			int y;
		};
		int v[2];
	};
} vec2i;

typedef struct {
	union {
		struct {
			uint8_t x;
			uint8_t y;
		};
		uint8_t v[2];
	};
} vec2ui8;

#endif
