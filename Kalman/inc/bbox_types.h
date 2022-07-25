#ifndef _BBOX_TYPES_H
#define _BBOX_TYPES_H

#include <stdint.h>

/* Float (dequantized) bounding boxes types */

typedef struct {
	float xmin;
	float ymin;
	float xmax;
	float ymax;
} BoxCornerEncodingFloat;

typedef struct {
	float y;
	float x;
	float h;
	float w;
} CenterSizeEncodingFloat;

/* Byte (quantized) bounding boxes types*/

typedef struct {
	uint8_t xmin;
	uint8_t ymin;
	uint8_t xmax;
	uint8_t ymax;
} BoxCornerEncodingByte;

typedef struct {
	uint8_t y;
	uint8_t x;
	uint8_t h;
	uint8_t w;
} CenterSizeEncodingByte;

#endif // !_BBOX_TYPES_H


