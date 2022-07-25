#ifndef _HUNGARIAN_H_
#define _HUNGARIAN_H_

#include "vec_types.h"
#include <stdint.h>

typedef struct {
	int8_t* const col_mate;
	int8_t* const row_mate;
	int8_t* const parent_row;
	int8_t* const unchosen_row;
	int* const row_dec;
	int* const col_inc;
	int* const slack;
	int8_t* const slack_row;
	int* const hun_costmat;
	uint8_t _h_rows;
	uint8_t _h_cols;
	uint8_t init;
} hungarian_t;

uint8_t hungarian(float* cost_matrix, int n_rows, int n_cols, vec2ui8* matchings, int* costs, hungarian_t* _hun);

#endif // _HUNGARIAN_H_