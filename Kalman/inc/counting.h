#ifndef _COUNTING_H
#define _COUNTING_H

#include <stdint.h>
#include <vec_types.h>

#define CNT_MAX_SEGMENTS 8

typedef struct {
	float a[CNT_MAX_SEGMENTS];
	float b[CNT_MAX_SEGMENTS];
	uint16_t N;
	float to_idx_multiplier;
	float half_hysteresis;

} counting_t;

int cnt_npts_init(float* y, unsigned int N, uint16_t input_w, uint16_t input_h, float hysteresis, float offset, counting_t* cnt_obj);
int cnt_npts_is_secondary(float x, float y, counting_t* cnt_obj);
int cnt_npts_is_primary(float x, float y, counting_t* cnt_obj);

void compute_count(vec2f* start, vec2f* end, vec2ui8* cnt, counting_t* cnt_obj);

#endif // _COUNTING_H