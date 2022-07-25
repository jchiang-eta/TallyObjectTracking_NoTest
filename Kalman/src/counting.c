#include "counting.h"
#include "debug.h"
#include <stdlib.h>
#include <stdint.h>
#include "assertion.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) < (b) ? (b) : (a))

#ifdef debug
// helper function to display counting canvas
static int display_counting_line(counting_t* cnt, int W, int H) {

	for (int i = 0; i < cnt->N; i++)
		debug("[%d] a: %f, b: %f\n", i, cnt->a[i], cnt->b[i]);

	debug("\n");
	int sum = 0;
	debug("y = 0.0 (PRIMARY)\n");
	int wi = 1;
	int hi = 2;
	for (int y = 0; y < H; y += hi) {
		for (int x = 0; x <= W; x += wi) {
			int s = cnt_npts_is_secondary((float)x, (float)y, cnt);
			int p = cnt_npts_is_primary((float)x, (float)y, cnt);
			assert(!(s && p)); // cant be below and above at once
			if (!s && !p) {
				debug("+"); // on hysteresis
			} else if (s) {
				debug("s"); // above
				sum += 1;
			} else if (p) {
				debug("p"); // below
			} else {
				assert(0);
			}
			
		}
		debug("\n");
	}
	debug("y = 1.0 (SECONDARY)\n");

	return sum;
}

#endif

void compute_count(vec2f* start, vec2f* end, vec2ui8* cnt, counting_t* cnt_obj){

	if (	cnt_npts_is_secondary(start->x, start->y, cnt_obj)
		&&	cnt_npts_is_primary(end->x, end->y, cnt_obj) ) 
	{
		debug("Count PRIMARY=%d+1, SECONDARY=%d\r\n", (int)cnt->v[0], (int)cnt->v[1]);
		cnt->v[0] += 1;
	}

	if (	cnt_npts_is_primary(start->x, start->y, cnt_obj)
		&&  cnt_npts_is_secondary(end->x, end->y, cnt_obj) )
	{
		debug("Count PRIMARY=%d, SECONDARY=%d+1\r\n", (int)cnt->v[0], (int)cnt->v[1]);
		cnt->v[1] += 1;
	}

	return;
}

int cnt_npts_init(float* y, unsigned int N, uint16_t input_w, uint16_t input_h, float hysteresis, float offset, counting_t* cnt_obj) {

	if (N < 2 || input_w < 0 || input_h < 0 || cnt_obj == 0 || N > (CNT_MAX_SEGMENTS+1) || hysteresis < 0)
		return -1;

	N--; // number of piecewise segments will be 1 smaller than the number of points
	cnt_obj->N = N; 
	cnt_obj->half_hysteresis = hysteresis*input_h*0.5f;

	if (cnt_obj->a == NULL || cnt_obj->b == NULL)
		return -1;

	cnt_obj->to_idx_multiplier = (float)N / (float)input_w;

	// find a, b coefficients for each piece-wise segment
	float x0, x1, y0, y1;
	debug("Line points in model input coordinates (x0, x1, y0, y1):\n");
	for (int i = 0; i < cnt_obj->N; i++) {
		x0 = (float)(i * input_w / cnt_obj->N);
		x1 = (float)((i+1) * input_w / cnt_obj->N);
		y0 =(float)( (y[i] + offset) * input_h);
		y1 =(float)( (y[i + 1] + offset) * input_h);
		debug("\t[%f, %f, %f, %f]\n", x0, x1, y0, y1);
		cnt_obj->a[i] = (y1 - y0) / (x1 - x0);
		cnt_obj->b[i] = y0 - cnt_obj->a[i] * x0;
	}

#ifdef debug
	display_counting_line(cnt_obj, input_w, input_h);
#endif

	return 0;

}

int cnt_npts_is_secondary(float x, float y, counting_t* cnt_obj) {

	assert(cnt_obj != NULL);

	y -= cnt_obj->half_hysteresis;
	int seg_idx = (int)(x*cnt_obj->to_idx_multiplier);

	// clip seg_idx to 0-N range
	seg_idx = MIN(seg_idx, cnt_obj->N - 1);
	seg_idx = MAX(seg_idx, 0);

	return y > cnt_obj->a[seg_idx] * x + cnt_obj->b[seg_idx];

}

int cnt_npts_is_primary(float x, float y, counting_t* cnt_obj) {

	y += cnt_obj->half_hysteresis;
	int seg_idx = (int)(x * cnt_obj->to_idx_multiplier);

	// clip seg_idx to 0-N range
	seg_idx = MIN(seg_idx, cnt_obj->N - 1);
	seg_idx = MAX(seg_idx, 0);

	return y <= cnt_obj->a[seg_idx] * x + cnt_obj->b[seg_idx];
}