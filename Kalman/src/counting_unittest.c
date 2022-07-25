#include "counting.h"
#include "counting_unittest.h"
#include "assertion.h"
#include "debug.h"

#define EPSILON 0.0001f

// helper function to compare two floats
static int appx_equalf(float ref, float val) {
	float diff = ref - val;
	if (diff < 0)
		diff *= -1.0f;
	return diff < EPSILON;
}

// helper function to display counting canvas
static int display_counting_line(counting_t* cnt, int W, int H) {

	for (int i = 0; i < cnt->N; i++)
		debug("[%d] a: %f, b: %f\n", i, cnt->a[i], cnt->b[i]);

	debug("\n");
	int sum = 0;
	debug("y = 0.0 (PRIMARY)\n");
	int wi = 2;
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

static void cnt_2pt_line_unittest() {

	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { 0.4f, 0.6f };
	int res = cnt_npts_init(y, 2, 96, 96, 0.0f, 0.0f, &cnt);
	assert(res == 0);

	for (int i = 0; i < cnt.N; i++)
		debug("[%d] a: %f, b: %f\n", i, cnt.a[i], cnt.b[i]);

	assert(cnt_npts_is_secondary(30.0f, 48.0f, &cnt) == 1);
	assert(cnt_npts_is_secondary(60.0f, 38.0f, &cnt) == 0);

	debug("PASSED\n");
}

static void cnt_2pt_line_inverted_unittest() {
	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { 1.0f, 0.0f };
	int res = cnt_npts_init(y, 2, 192, 96, 0.0f, 0.0f, &cnt);
	assert(res == 0);

	for (int i = 0; i < cnt.N; i++)
		debug("[%d] a: %f, b: %f\n", i, cnt.a[i], cnt.b[i]);

	assert(appx_equalf(-0.5f, cnt.a[0]));
	assert(appx_equalf(96.0f, cnt.b[0]));

	assert(cnt_npts_is_secondary(-5.0f, 96.0f, &cnt) == 0);
	assert(cnt_npts_is_secondary(-5.0f, 105.0f, &cnt) == 1);

	debug("PASSED\n");
}

static void cnt_5pt_line_unittest() {

	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { 0.4f, 0.6f, 0.6f, 0.3f, 0.2f };
	int res = cnt_npts_init(y, 5, 96, 96, 0.0, 0.0f, &cnt);
	assert(res == 0);

	for (int i = 0; i < cnt.N; i++)
		debug("[%d] a: %f, b: %f\n", i, cnt.a[i], cnt.b[i]);

	assert(appx_equalf(0.0f, cnt.a[1]));
	assert(appx_equalf(57.6f, cnt.b[1]));

	assert(cnt_npts_is_secondary(32.0f, 57.0f, &cnt) == 0);
	assert(cnt_npts_is_secondary(32.0f, 58.0f, &cnt) == 1);

	debug("PASSED\n");

}

static void cnt_5pt_circle_draw_unittest() {

	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { 0.65f, 0.5f, 0.4f, 0.55f, 0.9f };
	int res = cnt_npts_init(y, 5, 100, 50, 0.0f, 0.0f, &cnt);
	assert(res == 0);

	int sum = display_counting_line(&cnt, 100, 50);

	assert(sum < (50 / 2 * 100 / 2 / 2));

	debug("PASSED\n");

}

static void cnt_6pt_concave_with_negative_unittest() {

	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { 1.1f, 0.7f, 0.6f, 0.6f, 0.7f, 1.1f };
	int res = cnt_npts_init(y, 6, 100, 50, 0.0f, 0.0f, &cnt);
	assert(res == 0);

	int sum = display_counting_line(&cnt, 100, 50);

	assert(!cnt_npts_is_secondary(5.0f, 45.0f, &cnt));
	assert(cnt_npts_is_secondary(15.0f, 45.0f, &cnt));
	assert(sum < (50/2 * 100/2 / 2));

	debug("PASSED\n");

}

static void cnt_2pt_line_unittest_with_negative() {

	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { -0.1f, 0.6f };
	int res = cnt_npts_init(y, 2, 96, 96, 0.0f, 0.0f, &cnt);
	assert(res == 0);

	for (int i = 0; i < cnt.N; i++)
		debug("[%d] a: %f, b: %f\n", i, cnt.a[i], cnt.b[i]);

	assert(cnt_npts_is_secondary(30.0f, 91.0f, &cnt) == 1);
	assert(cnt_npts_is_secondary(60.0f, 30.0f, &cnt) == 0);
	assert(cnt_npts_is_secondary(-10.0f, 30.0f, &cnt) == 1);

	debug("PASSED\n");
}

static void cnt_6pt_zigzag_outofrange_unittest() {

	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { 0.2f, 0.5f, 0.3f, 0.8f, -0.3f, 1.2f };
	int res = cnt_npts_init(y, 6, 100, 50, 0.0f, 0.0f, &cnt);
	assert(res == 0);

	int sum = display_counting_line(&cnt, 100, 50);

	assert(!cnt_npts_is_secondary(5.0f, 13.74f, &cnt));
	assert(cnt_npts_is_secondary(25.0f, 22.6f, &cnt));
	assert(cnt_npts_is_secondary(-1.0f, 9.26f, &cnt));
	assert(!cnt_npts_is_secondary(110.0f, 97.4f, &cnt));

	debug("PASSED\n");

}

static void cnt_5pt_circle_draw_hist_unittest() {

	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { 1.0f, 0.5f, 0.4f, 0.5f, 1.0f };
	int res = cnt_npts_init(y, 5, 100, 50, 0.15f, 0.0f, &cnt);
	assert(res == 0);

	int sum = display_counting_line(&cnt, 100, 50);

	{
		vec2f s = { .x = 50.0f, .y = 10.0f }; // primary
		vec2f e = { .x = 50.0f, .y = 40.0f }; // secondary
		vec2ui8 out = { 0, 0 };
		compute_count(&s, &e, &out, &cnt);
		assert(out.v[0] == 0);
		assert(out.v[1] == 1);
	}

	{
		vec2f s = { .x = 50.0f, .y = 10.0f }; // primary
		vec2f e = { .x = 50.0f, .y = 22.0f }; // hysteresis
		vec2ui8 out = { 0, 0 };
		compute_count(&s, &e, &out, &cnt);
		assert(out.v[0] == 0);
		assert(out.v[1] == 0);
	}

	{
		vec2f s = { .x = 50.0f, .y = 23.0f }; // hysteresis
		vec2f e = { .x = 50.0f, .y = 40.0f }; // secondary
		vec2ui8 out = { 0, 0 };
		compute_count(&s, &e, &out, &cnt);
		assert(out.v[0] == 0);
		assert(out.v[1] == 0);
	}

	debug("PASSED\n");

}

// test with small hysteresis and an offset
static void cnt_6pt_hist_offset_unittest() {

	debug("testing: %s\r\n", __FUNCTION__);

	counting_t cnt;
	float y[] = { 1.0f, 0.5f, 0.4f, 0.4f, 1.0f, 1.0f };
	int res = cnt_npts_init(y, 6, 100, 50, 0.05f, -0.2f, &cnt);
	assert(res == 0);

	int sum = display_counting_line(&cnt, 100, 50);

	vec2f s = { .x = 99.0f, .y = 45.0f }; // secondary
	vec2f e = { .x = 99.0f, .y = 35.0f }; // primary
	vec2ui8 out = { 0, 0 };
	compute_count(&s, &e, &out, &cnt);
	assert(out.v[0] == 1);
	assert(out.v[1] == 0);

	debug("PASSED\n");

}

void counting_unittest() {

	debug("\n----- counting_unittest -----\n");

	cnt_2pt_line_unittest();
	cnt_5pt_line_unittest();
	cnt_2pt_line_inverted_unittest();
	cnt_5pt_circle_draw_unittest();
	cnt_6pt_concave_with_negative_unittest();

	cnt_2pt_line_unittest_with_negative();
	cnt_6pt_zigzag_outofrange_unittest();

	// hysteresis tests
	cnt_5pt_circle_draw_hist_unittest();
	cnt_6pt_hist_offset_unittest();

}
