

#include "tracking_unittest.h"
#include "pipeline_config.h"
#include "tracking.h"

#include "debug.h"
#include "assertion.h"

#ifndef NULL
#define NULL 0
#endif

#define TEST_STRESS_ITER 100

#define TEST_MATCH_COST_LIMIT 28*28

// extern object_tracking_t _track_obj;

void print_last_pos(object_tracking_t* track_obj);

// runs all test one after another
void tracking_unittest() {
	test_match_filter();
	bboxcorner2centroid_unittest();
	//test_tracking_1(); // fails
	test_tracking_2();
	test_tracking_clearing();

	//test_tracking_stress();
}

uint16_t test_idxs[] = { 0, 1, 2, 3, 4, 5 };
BoxCornerEncodingFloat test_dets[] = {

  { 0.0f,  0.0f,  0.0f,  0.0f},
  { 0.0f, 20.0f, 10.0f, 30.0f},
  {80.0f, 20.0f, 80.0f, 20.0f},
  {60.0f, 70.0f, 60.0f, 80.0f},
  {70.0f, 50.0f, 70.0f, 50.0f},
  { 0.0f, 50.0f,  0.0f, 50.0f}

};

#pragma message("UNITTESTING requires setup to work")
static object_tracking_cfg_t tracking_config = {0}; // <= note, this will fail
static kalman_cfg_t kalman_config = {0};

//void tracking_perf_init(object_tracking_t* track_obj) {
//	object_tracking_init(&track_obj);
//}
//
//void tracking_perf_loop(int n, object_tracking_t* track_obj) {
//	for (int i = 0; i < n; i++) {	// should link det 1 and 3 to existing obj and create new object for det 2
//		object_tracking(test_dets, test_idxs, 6, &track_obj);
//	}
//}
//
//void test_tracking_stress() {
//	extern object_tracking_t _track_obj;
//
//	tracking_perf_init((object_tracking_t*)&_track_obj);
//	tracking_perf_loop(TEST_STRESS_ITER, (object_tracking_t*)&_track_obj);
//
//}

// test filtering capability
void test_match_filter() {

	debug("test_match_filter\r\n");

	{
		vec2ui8 match_0[] = { {1,1},{2,2},{3,3},{4,0},{5,0},{6,0} };
		int costs_0[] = { 17, 20, 2600, 3000, 30, 2900 };
		int n = filter_matches(match_0, costs_0, 6, TEST_MATCH_COST_LIMIT);

		debug("Match filter test 0\r\n");
		for (int i = 0; i < n; i++) {
			debug("\tcosts[%d]=%d\r\n", i, costs_0[i]);
		}

		assert(n == 3);
		assert(costs_0[0] == 17);
		assert(costs_0[1] == 20);
		assert(costs_0[2] == 30);

	}

	{
		vec2ui8 match_1[] = { {1,1},{2,2},{3,3},{4,0},{5,0},{6,0},{7,0},{8,8} };
		int costs_1[] = { 3000, 5, 2, 3000, 2, 2900, 2600, 500 };
		int n = filter_matches(match_1, costs_1, 8, TEST_MATCH_COST_LIMIT);

		debug("Match filter test 1\r\n");
		for (int i = 0; i < n; i++) {
			debug("\tcosts[%d]=%d\r\n", i, costs_1[i]);
		}

		assert(n == 4);
		assert(costs_1[0] == 5);
		assert(costs_1[1] == 2);
		assert(costs_1[2] == 2);
		assert(costs_1[3] == 500);
	}

}

// from "optuna_2.2l_2_T4_T0_multiline4_cffi_test_office_v1\29bdc258"
//pipeline_config_t pipeline_config = {
//	.mdl_out0_scale = OUT0_QUANT_SCALE,
//	.mdl_out0_offset = OUT0_QUANT_OFFSET,
//	.mdl_out1_scale = OUT1_QUANT_SCALE,
//	.mdl_out1_offset = OUT1_QUANT_OFFSET,
//	.mdl_bboxes_first = MODEL_BBOX_FIRST,
//	.mdl_input_w = IN0_SIZE_DIM0,
//	.mdl_input_h = IN0_SIZE_DIM1,
//	.mdl_pad = {0},
//
//	.pp_score_threshold = 0.6048340228576267f,
//	.pp_score_prefilter_threshold = 0.6048340228576267f,
//	.pp_iou_threshold = 0.25f,
//	.pp_roi_xmin = 0.01f,
//	.pp_roi_xmax = 0.99f,
//	.pp_roi_ymin = 0.01f,
//	.pp_roi_ymax = 1.0f,
//	.pp_sumred_vect = {0.0f, 1.0f},
//	.pp_sumred_vect_n = 2,
//	.pp_filter_select_class_id = 1,
//	.pp_pad = {0},
//
//	.trk_association_cost_limit = 0.3f,
//	.trk_kalman_state_variance = 0.04871f,
//	.trk_kalman_measurement_variance = 0.13112f,
//	.trk_kalman_time_constant = 1.0f,
//	.trk_kalman_max_missed = 2,
//	.trk_pad = {0},
//
//,
//	.cnt_offsets = {
//		0.07045477953359523f,
//		-0.02061423818398246f,
//		-0.1377327331812282f,
//		-0.2587112553478026f
//	},
//	.cnt_hysteresis = {
//		0.08192360367077822f,
//		0.11068247617382919f,
//		0.13944134867688016f,
//		0.1682002211799311f
//	},
//	.cnt_pts_n = 6,
//	.cnt_lines_n = 4,
//	.cnt_pad = {0},
//};

static float _cnt_pts[] = { 0.5f, 0.5f };
static float _cnt_hysteresis[] = { 0.01f };
static float _cnt_offsets[] = { 0.0f };
static float _y[] = { 0.5f, 0.5f };

void test_tracking_1_init(object_tracking_t* track_obj) {
	track_obj->kalman_num_filter = 3;
	track_obj->max_detections = 5;

	object_tracking_cfg_t* cfg = &track_obj->cfg;
	kalman_cfg_t* kcfg = &cfg->kalman_cfg;

	kcfg->state_variance = 0.05f;
	kcfg->measurement_variance = 0.15f;
	kcfg->time_constant = 1.0f;

	cfg->cnt_pts = _y;
	cfg->cnt_pts_n = 2;
	cfg->width = 96;
	cfg->height = 96;
	cfg->cnt_hysteresis = _cnt_hysteresis;
	cfg->cnt_offsets = _cnt_offsets;
	cfg->cnt_lines_n = 1;

	cfg->max_missed = 2; // or 3?
	cfg->roi_min_x = 0.0f;
	cfg->roi_max_x = 100.0f;
	cfg->roi_min_y = 0.0f;
	cfg->roi_max_y = 100.0f;
	cfg->match_cost_limit = 1000;

	object_tracking_init(track_obj);
}

void test_tracking_1() {
	extern object_tracking_t _track_obj;

	debug("Pipeline init\r\n");
	test_tracking_1_init((object_tracking_t*)&_track_obj);

	debug("Step 0\r\n");
	uint16_t idxs_0[] = { 0, 1 };
	BoxCornerEncodingFloat dets_0[] = {
		{0.0f, 0.0f, 10.0f, 10.0f}, // new object, id 0
		{50.0f, 60.0f, 50.0f, 60.0f} // new object, id 1
	};
	// should add two new objects
	object_tracking(dets_0, idxs_0, 2, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 2);
	assert(_track_obj.trackers[0].missed == 0);
	assert(_track_obj.trackers[1].missed == 0);

	debug("Step 1\r\n");
	uint16_t idxs_1[] = { 1, 2, 3 };
	BoxCornerEncodingFloat dets_1[] = {
		{ 0.0f,  0.0f,  0.0f,  0.0f}, // invalid - missing index
		{ 0.0f, 20.0f, 10.0f, 30.0f}, // match with id 0
		{80.0f, 20.0f, 80.0f, 20.0f}, // valid but far - new object, id 2
		{60.0f, 70.0f, 60.0f, 80.0f}  // match with id 1
	}; // [0,0], [2,1]
	// should link det 1 and 3 to existing obj and create new object for det 2
	object_tracking(dets_1, idxs_1, 3, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 3);
	assert(_track_obj.trackers[0].missed == 0);
	assert(_track_obj.trackers[1].missed == 0);
	assert(_track_obj.trackers[2].missed == 0);

	debug("Step 2\r\n");
	uint16_t idxs_2[] = { 2, 3 };
	BoxCornerEncodingFloat dets_2[] = {
		{ 0.0f,  0.0f,  0.0f,  0.0f}, // invalid
		{ 0.0f, 20.0f, 10.0f, 30.0f}, // invalid
		{90.0f, 50.0f, 95.0f, 50.0f}, // new object - but no space, so dropped
		{60.0f, 80.0f, 80.0f, 90.0f}, // match with id 1
	}; // [0, 1]
	object_tracking(dets_2, idxs_2, 2, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 3);
	// id 0 missed 1, id 1 missed 0, id 2 missed 1
	assert(_track_obj.trackers[0].missed == 1);
	assert(_track_obj.trackers[1].missed == 0);
	assert(_track_obj.trackers[2].missed == 1);

	debug("Step 3\r\n");
	object_tracking(NULL, NULL, 0, (object_tracking_t*)&_track_obj); // 3 kalmans missed +1
	// id 0 missed 2, id 1 missed 1, id 2 missed 2
	assert(tracker_num_active(&_track_obj) == 3);
	assert(_track_obj.trackers[0].missed == 2);
	assert(_track_obj.trackers[1].missed == 1);
	assert(_track_obj.trackers[2].missed == 2);

	debug("Step 4\r\n");
	object_tracking(NULL, NULL, 0, (object_tracking_t*)&_track_obj); // 3 kalmans missed +1, 2 removed, one remaining
	// id 0 missed 3 (removed), id 1 missed 2, id 2 missed 3 (removed)
	assert(tracker_num_active((object_tracking_t*)&_track_obj) == 1);
	assert(_track_obj.trackers[1].missed == 2);
	// id 0 started below boundary and ended above boundary
	// id 2 started and ended below boundary
	assert(_track_obj.counts.v[1] == 1);

	debug("Step 5\r\n");
	object_tracking(NULL, NULL, 0, &_track_obj); // last kalman will be removed
	// id 1 missed 3 (removed)
	assert(tracker_num_active((object_tracking_t*)&_track_obj) == 0);

	debug("Step 6\r\n");
	uint16_t idxs_6[] = { 0, 1 };
	BoxCornerEncodingFloat dets_6[] = {
		{ 50.0f, 10.0f, 50.0f, 10.0f}, // new
		{ 10.0f, 20.0f, 10.0f, 20.0f}, // new
	};
	// create 2 new kalmans
	object_tracking(dets_6, idxs_6, 2, (object_tracking_t*)&_track_obj);
	// id 0 missed 0, id 1 missed 0
	assert(tracker_num_active((object_tracking_t*)&_track_obj) == 2);
	assert(_track_obj.trackers[0].missed == 0);
	assert(_track_obj.trackers[1].missed == 0);

	debug("step 7\r\n");
	// keep 2 kalmans, missed++ for both
	object_tracking(NULL, NULL, 0, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active((object_tracking_t*)&_track_obj) == 2);
	assert(_track_obj.trackers[0].missed == 1);
	assert(_track_obj.trackers[1].missed == 1);

	debug("step 8\r\n");
	// keep 2 kalmans, missed++ for both
	object_tracking(NULL, NULL, 0, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active((object_tracking_t*)&_track_obj) == 2);
	assert(_track_obj.trackers[0].missed == 2);
	assert(_track_obj.trackers[1].missed == 2);

	debug("step 9\r\n");
	uint16_t idxs_9[] = { 0, 1 };
	BoxCornerEncodingFloat dets_9[] = {
		{ 50.0f, -10.0f, 60.0f, -10.0f}, // match with id 0
		{ 70.0f,  70.0f, 70.0f,  70.0f}, // should be too far, therefore unmatched -> new object
	};
	// id 0 matches, miss = 0; id 1 misses 3 and removed; id 2 new object
	object_tracking(dets_9, idxs_9, 2, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active((object_tracking_t*)&_track_obj) == 2);
	assert(_track_obj.trackers[0].missed == 0);
	assert(_track_obj.trackers[2].missed == 0);

	debug("Step 10\r\n");
	uint16_t idxs_10[] = { 0, 1, 2, 3, 4, 5 };
	BoxCornerEncodingFloat dets_10[] = {
		{ 50.0f, -10.0f, 60.0f, -10.0f}, // should be too far, therefore unmatched -> new object
		{ 70.0f,  70.0f, 70.0f,  60.0f}, // match with id 2
		{ 50.0f, -10.0f, 60.0f, -10.0f}, // new object - but no space, so dropped
		{ 80.0f,  70.0f, 80.0f,  70.0f}, // new object - but no space, so dropped
		{ 50.0f, -10.0f, 60.0f, -10.0f}, // new object - but no space, so dropped
		{ 10.0f,  15.0f, 20.0f, -50.0f}, // detection out of limit, drop it
	};
	// id 0 miss 1, id 1 new object, id 2 miss 0
	object_tracking(dets_10, idxs_10, 6, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 3);
	assert(_track_obj.trackers[0].missed == 1);
	assert(_track_obj.trackers[1].missed == 0);
	assert(_track_obj.trackers[2].missed == 0); // Bug. Expect 0 but 1
}

void test_tracking_2() {

	extern object_tracking_t _track_obj;

	debug("Pipeline init\r\n");
	test_tracking_1_init((object_tracking_t*)&_track_obj);

	debug("Step 0\r\n");
	uint16_t idxs_0[] = { 0 };
	BoxCornerEncodingFloat dets_0[] = {
		{0.0f, 0.0f, 10.0f, 10.0f}, // new object, id 0
	};
	// should add two new objects
	object_tracking(dets_0, idxs_0, 1, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 1);

	debug("Step 1\r\n");
	uint16_t idxs_1[] = { 0 };
	BoxCornerEncodingFloat dets_1[] = {
		{ 10.0f, 10.0f, 20.0f, 20.0f}, // match with id 0
	};
	object_tracking(dets_1, idxs_1, 1, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 1);

	debug("Step 1\r\n");
	uint16_t idxs_2[] = { 0 };
	BoxCornerEncodingFloat dets_2[] = {
		{ 20.0f, 20.0f, 30.0f, 30.0f}, // match with id 0
	};
	object_tracking(dets_2, idxs_2, 1, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 1);

	debug("Printing last position: \r\n");
	print_last_pos((object_tracking_t*)&_track_obj);
}

void test_tracking_clearing() {

	extern object_tracking_t _track_obj;

	debug("Pipeline init\r\n");
	test_tracking_1_init((object_tracking_t*)&_track_obj);

	debug("Step 0\r\n");
	uint16_t idxs_0[] = { 0, 1, 2 };
	BoxCornerEncodingFloat dets_0[] = {
		{0.0f, 0.0f, 10.0f, 10.0f}, // new object, id 0
		{0.0f, 0.0f, 20.0f, 10.0f}, // new object, id 1
		{0.0f, 0.0f, 10.0f, 20.0f}, // new object, id 2
	};
	// should add three new objects
	object_tracking(dets_0, idxs_0, 3, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 3);

	// clear all objects - we should have 0 objects in memory
	object_tracking_clear(&_track_obj);
	assert(tracker_num_active(&_track_obj) == 0);

	debug("Step 1\r\n");
	uint16_t idxs_1[] = { 0, 1 };
	BoxCornerEncodingFloat dets_1[] = {
		{ 10.0f, 10.0f, 20.0f, 20.0f}, // match with id 0
		{ 10.0f, 10.0f, 25.0f, 20.0f}, // match with id 1

	}; 
	object_tracking(dets_1, idxs_1, 2, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 2);
	object_tracking(dets_1, idxs_1, 2, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 2);

	// clear all objects - we should have 0 objects in memory
	object_tracking_clear(&_track_obj);
	assert(tracker_num_active(&_track_obj) == 0);

	debug("Step 3\r\n");
	uint16_t idxs_2[] = { 0 };
	BoxCornerEncodingFloat dets_2[] = {
		{ 20.0f, 20.0f, 30.0f, 30.0f}, // match with id 0
	};
	object_tracking(dets_2, idxs_2, 1, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 1);

	// clear all objects - we should have 0 objects in memory
	object_tracking_clear(&_track_obj);
	assert(tracker_num_active(&_track_obj) == 0);

	debug("Step 4\r\n");
	object_tracking(NULL, NULL, 0, (object_tracking_t*)&_track_obj);
	assert(tracker_num_active(&_track_obj) == 0);

	// clear all objects - we should have 0 objects in memory
	object_tracking_clear(&_track_obj);
	assert(tracker_num_active(&_track_obj) == 0);

}

void print_last_pos(object_tracking_t* track_obj) {
	uint8_t kalman_num_filter = track_obj->kalman_num_filter;
	kalman_tracker* trackers = track_obj->trackers;

	for (int i = 0; i < kalman_num_filter; i++) {
		kalman_tracker* tracker = &trackers[i];
		if (tracker->active) {
#if DEBUG
			vec2f pos = get_position_from_state(tracker->kf_ptr);
			debug("\tTracker %d: uid=%d, x=%.2f, y=%.2f, missed=%d\r\n", i, tracker->uid, pos.x, pos.y, tracker->missed);
#endif
		}
	}
}

void bboxcorner2centroid_unittest() {
	vec2f c_det;
	BoxCornerEncodingFloat bboxes[1];
	BoxCornerEncodingFloat* bbox = &bboxes[0];
	bbox->xmax = 0.8f;
	bbox->xmin = -0.2f;
	bbox->ymax = 0.4f;
	bbox->ymin = 0.0f;
	bboxcorner2centroid(bbox, &c_det);
	assert(c_det.x == 0.3f);
	assert(c_det.y == 0.2f);
	debug("Passed bboxcorner2centroid_unittest\r\n");
}

