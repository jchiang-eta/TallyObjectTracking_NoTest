//#include <iostream>
//#include <fstream>
//using namespace std;

#include "pipeline_config.h"
#include "bbox_types.h"
#include "nms_byte.h"
#include "nms_float.h"
#include "pipeline.h"

#include "debug.h"
#include "assertion.h"

#include "postprocessing_unittest.h"

// test configuration
#define N_TEST_BOXES				50
#define N_TEST_OVER_THRESHOLD		10

#define BG_THRESHOLD				0.1f
#define OBJ_THRESHOLD				0.6f

#define NON_SUPRESSED				5

#define TEST_SCORE_THRESHOLD		0.5f


BoxCornerEncodingFloat test_boxes[N_TEST_BOXES];
float test_scores[2 * N_TEST_BOXES];
uint16_t test_kept_indicies[N_TEST_BOXES];
#define NMS_TEST_MAX_KEEP 50
static float _nms_kept_scores[NMS_TEST_MAX_KEEP];
static hard_nms_t _hard_nms = {
	_nms_kept_scores,
	NMS_TEST_MAX_KEEP
};

void init_boxes() {

	// fill with zeros
	for (int i = 0; i < N_TEST_BOXES; i++) {
		BoxCornerEncodingFloat* b = &test_boxes[i];
		b->xmin = 0;
		b->ymin = 0;
		b->xmax = 0;
		b->ymax = 0;
	}

	for (int i = 0; i < 2 * N_TEST_BOXES; i++)
		test_scores[i] = 0;

	// fill every N-th box with data
	int gap = N_TEST_BOXES / N_TEST_OVER_THRESHOLD;
	for (int i = 0, idx = 0; i < N_TEST_OVER_THRESHOLD; i++, idx += gap) {

		test_scores[2 * idx] = BG_THRESHOLD + (0.01f * i); // "residual" background confidence
		test_scores[2 * idx + 1] = OBJ_THRESHOLD + (0.01f * i); // object confidence

		// if box side is a then shift > 0.19*a will generate a non supressed box (for IoU == 0.5)
		// b - side intersection of intersection square, a - side of bounding box
		// general formula b = sqrt(2*IoU/(1+IoU))*a
		// for IoU = 0.5 => b = 0.816, thus 1-b = 0.184
		// here box side a=0.1 so moving for 0.02 will generate a new box - every second box should persist

		//	n	IoU		b			1 - b
		//	0	0.00	0.000000	1.000000 <- None
		//	1	0.05	0.308607	0.691393 <- One in 7
		//	2	0.10	0.426401	0.573599 <- One in 6
		//	3	0.15	0.510754	0.489246 <- One in 5
		//	4	0.20	0.577350	0.422650 <- One in 5
		//	5	0.25	0.632456	0.367544 <- One in 4 <- this one currently
		//	6	0.30	0.679366	0.320634 <- One in 4
		//	7	0.35	0.720082	0.279918 <- One in 3
		//	8	0.40	0.755929	0.244071 <- One in 3
		//	9	0.45	0.787839	0.212161 <- One in 3
		//	10	0.50	0.816497	0.183503 <- One in 2
		//	11	0.55	0.842424	0.157576 <- One in 2
		//	12	0.60	0.866025	0.133975 <- One in 2
		//	13	0.65	0.887625	0.112375 <- One in 2
		//	14	0.70	0.907485	0.092515 <- Every one
		//	15	0.75	0.925820	0.074180 <- Every one
		//	16	0.80	0.942809	0.057191 <- Every one
		//	17	0.85	0.958603	0.041397 <- Every one
		//	18	0.90	0.973329	0.026671 <- Every one
		//	19	0.95	0.987096	0.012904 <- Every one

		BoxCornerEncodingFloat* b = &test_boxes[idx];
		b->xmin = 0.0f + 0.01f * i;
		b->ymin = 0.9f - 0.01f * i;
		b->xmax = 0.1f + 0.01f * i;
		b->ymax = 1.0f - 0.01f * i;

	}

}

extern uint16_t kept_indicies_tmp[MAX_INDICIES];

void filtering_unittest()
{

	debug("\tFiltering unittest\r\n");

	init_boxes();

	int n_kept_tmp = FilterByScoreSingleClassExclBG_Float(test_scores, N_TEST_BOXES, TEST_SCORE_THRESHOLD, kept_indicies_tmp, MAX_INDICIES);

	//// check if numer of boxes is correct
	assert(n_kept_tmp == N_TEST_OVER_THRESHOLD);
	//// check if each box is ok
	for (int i = 0, idx = 0; i < N_TEST_OVER_THRESHOLD; i++, idx += N_TEST_BOXES / N_TEST_OVER_THRESHOLD)
		assert(kept_indicies_tmp[i] == idx);
}

void filtering_unittest_1()
{

	debug("\tFiltering unittest 1\r\n");

	// test FilterWeighedSumMulticlass_Float
	debug("FilterWeighedSumMulticlass_Float test\r\n");
	init_boxes();
	const float weights_vect[2] = { 0.8f, 0.8f };
	int count_float = FilterWeighedSumMulticlass_Float(test_scores, N_TEST_BOXES, TEST_SCORE_THRESHOLD, weights_vect, kept_indicies_tmp, MAX_INDICIES, 2);
	assert(count_float == 10);

	// test FilterOneClassByScoreMulticlass_Byte
	debug("FilterOneClassByScoreMulticlass_Byte test\r\n");
	int8_t scores_byte[2 * N_TEST_BOXES];
	for (int i = 0; i < N_TEST_BOXES; i++) {
		scores_byte[2 * i + 1] = -128 + 5 * i;
	}
	int8_t threshold = 0;
	int16_t keep_indicies[N_TEST_BOXES];

	int16_t count_byte = FilterOneClassByScoreMulticlass_Byte(scores_byte, N_TEST_BOXES, threshold, keep_indicies, N_TEST_BOXES, 2, 1);
	assert(count_byte == 24);
}

BoxCornerEncodingFloat nms_bboxes[10];
float scores[6] = { 0.89f, 0.11f, 0.05f, 0.44f, 0.02f, 0.60f };
uint16_t kept_indices[6] = { 0, 1, 2, 3, 4, 5 };

void nms_init() {
	BoxCornerEncodingFloat* b;

	b = &nms_bboxes[0];
	b->xmin = 0.72f;
	b->xmax = 0.96f;
	b->ymin = 0.28f;
	b->ymax = 0.55f;

	b = &nms_bboxes[1];
	b->xmin = 0.30f;
	b->xmax = 0.55f;
	b->ymin = 0.33f;
	b->ymax = 0.71f;

	b = &nms_bboxes[2];
	b->xmin = 1.00f;
	b->xmax = 1.00f;
	b->ymin = 0.37f;
	b->ymax = 0.59f;

	b = &nms_bboxes[3];
	b->xmin = 0.53f;
	b->xmax = 0.78f;
	b->ymin = 0.28f;
	b->ymax = 0.92f;

	b = &nms_bboxes[4];
	b->xmin = 0.17f;
	b->xmax = 0.49f;
	b->ymin = 0.54f;
	b->ymax = 0.58f;

	b = &nms_bboxes[5];
	b->xmin = 0.37f;
	b->xmax = 0.69f;
	b->ymin = 0.05f;
	b->ymax = 0.15f;
}

void nms_unittest(){
	uint16_t nms_kept_n;
	uint16_t nms_kept_indicies[6];
	nms_init();
	nms_kept_n = hard_nms_single_class(nms_bboxes, scores, kept_indices, 6, nms_kept_indicies, 0.01f, 0, &_hard_nms);
	uint16_t expected[4] = { 0, 5, 1, 2 };

	assert(nms_kept_n == 4);

	for (int i = 0; i < nms_kept_n; i++) {
		assert(nms_kept_indicies[i] == expected[i]);
	}
	debug("NMS passed\n");
}


// joint unittest
void postprocessing_unittest()
{

	//assert(0);
	debug("Postprocessing unittest\r\n");
	
	filtering_unittest();
	filtering_unittest_1();
	nms_unittest();
	// postproc_pipeline_unittest();

}