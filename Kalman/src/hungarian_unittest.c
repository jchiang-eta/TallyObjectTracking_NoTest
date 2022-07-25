#include "pipeline_config.h"
//#include "bbox_types.h"
//#include "nms_float.h"
//#include "pipeline.h"

#include "debug.h"
#include "assertion.h"
#include "hungarian.h"

#include "postprocessing_unittest.h"

#define TEST_SIZE_ROW 6
#define TEST_SIZE_COL 6

static int8_t _col_mate[TEST_SIZE_ROW];
static int8_t _row_mate[TEST_SIZE_COL];
static int8_t _parent_row[TEST_SIZE_COL];
static int8_t _unchosen_row[TEST_SIZE_ROW];
static int _row_dec[TEST_SIZE_ROW];
static int _col_inc[TEST_SIZE_COL];
static int _slack[TEST_SIZE_COL];
static int8_t _slack_row[TEST_SIZE_COL];
static int _hun_costmat[TEST_SIZE_ROW * TEST_SIZE_COL];

static hungarian_t _hun = {
	_col_mate,
	_row_mate,
	_parent_row,
	_unchosen_row,
	_row_dec,
	_col_inc,
	_slack,
	_slack_row,
	_hun_costmat,
	TEST_SIZE_ROW,
	TEST_SIZE_COL
};

void hungarian_test_recth() {

	debug("testing: %s\r\n", __FUNCTION__);

	// inputs
	float cost_mat[] = { 9, 9, 8, 3,
						 9, 1, 4, 8,
						 9, 2, 7, 9 };
	int n_rows = 3;
	int n_cols = 4;

	// outputs
	vec2ui8 matchings[3];
	int costs[3];

	// compute
	int matches = hungarian(cost_mat, n_rows, n_cols, matchings, costs, &_hun);

	// assert
	debug("matchings: %d\r\n", matches);
	for (int i = 0; i < matches; i++) {
		debug("match %d: (%d, %d) => %d\r\n", i, matchings[i].x, matchings[i].y, costs[i]);
	}

	assert(matches == 3);

	assert(matchings[0].x == 0 && matchings[0].y == 3 && costs[0] == 3);
	assert(matchings[1].x == 1 && matchings[1].y == 2 && costs[1] == 4);
	assert(matchings[2].x == 2 && matchings[2].y == 1 && costs[2] == 2);

}

void hungarian_test_rectv() {

	debug("testing: %s\r\n", __FUNCTION__);

	// inputs
	float cost_mat[] = { 9, 9, 8,
						 3, 5, 4,
						 2, 2, 7,
						 7, 5, 9 };
	int n_rows = 4;
	int n_cols = 3;

	// outputs
	vec2ui8 matchings[3];
	int costs[3];

	// compute
	int matches = hungarian(cost_mat, n_rows, n_cols, matchings, costs, &_hun);

	// assert
	debug("matchings: %d\r\n", matches);
	for (int i = 0; i < matches; i++) {
		debug("match %d: (%d, %d) => %d\r\n", i, matchings[i].x, matchings[i].y, costs[i]);
	}

	assert(matches == 3);

	assert(matchings[0].x == 1 && matchings[0].y == 2 && costs[0] == 4);
	assert(matchings[1].x == 2 && matchings[1].y == 0 && costs[1] == 2);
	assert(matchings[2].x == 3 && matchings[2].y == 1 && costs[2] == 5);


}

void hungarian_test_square() {

	debug("testing: %s\r\n", __FUNCTION__);

	// inputs
	float cost_mat[] = { 4, 2, 5, 7,
						9, 3, 10, 9,
						12, 5, 4, 5,
						6, 3, 7, 14 };
	int n_rows = 4;
	int n_cols = 4;

	// outputs
	vec2ui8 matchings[4];
	int costs[4];

	int matches = hungarian(cost_mat, n_rows, n_cols, matchings, costs, &_hun);

	debug("matchings: %d\r\n", matches);
	for (int i = 0; i < matches; i++) {
		debug("match %d: (%d, %d) => %d\r\n", i, matchings[i].x, matchings[i].y, costs[i]);
	}

	assert(matches == 4);

	assert(matchings[0].x == 0 && matchings[0].y == 0 && costs[0] == 4);
	assert(matchings[1].x == 1 && matchings[1].y == 1 && costs[1] == 3);
	assert(matchings[2].x == 2 && matchings[2].y == 3 && costs[2] == 5);
	assert(matchings[3].x == 3 && matchings[3].y == 2 && costs[3] == 7);

}

void hungarian_test_zeros_filled_row() {

	debug("testing: %s\r\n", __FUNCTION__);

	// inputs
	float cost_mat[] = { 0, 0, 0, 0, 0, 0 };
	int n_rows = 1;
	int n_cols = 6;

	// outputs
	vec2ui8 matchings[1];
	int costs[1];

	// compute
	int matches = hungarian(cost_mat, n_rows, n_cols, matchings, costs, &_hun);

	// if get stuck in infinite loop, matches will be 255
	assert(matches == 1);

	for (int i = 0; i < matches; i++) {
		debug("match %d: (%d, %d) => %d\r\n", i, matchings[i].x, matchings[i].y, costs[i]);
	}

	assert(matches == 1);
	assert(matchings[0].x == 0 && (0 <= matchings[0].y < 6) && costs[0] == 0);

}

void hungarian_test_zeros_filled_col() {

	debug("testing: %s\r\n", __FUNCTION__);

	// inputs
	float cost_mat[] = { 0, 0, 0, 0, 0, 0 };
	int n_rows = 6;
	int n_cols = 1;

	// outputs
	vec2ui8 matchings[1];
	int costs[1];

	// compute
	int matches = hungarian(cost_mat, n_rows, n_cols, matchings, costs, &_hun);

	// if get stuck in infinite loop, matches will be 255
	assert(matches == 1);

	for (int i = 0; i < matches; i++) {
		debug("match %d: (%d, %d) => %d\r\n", i, matchings[i].x, matchings[i].y, costs[i]);
	}

	assert(matches == 1);
	assert((0 <= matchings[0].x < 6) && matchings[0].y == 0 && costs[0] == 0);

}

void hungarian_test_infinite_loop() {

	debug("testing: %s\r\n", __FUNCTION__);

	// inputs
	float cost_mat[] = { 0, 0 };
	int n_rows = 1;
	int n_cols = 2;

	// outputs
	vec2ui8 matchings[1];
	int costs[1];

	// compute
	int matches = hungarian(cost_mat, n_rows, n_cols, matchings, costs, &_hun);
	
	// if get stuck in infinite loop, matches will be 255
	assert(matches == 1);

	for (int i = 0; i < matches; i++) {
		debug("match %d: (%d, %d) => %d\r\n", i, matchings[i].x, matchings[i].y, costs[i]);
	}

	assert(matches == 1);
	assert(matchings[0].x == 0 && matchings[0].y == 0 && costs[0] == 0);

}

void hungarian_test_negative_cost() {

	debug("testing: %s\r\n", __FUNCTION__);

	// inputs
	float cost_mat[] = { 7,  6,  6,  1,  9, 
						-1,  7,  8,  2,  2,  
						 1, -1,  6,  2,  4,  
						 3,  6,  2, -1,  7,  
						 3,  7,  9,  3, -1 };
	int n_rows = 5;
	int n_cols = 5;

	// outputs
	vec2ui8 matchings[5];
	int costs[5];

	int matches = hungarian(cost_mat, n_rows, n_cols, matchings, costs, &_hun);

	debug("matchings: %d\r\n", matches);
	for (int i = 0; i < matches; i++) {
		debug("match %d: (%d, %d) => %d\r\n", i, matchings[i].x, matchings[i].y, costs[i]);
	}

	assert(matches == 5);

	assert(matchings[0].x == 0 && matchings[0].y == 3 && costs[0] == 1);
	assert(matchings[1].x == 1 && matchings[1].y == 0 && costs[1] == -1);
	assert(matchings[2].x == 2 && matchings[2].y == 1 && costs[2] == -1);
	assert(matchings[3].x == 3 && matchings[3].y == 2 && costs[3] == 2);
	assert(matchings[4].x == 4 && matchings[4].y == 4 && costs[4] == -1);

}

/*!
* \brief Test correctness of hungarian algorithm
*/
void hungarian_unittest(){

	// Failed when run it in the beginning
	hungarian_test_infinite_loop();

	// checks if row filled with zeros returns a proper result
	hungarian_test_zeros_filled_row();

	// checks if column filled with zeros returns a proper result
	hungarian_test_zeros_filled_col();

	// Passed tests
	hungarian_test_square();
	hungarian_test_recth();
	hungarian_test_rectv();

	// Passed tests when not running it in the beginning
	hungarian_test_infinite_loop();
	// Failed
	hungarian_test_negative_cost();
}