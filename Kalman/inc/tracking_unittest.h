#ifndef _TRACKING_UNITTESTING_H
#define _TRACKING_UNITTESTING_H

#include <stdint.h>

#include "bbox_types.h"
#include "vec_types.h"

void tracking_unittest();

void test_match_filter();
void test_tracking_1();
void test_tracking_2();
void test_tracking_clearing();
void bboxcorner2centroid_unittest();

#endif // !_TRACKING_UNITTESTING_H

