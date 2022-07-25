/* Copyright 2018 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "nms_byte.h"
#include "debug.h"

#ifdef __cplusplus
extern    "C"
{
#endif

/*
	return indicies of predictions where at least one score (i.e. of at least one class) is larger than threshold
*/
int16_t FilterByScoreMulticlassExclBG_Byte(
	const int8_t * scores, // scores array of length n*n_classes
	int16_t n,
	const int8_t threshold, // single value to apply to all non-background columns
	int16_t * keep_indices,
	uint16_t keep_n_limit,
	uint8_t n_classes
)
{
	// scores[n, n_classes]: [nBoxes, bg or class_i]
	int16_t	count = 0;  // # of valid data

	// iterate over predictions (rows)
	for ( uint16_t i = 0, jj = 0; i < n; i++, jj += n_classes ) {
		// interate over scores in each row (columns), excluding BG (first column)
		for( uint16_t j = 1; j < n_classes; j++){
			if ( scores[jj+j] >= threshold ) {
				keep_indices[count] = i;
				count++;
				if (count >= keep_n_limit) {
					return count;
				}
				// if any of the boxes exceeds the score, no need to check the rest
				break;
			}
		}
	}

	return count;	
}

int16_t FilterOneClassByScoreMulticlass_Byte(
	const int8_t * scores, // scores array of length n*n_classes
	int16_t n, // number of predictions (rows)
	const int8_t threshold, // single value to apply to all non-background columns
	int16_t * keep_indices,
	uint16_t keep_n_limit,
	uint8_t n_classes,
	uint8_t class_select // id of the class to filter
)
{
	debug("\r\nFilterOneClassByScoreMulticlass_Byte\r\n");
	// scores[n, n_classes]: [nBoxes, bg or class_i]
	int16_t	count = 0;  // # of valid data

	debug("threshold=%d, class_select=%d\r\n", threshold, class_select);
	// iterate over predictions (scores of selected class)
	for ( uint16_t i = 0, jj = class_select; i < n; i++, jj += n_classes ) {
		if ( scores[jj] >= threshold ) {
			
			keep_indices[count] = i;
			count++;
			//debug("sample at: count=%d, i=%d, jj=%d, class_select=%d\r\n", count, i, jj, class_select);
			if (count >= keep_n_limit) {
				debug("quitting at count=%d\r\n", count);
				return count;
			}
		}	
	}

	debug("quitting at end: count=%d\r\n", count);
	return count;	
}

// #define __SSAT(x,y) 0

// #define	max(a, b)	((a) >= (b) ? (a) : (b))
// #define	min(a, b)	((a) <= (b) ? (a) : (b))

// #define areaBox(b)  ( ((int16_t)((b).ymax) - (int16_t)((b).ymin)) * ((int16_t)((b).xmax) - (int16_t)((b).xmin)) )

// Returns approximate value of e^x
// using sum of first n terms of Taylor Series
// float fexpf( int n, float x )
// {
// 	float sum = 1.0f; // initialize sum of series
// 	float fi = n - 1.0f;

// 	for (int i = n - 1; i > 0; --i, fi -= 1.0f )
// 		sum = 1 + x * sum / fi;

// 	return sum;
// }

// #ifdef STD_LIB

// #include <stdio.h>
// #include <math.h>
// #include <stdint.h>

// // if platform does not support __SSAT (e.g. on PC)
// // Reference: https://www.keil.com/support/man/docs/armasm/armasm_dom1361289904320.htm
// q7_t __SSAT(int32_t operand, uint8_t shift){

// 	int32_t sat_value = 1 << shift;

// 	if(operand < -sat_value)
// 		return -sat_value;
// 	else if(operand > sat_value)
// 		return sat_value;
// 	else
// 		return operand;
// }

// #else

// // #include "arm_math.h"
// // #include "qfplib-m3.h"
// // #define expf    qfp_fexp
  
// #endif // STD_LIB

#ifdef __cplusplus
}
#endif
