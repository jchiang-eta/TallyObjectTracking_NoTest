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

#include "nms_float.h"

#include "pipeline_config.h"

#include "debug.h"
#include "assert.h"

#ifdef STD_LIB

#include <stdio.h>
#include <math.h>
#include <stdint.h>

// if platform does not support __SSAT (e.g. on PC)
// Reference: https://www.keil.com/support/man/docs/armasm/armasm_dom1361289904320.htm
// float __SSAT(int32_t operand, uint8_t shift){

// 	int32_t sat_value = 1 << shift;

// 	if(operand < -sat_value)
// 		return -sat_value;
// 	else if(operand > sat_value)
// 		return sat_value;
// 	else
// 		return operand;
// }

#else

#include "arm_math.h"
#include "qfplib-m3.h"
#define expf    qfp_fexp
  
#endif // STD_LIB

#ifdef ETA_SDK
#include "eta_types.h" // we only use eta_rc (return code, int, -1 or 0)
#include "FreeRTOS.h"
#include "portable.h"
#include "print_util.h"
#else

#endif // ETA_SDK

#ifdef __cplusplus
extern    "C"
{
#endif

/**
 * @brief Example showing how to document a function with Doxygen.
 *
 * Description of what the function does. This part may refer to the parameters
 * of the function, like @p param1 or @p param2. A word of code can also be
 * inserted like @c this which is equivalent to <tt>this</tt> and can be useful
 * to say that the function returns a @c void or an @c int. If you want to have
 * more than one word in typewriter font, then just use @<tt@>.
 * We can also include text verbatim,
 * when the language is not the one used in the current source file (but
 * <b>be careful</b> as this may be supported only by recent versions
 * of Doxygen). By the way, <b>this is how you write bold text</b> or,
 * if it is just one word, then you can just do @b this.
 * @param values pointer to float array contiaining thresholds for each detection.
 * @param n number @p values
 * @param threshold decision threshold
 * @return @c int Numer of found predictions with confidence of grater or equal to @p threshold
 * @see Box_The_Second_Function
 * @see Box_The_Last_One
 * @see http://website/
 * @note Something to note.
 * @warning Warning.
 */

#define	max(a, b)	((a) >= (b) ? (a) : (b))
#define	min(a, b)	((a) <= (b) ? (a) : (b))

#define areaBox(b)  ( ((float)((*b).ymax) - (float)((*b).ymin)) * ((float)((*b).xmax) - (float)((*b).xmin)) )

/**
 * @brief filter detection based on confidence threshold
 * 
 * function returns indicies of from @p values array which are greater or equal to @p threshold
 * *
 * @param scores - pointer to float array contiaining confidences for and detected class in each 'row' [n, 2 (bg or person)]. This arrauy is flattened (1D)
 * @param n number of rows in @p values array
 * @param threshold decision threshold
 * @param[out] keep_indices array where indicies of scores above the specified threshold will be written
 * @param[in] max_keep maximum number of scores to select 
 * @return @c int Numer of found predictions with confidence of grater or equal to @p threshold
 */
// uint16_t SelectDetectionsAboveScoreThresholdMulticlass_Float( const float * scores, int16_t n,
// 											const float threshold,
// 											uint16_t * keep_indices,
// 											uint16_t max_keep,
// 											uint8_t n_classes)
// {
// 	debug( "SelectDetectionsAboveScoreThresholdMulticlass\n" );

// 	// scores => [n, n_classes] but as flat buffer
// 	int16_t	count = 0;  // # of valid data
// 	int16_t jj = 1;     // index to person's conf-value
// 	// i => prediction iterator
// 	for ( int16_t i = 0; i < n; i++, jj += n_classes ) {
// 		if ( scores[jj] >= threshold ) {
// 			keep_indices[count] = i;
// 			count++;
// 			if (count >= max_keep) {
// 				return count;
// 			}
// 		}
// 	}
// 	return count;
// }

/**
 * @brief filter detections by comparing a weighed sum of multiclass scores to a confidence threshold
 *
 * function returns indicies of from @p scores array which meet the condition:
 * 		dot(S,mask') > @p threshold, where S is a row from @p scores
 * *
 * @param[in] scores pointer to float array contiaining confidences for and detected class in each 'row' [n, bg or person]. This arrauy is flattened (1D)
 * 					 this array will be overwritten with a single column of weighed vect sum of scores which surpass the threshold
 * @param[in] n number of rows in @p scores array
 * @param[in] threshold decision threshold
 * @param[in] mask weights vector of length @p n_classes
 * @param[out] keep_indices array where indicies of scores above the specified threshold will be written
 * @param[in] max_keep maximum number of scores to select
 * @param[in] n_classes number of classes (cols) in @p scores array
 * @return @c int Numer of found predictions with confidence of grater or equal to @p threshold
 */
uint16_t FilterWeighedSumMulticlass_Float( 	float * scores, 
											int16_t n, // number of predictions (rows)
											float threshold,
											const float* weights_vect, // weights array, must be ength of n_classes
											uint16_t * keep_indices,
											uint16_t max_keep,
											uint8_t n_classes)
{

	debug("FilterWeighedSumMulticlass_Float. threshold=%f, weights_vect: [", threshold);
    for(int i=0; i<n_classes; i++)
        debug("%f, ", weights_vect[i]);
    debug("\b]\n");
	// scores => [n, n_classes] but as flat buffer

	int16_t	count = 0;  // # of valid data
	int16_t iter = 0;
	// iterate over predictions (rows)
	for ( int16_t i = 0, j = 0; i < n*n_classes; i += n_classes, j++ )
	{
		float vect_sum = 0;
		for (int16_t j = 0; j < n_classes; j++)
		{
			vect_sum += weights_vect[j]*scores[iter];
			iter++;
		}
		debug("iter=%d vect_sum=%f\r\n", iter, vect_sum);
		if ( vect_sum >= threshold ) {
			keep_indices[count] = j;
			scores[keep_indices[count]] = vect_sum;

			debug("sample at: count=%d, row=%i score=%f\r\n", count, j, vect_sum);
			count++;
			if (count >= max_keep) {
				return count;
			}
		}
	}

	return count;
}

/**
 * @brief filter detection based on confidence threshold
 *
 * function returns indicies of from @p values array which are greater or equal to @p threshold
 * *
 * @param scores - pointer to float array contiaining confidences for and detected class in each 'row' [n, 2 (bg or person)]. This arrauy is flattened (1D)
 * @param n number of rows in @p values array
 * @param threshold decision threshold
 * @param[out] keep_indices array where indicies of scores above the specified threshold will be written
 * @param[in] max_keep maximum number of scores to select 
 * @return @c int Numer of found predictions with confidence of grater or equal to @p threshold
 */
uint16_t FilterByScoreSingleClassExclBG_Float( const float * scores, int16_t n,
											const float threshold,
											uint16_t * keep_indices,
											uint16_t max_keep)
{
	// values[n, 2]: [nBoxes, bg or person]
	int16_t	count = 0;  // # of valid data
	int16_t jj = 1;     // index to person's conf-value

	for ( int16_t i = 0; i < n; i++, jj += 2 ) {
		if ( scores[jj] >= threshold ) {
			keep_indices[count] = i;
			count++;
			if (count >= max_keep) {
				return count;
			}
		}
	}

	return count;
}

/**
 * @brief filter detections based on confidence threshold (for a single float array)
 *
 * function returns indicies of from @p values array which are greater or equal to @p threshold
 * *
 * @param scores - pointer to float array contiaining confidences for and detected class in each 'row' [n, 1 (score)]. This array is flattened (1D)
 * @param n number of rows in @p values array
 * @param threshold decision threshold
 * @param[out] keep_indices array where indicies of scores above the specified threshold will be written
 * @param[in] max_keep maximum number of scores to select 
 * @return @c int Numer of found predictions with confidence of grater or equal to @p threshold
 */
uint16_t FilterByScoreSingleClass_Float( const float * scores, int16_t n,
											const float threshold,
											uint16_t * keep_indices,
											uint16_t max_keep)
{
	// values[n, 2]: [nBoxes, bg or person]
	int16_t	count = 0;  // # of valid data

	for ( int16_t i = 0; i < n; i++) {
		if ( scores[i] >= threshold ) {
			keep_indices[count] = i;
			count++;
			if (count >= max_keep) {
				return count;
			}
		}
	}

	return count;
}

// uint16_t SelectDetectionsAboveScoreThresholdByte( const uint8_t * scores, int16_t n,
// 											const uint8_t threshold,
// 											uint16_t * keep_indices,
// 											uint16_t max_keep)
// {
// 	// values[n, 2]: [nBoxes, bg or person]
// 	int16_t	count = 0;  // # of valid data
// 	int16_t jj = 1;     // index to person's conf-value

// 	for ( int16_t i = 0; i < n; i++, jj += 2 ) {
// 		if ( scores[jj] >= threshold ) {
// 			keep_indices[count] = i;
// 			count++;
// 			if (count >= max_keep) {
// 				return count;
// 			}
// 		}
// 	}

// 	return count;
// }

int16_t SelectDetectionsAboveScoreThresholdConsideringBG( const float * scores, int16_t n,
															const float threshold,
															float * keep_values,
															int16_t * keep_indices )
{
	//printf( "\nSelectDetectionsAboveScoreThresholdConsideringBG\n" );

	// values[n, 2]: [nBoxes, bg or person]
	int16_t	count = 0;  // # of valid data
	int16_t jj = 1;     // index to person's conf-value

	for ( int16_t i = 0; i < n; i++, jj += 2 ) {
		//printf( "%4d: %3d %3d", i, values[jj - 1], values[jj] );
		//if ( (values[jj - 1] >> 1) <= values[jj]                          // background conf * 4/8 <= person conf
		if ( (scores[jj - 1] / 4) + (scores[jj - 1] / 8) <= scores[jj]    // background conf * 3/8 <= person conf
			  &&  scores[jj] >= threshold ) {
			keep_values[count] = scores[jj];
			keep_indices[count] = i;
			count++;
			//printf( "  %4d", count );
		}
		//printf( "\n" );
	}

	return count;
}

/*
 * compute similarity between box_i and box_j by their aspect ratio and area ratio
 * input: bboxes in float (corner format)
*/
float ComputeSimilarity( const BoxCornerEncodingFloat* box_i, const BoxCornerEncodingFloat* box_j )
{
	float wi_hj = ( (*box_i).xmax - (*box_i).xmin ) * ( (*box_j).ymax - (*box_j).ymin );
	float wj_hi = ( (*box_j).xmax - (*box_j).xmin ) * ( (*box_i).ymax - (*box_i).ymin );
	float aspect_ratio = wi_hj <= wj_hi ? wi_hj/wj_hi : wj_hi/wi_hj;
	float area_i = areaBox( box_i );
	float area_j = areaBox( box_j );
	float area_ratio = area_i <= area_j ? area_i/area_j : area_j/area_i; 

	return area_ratio * aspect_ratio;
}


/*
 * compute Intersection-Over-Union between box_i and box_j
 * input: bboxes in float format
 * return: intersection / union in Qx.14
*/
float ComputeIoU( const BoxCornerEncodingFloat* box_i, const BoxCornerEncodingFloat* box_j )
{
	const float   intersection_ymin = max(box_i->ymin, box_j->ymin);
	const float   intersection_xmin = max(box_i->xmin, box_j->xmin);
	const float   intersection_ymax = min(box_i->ymax, box_j->ymax);
	const float   intersection_xmax = min(box_i->xmax, box_j->xmax);
	const float   area_intersection = max(intersection_ymax - intersection_ymin, 0 ) *
										max(intersection_xmax - intersection_xmin, 0 );
	const float   area_i = areaBox( box_i );
	const float   area_j = areaBox( box_j );
	const float   area_union = area_i + area_j - area_intersection;

	// return area_intersection < area_union ? area_intersection/area_union : (1 << 14);  // Qx.14  // this condition should be true by definition
	return area_intersection/area_union;
}

uint16_t hard_nms_single_class(  const BoxCornerEncodingFloat* bboxes, // all bboxes [MAX_NUM_BOXES]
											float* scores, // all scores of valid boxes: pointer to float array contiaining confidence background and backgrond or detected class in each 'row' [MAX_NUM_BOXES, 2 (bg or person)]
											uint16_t* kept_indicies, // indicies of valid boxes
											uint16_t kept_N, // number of valid detections (indicies/scores)
											uint16_t* kept_indicies_out,
											float threshold_IoU, // max IoU allowed
											uint8_t first_background, // if true "first column will be omitted"
											hard_nms_t* hard_nms_obj) 
{
	// get hold off hard_nms variables
	float* kept_scores = hard_nms_obj->kept_scores;
	int max_to_keep = hard_nms_obj->num_scores;

	debug("hardNMS : kept_N=%d, IoU_threshold=0.%02d, MAX_TO_KEEP=%d\r\n", kept_N, (int)(100*threshold_IoU), max_to_keep);

	kept_N = MIN(NMS_MAX_KEEP, kept_N);
	if(first_background){
		// copy only actual class scores, skiping background
		for(uint16_t i=0; i<kept_N; i++)
			kept_scores[i] = scores[2*kept_indicies[i]+1];
	} else {
		// since we don't have a background class - copy starightaway
		for(uint16_t i=0; i<kept_N; i++)
			kept_scores[i] = scores[kept_indicies[i]];
			// kept_scores[i] = scores[kept_indicies[i]];
	}

	for ( int i = 0; i < kept_N; i++ ){
		debug( "score[%d]=0.%02d\r\n", kept_indicies[i], (int)(100*kept_scores[i]) );
	}

	// output no more than MAX_TO_KEEP detections
	const uint16_t	max_dets = min( max_to_keep , kept_N );

	uint16_t i = 0;
	for ( ; i < max_dets; i++ ) {

		// find the largest score in score array
		float maxV = 0;
		int16_t maxI = -1;
		for ( int16_t j = 0; j < kept_N; j++ ){
			if ( maxV < kept_scores[j] ) {
				maxV = kept_scores[j];
				maxI = j;
			}
		}
		if ( maxI < 0 )
			break;

		//debug( "max_score %f at idx %d\r\n", maxV, maxI );
		kept_indicies_out[i] = kept_indicies[maxI]; // now we have the index of box that will be preserved.
		kept_scores[maxI] = 0; // we set it's score to 0, not to consider it in next iteration

		// getting hold of current box
		const BoxCornerEncodingFloat* bbox = &bboxes[kept_indicies[maxI]];

		for ( uint16_t j = 0; j < kept_N; ++j ) {
			// skip boxes which were already processed
			if(kept_scores[j] == 0)
				continue;

			uint16_t bbox_j = kept_indicies[j];
			float iou = ComputeIoU( bbox, (BoxCornerEncodingFloat *)(bboxes+bbox_j) );  // Qx.14

			if ( iou >= threshold_IoU )	// overlapped
				kept_scores[j] = 0; // eliminate ovelapping box from future consideration

		}
	}

	return i;
}

#ifdef __cplusplus
}
#endif
