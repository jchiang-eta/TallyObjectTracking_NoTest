#ifndef SSD_DETECTION_H
#define SSD_DETECTION_H

// #include "nms_float_def.h"

#ifndef STD_LIB
#include <stdint.h>
#endif  // STD_LIB

# define STD_LIB
// #define ETA_SDK
#define HARD_NMS

#define FLOAT_NMS_MAX_TO_KEEP 50
#ifdef STD_LIB
float __SSAT(int32_t operand, uint8_t shift);
#endif

#ifdef __cplusplus
extern    "C"
{
#endif

#ifndef _NMS_FLOAT_DEF_H
#define _NMS_FLOAT_DEF_H

	typedef enum
	{
		ETA_STATUS_OK = 0,        /**< No error */
		ETA_GEN_ERR = -1,        /**< Error in the compute of the function */
	} eta_rc;

#include "bbox_types.h"

typedef struct {
	float* kept_scores;
	int num_scores;
} hard_nms_t;


float ComputeSimilarity(const BoxCornerEncodingFloat* box_i, const BoxCornerEncodingFloat* box_j);
float ComputeIoU(const BoxCornerEncodingFloat* box_i, const BoxCornerEncodingFloat* box_j);

/**
 * @brief filter detection based on confidence threshold
 *
 * function returns indicies of from @p scores array which meet the condition:
 * 		dot(S,mask') > @p threshold, where S is a row from @p scores
 * *
 * @param[in] scores pointer to float array contiaining confidences for and detected class in each 'row' [n, bg or person]. This arrauy is flattened (1D)
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
											uint8_t n_classes);

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
uint16_t FilterByScoreSingleClassExclBG_Float(const float* scores, int16_t n,
	const float threshold,
	//float * keep_values,
	uint16_t* keep_indices,
	uint16_t max_keep);

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
											uint16_t max_keep);

uint16_t hard_nms_single_class(const BoxCornerEncodingFloat* bboxes, // all bboxes [MAX_NUM_BOXES]
	float* scores, // all scores of valid boxes: pointer to float array contiaining confidence background and backgrond or detected class in each 'row' [MAX_NUM_BOXES, 2 (bg or person)]
	uint16_t* kept_indicies, // indicies of valid boxes
	uint16_t kept_N, // number of valid detections (indicies/scores)
	uint16_t* kept_indicies_out,
	float threshold_IoU, // max IoU allowed;
	uint8_t first_background, // if true "first column will be omitted"
	hard_nms_t* hard_nms_obj 
);

#endif


#ifdef __cplusplus
}
#endif

#endif  // SSD_DETECTION_H
