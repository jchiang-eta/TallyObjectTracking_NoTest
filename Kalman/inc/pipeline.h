#ifndef _PIPELINE_H
#define _PIPELINE_H

#include <stdint.h>

#include "bbox_types.h"
#include "vec_types.h"
#include "pipeline_config.h"

/*
	Type definitions
*/

typedef struct {
	int8_t* data_ptr;
	uint16_t n_preds;
	uint16_t n_cols;
	float quant_scale;
	float quant_offset;
} model_output_t;

typedef struct {
	int count_up;
	int count_down;
	uint16_t n_detected; // number of objects detected in last frame
	uint16_t n_tracked; // numer of currently tracked objects
} pipeline_out_t;

typedef struct {
	float mdl_out0_scale;
    float mdl_out0_offset;
    float mdl_out1_scale;
    float mdl_out1_offset;
	uint8_t mdl_bboxes_first;
	uint8_t mdl_input_w;
	uint8_t mdl_input_h;
	uint8_t mdl_pad[1];
	
	float pp_score_threshold;
	float pp_score_prefilter_threshold;
	float pp_iou_threshold;
	float pp_roi_xmin;
	float pp_roi_xmax;
	float pp_roi_ymin;
	float pp_roi_ymax;
    float pp_sumred_vect[4];
    uint8_t pp_sumred_vect_n;
	uint8_t pp_filter_select_class_id;
	uint8_t pp_pad[2];

	float trk_association_cost_limit;
	float trk_kalman_state_variance;
	float trk_kalman_measurement_variance;
	float trk_kalman_time_constant;
	uint8_t trk_kalman_max_missed;
	uint8_t trk_pad[3];

	float cnt_pts[6]; // points which constitute couting lines
	float cnt_offsets[6]; // offset for each of the counting lines
	float cnt_hysteresis[6]; // hysteresis ...
	uint8_t cnt_pts_n; // number of point in line
	uint8_t cnt_lines_n; // number of lines
	uint8_t cnt_pad[2];

} pipeline_config_t;

typedef struct {
	model_output_t scores;
	model_output_t bboxes;
} pipeline_quant_in_t;

/*
	Function declarations
*/

void pipeline_init(pipeline_config_t* config);

void pipeline_run(pipeline_quant_in_t* in, pipeline_out_t* out, pipeline_config_t* config);

// should be called to clear all tracked objects (when tracking "goes to sleep")
void pipeline_idle(pipeline_config_t* config);

// helper used by cffi
BoxCornerEncodingFloat* get_bboxes_ptr();
float* get_scores_ptr();

#endif // _PIPELINE_H