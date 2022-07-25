#include <stdint.h>

// #include "../../../include/input_data.h"
#include "pipeline_config.h"

#include "pipeline.h"
#include "hungarian.h"
#include "tracking.h"
#include "nms_float.h"
#include "quant_utils.h"
#include "nms_byte.h"

#include "debug.h"
#include "assertion.h"

#if PIPELINE_VARIANT == PIPELINE_QUANT
#pragma message("Using PIPELINE_QUANT variant.")

/************************************************************************/
/* Global static vars for pipeline                                      */
/************************************************************************/

/*****************************   KALMAN   *******************************/
// #include "kalman_cleanup_factory.h"
#define KALMAN_NUM_FILTERS MAX_OBJECTS
#include "kalman_filters_factory.h"
#include "kalman_measurements_factory.h"

#define MAX_SIZE_ROW MAX(MAX_OBJECTS, MAX_DETECTIONS)
#define MAX_SIZE_COL MAX(MAX_OBJECTS, MAX_DETECTIONS)

static int8_t _col_mate[MAX_SIZE_ROW];
static int8_t _row_mate[MAX_SIZE_COL];
static int8_t _parent_row[MAX_SIZE_COL];
static int8_t _unchosen_row[MAX_SIZE_ROW];
static int _row_dec[MAX_SIZE_ROW];
static int _col_inc[MAX_SIZE_COL];
static int _slack[MAX_SIZE_COL];
static int8_t _slack_row[MAX_SIZE_COL];
static int _hun_costmat[MAX_SIZE_ROW * MAX_SIZE_COL];

// NOTE: we have a weird approach to response to a weird problem
// the problem comes from the fact that size of hungarian function changes as a function of 
// MAX_SIZE_ROW and MAX_SIZE_COL
hungarian_t _hun = {
	_col_mate,
	_row_mate,
	_parent_row,
	_unchosen_row,
	_row_dec,
	_col_inc,
	_slack,
	_slack_row,
	_hun_costmat,
	MAX_SIZE_ROW,
	MAX_SIZE_COL
};
   
static int8_t _active_trackers_idx[MAX_OBJECTS]; // array to keep indicies of active trackers/filter
static vec2ui8 _matches[N_MATCHES];
static vec2f _objs_position[MAX_OBJECTS];
static int _costs[N_MATCHES];
static int8_t _matched_kf[MAX_OBJECTS]; // this array keeps flags on whether given kalman had a matching detection
static kalman_tracker _trackers[MAX_OBJECTS];
static matrix_data_t _cost_matrix_buffer[MAX_DETECTIONS * MAX_OBJECTS];
static counting_t _counting_obj[MAX_COUNT_LINES];

object_tracking_t _track_obj = {
    _matches,
    _objs_position,
    _costs,
    _matched_kf,
    _active_trackers_idx,
    _trackers,
    MAX_OBJECTS,
    MAX_DETECTIONS,
    0,
    _cost_matrix_buffer,
    {0}, 					// cost_matrix
    {0,0}, 					// counts
    &_hun, 					// hungarian object
	_counting_obj, 			// counting_obj
	6, 						// counting_obj_n
	{0}						// cfg
};

// keep track of supressed variables
uint16_t nms_kept_indicies[MAX_INDICIES];
uint16_t kept_indicies_tmp[MAX(MAX_DEQUANT,MAX_INDICIES)];
uint16_t nms_kept_n;

// hard nms vars NMS
static float _nms_kept_scores[NMS_MAX_KEEP];
static hard_nms_t _hard_nms = {
	_nms_kept_scores,
	NMS_MAX_KEEP
};

// for dequantized variables (we are conservative - take larger number, 4 rather than 2)
static float float_scores[4*MAX_DEQUANT];
static float float_bboxes[4*MAX_DEQUANT];

BoxCornerEncodingFloat* get_bboxes_ptr(){
	return (BoxCornerEncodingFloat*)float_bboxes;
}

float* get_scores_ptr(){
	return float_scores;
}

/*
	Function definitions
*/
void pipeline_init(pipeline_config_t* config) {

	object_tracking_cfg_t* cfg = &_track_obj.cfg;

	kalman_cfg_t* kcfg = &cfg->kalman_cfg;
	kcfg->state_variance = config->trk_kalman_state_variance;
	kcfg->measurement_variance = config->trk_kalman_measurement_variance;
	kcfg->time_constant = config->trk_kalman_time_constant;

	cfg->width = config->mdl_input_w;
	cfg->height = config->mdl_input_h;
	cfg->cnt_pts = config->cnt_pts;
	cfg->cnt_offsets = config->cnt_offsets;
	cfg->cnt_hysteresis = config->cnt_hysteresis;
	cfg->cnt_pts_n = config->cnt_pts_n;
	cfg->cnt_lines_n = config->cnt_lines_n;

	cfg->max_missed = config->trk_kalman_max_missed;
	cfg->roi_min_x = config->mdl_input_w*config->pp_roi_xmin;
   	cfg->roi_max_x = config->mdl_input_w*config->pp_roi_xmax;
   	cfg->roi_min_y = config->mdl_input_h*config->pp_roi_ymin;
   	cfg->roi_max_y = config->mdl_input_h*config->pp_roi_ymax;
	cfg->match_cost_limit = (int)(config->trk_association_cost_limit*config->trk_association_cost_limit*config->mdl_input_w*config->mdl_input_h);

	debug("roi_x: [%.1f, %.1f], roi_y: [%.1f, %.1f]", cfg->roi_min_x, cfg->roi_max_x, cfg->roi_min_y, cfg->roi_max_y);

	object_tracking_init(&_track_obj);
}

void pipeline_idle(pipeline_config_t* config){
	object_tracking_clear(&_track_obj);
}

static int iterator = 0;

void pipeline_run(pipeline_quant_in_t* in, pipeline_out_t* out, pipeline_config_t* config){

	uint8_t input_w = config->mdl_input_w; // IN0_SIZE_DIM1
	uint8_t input_h = config->mdl_input_h; // IN0_SIZE_DIM0

	//assert(in->scores.n_cols == N_CLASSES);

	debug("\r\n================= Running Pipeline: %d ===============\r\n\r\n", iterator++);

	model_output_t* scores_output = &in->scores;
	model_output_t* bboxes_output = &in->bboxes;

	uint8_t N_classes = (uint8_t)scores_output->n_cols;
	debug("N classes: %d\r\n", N_classes);

	debug("Quant scores: s=%f, o=%f\r\n", scores_output->quant_scale, scores_output->quant_offset);
	debug("Scores size: %d, %d\r\n", scores_output->n_preds, scores_output->n_cols);
	debug("Quant bboxes: s=%f, o=%f\r\n", bboxes_output->quant_scale, bboxes_output->quant_offset);
	debug("Bboxes size: %d, %d\r\n", bboxes_output->n_preds, bboxes_output->n_cols);
	
	const int _FIRST_N = 5;
	debug("\r\nScores (first %d)\r\n", _FIRST_N);
	const int8_t* scores_ptr = scores_output->data_ptr;
	for(int i=0; i<MIN(scores_output->n_preds, _FIRST_N); i++){
		for(int j=0; j<scores_output->n_cols; j++){
			debug("%d,", *scores_ptr);
			scores_ptr++;
		}
		debug("\r\n");
	}

	debug("\r\nBboxes (first %d)\r\n", _FIRST_N);
	const int8_t* bboxes_ptr = bboxes_output->data_ptr;
	for(int i=0; i<MIN(bboxes_output->n_preds, _FIRST_N); i++){
		for(int j=0; j<bboxes_output->n_cols; j++){
			debug("%d,", *bboxes_ptr);
			bboxes_ptr++;
		}
		debug("\r\n");
	}

	// find quantized score and clamp its value to int8 range
	int16_t score_threshold = (int16_t)((float)config->pp_score_prefilter_threshold/(float)scores_output->quant_scale + (float)scores_output->quant_offset);
	score_threshold = score_threshold < -128 ? -128 : score_threshold;
	score_threshold = score_threshold >  127 ?  127 : score_threshold;

	debug("Quant score threshold: %d\r\n", score_threshold);

	int n_kept_temp = FilterOneClassByScoreMulticlass_Byte(
		scores_output->data_ptr,
		scores_output->n_preds, 
		(int8_t)score_threshold,
		kept_indicies_tmp, 
		MAX_DEQUANT,
		N_classes,
		//3 // id of the class to filter - 1-head, 2-upper_body, 3-person
		config->pp_filter_select_class_id // for 1 class model - always 1. TODO: add it as a parameter (e.g. argmax[weight_vect] => in python)
		// TODO <= this needs to be parametrized too
	);

	debug("Prefiltered by score: %d\r\n", n_kept_temp);

	debug("Dequant scores\r\n");
	// dequantize outputs
    dequantize_to_float_by_index(
		scores_output->data_ptr, float_scores, 
		kept_indicies_tmp, n_kept_temp, 
		N_classes,
		scores_output->quant_scale, scores_output->quant_offset
	);

	debug("Dequant bboxes\r\n");
    dequantize_to_float_by_index(
		bboxes_output->data_ptr, float_bboxes, 
		kept_indicies_tmp, n_kept_temp, 
		(uint8_t)bboxes_output->n_cols,
		bboxes_output->quant_scale, bboxes_output->quant_offset
	);

	debug("Dequant done.\r\n");

	// here kept_indicies_tmp is no longer needed. Boxes/scores are all in range <0, n_kept_temp)

	// #define BBOX_ENLARGEMENT 0.06f
	// BoxCornerEncodingFloat *bbox = (BoxCornerEncodingFloat *)float_bboxes;
	// for(int i=0; i<n_kept_temp; i++){
	// 	bbox[i].xmax += BBOX_ENLARGEMENT;
	// 	bbox[i].ymax += BBOX_ENLARGEMENT;
	// 	bbox[i].xmin -= BBOX_ENLARGEMENT;
	// 	bbox[i].ymin -= BBOX_ENLARGEMENT;
	// }

	//display scores
	debug("Dequant scores:");
	for(int i=0; i<n_kept_temp*N_classes; i++){
		if(i % N_classes == 0)
			debug("\t%d => ", i);
		debug("%f,", float_scores[i]);
		if((i+1) % N_classes == 0)
			debug("\r\n");
	}
	//display bboxes
	debug("Dequant bboxes:");
	for(int i=0; i<n_kept_temp; i++){
		BoxCornerEncodingFloat* b = (BoxCornerEncodingFloat*)float_bboxes;
		debug("\t%d => %.2f, %.2f, %.2f, %.2f\r\n", i, b[i].xmin, b[i].ymin, b[i].xmax, b[i].ymax);
	}

	n_kept_temp = FilterWeighedSumMulticlass_Float( 	
		float_scores, 
		n_kept_temp, // number of predictions (rows)
		config->pp_score_threshold,
		config->pp_sumred_vect,
		kept_indicies_tmp,
		MAX_INDICIES,
		N_classes
	);

	debug("Post FilterWeighedSumMulticlass_Float\r\n");
	for(int i=0; i<n_kept_temp; i++){
			// if model has bbox coordinate value range 0-1, scale it up to h, w
		BoxCornerEncodingFloat* bboxs = (BoxCornerEncodingFloat*)float_bboxes;
		for(int i = 0; i < nms_kept_n; i++){
			BoxCornerEncodingFloat bbox = bboxs[kept_indicies_tmp[i]];
			bbox.xmin *= input_w;
			bbox.ymin *= input_h;
			bbox.xmax *= input_w;
			bbox.ymax *= input_h;

			debug(
				"\t%d => b: %d, %d, %d, %d, s: 0.%02d\r\n", 
				kept_indicies_tmp[i], 
				(int)bbox.xmin, (int)bbox.ymin, (int)bbox.xmax, (int)bbox.ymax, 
				(int)(100*float_scores[kept_indicies_tmp[i]]) 
			);
		}
	}

	debug("Non max supression\r\n");
	nms_kept_n = hard_nms_single_class(
		(BoxCornerEncodingFloat*)float_bboxes, // all bboxes
		float_scores, // all scores: pointer to float array contiaining confidence background and backgrond or detected class in each 'row' [MAX_NUM_BOXES, 2 (bg or person)]
		kept_indicies_tmp, // indicies of valid boxes
		n_kept_temp, // number of valid detections (indicies/scores)
		nms_kept_indicies, // array to store selected indicies
		config->pp_iou_threshold, // max IoU allowed,
		0, // first class background?
		&_hard_nms
	 );
	 // internal MAX_TO_KEEP <- this should be linked to our global config somehow

	// TOP K - keep highest k-highest score indicies
	nms_kept_n = MIN(nms_kept_n, MAX_DETECTIONS);

	debug("NMS kept: %d\r\n", nms_kept_n);
	debug("Post NMS bboxes\r\n");
#if MODEL_OUTPUT_ABSOLUTE == 0
	// if model has bbox coordinate value range 0-1, scale it up to h, w
	BoxCornerEncodingFloat* bboxs = (BoxCornerEncodingFloat*)float_bboxes;
	for(int i = 0; i < nms_kept_n; i++){
		BoxCornerEncodingFloat* bbox = &bboxs[nms_kept_indicies[i]];
		bbox->xmin *= input_w;
		bbox->ymin *= input_h;
		bbox->xmax *= input_w;
		bbox->ymax *= input_h;

		debug("\t%d => b: %d, %d, %d, %d, s: 0.%02d\r\n", nms_kept_indicies[i], (int)bbox->xmin, (int)bbox->ymin, (int)bbox->xmax, (int)bbox->ymax, (int)(100*float_scores[nms_kept_indicies[i]]) );
	}
#endif

	debug("Running Object Tracking\r\n");

#endif

#if TRACKING_ACCUMULATE_COUNTS == 0
	_track_obj.counts.v[0] = 0;
	_track_obj.counts.v[1] = 0;
#endif

	vec2i counts = object_tracking(
		(BoxCornerEncodingFloat*)float_bboxes,
		nms_kept_indicies, 
		nms_kept_n, 
		&_track_obj
	);
	out->count_up = counts.v[0];
	out->count_down = counts.v[1];

	out->n_detected = nms_kept_n;
	out->n_tracked = tracker_num_active(&_track_obj);

    debug("Tracking counts: UP=%d, DOWN=%d\r\n", out->count_up, out->count_down);
	debug("====== Finished pipeline ======\r\n\r\n");
}