/************************************************************************/
/* CFFI interface functions                                              */
/************************************************************************/

#include "vec_types.h"
#include "tracking.h"
#include "pipeline.h"
#include "assertion.h"
#include "debug.h"
#include "../../../include/project_defines.h"

// add externs
extern volatile object_tracking_t _track_obj;

static BoxCornerEncodingFloat * last_bboxes_in = 0;
static float * last_scores_in = 0;

/* outputs kalmans states and object ids into profided arrays */
int cffi_get_current_objects(int* uids, float* x, float* y) {
    kalman_tracker* trackers = _track_obj.trackers;
    int max_objects = _track_obj.max_detections;
    int count = 0;
    for (int i = 0; i < max_objects; i++) {
        if (trackers[i].active) {
            vec2f pos = trackers[i].last_pos;
            uids[count] = trackers[i].uid;
            x[count] = pos.x;
            y[count] = pos.y;
            count++;
        }
    }
    return count;
}

/* kalman predictions */
int cffi_get_projected_objects(int* uids, float* x, float* y) {
    kalman_tracker* trackers = _track_obj.trackers;
    int max_objects = _track_obj.max_detections;
    int count = 0;
    for (int i = 0; i < max_objects; i++) {
        if (trackers[i].active) {
            vec2f pos = get_position_from_state(trackers[i].kf_ptr);
            uids[count] = trackers[i].uid;
            x[count] = pos.x;
            y[count] = pos.y;
            count++;
        }
    }
    return count;
}

extern uint16_t nms_kept_indicies[];
extern uint16_t nms_kept_n;
extern float _bboxes[];
extern BoxCornerEncodingFloat* float_bboxes;

/* output boxes indicies from postprocessing */
int cffi_get_postprocessed_predictions(float* bboxes, float* scores, int* kept_indicies) {

    if(last_bboxes_in == 0)
        return 0;

    if(last_scores_in == 0)
        return 0;

    debug("getting postprocessed\r\n");

    // copy bounding boxes
    for(int i=0; i<nms_kept_n; i++){
        // copy bbox
        BoxCornerEncodingFloat bbox = last_bboxes_in[nms_kept_indicies[i]];
        *bboxes = bbox.xmin;
        bboxes++;
        *bboxes = bbox.ymin;
        bboxes++;
        *bboxes = bbox.xmax;
        bboxes++;
        *bboxes = bbox.ymax;
        bboxes++;

        // copy score
        *scores++ = last_scores_in[nms_kept_indicies[i]];

        // copy index
        kept_indicies[i] = nms_kept_indicies[i];
    }

    // return numer of predictions
    return nms_kept_n;
}

  
static vec2i counts_accumulator = {0, 0};

pipeline_config_t pipeline_config = {0};

static pipeline_config_t* pipeline_config_ptr;

void cffi_pipeline_init(
    // pipeline_config_t cfg
	uint8_t mdl_bboxes_first,
	uint8_t mdl_input_w,
	uint8_t mdl_input_h,
    float mdl_out0_scale,
    float mdl_out0_offset,
    float mdl_out1_scale,
    float mdl_out1_offset,

	float pp_score_threshold,
	float pp_score_prefilter_threshold,
	float pp_iou_threshold,
    float pp_roi_xmin,
    float pp_roi_xmax,
	float pp_roi_ymin,
	float pp_roi_ymax,
    float* pp_sumred_vect,
    uint8_t pp_sumred_vect_n,
    uint8_t pp_filter_select_class_id,

	uint8_t trk_kalman_max_missed,
	float trk_association_cost_limit,
	float trk_kalman_state_variance,
	float trk_kalman_measurement_variance,
	float trk_kalman_time_constant,

    float* cnt_pts, // points which constitute couting lines
	float* cnt_offsets, // offset for each of the counting lines
	float* cnt_hysteresis, // hysteresis ...
	uint8_t cnt_pts_n, // number of point in line
	uint8_t cnt_lines_n // number of lines
) {

    // pipeline_config_t
    pipeline_config.mdl_bboxes_first = mdl_bboxes_first;
    pipeline_config.mdl_input_w = mdl_input_w;
    pipeline_config.mdl_input_h = mdl_input_h;
    pipeline_config.mdl_out0_scale = mdl_out0_scale; // new
    pipeline_config.mdl_out0_offset = mdl_out0_offset; // new
    pipeline_config.mdl_out1_scale = mdl_out1_scale; // new
    pipeline_config.mdl_out1_offset = mdl_out1_offset; // new

    pipeline_config.pp_score_threshold = pp_score_threshold;
    pipeline_config.pp_score_prefilter_threshold = pp_score_prefilter_threshold;
    pipeline_config.pp_iou_threshold = pp_iou_threshold;
    pipeline_config.pp_roi_xmin = pp_roi_xmin;
    pipeline_config.pp_roi_xmax = pp_roi_xmax;
	pipeline_config.pp_roi_ymin = pp_roi_ymin;
	pipeline_config.pp_roi_ymax = pp_roi_ymax;
    for(int i=0; i<pp_sumred_vect_n; i++)
        pipeline_config.pp_sumred_vect[i] = pp_sumred_vect[i];
    pipeline_config.pp_sumred_vect_n = pp_sumred_vect_n;
    pipeline_config.pp_filter_select_class_id = pp_filter_select_class_id;

    pipeline_config.trk_kalman_max_missed = trk_kalman_max_missed;
    pipeline_config.trk_association_cost_limit = trk_association_cost_limit;
    pipeline_config.trk_kalman_state_variance = trk_kalman_state_variance;
    pipeline_config.trk_kalman_measurement_variance = trk_kalman_measurement_variance;
    pipeline_config.trk_kalman_time_constant = trk_kalman_time_constant;

    for(int i=0; i<cnt_pts_n; i++)
        pipeline_config.cnt_pts[i] = cnt_pts[i];
    for(int i=0; i<cnt_lines_n; i++){
        pipeline_config.cnt_hysteresis[i] = cnt_hysteresis[i];
        pipeline_config.cnt_offsets[i] = cnt_offsets[i];
    }
    pipeline_config.cnt_pts_n = cnt_pts_n;
    pipeline_config.cnt_lines_n = cnt_lines_n;

    debug("\nModel config\n");
    debug("mdl_bboxes_first: %d, mdl_input_w: %d, mdl_input_h: %d\n", pipeline_config.mdl_bboxes_first, pipeline_config.mdl_input_w, pipeline_config.mdl_input_h);
    debug("mdl_out0_scale: %f, mdl_out0_offset %f\n", pipeline_config.mdl_out0_scale,  pipeline_config.mdl_out0_offset);
    debug("mdl_out1_scale: %f, mdl_out1_offset %f\n", pipeline_config.mdl_out1_scale,  pipeline_config.mdl_out1_offset);
    
    debug("\nPostprocessing config\n");
    debug("pp_score_prefilter_threshold: %f, pp_score_threshold: %f, pp_iou_threshold: %f\n", pipeline_config.pp_score_prefilter_threshold, pipeline_config.pp_score_threshold, pipeline_config.pp_iou_threshold);
    debug("pp_roi_xmin: %f, pp_roi_xmax %f, pp_roi_ymin: %f, pp_roi_ymax %f\n", pipeline_config.pp_roi_xmin, pipeline_config.pp_roi_xmax, pipeline_config.pp_roi_ymin, pipeline_config.pp_roi_ymax);
    debug("pp_sumred_vect: [");
    for(int i=0; i<pipeline_config.pp_sumred_vect_n; i++)
        debug("%f, ", pipeline_config.pp_sumred_vect[i]);
    debug("\b]\n");
    debug("pp_sumred_vect_n: %d\n", pipeline_config.pp_sumred_vect_n);
    debug("pp_filter_select_class_id: %d\n", pipeline_config.pp_filter_select_class_id);

    debug("\nTracking config\n");
    debug("trk_kalman_max_missed: %d, trk_association_cost_limit: %f\n", pipeline_config.trk_kalman_max_missed, pipeline_config.trk_association_cost_limit);
    debug("trk_kalman_state_variance: %f, trk_kalman_measurement_variance: %f, trk_kalman_time_constant: %f\n", 
        pipeline_config.trk_kalman_state_variance, pipeline_config.trk_kalman_measurement_variance, pipeline_config.trk_kalman_time_constant);

    debug("\nCounting config\n");
    debug("cnt_pts: [");
    for(int i=0; i<pipeline_config.cnt_pts_n; i++)
        debug("%f, ", pipeline_config.cnt_pts[i]);
    debug("\b]\n");
    debug("\nCounting hysteresis\n");
    debug("cnt_hysteresis: [");
    for(int i=0; i<pipeline_config.cnt_lines_n; i++){
        debug("%f, ", pipeline_config.cnt_hysteresis[i]);
    }
    debug("\b]\n");
    debug("\nCounting offsets\n");
    debug("cnt_offsets: [");
    for(int i=0; i<pipeline_config.cnt_lines_n; i++){
        debug("%f, ", pipeline_config.cnt_offsets[i]);
    }
    debug("\b]\n");
    debug("\n");

    // pipeline_config = cfg;
    counts_accumulator.v[0] = 0;
    counts_accumulator.v[1] = 0;

    pipeline_config_ptr = &pipeline_config;
    pipeline_init(pipeline_config_ptr);
}

void cffi_pipeline_run(unsigned char* bboxes_in, unsigned char* scores_in, int N, unsigned char n_classes, int* count_up_out, int* count_down_out) {
    last_bboxes_in = get_bboxes_ptr();
    last_scores_in = get_scores_ptr();

    pipeline_quant_in_t in;

    in.scores.data_ptr = scores_in;
    in.scores.n_preds = N;
    in.scores.n_cols = n_classes;

    in.bboxes.data_ptr = bboxes_in;
    in.bboxes.n_preds = N;
    in.bboxes.n_cols = 4;

    if(pipeline_config_ptr->mdl_bboxes_first){
        in.scores.quant_scale = pipeline_config_ptr->mdl_out1_scale;
        in.scores.quant_offset = pipeline_config_ptr->mdl_out1_offset;
        in.bboxes.quant_scale = pipeline_config_ptr->mdl_out0_scale;
        in.bboxes.quant_offset = pipeline_config_ptr->mdl_out0_offset;
    } else {
        in.scores.quant_scale = pipeline_config_ptr->mdl_out0_scale;
        in.scores.quant_offset = pipeline_config_ptr->mdl_out0_offset;
        in.bboxes.quant_scale = pipeline_config_ptr->mdl_out1_scale;
        in.bboxes.quant_offset = pipeline_config_ptr->mdl_out1_offset;
    }
    
    pipeline_out_t out = { 0, 0, 0, 0 };
    pipeline_run(&in, &out, pipeline_config_ptr);

#if TRACKING_ACCUMULATE_COUNTS == 1
// if tracking accumulate counts internally, pass through the values
    counts_accumulator.v[0] = out.count_up;
    counts_accumulator.v[1] = out.count_down;
#else
// if tracking given instantenous counts, accumulate the values
    counts_accumulator.v[0] += out.count_up;
    counts_accumulator.v[1] += out.count_down;
#endif

    *count_up_out = counts_accumulator.v[0];
    *count_down_out = counts_accumulator.v[1];

    debug("Accumulated counts: UP=%d, DOWN=%d, detected=%d, tracked=%d\r\n", *count_up_out, *count_down_out, out.n_detected, out.n_tracked);
}

int cffi_get_input_height(){
    return pipeline_config_ptr->mdl_input_h;
}

int cffi_get_input_width(){
    return pipeline_config_ptr->mdl_input_w;
}

void unittest(){
	// empty for now
}

