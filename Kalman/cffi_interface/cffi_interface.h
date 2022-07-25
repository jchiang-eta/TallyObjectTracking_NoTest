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
);

void cffi_pipeline_run(unsigned char* bboxes_in, unsigned char* scores_in, int N, unsigned char n_classes, int* count_up_out, int* count_down_out);

int cffi_get_postprocessed_predictions(float* bboxes, float* scores, int* kept_indicies);

int cffi_get_current_objects(int* uids, float* x, float* y);

int cffi_get_projected_objects(int* uids, float* x, float* y);

int cffi_get_input_height();

int cffi_get_input_width();		

void unittest();