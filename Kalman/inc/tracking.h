#ifndef _TRACKING_H_
#define _TRACKING_H_

#include "kalman.h"
#include "vec_types.h"
#include "bbox_types.h"
#include "pipeline_config.h"
#include "hungarian.h"
#include "counting.h"

typedef struct {
    kalman_t* kf_ptr;
    kalman_measurement_t* kfm_ptr;
    uint8_t active;
	uint8_t missed;

    vec2f start_point;
    int iterator;
    uint8_t uid;
    
    vec2f last_pos;
} kalman_tracker;

typedef struct {
    float state_variance;
    float measurement_variance;
    float time_constant;
} kalman_cfg_t;

typedef struct {
    uint8_t max_missed;
    float roi_min_x;
    float roi_max_x;
    float roi_min_y;
    float roi_max_y;
    int match_cost_limit;
    uint16_t width;
    uint16_t height;
    kalman_cfg_t kalman_cfg;
    float* cnt_pts;
    float* cnt_hysteresis;
    float* cnt_offsets;
    uint8_t cnt_pts_n;
    uint8_t cnt_lines_n;
} object_tracking_cfg_t;

// container for data (arrays) used in computation
typedef struct {
    vec2ui8* matches;
    vec2f* objs_position;
    int* costs;
    int8_t* matched_kf; // this array keeps flags on whether given kalman had a matching detection
    int8_t* active_trackers_idx;
    kalman_tracker* trackers;
    uint8_t kalman_num_filter;
    uint8_t max_detections;
    uint8_t uid_counter;
    matrix_data_t* cost_matrix_buffer;
    matrix_t cost_matrix;
    vec2i counts;
    hungarian_t* hungarian_obj;
    counting_t* counting_obj;
    uint8_t counting_obj_n;
    object_tracking_cfg_t cfg;
} object_tracking_t;

#define KALMAN_NAME trackers
#define KALMAN_NUM_STATES 4
#define KALMAN_NUM_INPUTS 0
#define KALMAN_MEASUREMENT_NAME position
#define KALMAN_NUM_MEASUREMENTS 2
#define KALMAN_SHARE_MATRICES

int tracker_num_active(object_tracking_t* track_obj);
int tracker_num_free(object_tracking_t* track_obj);

// initializes tracking (filters, counting etc.)
int object_tracking_init(object_tracking_t* track_obj);

// runs tracking
vec2i object_tracking(BoxCornerEncodingFloat* bboxes, uint16_t* kept_indicies, uint16_t N, object_tracking_t* track_obj);
// clears any objects which are active
void object_tracking_clear(object_tracking_t* track_obj);

void tracker_reinit(kalman_tracker* tracker, vec2f* pos, vec2f* vel, uint8_t uid);
int tracker_alloc(object_tracking_t* track_obj);
void tracker_free(int i, object_tracking_t* track_obj);

void bboxcorner2centroid(BoxCornerEncodingFloat* bbox, vec2f* out);

// cost function computes sqrted distance between box centers and centroid cx, cy. If cost is higher that limit outputs a very large number
float cost_function(BoxCornerEncodingFloat* bbox, vec2f* pred, int cost_limit);
void counting_func(kalman_tracker* kf, vec2i* counts, object_tracking_t* track_obj);
vec2f get_position_from_state(kalman_t* kf);

int filter_matches(vec2ui8* matchings, int* costs, int n_matchings, int cost_limit);

// ------------ DEBUGGING FUNCTIONS ---------------------

void debug_tracker(kalman_tracker* tracker, int idx);
void debug_trackers(object_tracking_t* track_obj);

#endif // _TRACKING_H_
