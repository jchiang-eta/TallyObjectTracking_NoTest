#include "tracking.h"
#include "debug.h"
#include "counting.h"
/************************************************************************/
/* Initialization functions                                             */
/************************************************************************/

extern kalman_measurement_t* kalman_filter_trackers_measurement_position_init();
extern kalman_t* kalman_filter_trackers_init();

void kalman_model_init(kalman_t* kf, kalman_measurement_t* kfm, kalman_cfg_t* kcfg) {
    // BUILD MODEL

    /************************************************************************/
    /* Time constant and variance setup                                     */
    /************************************************************************/
    const matrix_data_t T = kcfg->time_constant;
    matrix_data_t state_var = kcfg->state_variance; // Q - should state variance be the same for x and x' ?
    matrix_data_t measurement_var = kcfg->measurement_variance; //

    /************************************************************************/
    /* set initial state                                                    */
    /************************************************************************/
    matrix_t* x = kalman_get_state_vector(kf);
    x->data[0] = 0; // x
    x->data[1] = 0; // x'
    x->data[2] = 0; // y
    x->data[3] = 0; // y'

    /************************************************************************/
    /* set state transition                                                 */
    /************************************************************************/
    matrix_t* A = kalman_get_state_transition(kf);

    //# initalize Kalman matrices
    //    self.A = np.matrix([  [1, self.dt,    0, 0 ],
    //                          [0, 1,          0, 0],
    //                          [0, 0,          1, self.dt],
    //                          [0, 0,          0, 1]] )

    // transition in X direction
    matrix_set(A, 0, 0, 1);   // 1
    matrix_set(A, 0, 1, T);   // T
    matrix_set(A, 1, 1, 1);   // 1

    // transition in Y direction
    matrix_set(A, 2, 2, 1);   // 1
    matrix_set(A, 2, 3, T);   // T
    matrix_set(A, 3, 3, 1);   // 1

    /************************************************************************/
    /* set system covariance                                                */
    /************************************************************************/

    //self.P = np.matrix(self.stateVariance * np.identity(self.A.shape[0])) # 4x4

    matrix_t* P = kalman_get_system_covariance(kf);

    matrix_set(P, 0, 0, state_var);
    matrix_set(P, 1, 1, state_var);
    matrix_set(P, 2, 2, state_var);
    matrix_set(P, 3, 3, state_var);

    /************************************************************************/
    /* set process covariance covariance                                                 */
    /************************************************************************/

    //self.G = np.matrix([[1 / 2 * self.dt * *2, 0 ],
    //    [self.dt, 0],
    //    [0, 1 / 2 * self.dt * *2],
    //    [0, self.dt]] )
    //self.Q = self.G * self.G.T * self.stateVariance

    matrix_t* Q = kalman_get_input_covariance(kf);

    matrix_set(Q, 0, 0, (matrix_data_t)0.25 * T * T * T * T * state_var);
    matrix_set_symmetric(Q, 0, 1, (matrix_data_t)0.5 * T * T * state_var);
    matrix_set(Q, 1, 1, T * state_var);

    matrix_set(Q, 2, 2, (matrix_data_t)0.25 * T * T * T * T * state_var);
    matrix_set_symmetric(Q, 2, 3, (matrix_data_t)0.5 * T * T * state_var);
    matrix_set(Q, 3, 3, (matrix_data_t)T * state_var);

    /************************************************************************/
    /* set measurement transformation                                       */
    /************************************************************************/
    //self.H = np.matrix([  [1, 0, 0, 0],
    //                      [0, 0, 1, 0]] )
    matrix_t* H = kalman_get_measurement_transformation(kfm);

    matrix_set(H, 0, 0, 1); // x
    matrix_set(H, 1, 2, 1); // y

    /************************************************************************/
    /* set process noise                                                    */
    /************************************************************************/
    // self.R = np.matrix(self.measurementVariance * np.identity(self.H.shape[0])) # 2x2
    matrix_t* R = kalman_get_process_noise(kfm);

    matrix_set(R, 0, 0, measurement_var);
    matrix_set(R, 1, 1, measurement_var);

}

int object_tracking_init(object_tracking_t* track_obj)
{

    object_tracking_cfg_t* cfg = &track_obj->cfg;

    int status;

    // unpack local vars
    kalman_tracker* trackers = track_obj->trackers;
    uint8_t kalman_num_filter = track_obj->kalman_num_filter;
    vec2i* counts = &track_obj->counts;

    track_obj->uid_counter = 0;

    // reset counts
    counts->v[0] = 0;
    counts->v[1] = 0;

    /************************************************************************/
    /* initialize the filter structures                                     */
    /************************************************************************/

    kalman_t* kfs = kalman_filter_trackers_init(); // this is an array now
    kalman_measurement_t* kfms = kalman_filter_trackers_measurement_position_init(); // this is an array now
    kalman_cfg_t* kcfg = &cfg->kalman_cfg;

    for (int i = 0; i < kalman_num_filter; i++) {

        kalman_t* kf = &kfs[i];
        kalman_measurement_t* kfm = &kfms[i];

        kalman_model_init(kf, kfm, kcfg);

        // link kalman filters to trackers
        kalman_tracker* tracker = &trackers[i];

        tracker->kf_ptr = kf;
        tracker->kfm_ptr = kfm;
        tracker->active = 0;
        tracker->missed = 0;

        debug_tracker(tracker, i);

    }

    // init counting lines
    track_obj->counting_obj_n = cfg->cnt_lines_n;
    for (int i = 0; i < cfg->cnt_lines_n; i++) {
        status = cnt_npts_init(
            cfg->cnt_pts,
            cfg->cnt_pts_n,
            cfg->width, cfg->height,
            cfg->cnt_hysteresis[i], cfg->cnt_offsets[i],
            &track_obj->counting_obj[i]
        );
        if (status < 0) return status;
    }

    return 0;
}

vec2f get_position_from_state(kalman_t* kf) {
    vec2f pos;

    matrix_t* x = kalman_get_state_vector(kf);
    pos.x = x->data[0]; // x
    pos.y = x->data[2]; // y
    return pos;
}

// uint8_t centroid_out_of_scope(float x, float y, object_tracking_cfg_t* track_cfg) {
//     return (x < CANVAS_MAX_LIM_X*0.05) || (y < CANVAS_MAX_LIM_Y*0.075)
//         || (x > CANVAS_MAX_LIM_X*0.95) || (y > CANVAS_MAX_LIM_Y);
// }

uint8_t centroid_out_of_scope(float x, float y, object_tracking_cfg_t* track_cfg) {

    debug("obj_pos: [%.1f, %.1f]\n", x, y);
    debug("roi_x: [%.1f, %.1f], roi_y: [%.1f, %.1f]\n", track_cfg->roi_min_x, track_cfg->roi_max_x, track_cfg->roi_min_y, track_cfg->roi_max_y);
    return (x < track_cfg->roi_min_x) || (y < track_cfg->roi_min_y)
        || (x > track_cfg->roi_max_x) || (y > track_cfg->roi_max_y);
}

/*
 *	OBJECT TRACKING
 *	- make kalman predictions for all active kalman filters (existing from previous iterations)
 *	- compute cost matrix between current detections and predictions
 *	- calculate 
 *	- filter unreasonable matchings (e.g. very distant from each other)
 *	- For each matching: update kalman filters
 *			- add measurement to matching kalman filters and zero "missed" counter
 *			- if no matching found increment "missed" counter
 *			- if "missed" counter above threshold
 *				- apply counting function to indicate increment, decrement or no-action
 *				- deactivate filter (return to the "pool")
 *	- For each detection without matching:
 *			- Activate (new filter from the "pool") with centroid position as initial condition
 */
vec2i object_tracking(BoxCornerEncodingFloat* bboxes, uint16_t* kept_indicies, uint16_t N, object_tracking_t* track_obj) {

    vec2ui8* matches = track_obj->matches;
    vec2f* objs_position = track_obj->objs_position;
    int* costs = track_obj->costs;
    int8_t* matched_kf = track_obj->matched_kf; // this array keeps flags on whether given kalman had a matching detection
    int8_t* active_trackers_idx = track_obj->active_trackers_idx;
    kalman_tracker* trackers = track_obj->trackers;
    uint8_t kalman_num_filter = track_obj->kalman_num_filter;
    matrix_data_t* cost_matrix_buffer = track_obj->cost_matrix_buffer;
    matrix_t* cost_matrix = &track_obj->cost_matrix;
    vec2i* counts = &track_obj->counts;
    hungarian_t* hungarian_obj = track_obj->hungarian_obj;
    object_tracking_cfg_t* track_cfg = &track_obj->cfg;

    int match_limit = track_cfg->match_cost_limit;

    debug("Match cost limit: %d\r\n", match_limit);

    debug("Detections:\r\n");
	for(int i=0; i<N; i++){
		debug("\t%d => %d, %d, %d, %d\r\n", 
            kept_indicies[i], (int)bboxes[kept_indicies[i]].xmin, (int)bboxes[kept_indicies[i]].ymin, (int)bboxes[kept_indicies[i]].xmax, (int)bboxes[kept_indicies[i]].ymax );
	}

    // if we get more detections that we can handle -> disregard the last ones
    N = MIN(N, track_obj->max_detections);
    
    int n_active_trackers = 0;

    debug("Trackers:\r\n");
    // collect filter predictions
    for (int i = 0; i < kalman_num_filter; i++) {
        kalman_tracker* tracker = &trackers[i];
        if (tracker->active) {
            // NOTE: we used to advance filter here, however now we do it at the end of the loop
            //kalman_predict(tracker->kf_ptr);

            // obtain predicted object position from kalman state vector
            vec2f pos = get_position_from_state(tracker->kf_ptr);
            debug("\tTracker %d: uid=%d, x=%d, y=%d, missed=%d\r\n", i, tracker->uid, (int)pos.x, (int)pos.y, tracker->missed);
            // if object went outsize of scope (beyond image borders)
            // then deactivate it and check the count
            if (centroid_out_of_scope(pos.x, pos.y, track_cfg)) {
                counting_func(tracker, counts, track_obj);
                // deactivate tracker (return to the pool)
                debug("\tCentroid out of scope.\r\n");
                tracker_free(i, track_obj);
                continue;
            }

            // collect predictions from active filters
            objs_position[n_active_trackers] = pos; // x
            active_trackers_idx[n_active_trackers] = i;

            debug("\tTracker %d pred.: x=%d, y=%d\r\n", i, (int)objs_position[n_active_trackers].x, (int)objs_position[n_active_trackers].y);

            n_active_trackers++;
        }
        debug("\n");
    }

    // create cost matrix
    matrix_init(cost_matrix, (uint8_t)N, n_active_trackers, cost_matrix_buffer);

    // compute cost and fill cost matrix
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < n_active_trackers; j++) {
            float cost = cost_function(&bboxes[kept_indicies[i]], &objs_position[j], match_limit);
            debug("cost (obj->%d,trk->%d) = %d\r\n", i, j, (int)cost);
            matrix_set(cost_matrix, i, j, cost);
        }
    }

    // run hungarian algorithm to solve linear assignment problem
    int n_matches = hungarian(cost_matrix->data, cost_matrix->rows, cost_matrix->cols, matches, costs, hungarian_obj);

    // filter match list by cost - e.g. detections too far away
    n_matches = filter_matches(matches, costs, n_matches, match_limit);

    // for debugging purposes - display matches with cost
    debug("Matched %d trackers\r\n", n_matches);
    for (int i = 0; i < n_matches; i++) {
        debug("\t[%d] %d => %d : cost=%d\r\n", i, matches[i].v[0], matches[i].v[1], costs[i]);
    }

    // analyze the matches and advance kalman trackers
    // inputs:
    //  kf_idicies, n_obj
    //  matches,	n_matches

    // classify:
    // case 1: detections with a matched object (active filter) -> advance filter
    // case 2: case detections without a filter match i.e. not being in the list
    // case 3: object (filter) without a detection -> increment the missing counter, and if over the limit - deactivate

    // note - matches are a 1d array of size 2*n_matches as follows:
    // [det0, kal0, det1, kal1, ... ] where det0, det1 ... are ordered from lowest to highest
    int det_iter = 0;

    for (int i = 0; i < kalman_num_filter; i++)
        matched_kf[i] = 0;

    uint8_t match;
    // iterate over detections
    debug("Analyzing matches:\r\n");
    for (int i = 0; i < N; i++) {

        match = 0;
        if (det_iter < n_matches)
            if (matches[det_iter].v[0] == i)
                match = 1;

        if (match) {

            // case 1: detection has a match 
            // update kalman filter

            // get centroid of the detection
            vec2f c_det;
            bboxcorner2centroid(&bboxes[kept_indicies[i]], &c_det);

            int tracker_idx = active_trackers_idx[matches[det_iter].v[1]];

            debug("\tMatch for tracker %d\r\n", tracker_idx);

            // get kalman primitives
            kalman_tracker* tracker = &trackers[tracker_idx];
            kalman_t* kf = tracker->kf_ptr;
            kalman_measurement_t* kfm = tracker->kfm_ptr;
            matrix_t* z = kalman_get_measurement_vector(kfm);

            // reset missed counter, since we have a match
            tracker->missed = 0;

            // apply measurement
            matrix_set(z, 0, 0, c_det.x);
            matrix_set(z, 1, 0, c_det.y);

            // update filter
            kalman_correct(kf, kfm);

            // add tracker to the array of matches (used later on to find unmatched trackers)
            matched_kf[tracker_idx] = 1;
            det_iter++;
        }
        else {
            // case 2: a detection does not have a match - create new filter
            int idx = tracker_alloc(track_obj);
            if (idx >= 0) {
                vec2f pos;
                vec2f vel = { 0,0 };
                // convert detection to centroid
                bboxcorner2centroid(&bboxes[kept_indicies[i]], &pos);

                tracker_reinit(&trackers[idx], &pos, &vel, track_obj->uid_counter);
                debug("\tObject without a match\r\n");
                debug("\tNew tracker with idx %d and uid %d\r\n", idx, track_obj->uid_counter);
                track_obj->uid_counter++;
            }
            else {
                debug("\tFailed to alloc new tracker - pool exausted\r\n");
            }
        }
    }

    debug("Trackers without a match - increment missing:\r\n");

    // look for filters without a match and increment their missing counter
    // if missed to many times deactivate, count and return to the pool
    for (int i = 0; i < n_active_trackers; i++) {
        if (matched_kf[i] == 0) {
            // case 3: filter without a mach
            int tracker_idx = active_trackers_idx[i];
            kalman_tracker* tracker = &trackers[tracker_idx];
            tracker->missed++;
            debug("\tTracker %d (uid=%d) missed %d\r\n", tracker_idx, tracker->uid, tracker->missed);
            if (tracker->missed > track_cfg->max_missed) {

                // "count" and deactivate filter
                counting_func(tracker, counts, track_obj);
                // deactivate tracker (return to the pool)
                debug("\tMissed too much.\r\n");
                tracker_free(tracker_idx, track_obj);
            }
        }
    }

    // Advance all the filters (for the next iteration)
    for (int i = 0; i < kalman_num_filter; i++) {
        kalman_tracker* tracker = &trackers[i];
        if (tracker->active) {
            // keep last position before updating (mainly for testing/visualization purposes)
            tracker->last_pos = get_position_from_state(tracker->kf_ptr);
            // make a prediction (state will be updated)
            kalman_predict(tracker->kf_ptr);
        }
    }

    // return copy of the counts
    return *counts;
}

// check if object (described by kalman_tracker) went up or down or neither and add count to the "counts" vector
void counting_func(kalman_tracker* kf, vec2i* counts, object_tracking_t* track_obj) {

    debug("Counting tracklet %d\r\n", kf->uid);
    // simple "up/down" counting

    // NOTE: coiunterintutively UP is where y corrinate is 0, whereas DOWN is where y coordinate is largest (e.g. 96 or so)
    // therefore counting function should be designed carefully
    // matrix_t* X = kalman_get_state_vector(kf->kf_ptr);


    vec2f end_point = get_position_from_state(kf->kf_ptr);
    vec2f start_point = kf->start_point;

    debug("\tstart_pt: [%f, %f]\n", start_point.x, start_point.y);
    debug("\tend_pt: [%f, %f]\n", end_point.x, end_point.y);

    // TODO: problem is probably herem ths value is 0
    int n = track_obj->counting_obj_n;
    debug("\tIterating over %d counting lines\n", n);
    vec2ui8 cnt = { 0,0 };

    // multiline counting - iterate through counting objects
    for (int i = 0; i < n; i++) {
        debug("\tcount line %d:", i);
        counting_t* cnt_obj = &track_obj->counting_obj[i];

        compute_count(&start_point, &end_point, &cnt, cnt_obj);
        debug(" p=%d, s=%d\n", cnt.x, cnt.y);
        // stop at first count found
        if (*((uint16_t*)cnt.v) > 0)
            break;

    }

    // clip counts at 1
    counts->v[0] += MIN(cnt.v[0], 1);
    counts->v[1] += MIN(cnt.v[1], 1);    

    debug("Count UP=%d, DOWN=%d [unchanged]\r\n", counts->v[0], counts->v[1]);
}

// computes centroid coordinates of an object represented by BoxCornerEncoding
// saves results to provide vec2* pointer
void bboxcorner2centroid(BoxCornerEncodingFloat* bbox, vec2f* out) {
    out->x = (bbox->xmax - bbox->xmin) * 0.5f + bbox->xmin; // centroid x
    out->y = (bbox->ymax - bbox->ymin) * 0.5f + bbox->ymin; // centroid y
}

// NOTE - we could easily make this math in int32 (int16 may have a risk of overflow in extreme cases)
// cost function computes sqrted distance between box centers and centroid cx, cy 
float cost_function(BoxCornerEncodingFloat* bbox, vec2f* pred, int cost_limit) {
    vec2f c_det;
    bboxcorner2centroid(bbox, &c_det);
    // squared distance between object detected and kalman tracker prediction (tracked object state)
    float cost = (pred->x - c_det.x) * (pred->x - c_det.x) + (pred->y - c_det.y) * (pred->y - c_det.y);
    // if cost is higher than the limit - bump it up, so it will never be considered a valid assignment
    if((int)cost > cost_limit)
        return 1000000.0f;
    return cost;
}

int filter_matches(vec2ui8* matchings, int* costs, int n_matchings, int cost_limit) {
    // filter based on MATCH_COST_LIMIT
    int last_valid = 0;
    int n = n_matchings;
    for (int i = 0; i < n_matchings; i++) {
        if (costs[i] > cost_limit) { // filtering condition
            n--;
            debug("match %d with cost %d filtered out\r\n", i, costs[i]);
        }
        else { // "pop" invalid items from the list
            if (last_valid < i) { // "less" or "less or equal"
                costs[last_valid] = costs[i];
                matchings[last_valid] = matchings[i];
                debug("match %d with cost %d remains\r\n", i, costs[i]);
            }
            last_valid++;
        }
    }

    return n;
}

void object_tracking_clear(object_tracking_t* track_obj){

    // unpack local vars
    kalman_tracker* trackers = track_obj->trackers;
    uint8_t kalman_num_filter = track_obj->kalman_num_filter;

    // free any active tracks
    for (int i = 0; i < kalman_num_filter; i++) {
        if (trackers[i].active == 1) {
            tracker_free(i, track_obj);
        }
    }

}

// this doesn't work like a real alloc, but we may extend it in the future to handle a mempool like structure
void tracker_reinit(kalman_tracker* tracker, vec2f* pos, vec2f* vel, uint8_t uid) {
    tracker->active = 1;
    tracker->missed = 0;
    tracker->iterator = 0;
    tracker->start_point.x = pos->x;
    tracker->start_point.y = pos->y;
    tracker->uid = uid;

    matrix_t* x = kalman_get_state_vector(tracker->kf_ptr);
    x->data[0] = pos->x; // x
    x->data[1] = vel->x; // x'
    x->data[2] = pos->y; // y
    x->data[3] = vel->y; // y'
}

int tracker_alloc(object_tracking_t* track_obj)
{
    // unpack local vars
    kalman_tracker* trackers = track_obj->trackers;
    uint8_t kalman_num_filter = track_obj->kalman_num_filter;

    for (int i = 0; i < kalman_num_filter; i++) {
        if (trackers[i].active == 0) {
            trackers[i].active = 1;
            return i;
        }
    }

    return -1;
}

void tracker_free(int i, object_tracking_t* track_obj)
{
    // unpack local vars
    kalman_tracker* trackers = track_obj->trackers;

    // deactivate tracker
    debug("Removing tracker %d (uid=%d)\r\n", i, trackers[i].uid);
    trackers[i].active = 0;
}

int tracker_num_active(object_tracking_t* track_obj)
{
    // unpack local vars
    kalman_tracker* trackers = track_obj->trackers;
    uint8_t kalman_num_filter = track_obj->kalman_num_filter;

    int count = 0;
    for (int i = 0; i < kalman_num_filter; i++)
        if (trackers[i].active)
            count++;
    return count;
}

int tracker_num_free(object_tracking_t* track_obj) {
    return track_obj->kalman_num_filter - tracker_num_active(track_obj);
}

// ------------ DEBUGGING FUNCTIONS ---------------------

void debug_tracker(kalman_tracker* tracker, int idx){
    debug("Created a tracker[%d] with fields =>\n", idx);
    debug("\t.kf_ptr=0x%08x\n", (unsigned int)(uintptr_t)tracker->kf_ptr);
    debug("\t.kfm_ptr=0x%08x\n", (unsigned int)(uintptr_t)tracker->kfm_ptr);
    debug("\t.active=%d\n", tracker->active);
    debug("\t.missed=%d\n", tracker->missed);
}

void debug_trackers(object_tracking_t* track_obj){
    for (int i = 0; i < track_obj->kalman_num_filter; i++) {
        // link kalman filters to trackers
        kalman_tracker* tracker = &track_obj->trackers[i];

        debug_tracker(tracker, i);
    }
}
