#ifndef SSD_DETECTION_H
#define SSD_DETECTION_H

#ifndef STD_LIB
#include <stdint.h>
typedef int8_t  q7_t;
typedef int16_t  q15_t;
#endif  // STD_LIB

# define STD_LIB
// #define ETA_SDK
// #define HARD_NMS
# define FLOAT_PRECISION 0
# define Q7_PRECISION 1

#define MAX_NUM_BOXES 100
#define MAX_TO_KEEP 50
#ifdef STD_LIB
q7_t __SSAT(int32_t operand, uint8_t shift);
#endif

int16_t SelectDetectionsAboveScoreThresholdFloat( const q7_t * values, int16_t n,
                                            const q7_t threshold,
                                            q7_t * keep_values,
                                            int16_t * keep_indices );

# define ARITHMETIC_PRECISION Q7_PRECISION // not implemented yet, Q7 by default

#ifdef __cplusplus
extern    "C"
{
#endif

typedef enum
{
  ETA_STATUS_OK     =  0,        /**< No error */
  ETA_GEN_ERR       = -1,        /**< Error in the compute of the function */
} eta_rc;

typedef struct ssd_detector_opt_st
{
    int16_t max_detections; // max number of output detections, all classes
    int16_t num_classes;    // number of class should be 1
    int16_t num_boxes;      // number of boxes (= locations = scores = anchor boxes)
    q7_t scale_y;           // scales for box encoding system; default 10
    q7_t scale_x;           // scales for box encoding system; default 10
    q7_t scale_h;           // scales for box encoding system; default  5
    q7_t scale_w;           // scales for box encoding system; default  5
    q7_t considering_BG;    // 0: ignore BG score, 1: if BG score is much higher than that of person, there is no person to decrease false positives
    q7_t threshold_score;   // Qm.n as the same as conf format
    q7_t threshold_IOU;     // Q0.7 format, [0, 1) ==> [0, 127]
    q7_t loc_dec_bits;      // number of decimal bits in location??
    q7_t conf_dec_bits;     // number of decimal bits in confidence score; it should be 7; Q0.7 [0, 1)??
    q7_t anchor_dec_bits;   // number of decimal bits in anchor??
    q7_t bbox_dec_bits;     // number of decimal bits in bounding box system; it should be 6; Q1.6 [0, 2)??
} ssd_detector_opt;


/**
 * @brief SSD (Single Shot MultiBox Detector)
 * @param[in]   loc         pointer to locations (y-center, x-center, height, width) of [num_boxes, 4]
 * @param[in]   conf        pointer to confidence scores (backgroudn, person) of [num_boxes, 2]
 * @param[in]   anchor      pointer to anchor boxes (y-center, x-center, height, width) of [num_boxes, 4]
 * @param[out]  out_bbox    pointer to selected bounding-boxes (ymin, x-min, y-max, x-max) of [out_count, 4]
 * @param[out]  out_class   pointer to selected classes of [out_count, 4]
 *                                      (in person counting, there is one class, person, so these are all 0)
 * @param[out]  out_conf    pointer to selected confidence scores for person class of [out_count, 4]
 * @param[out]  out_count   pointer to number of persons in the input image (== number of output bboxes)
 * @param[in]   opt         struct for SSD options
 * @return     The function returns <code>ETA_STATUS_OK</code>
 */

eta_rc eta_ssd_detector_q7( const q7_t * loc, 
                            const q7_t * conf, 
                            const q7_t * anchor,
                            q7_t * out_bbox,
                            q7_t * out_class,
                            q7_t * out_conf,
                            int16_t * out_count,
                            const ssd_detector_opt opt );


#ifdef __cplusplus
}
#endif

#endif  // SSD_DETECTION_H
