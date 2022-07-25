//#include "../../../include/project_defines.h"
#include "project_defines_standalone.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) < (b) ? (b) : (a))

// is model output in relative (0.0-0.1, 0.0-0.1) or absolute (0-h, 0-w) coordinates
#define MODEL_OUTPUT_ABSOLUTE 0

// maximum number of bboxes to keep after dequantization
#define MAX_DEQUANT 256

// maximum number of bboxes to keep after nms
#define MAX_INDICIES 64
#define MAX_DETECTIONS 6
#define MAX_OBJECTS 6
#define KALMAN_NUM_FILTERS MAX_OBJECTS

#define MAX_COUNT_LINES 5

#define NMS_MAX_KEEP 6

#define N_MATCHES MIN(MAX_DETECTIONS, MAX_OBJECTS)