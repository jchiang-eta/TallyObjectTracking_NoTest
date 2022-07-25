#ifndef _PROJ_DEF_STANDALONE_H
#define _PROJ_DEF_STANDALONE_H

#ifdef UNITTEST_STANDALONE

// must be 1 when run on Katana board
#define KATANA_RUNTIME  0
// must be 0 when run on Katana board
#define CFFI_RUNTIME    1

// For the following, a value of 1 enables a feature, 0 disables
// #define ML_DEVELOPMENT 1
// #define ML_OUTPUT_PRINT 1
// Tracking setup
#define TRACKING_ACCUMULATE_COUNTS   0

#else
#include "../../../include/project_defines.h"
#endif

#endif // _PROJ_DEF_STANDALONE_H
