#ifndef _DEBUG_H
#define _DEBUG_H

//#include "../../../include/project_defines.h"
#include "project_defines_standalone.h"


#ifdef KATANA_TRACKING_DEBUG
#define debug(...) mcu_print(__VA_ARGS__)
#endif

#ifdef STDLIB_TRACKING_DEBUG
#include <stdio.h>
#define debug(...) printf(__VA_ARGS__)
#endif

#if !defined(KATANA_TRACKING_DEBUG) && !defined(STDLIB_TRACKING_DEBUG)
#define debug(...) 
#endif

#endif