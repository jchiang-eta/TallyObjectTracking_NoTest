
#include <stdio.h>

#include "pipeline_config.h"
#include "pipeline.h"
#include "debug.h"

#ifdef TESTING
#include "matrix_unittest.h"
#include "tracking_unittest.h"
#include "pipeline_unittest.h"
#include "postprocessing_unittest.h"
#include "hungarian_unittest.h"
#include "counting_unittest.h"
#include "quant_unittest.h"

void unittest() {
    matrix_unittest();
    hungarian_unittest();
    counting_unittest();
    tracking_unittest();
    postprocessing_unittest();
    quant_unittest();
    filtering_unittest();

    //pipeline_float_unittest();

    debug("Testing completed successfully\r\n");
}
#endif

/**
* \brief Main entry point
*/
void main()
{

#ifdef TESTING
    unittest();
#endif

}