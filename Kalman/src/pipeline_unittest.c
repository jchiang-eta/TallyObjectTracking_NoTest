#include "pipeline_unittest.h"
#include "pipeline.h"
#include "pipeline_config.h"
#include "debug.h"
#include "assertion.h"

#if 0

void pipeline_float_unittest() {

    debug("Pipeline unittest\r\n");
    pipeline_out_t out = { 0, 0 };

    pipeline_config_t config = {0}; // for the time being it's null initialized
    // TODO: pass real config when ready
    pipeline_init(config);

    // for reference, box format: [xmin, ymin, xmax, ymax]
    debug("Step 0\r\n");
    {
        BoxCornerEncodingFloat bboxes_in[] = { 
            {10.0, 20.0, 20.0, 30.0}, // supressed
            {11.0, 21.0, 20.0, 30.0}, // obj 0 - will be moving up
            {80.0,  80.0, 85.0, 90.0} // obj 1 - will be moving down
        };
        // each row: [background, head]
        float scores_in[] = {   0.1f, 0.78f, // supressed
                                0.1f, 0.9f, 
                                0.3f, 0.8f 
                            };
        int N = 3;
        pipeline_float_in_t in = { (BoxCornerEncodingFloat*)bboxes_in, scores_in, N, 2};

        pipeline_float_run(&in, &out);
    }

    debug("Step 1\r\n");
    {
        BoxCornerEncodingFloat bboxes_in[] = { 
            {15.0, 40.0, 20.0, 50.0}, // obj 0
            {60.0, 70.0, 70.0, 80.0} // obj 1
        };
        // each row: [background, head]
        float scores_in[] = {   0.1f, 0.78f, 
                                0.1f, 0.9f, 
                            };
        int N = 2;
        pipeline_float_in_t in = { (BoxCornerEncodingFloat*)bboxes_in, scores_in, N, 2};

        pipeline_float_run(&in, &out);
    }

    debug("Step 2\r\n");
    {
        BoxCornerEncodingFloat bboxes_in[] = {
            {10.0, 70.0, 20.0, 80.0}, // obj 0
            {70.0, 55.0, 80.0, 65.0}, // obj 1
            { 1.0,  1.0,  2.0,  2.0}  // under threshold
        };
        // each row: [background, head]
        float scores_in[] = {   0.1f, 0.78f, 
                                0.1f, 0.9f, 
                                0.3f, 0.05f 
                            };
        int N = 3;
        pipeline_float_in_t in = { (BoxCornerEncodingFloat*)bboxes_in, scores_in, N, 2};

        pipeline_float_run(&in, &out);
    }

    debug("Step 3\r\n");
    {
        BoxCornerEncodingFloat bboxes_in[] = { 
            {10.0, 98.0, 20.0, 100.0}, // obj 0 - this will be counted as UP, but in the next iteration
                                       // obj 1 - disappears temporarily
        };
        float scores_in[] = { 0.1f, 0.9f };
        int N = 1;
        pipeline_float_in_t in = { (BoxCornerEncodingFloat*)bboxes_in, scores_in, N, 2};

        pipeline_float_run(&in, &out);
    }

    debug("Step 4\r\n");
    {
        BoxCornerEncodingFloat bboxes_in[] = { 
            {70.0, 35.0, 80.0, 40.0}, // obj 1 - reappears
        };
        float scores_in[] = { 0.1f, 0.9f};
        int N = 1;
        pipeline_float_in_t in = { (BoxCornerEncodingFloat*)bboxes_in, scores_in, N, 2};

        pipeline_float_run(&in, &out);
    }

    debug("up=%d, down=%d\r\n", out.count_up, out.count_down);

    assert(out.count_up == 1);
    assert(out.count_down == 0);

    debug("Step 5\r\n");
    {
        BoxCornerEncodingFloat bboxes_in[] = { 
            {70.0, 0.0, 85.0, -5.0}, // obj 1 - should be counted as DOWN, but in the next iteration
        };
        float scores_in[] = { 0.1f, 0.9f};
        int N = 1;
        pipeline_float_in_t in = { (BoxCornerEncodingFloat*)bboxes_in, scores_in, N, 2};

        pipeline_float_run(&in, &out);
    }
    // Note - counting is based tracklet which is composed of predictions. 
    // So if measurement goes out of scope, it is the kalman filter predicition that will
    // be used for counting. That's why counting is "delayed" by one iteration w.r.t. measurements

    debug("Step 6\r\n");
    {
        BoxCornerEncodingFloat bboxes_in[1];
        float scores_in[1];
        int N = 0;
        pipeline_float_in_t in = { (BoxCornerEncodingFloat*)bboxes_in, scores_in, N, 2};

        pipeline_float_run(&in, &out);
    }

    debug("up=%d, down=%d\r\n", out.count_up, out.count_down);

    assert(out.count_up == 1);
    assert(out.count_down == 1);


}

#endif