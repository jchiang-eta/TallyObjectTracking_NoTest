#include <stdint.h>

#include "debug.h"
#include "assertion.h"

#include "quant_utils.h"
#include "quant_unittest.h"

void dequantize_class_unittest();

// testdata from tflite model:

// scale=0.00390625 and zero_point=-128
// [ 19, -19,  88, -88,  88, -88,  88, -88,  36, -36]
// [0.57421875, 0.42578125, 0.84375   , 0.15625   , 0.84375   , 0.15625   , 0.84375   , 0.15625   , 0.640625  , 0.359375  ]

// scale=0.39635542035102844 and zero_point=-119
// [-83, -73, -48, -41, -90, -91, -61, -61, -90, -92],
// [14.26879513, 18.23234934, 28.14123484, 30.91572279, 11.49430719, 11.09795177, 22.98861438, 22.98861438, 11.49430719, 10.70159635]

// main test function
void quant_unittest(){

    debug("Quantization unittest\r\n");

    dequantize_class_unittest();
    dequantize_by_index_unittest();
}

// helper function to 
int appx_equal(float val1, float val2){
    float diff = val1 - val2;
    if(diff < 0)
        diff = -diff;
    return diff <= FLOAT_EQUAL_PRECISION;
}

#define QUANT_CLASS_TEST_N          10
#define TEST_CLASS_QUANT_SCALE      0.00390625f
#define TEST_CLASS_QUANT_OFFSET     -128

const int8_t quant_class_raw_array[QUANT_CLASS_TEST_N] = { 19, -19,  88, -88,  88, -88,  88, -88,  36, -36 };
float quant_class_test_array[QUANT_CLASS_TEST_N];
const float quant_class_ref_array[QUANT_CLASS_TEST_N] = { 0.57421875f, 0.42578125f, 0.84375f   , 0.15625f   , 0.84375f   , 0.15625f   , 0.84375f   , 0.15625f   , 0.640625f  , 0.359375f };

void dequantize_class_unittest()
{
    dequantize_to_float(
        (const int8_t*)quant_class_raw_array, 
        (float*)quant_class_test_array, QUANT_CLASS_TEST_N, TEST_CLASS_QUANT_SCALE, TEST_CLASS_QUANT_OFFSET);

    for (int i = 0; i < QUANT_CLASS_TEST_N; i++)
	    assert(appx_equal(quant_class_ref_array[i], quant_class_test_array[i]));
    debug("class unittest passed\r\n");
}

#define N_SAMPLES                   10
#define N_KEPT                      8
#define ROW_LENGTH                  4
#define SCALE                       0.11452088f
#define ZERO_POINT                  62

const int8_t dequantize_input_array[N_SAMPLES * ROW_LENGTH] = { 82,    3,  -60,  -55, -100,  -86,  -79,    5,   96,   -7,   41,
        -26, -103,   63,   37,  125,   88,   47, -100,  -86,  -21,  -16,
        -73,  -43,    9,  -24, -117,   72,  -77,  125,   -2,   89,   15,
         -3,  -18,   92,   83,   59,   96,   47 };
float output_array[N_KEPT * ROW_LENGTH];
const float expected_output_array[N_KEPT * ROW_LENGTH] = { 2.29041762f,  -6.75673199f, -13.9715475f , -13.39894309f,
        -9.50523313f,  -8.93262873f, -15.46031895f, -12.02469252f,
         2.4049385f ,  -0.34356264f,   3.89370996f,  -1.71781322f,
        -5.38248141f,  -7.44385727f,  -9.16167049f,   3.43562643f,
       -18.55238274f, -16.9490904f , -16.14744424f,  -6.52769022f,
         2.97754291f,  -1.71781322f, -18.55238274f, -16.9490904f ,
        -6.0696067f ,  -9.84879578f, -20.49923772f,   1.14520881f,
         3.89370996f,  -7.9019408f ,  -2.4049385f , -10.07783754f };
uint16_t kept_indicies[N_KEPT] = { 0, 5, 9, 8, 1, 4, 6, 2 };

void dequantize_by_index_unittest()
{
    dequantize_to_float_by_index(dequantize_input_array, output_array, kept_indicies, N_KEPT, ROW_LENGTH, SCALE, ZERO_POINT);

    for (int i = 0; i < N_KEPT * ROW_LENGTH; i++)
        assert(appx_equal(output_array[i], expected_output_array[i]));
    debug("index unittest passed\r\n");
}
