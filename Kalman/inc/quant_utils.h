#ifndef _QUANT_UTILS_H
#define _QUANT_UTILS_H

#include <stdint.h>

/**
 * Converts int8 quantized input array to floats, using provided quantization parameters
 * Formula: out = in*scale + offset
 * @param[in] pointer to int8 quantized data
 * @param[out] pointer to output array where conveted data will be written
 * @param[in] length of in/out array
 * @param[in] quantization multiplier (scale)
 * @param[in] quantization offset
*/
void dequantize_to_float(const int8_t* in, float* out, unsigned int n, float scale, float offset);

void dequantize_to_float_by_index(
	const int8_t* in,
	float* out,
	uint16_t* indicies,
	unsigned int n,
	uint8_t row_length,
	float scale,
	float offset
);

#endif // _QUANT_UTILS_H