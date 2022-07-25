#include "quant_utils.h"

void dequantize_to_float(const int8_t* in, float* out, unsigned int n, float scale, float offset) 
{
	for (unsigned int i = 0; i < n; i++) {
		out[i] = scale * (((float)in[i]) - offset);
	}
}
#include "debug.h"
void dequantize_to_float_by_index(
	const int8_t* in,
	float* out,
	uint16_t* indicies,
	unsigned int n,
	uint8_t row_length,
	float scale,
	float offset
)
{
	// iterate over rows indicated by indicies
	for (uint16_t i = 0; i < n; i++) 
	{
		// iterate over columns in each row
		for(uint16_t j = 0; j < row_length; j++){
			int read_idx = indicies[i] * row_length + j;
			int write_idx = i* row_length + j;
			int8_t value = in[ read_idx ];
			out[ write_idx ] = scale * ( ((float)value) - offset );
			// debug("I:%d, R: %d => W: %d (%d => %f)\r\n", indicies[i], read_idx, write_idx, value, out[ write_idx ] );
		}
	}
}