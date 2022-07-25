#ifndef NMS_BYTE_H
#define NMS_BYTE_H

#ifdef __cplusplus
extern    "C"
{
#endif

#include <stdint.h>

int16_t FilterByScoreMulticlassExclBG_Byte(
	const int8_t * scores, // scores array of length n*n_classes
	int16_t n,
	const int8_t threshold, // single value to apply to all non-background columns
	int16_t * keep_indices,
	uint16_t keep_n_limit,
	uint8_t n_classes
);

int16_t FilterOneClassByScoreMulticlass_Byte(
	const int8_t * scores, // scores array of length n*n_classes
	int16_t n,
	const int8_t threshold, // single value to apply to all non-background columns
	int16_t * keep_indices,
	uint16_t keep_n_limit,
	uint8_t n_classes,
	uint8_t class_select // id of the class to filter
);

// #ifdef STD_LIB
// int8_t __SSAT(int32_t operand, uint8_t shift);
// #endif

#ifdef __cplusplus
}
#endif

#endif  // NMS_BYTE_H
