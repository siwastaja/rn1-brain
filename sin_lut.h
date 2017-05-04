#ifndef SIN_LUT_H
#define SIN_LUT_H

#include <inttypes.h>

#define SIN_LUT_POINTS 4096
#define SIN_LUT_BITS 12
#define SIN_LUT_SHIFT (32-SIN_LUT_BITS)

extern const int16_t sin_lut[SIN_LUT_POINTS] __attribute__((section(".sin_lut")));


#endif
