/*
 * pwm_tables.c
 * Lookup table definitions for gamma correction and dithering.
 *
 * XC8 places `const` variables in flash (program memory).
 * The __at(0x200) attribute pins GAMMA_LUT256 to page 2 so that the linker
 * does not scatter it across page boundaries, which would cost extra cycles.
 */

#include <xc.h>
#include <stdint.h>
#include "pwm_tables.h"

/* --------------------------------------------------------------------------
 * Gamma-correction LUT  — 256 entries, γ = 2.2, output range 1..32
 *
 * Usage:  duty = GAMMA_LUT[ADRESH];
 *
 * The first ~40 ADC values all map to duty=1 (minimum visible brightness).
 * Value 255 maps to duty=32 (full on, no dithering applied).
 * -------------------------------------------------------------------------- */
const uint8_t __at(0x200) GAMMA_LUT256[256] = {
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,  /* 0x00 */
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,  /* 0x10 */
     1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,  /* 0x20 */
     2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,  /* 0x30 */
     2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,  /* 0x40 */
     3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   5,  /* 0x50 */
     5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,  /* 0x60 */
     6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   7,   7,   8,   8,  /* 0x70 */
     8,   8,   8,   8,   8,   8,   9,   9,   9,   9,   9,   9,   9,   9,  10,  10,  /* 0x80 */
    10,  10,  10,  10,  10,  11,  11,  11,  11,  11,  11,  11,  12,  12,  12,  12,  /* 0x90 */
    12,  12,  12,  13,  13,  13,  13,  13,  13,  14,  14,  14,  14,  14,  14,  15,  /* 0xA0 */
    15,  15,  15,  15,  15,  16,  16,  16,  16,  16,  16,  17,  17,  17,  17,  17,  /* 0xB0 */
    18,  18,  18,  18,  18,  19,  19,  19,  19,  19,  20,  20,  20,  20,  20,  21,  /* 0xC0 */
    21,  21,  21,  21,  22,  22,  22,  22,  23,  23,  23,  23,  23,  24,  24,  24,  /* 0xD0 */
    24,  25,  25,  25,  25,  25,  26,  26,  26,  26,  27,  27,  27,  27,  28,  28,  /* 0xE0 */
    28,  28,  29,  29,  29,  29,  30,  30,  30,  30,  31,  31,  31,  31,  31,  32,  /* 0xF0 */
};

/* --------------------------------------------------------------------------
 * run_start[]  — GAMMA_LUT index where each run of identical values begins
 *
 * run_start[0] = 0   (value 1 first appears at index 0)
 * run_start[1] = 40  (value 2 first appears at index 40)
 * …
 * Used by get_pwm_duty() to calculate where within a run the ADC reading falls.
 * -------------------------------------------------------------------------- */
const uint8_t run_start[32] = {
      0,  40,  65,  82,  95, 107, 117, 126, 134, 142,
    149, 156, 163, 169, 175, 181, 187, 192, 197, 202,
    207, 212, 216, 221, 225, 230, 234, 238, 242, 246,
    250, 255
};

/* --------------------------------------------------------------------------
 * run_inc[]  — fixed-point dither increment per ADC step within each run
 *
 * Approximates  256 / run_length  for each run.
 * Stored as uint8_t; intentional wrapping of the 8-bit accumulator in
 * get_pwm_duty() produces a carry (dither_bump) at the correct average rate.
 *
 * Index 31 (base level 32, full brightness) is 255: dithering is suppressed
 * and the accumulator is cleared instead — see get_pwm_duty().
 * -------------------------------------------------------------------------- */
const uint8_t run_inc[32] = {
      6,  10,  15,  20,  21,  26,  28,  32,  32,  36,
     36,  36,  42,  42,  42,  42,  51,  51,  51,  51,
     51,  64,  51,  64,  51,  64,  64,  64,  64,  64,
     51, 255
};
