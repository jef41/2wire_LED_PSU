/*
 * pwm_tables.h
 * Declarations for gamma-correction lookup table and dither helper tables.
 *
 * Definitions live in pwm_tables.c 
 */

#ifndef PWM_TABLES_H
#define PWM_TABLES_H

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Gamma correction LUT  (256 entries, pinned to flash page 2 at 0x200)
 *
 * Maps 8-bit ADC reading → perceptually-linear PWM duty step (1..32).
 * Generated for γ = 2.2 with PWM_STEPS = 32 output levels.
 * -------------------------------------------------------------------------- */
extern const uint8_t __at(0x200) GAMMA_LUT256[256];

/* Alias — change this #define to switch to a different table if needed */
#define GAMMA_LUT GAMMA_LUT256

/* --------------------------------------------------------------------------
 * Dither helper tables  (32 entries each, one per PWM base level)
 *
 *  run_start[i]  — index into GAMMA_LUT where the run of value (i+1) begins
 *  run_inc[i]    — fractional increment per ADC step within that run
 *                  = 256 / run_length  (stored as uint8_t, wraps intentionally)
 * -------------------------------------------------------------------------- */
extern const uint8_t run_start[32];
extern const uint8_t run_inc[32];

#endif /* PWM_TABLES_H */
