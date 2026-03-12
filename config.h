/*
 * config.h
 * Hardware configuration, PIC fuse bits, pin assignments,
 * timing constants and state definitions for dithered PWM dimmer.
 *
 * PIC12F675 / XC8
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <xc.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * PIC12F675 Configuration Fuse Bits
 * -------------------------------------------------------------------------- */
#pragma config FOSC  = INTRCIO   // Internal oscillator, GPIO on OSC pins
#pragma config WDTE  = ON        // Watchdog timer enabled
#pragma config PWRTE = ON        // Power-up timer enabled
#pragma config MCLRE = OFF       // GP3/MCLR pin as digital input
#pragma config BOREN = ON        // Brown-out reset enabled
#pragma config CP    = OFF       // Code protection off
#pragma config CPD   = OFF       // Data memory protection off

/* --------------------------------------------------------------------------
 * Oscillator
 * -------------------------------------------------------------------------- */
#define _XTAL_FREQ 4000000       // 4 MHz internal RC — required by __delay macros

/* --------------------------------------------------------------------------
 * Pin Assignments  (GPIO port bit aliases)
 * -------------------------------------------------------------------------- */
#define LED_A  GPIObits.GP0      // Complementary PWM output A
#define LED_B  GPIObits.GP1      // Complementary PWM output B
//  GP2  AN2  — analogue input (potentiometer / light sensor)
//  GP3  MCLR — digital input, manual-override switch (active high)
//  GP4/GP5  xtal clock pins

/* --------------------------------------------------------------------------
 * State Machine States
 * -------------------------------------------------------------------------- */
#define STATE_MANUAL  1          // Dimmer active, no timer
#define STATE_BRIGHT  2          // Full PWM, timed (morning / bright period)
#define STATE_DIM     4          // Dithered PWM, timed (evening / dim period)
#define STATE_OFF     8          // Outputs off, sleeping

/* --------------------------------------------------------------------------
 * PWM Parameters
 * each pwm step ≈ 22 µs  →  ~595 Hz base rate
 * ADC uses ~75 µs acquisition + conversion time; code overhead ≈ 35 µs.
 * frequency = 1 / (2 * PWM_STEPS * 22µs  +  75µs + 35µs)
 * PWM_STEPS must equal the maximum value produced by GAMMA_LUT
 * -------------------------------------------------------------------------- */
#define PWM_STEPS  32            // Coarse resolution / frequency control

/* --------------------------------------------------------------------------
 * Timer Tick Preloads  (1 Hz ticks from TMR1 with 32.768 kHz crystal)
 * Timer1: async external crystal, 1:1 prescaler, TMR1H=0x80 → 1 s overflow
 *
 * Encode tick counts as two uint8_t values (tick_hi : tick_lo).
 * The ISR decrements this pair and sets state_change_flag on underflow to 0.
 *
 *   BRIGHT period  10h 35m  = 38 100 s  = 0x94D4
 *   DIM    period   6h 25m  = 23 100 s  = 0x5A3C
 *   OFF    period   7h 00m  = 25 200 s  = 0x6270
 *
 * During development, replace with small values e.g. tick_lo=0x0A, tick_hi=0x00
 * -------------------------------------------------------------------------- */
//#define DEV_LO          0x0A
//#define DEV_HI          0x00
#define TICK_BRIGHT_LO  0xD4     // 38100 = 0x94D4
#define TICK_BRIGHT_HI  0x94

#define TICK_DIM_LO     0x3C     // 23100 = 0x5A3C
#define TICK_DIM_HI     0x5A

#define TICK_OFF_LO     0x70     // 25200 = 0x6270
#define TICK_OFF_HI     0x62

#endif /* CONFIG_H */
