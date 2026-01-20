/*
 * File:   pwm_period_test3.c  
 * Testing single psm period
 * PIC12F675 Code Workspace - OPTIMIZED VERSION
 * 
 * single loop takes approx 43us
 * pwm_steps * 2 * 43us = 8.6ms = 116.28Hz for 100 steps Measured on scope @ 127.2201Hz
 *                              = 181.69Hz for 64 steps
 * a single loop must be at least as long as an ADC read
 * otherwise set ADC counter & read less frequently
 * 
 * increase from 127 to 152Hz with xc8 optimisation set in project properties
 * 157Hz with 100 steps
 * 238Hz 64 steps
 * 256Hz 60 steps
 * 300Hz 50 steps
 * measured OEM supply as 50V p2p, 242Hz
 */

#include <xc.h>
#include <stdint.h>

// CONFIG
#pragma config FOSC = INTRCIO
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config MCLRE = ON
#pragma config BOREN = ON
#pragma config CP = OFF
#pragma config CPD = OFF

#define _XTAL_FREQ 4000000

#define LED_A GPIObits.GP4
#define LED_B GPIObits.GP5
#define PWM_STEPS 100

// Timing constants
uint8_t pwm_counter = 0;
uint8_t pwm_duty = 90;  // initial value
uint8_t phase = 0;      // 0 = A, 1 = B
// State machine approach - spreads ADC across multiple ISR calls
uint8_t adc_state = 0;  // 0=idle, 1=converting
// Pre-calculated gamma correction table (1-100% range)
// Gamma = 2.2, Input: 0-127, Output: 1-100, val^gamma
// const saves to flash
const uint8_t gamma[128] = {
    1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,
    2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   5,
    5,   5,   6,   6,   6,   7,   7,   7,   8,   8,   9,   9,   9,  10,  10,  11,
   11,  12,  12,  13,  13,  14,  14,  15,  16,  16,  17,  17,  18,  19,  19,  20,
   21,  21,  22,  23,  24,  24,  25,  26,  27,  27,  28,  29,  30,  31,  32,  32,
   33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,
   49,  50,  51,  53,  54,  55,  56,  57,  58,  60,  61,  62,  63,  65,  66,  67,
   69,  70,  71,  73,  74,  76,  77,  78,  80,  81,  83,  84,  86,  87,  89,  100,
};

void setup() {
    // Analogue channel setup
    //ANSEL = 0b00010100;     // 2us per sample (2x11) + 11.5 + 0.5 = 34us
    ANSEL = 0b01010100;   // 4us per sample = (4*11) + 11.5 +0.5 = 56us
    //        -|||---- bits 6:4 ADC conversion clock 101=4us 001=2us
    //        ----|||| bits 3:0 set AN3:0 channels as analogue
    // since we are not wating for ADC to complete 4us is better bet at 3V
    
    // GP4 & GP5 outputs
    TRISIO = 0b00001100;         // all outputs except AN2 GP3(MCLRE))
    
    //ADC configuration
    ADCON0 = 0b00001001;         // left justified, Vdd ref, AN2, ADC ON
    // Disable comparator
    CMCON = 0b00000111;         // Comparator Off (Lowest power)
    
    // Initialise outputs
    GPIO = 0x00;
}

void main(void) {
    setup();
    while(1) {
       if (pwm_counter == 0) {
           // Start ADC conversion (non-blocking)
           ADCON0bits.GO_nDONE = 1;
           adc_state = 1;
       }

       // Check if ADC is done (runs every cycle, minimal overhead)
       if (adc_state && !ADCON0bits.GO_nDONE) {
           adc_state = 0;
           // ADC complete - read and update
           // read only top 8 bits and shift  right to 7 bits
           pwm_duty = gamma[ADRESH >> 1];  // Combined: read, shift, lookup
       }

       // PWM counter (optimised)
       if (++pwm_counter >= PWM_STEPS) {
           pwm_counter = 0;
           phase ^= 1;
       }

       // update LED outputs
       uint8_t pwm_active = (pwm_counter < pwm_duty);
       if (phase) {
           LED_A = 0;           // BCF GPIO, 4 - Turn off A first
           LED_B = pwm_active;  // BCF/BSF GPIO, 5 - Then update B
       } else {
           LED_B = 0;           // BCF GPIO, 5 - Turn off B first
           LED_A = pwm_active;  // BCF/BSF GPIO, 4 - Then update A
       }
    }
}
