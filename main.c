/*
 * File:   pwm_period_test3.c  
 * beta release
 * PIC12F675 Code Workspace
 * 
 * each loop (pwm step) is ~32us independent of anything else
 * PWM_STEPS = length of half a duty cycle
 * frequency = 1/(2*PWM_STEPS*.000030)
 * increasing PWM_STEPS = slower frequency, but smoother transition
 * the smaller lookup table is only really useful to save space (on flash)
 * 
 * increase from 127 to 152Hz with xc8 optimisation set in project properties
 * 122Hz with 128 steps
 * 157Hz with 100 steps
 * 241Hz 64 steps
 * 256Hz 60 steps
 * 311Hz 50 steps
 * measured OEM supply as 50V p2p, 242Hz
 */

#include <xc.h>
#include <stdint.h>

// CONFIG
#pragma config FOSC = INTRCIO
#pragma config WDTE = ON
#pragma config PWRTE = ON
#pragma config MCLRE = ON
#pragma config BOREN = ON
#pragma config CP = OFF
#pragma config CPD = OFF

#define _XTAL_FREQ 4000000
#define LED_A GPIObits.GP4
#define LED_B GPIObits.GP5

// Pre-calculated gamma correction table (1-100% range)
// Gamma = 2.2, Input: 0-127, Output: 1-100, val^gamma
// const saves to flash
const uint8_t GAMMA_LUT128[128] = {
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
     2,   2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,
     6,   6,   6,   7,   7,   8,   8,   8,   9,   9,  10,  10,  11,  11,  12,  12,
    13,  13,  14,  14,  15,  15,  16,  17,  17,  18,  19,  19,  20,  21,  21,  22,
    23,  24,  24,  25,  26,  27,  28,  29,  29,  30,  31,  32,  33,  34,  35,  36,
    37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,  49,  50,  51,  52,  53,
    54,  56,  57,  58,  60,  61,  62,  63,  65,  66,  68,  69,  70,  72,  73,  75,
    76,  78,  79,  81,  82,  84,  85,  87,  88,  90,  92,  93,  95,  97,  98, 100,
};
// Input: 0-127, Range = 1-100, Steps = 64, Gamma = 2.2
const uint8_t GAMMA_LUT64[64] = {
    1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  4,  4,  5,  5,
    6,  7,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 22,
    23, 25, 26, 28, 30, 32, 34, 35, 37, 39, 42, 44, 46, 48, 51, 53,
    55, 58, 61, 63, 66, 69, 72, 74, 77, 80, 84, 87, 90, 93, 97, 100
};

#define GAMMA_LUT GAMMA_LUT128   // which lookup table to use
#define PWM_STEPS 100            // coarse requency control
#define SHIFT_BITS 1             // ADC reads as 8bit, left shift to match lookup table length
/* 64 lookup
#define GAMMA_LUT GAMMA_LUT64
#define PWM_STEPS 64
#define SHIFT_BITS 2
*/

// Timing constants
uint8_t pwm_counter = 0;
uint8_t pwm_duty = 90;  // initial value
uint8_t phase = 0;      // 0 = A, 1 = B
// State machine approach - spreads ADC across multiple ISR calls
uint8_t adc_state = 0;  // 0=idle, 1=converting

void setup() {
    // Calibrate oscillator
    //asm("call 0x348C"); //523Hz
    //asm("call 0b10000000");
    //asm("movwf OSCCAL");
    // memory address 3fff goes back to 3444
    //OSCCAL = 0b11111100; // max value
    //OSCCAL = 0b10000000; //0x80 centre value
    //OSCCAL = 0b00000000; //min value
    
    // Analogue channel setup
    //ANSEL = 0b00010100;     // 2us per sample (2x11) + 11.5 + 0.5 = 34us
    ANSEL = 0b01010100;   // 4us per sample = (4*11) + 11.5 +0.5 = 56us
    //        -|||---- bits 6:4 ADC conversion clock 101=4us 001=2us
    //        ----|||| bits 3:0 set AN3:0 channels as analogue
    // since we are not waiting for ADC to complete 4us is better bet at 3V
    
    // GP4 & GP5 outputs
    TRISIO = 0b00001100;         // all outputs except AN2 GP3(MCLRE))
    //ADC configuration
    ADCON0 = 0b00001001;         // left justified, Vdd ref, AN2, ADC ON
    // Disable comparator
    CMCON = 0b00000111;          // Comparator Off (Lowest power)
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
           CLRWDT();    // reset watchdog timer, default 18ms timeout
       }

       // Check if ADC is done (runs every cycle, minimal overhead)
       if (adc_state && !ADCON0bits.GO_nDONE) {
           adc_state = 0;
           // ADC complete - read and update
           // read only top 8 bits and shift  right to 7 bits
           pwm_duty = GAMMA_LUT[ADRESH >> SHIFT_BITS];  // Combined: read, shift, lookup
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
