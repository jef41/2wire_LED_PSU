/*
 * base File:   dithered.c 
 * reduce PWM steps to increase frequency 
 * then implement dithering to restore more levels
 * especially at low duty cycles where step changes are visible 
 * 
 * PIC12F675 Code Workspace
 * 
 * each pwm step is 22us independent of anything else ~595Hz
 * ADC read once PWM period & takes ~75us aquisition & processing time, 
 * an additional ~50us is code related:
 * At duty=1, get_pwm_duty() includes the multiply position * run_inc[idx] which is expensive. 
 * At duty=32 it hits the else branch which just clears the accumulator — much faster
 * PWM_STEPS = length of half a duty cycle
 * frequency = 1/(2*PWM_STEPS*.0000222 + 0.000075 + 0.000035)
 * increasing PWM_STEPS = slower frequency, but smoother transition
 * a smaller lookup table is only really useful to save space (on flash)
 * a large lookup, but with only 32 discrete values
 * allows higher frequency & room for dithering, 
 * result is better than 128 discrete birghtness levels
 * 
 * at my location sunset 15:50, dusk 16:35
 * around 160Hz with 128 levels
 * around 653Hz with 32 levels
 * measured OEM supply as 50V p2p, 242Hz
 * 
 * TODO
 * 
 * POSSIBLE
 * only read ADC if button pressed
 * save ADC value when button released
 * save PWM for bright & dim to flash memory?
 * 
 * DONE
 * override switch to keep in PWM mode with no timer
 * accurate delay full on, then switch to PWM, then off, repeat
 * switch for always on PWM (no timer)
 * start timer
 * add state test
 * when timer elapsed move to next state
 * pwm steps to use 2x8bit values
 * 
 */

#include <xc.h>
#include <stdint.h>

// CONFIG
#pragma config FOSC = INTRCIO
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config MCLRE = OFF // testing input on GP3 // ON
#pragma config BOREN = ON
#pragma config CP = OFF
#pragma config CPD = OFF

#define _XTAL_FREQ 4000000
//#define _XTAL_FREQ FOSC_    // * required for __delay_ms, __delay_us macros * //
#define LED_A GPIObits.GP0
#define LED_B GPIObits.GP1
#define STATE_DIM 2
#define STATE_BRIGHT 4
#define STATE_OFF 8
#define STATE_MANUAL 1
  
// Pre-calculated gamma correction table (1-100% range)
// Gamma = 2.2, Input: 0-127, Output: 1-100, val^gamma
// const saves to flash

// bigger lookup uses more program space, but saves doing a shift op when reading ADC
// use 128 values for PWM because more decreases frequency 
// store on page 2, all of it
/*const uint8_t GAMMA_LUT256[256] __at(0x200) = {
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
     2,   2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,
     4,   4,   5,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   6,   7,   7,
     7,   7,   7,   8,   8,   8,   8,   9,   9,   9,   9,  10,  10,  10,  10,  11,
    11,  11,  11,  12,  12,  12,  13,  13,  13,  14,  14,  14,  14,  15,  15,  15,
    16,  16,  16,  17,  17,  18,  18,  18,  19,  19,  19,  20,  20,  21,  21,  21,
    22,  22,  23,  23,  23,  24,  24,  25,  25,  26,  26,  27,  27,  27,  28,  28,
    29,  29,  30,  30,  31,  31,  32,  32,  33,  33,  34,  34,  35,  35,  36,  37,
    37,  38,  38,  39,  39,  40,  41,  41,  42,  42,  43,  43,  44,  45,  45,  46,
    47,  47,  48,  48,  49,  50,  50,  51,  52,  52,  53,  54,  54,  55,  56,  56,
    57,  58,  59,  59,  60,  61,  61,  62,  63,  64,  64,  65,  66,  67,  67,  68,
    69,  70,  71,  71,  72,  73,  74,  75,  75,  76,  77,  78,  79,  80,  80,  81,
    82,  83,  84,  85,  86,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,
    96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
   112, 113, 114, 115, 116, 117, 118, 119, 120, 122, 123, 124, 125, 126, 127, 128,
}; 
const uint8_t GAMMA_LUT64[64] = {
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,
     2,   3,   3,   3,   3,   4,   4,   4,   5,   5,   5,   6,   6,   6,   7,   7,
     8,   8,   9,   9,  10,  10,  11,  11,  12,  13,  13,  14,  15,  15,  16,  17,
    17,  18,  19,  20,  21,  22,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,
};
const uint8_t GAMMA_LUT128[128] = {
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
     2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,
     5,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   7,   7,   7,   7,   7,
     8,   8,   8,   8,   9,   9,   9,   9,  10,  10,  10,  10,  11,  11,  11,  12,
    12,  12,  12,  13,  13,  13,  14,  14,  14,  15,  15,  15,  16,  16,  16,  17,
    17,  18,  18,  18,  19,  19,  20,  20,  20,  21,  21,  22,  22,  22,  23,  23,
    24,  24,  25,  25,  26,  26,  27,  27,  27,  28,  28,  29,  29,  30,  31,  32,
};*/

// 2.2 gamma, 32 levels
const uint8_t GAMMA_LUT256[256] = {
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,
     2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
     2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,
     3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   5,
     5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,
     6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   7,   7,   8,   8,
     8,   8,   8,   8,   8,   8,   9,   9,   9,   9,   9,   9,   9,   9,  10,  10,
    10,  10,  10,  10,  10,  11,  11,  11,  11,  11,  11,  11,  12,  12,  12,  12,
    12,  12,  12,  13,  13,  13,  13,  13,  13,  14,  14,  14,  14,  14,  14,  15,
    15,  15,  15,  15,  15,  16,  16,  16,  16,  16,  16,  17,  17,  17,  17,  17,
    18,  18,  18,  18,  18,  19,  19,  19,  19,  19,  20,  20,  20,  20,  20,  21,
    21,  21,  21,  21,  22,  22,  22,  22,  23,  23,  23,  23,  23,  24,  24,  24,
    24,  25,  25,  25,  25,  25,  26,  26,  26,  26,  27,  27,  27,  27,  28,  28,
    28,  28,  29,  29,  29,  29,  30,  30,  30,  30,  31,  31,  31,  31,  31,  32,
};
/*
// 1.8 gamma, 32 levels - smoother at higher brightness?
const uint8_t GAMMA_LUT256[256] = {
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,
     2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,
     3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,
     4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   4,   5,   5,   5,   5,
     5,   5,   5,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   6,   6,   6,
     6,   6,   7,   7,   7,   7,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,
     8,   8,   8,   8,   9,   9,   9,   9,   9,   9,   9,   9,   9,  10,  10,  10,
    10,  10,  10,  10,  10,  11,  11,  11,  11,  11,  11,  11,  12,  12,  12,  12,
    12,  12,  12,  13,  13,  13,  13,  13,  13,  13,  14,  14,  14,  14,  14,  14,
    14,  15,  15,  15,  15,  15,  15,  15,  16,  16,  16,  16,  16,  16,  17,  17,
    17,  17,  17,  17,  18,  18,  18,  18,  18,  18,  19,  19,  19,  19,  19,  19,
    20,  20,  20,  20,  20,  20,  21,  21,  21,  21,  21,  22,  22,  22,  22,  22,
    22,  23,  23,  23,  23,  23,  24,  24,  24,  24,  24,  25,  25,  25,  25,  25,
    26,  26,  26,  26,  26,  27,  27,  27,  27,  27,  28,  28,  28,  28,  28,  29,
    29,  29,  29,  29,  30,  30,  30,  30,  30,  31,  31,  31,  31,  32,  32,  32,
};*/
#define GAMMA_LUT GAMMA_LUT256  // which lookup table to use
#define PWM_STEPS 32         // coarse requency control - should match max value in LUT
#define SHIFT_BITS 1             // ADC reads as 8bit, left shift to match lookup table length
/* 64 lookup
#define GAMMA_LUT GAMMA_LUT64
#define PWM_STEPS 64
#define SHIFT_BITS 2
*/

// PWM timing variables
volatile uint8_t pwm_counter = 0; //= PWM_STEPS;
uint8_t myint_ADRESH = 1;
volatile uint8_t pwm_duty = 1;  // initial value
volatile uint8_t pwm_active;  // flag to indicate if PWM output should be on or off
volatile uint8_t phase = 0;      // A or B on, used to alternate which LED is on for PWM dimming
// State machine approach - spreads ADC across multiple ISR calls
volatile uint8_t tick_lo = 0x14;   // 9450 = 0x24EA
volatile uint8_t tick_hi = 0x00;
volatile uint8_t state_change_flag = 0;   //volatile ensures value is evaluated at each use
volatile uint8_t next_status = STATE_BRIGHT;  // start in bright state
uint8_t adc_state = 0;  // 0=idle, 1=converting
uint8_t gp3_last_state;

void setup() {
    OSCCAL = __osccal_val(); //load the calibration - important
    // Calibrate oscillator
    //asm("call 0x348C");
    //asm("call 0b10000000");
    //asm("movwf OSCCAL");
    // memory address 3fff goes back to 3444
    //OSCCAL = 0b11111100; // max value
    //OSCCAL = 0b10000000; //0x80 centre value
    //OSCCAL = 0b00000000; //min value
      
    // Timer1 enable async external crystal, 1:2 prescalar
    //T1CON = 0b00011110; // 4 sec ticks
    // should wait for settling period before TMR1ON (bit0))
    // Timer1 enable async external crystal, 1:1 prescalar
    T1CON = 0b00001110; // 2 sec ticks
    // no preload = 2sec interupt
    TMR1H = 0x80;     // 1sec overflow
    //TMR1H = 0xC0;     // 0.5sec overflow
    //TMR1H = 0xE0;     // 0.25sec overflow
    //TMR1H = 0xF0;     // 0.125sec overflow
            
    // Analogue channel setup
    //ANSEL = 0b01110100; //FRC internal clock?
    ANSEL = 0b00010100;     // 2us per sample (2x11) + 11.5 + 0.5 = 34us
    //ANSEL = 0b01010100;   // 4us per sample = (4*11) + 11.5 +0.5 = 56us
    //        -|||---- bits 6:4 ADC conversion clock 101=4us 001=2us
    //        ----|||| bits 3:0 set AN3:0 channels as analogue
    // since we are not waiting for ADC to complete 4us is better bet at 3V
    
    // GP0 & GP1 outputs
    TRISIO = 0b00001100;         // all outputs except AN2 & GP3(MCLRE))
    //ADC configuration
    ADCON0 = 0b00001001;         // left justified, Vdd ref, AN2, ADC ON
    __delay_us(20); // allow settling time
    //TMR1ON = 1;
    CMCON = 0b00000111;          // Comparator Off (Lowest power)
    // Initialise outputs
    GPIO = 0x00;
    IOC = 0b00001000;    //enable interrupt on GP3
    // Setup interrupts
    PIE1bits.ADIE = 1;   // Enable ADC interrupt
    PIE1bits.TMR1IE =1;  // Enable TMR1 interrupt
    //INTCONbits.T0IE = 1; // Enable TMR0 interrupt for WDT
    INTCONbits.PEIE = 1; // Enable peripheral interrupts
    INTCONbits.GIE = 1;  // Enable global interrupts
    INTCONbits.GPIE =1;  // Enable GPIO interrupt

    OPTION_REGbits.PSA = 1;
    OPTION_REGbits.PS = 0b111; // WDT prescaler 1:128, 18ms * 128 = ~2.3s timeout
    
    // set up GP3 state
    gp3_last_state = GPIObits.GP3;
    if (!GPIObits.GP3) {// is low - most often the case, timer is on
        next_status = STATE_BRIGHT;  //restart in default state
    } else {  // manual override, switch closed, GP3 high
        next_status = STATE_MANUAL;
            }
}

void __interrupt() ISR(void) {
    // ADC interrupt
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;  // Clear flag
        //pwm_duty = GAMMA_LUT[ADRESH >> 3];
        //pwm_duty = GAMMA_LUT[ADRESH]; //no shift if using 256 LUT saves 3us
        myint_ADRESH = ADRESH; //>> SHIFT_BITS;
    }
    // GPIO interrupt GP3
    else if (INTCONbits.GPIF) {
        uint8_t gp3 = GPIObits.GP3;  // latch the pin state first
        INTCONbits.GPIF = 0;         // Clear flag
        if (gp3 != gp3_last_state){
            // gp3 state changed
            gp3_last_state = gp3;
            state_change_flag = 1;
            if (!gp3) {// is  low - most often the case, switch ON/1; timer ON
                next_status = STATE_BRIGHT;  //restart in default state
            }
            else {   // manual override, switch OFF/0, GP3 high
                next_status = STATE_MANUAL;
            }
        }
    }
    // TMR1 interrupt - timing for state changes
    else if (PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;  // Clear flag
        TMR1H = 0x80; // reset overflow for 1sec ticks
        CLRWDT();
        //TMR1H = 0xF0;         // 8Hz 0.125ms 
        /*TMR1H = 0xff;
        TMR1L = 0xE0;*/         // 1024Hz
        //LED_A ^= 1;      // toggle GPIO
        
        /*if (delay_ticks) {
            --delay_ticks;
            if (delay_ticks == 0) {
                //LED_A = 1;
                state_change_flag = 1;   // signal main loop
            }
        }*/
        // 1Hz tick counter for state changes
        // 2 x 8bit faster than 16bit counter
        if (tick_lo || tick_hi) {
            if (--tick_lo == 0xFF && tick_hi) {    // underflow
                tick_hi--;
            }
            if (!(tick_lo | tick_hi)) { // | cheaper than two compares
                state_change_flag = 1;
            }
        }
    }
}
/*
// LUT index where each run of identical values starts
// base value:                1   2   3   4   5   6
static const uint8_t run_start[6] = {  0, 21, 34, 42, 51, 57 };

// 255 / run_length for each run
// base value:                1   2   3   4   5   6
// run length:               21  13   8   9   6   6
static const uint8_t run_inc[6]   = { 12, 20, 32, 28, 43, 43 };
static uint8_t accumulator  = 0;
static uint8_t dither_bump  = 0;

void apply_dither(void) {
    uint8_t base = GAMMA_LUT[myint_ADRESH];  // always reset to LUT value
    pwm_duty = base;

    if (base < 7)
    {
        uint8_t idx      = base - 1;
        uint8_t position = myint_ADRESH - run_start[idx];
        uint8_t fraction = position * run_inc[idx];

        uint8_t old_acc  = accumulator;
        accumulator      = old_acc + fraction;
        dither_bump      = (accumulator < old_acc) ? 1 : 0;  // carry
    }
    else
    {
        accumulator = 0;
        dither_bump  = 0;
    }
}*/
// LUT index where each run of identical values starts
// base value:                1   2   3   4   5   6
// base value:                  1   2   3   4   5   6   7
//static const uint8_t run_start[7] = {  0, 10, 17, 21, 24, 27, 30 };
// run length:                 


// 31 / run_length for each run
// base value:                1   2   3   4   5   6   7
// run length:                10  7   4   3   3   3   2
//static const uint8_t run_inc[7]   = { 26, 36, 64, 85, 85, 85, 128 };
// base value:                    1    2    3    4    5    6    7    8    9   10
/*static const uint8_t run_start[10] = {  0,  20,  33,  42,  48,  54,  59,  64,  68,  72 };
// run length:                   20   13    9    6    6    5    5    4    4    4
static const uint8_t run_inc[10]   = { 13,  20,  28,  42,  42,  51,  51,  64,  64,  64 };
*/
// base value:                    1    2    3    4    5    6    7    8    9   10
static const uint8_t run_start[32] = {
      0,  40,  65,  82,  95, 107, 117, 126, 134, 142,
    149, 156, 163, 169, 175, 181, 187, 192, 197, 202,
    207, 212, 216, 221, 225, 230, 234, 238, 242, 246,
    250, 255
};
// run length
static const uint8_t run_inc[32] = {
      6,  10,  15,  20,  21,  26,  28,  32,  32,  36,
     36,  36,  42,  42,  42,  42,  51,  51,  51,  51,
     51,  64,  51,  64,  51,  64,  64,  64,  64,  64,
     51,  255
};
/*
static const uint8_t run_start[32] = {
      0,  26,  48,  63,  76,  88,  98, 108, 116, 125, 133, 140, 147, 154, 161, 168,
    174, 180, 186, 192, 198, 203, 209, 214, 219, 224, 229, 234, 239, 244, 249, 253,
};

static const uint8_t run_inc[32] = {
     10,  12,  17,  20,  21,  26,  26,  32,  28,  32,  36,  36,  36,  36,  36,  42,
     42,  42,  42,  42,  51,  42,  51,  51,  51,  51,  51,  51,  51,  51,  64, 128,
};*/
static uint8_t last_base    = 0;
static uint8_t accumulator = 0;
static uint8_t dither_bump = 0;
uint8_t get_pwm_duty(void)
{
    uint8_t base = GAMMA_LUT[myint_ADRESH];
    dither_bump = 0;
    if (myint_ADRESH == 0)  // suppress dimming at lowest level
    {
        accumulator = 0;
        dither_bump = 0;
    }
    else if (base < 32)  //leaves max brightness undithered
    {
        if (base != last_base)
        {
            accumulator = 0;
            last_base   = base;
        }
        uint8_t idx      = base - 1;
        uint8_t position = myint_ADRESH - run_start[idx];
        uint8_t fraction = position * run_inc[idx];
        uint8_t old_acc  = accumulator;
        accumulator      = old_acc + fraction;
        if (accumulator < old_acc)
        {
            dither_bump = 1;   // flag only, do NOT increment base
        }
    }
    else
    {
        accumulator = 0;
        last_base   = 0;
    }
    return base;               // always return unbumped base
}

void run_dim(void) {
    // run PWM & read ADC until timer elapsed
    while(!state_change_flag) {
        if (++pwm_counter > PWM_STEPS) {
                pwm_counter -= PWM_STEPS;  // 30us Cheaper than = 0 with modulo
                phase ^= 1;
                //if (++adc_divider == 4 ) {  // Only start ADC every nth PWM cycle but that introduces jitter
                //150Hz, keep ADC at 2ms (50Hz)
                //adc_divider = 0;
                ADCON0bits.GO_nDONE = 1; //with 10k pot acquisition time is ~20us delay here
                // 5k pot would reduce to ~15us
                //ADCON0 |= 0x02;  // Direct bit set instead of ADCON0bits.GO_nDONE = 1
                //adc_state = 1;
                //} //2.49ms pwm loop 158Hz
                //CLRWDT();
            }
        // LED update
        //pwm_active = (pwm_counter > pwm_duty_eff);
        pwm_active = (pwm_counter > pwm_duty);
        if (phase) {
            LED_A = 1;
            LED_B = pwm_active;
        } else {
            LED_B = 1;
            LED_A = pwm_active;
        }
    }
    if(next_status != STATE_MANUAL){
        next_status = STATE_OFF;  // move to next state
    }
}

void run_bright(void) {
    // run PWM at max until timer elapsed -no need to read ADC
    while(!state_change_flag) {
        pwm_duty = PWM_STEPS;  // full on
        if (++pwm_counter > PWM_STEPS) {
                pwm_counter -= PWM_STEPS;  // 30us Cheaper than = 0 with modulo
                phase ^= 1;
                //ADCON0bits.GO_nDONE = 1; //with 10k pot acquisition time is ~20us delay here
                // 5k pot would reduce to ~15us
                //CLRWDT();
        }
        // LED update
        //pwm_active = (pwm_counter > pwm_duty_eff);
        pwm_active = (pwm_counter > pwm_duty);
        if (phase) {
            LED_A = 1;
            LED_B = pwm_active;
        } else {
            LED_B = 1;
            LED_A = pwm_active;
        }
    }
    if(next_status != STATE_MANUAL){
      next_status = STATE_DIM;  // move to next state  
    }
}

void lights_off(void) {
    // keep lights off until timer elapsed
    LED_A = 1;
    LED_B = 1;
    while(!state_change_flag) {
        SLEEP();
        NOP();
    }
    if(next_status != STATE_MANUAL){
        next_status = STATE_BRIGHT;  // move to next state
    }
}

void manual_override(void) {
    // run on dimmer, with no timer
    uint8_t pwm_duty_dithered = 0;
    static uint8_t adc_divider = 0;
    while(!state_change_flag) {
        // phase A
        while(!state_change_flag) {
            ++pwm_counter;
            LED_A = 1;
            //LED_B = (pwm_counter > pwm_duty);
            LED_B = (pwm_counter > pwm_duty_dithered);
            /*if (pwm_counter == PWM_STEPS - 10) {   // 10 steps from end, always in off period
                ADCON0bits.GO_nDONE = 1;           // as long as pwm_duty < PWM_STEPS - 10
            }*/
            //ADCON0bits.GO_nDONE = 1;
            if (pwm_counter >= PWM_STEPS) {
                pwm_counter = 0;
                phase ^= 1;
                pwm_duty = get_pwm_duty();   // update duty, dither_bump set inside
                pwm_duty_dithered = pwm_duty + dither_bump;  // computed once per period
            // at phase A rollover, replace ADCON0bits.GO_nDONE = 1 with:
            /*if (++adc_divider >= 60) {
                adc_divider = 0;
                ADCON0bits.GO_nDONE = 1;
            }*/
                break;
            }
        }
        // phase B - 
        while(!state_change_flag) {
            ++pwm_counter;
            LED_B = 1;
            //LED_A = (pwm_counter > pwm_duty);
            LED_A = (pwm_counter > pwm_duty_dithered);
            if (pwm_counter == PWM_STEPS - 10) {   // 10 steps from end, always in off period
                ADCON0bits.GO_nDONE = 1;           // as long as pwm_duty < PWM_STEPS - 10
            }
            //ADCON0bits.GO_nDONE = 1;
            if (pwm_counter >= PWM_STEPS) {
                pwm_counter = 0;
                phase ^= 1;
                break;
            }
        }
        CLRWDT();
    }
    if(next_status != STATE_MANUAL){
        next_status = STATE_BRIGHT;
    }
}

void main(void) {
    setup();
    //static uint8_t adc_divider = 0;
    while(1) {
        // check state and run appropriate function
        state_change_flag = 0;
        switch(next_status) {;
            case STATE_MANUAL: 
                //testing, no timer, read ADC
                TMR1ON = 0; //stop the timer
                manual_override();
                break;
            case STATE_BRIGHT: 
                //start timer for bright state 0600-1635
                // 10 hours 35 minutes = 38,100 seconds
                tick_lo = 0xD4;   // 38100 = 0x94D4
                tick_hi = 0x94;
                /*tick_lo = 0x0a;
                tick_hi = 0x00;*/
                TMR1ON = 1;
                run_bright();
                break;
            case STATE_DIM:
                //start timer for dim state 1635-2300
                // 6 hours 25 minutes = 23,100 seconds
                tick_lo = 0x3C;   // 23100 = 0x5A3C
                tick_hi = 0x5A;
                /*tick_lo = 0x0a;
                tick_hi = 0x00;*/
                run_dim();
                break;
            default:
                //start timer for off state
                // 7 hours 0 minutes = 25,200 seconds
                tick_lo = 0x70;    // 25200 = 0x6270
                tick_hi = 0x62; 
                /*tick_lo = 0x0a;
                tick_hi = 0x00;*/
                lights_off();
                break;
        }
    }
}

