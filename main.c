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
#pragma config WDTE = ON
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
  
// Pre-calculated gamma correction table (val^gamma)
// 2.2 gamma, 32 levels, store on all of page 2
const uint8_t __at(0x200) GAMMA_LUT256[256] = {
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
#define GAMMA_LUT GAMMA_LUT256  // which lookup table to use
#define PWM_STEPS 32         // coarse requency control - should match max value in LUT
//#define SHIFT_BITS 1             // ADC reads as 8bit, left shift to match lookup table length

// base value:                    1    2    3    4    5    6    7    8    9   10
// LUT index where each run of identical values starts
static const uint8_t run_start[32] = {
      0,  40,  65,  82,  95, 107, 117, 126, 134, 142,
    149, 156, 163, 169, 175, 181, 187, 192, 197, 202,
    207, 212, 216, 221, 225, 230, 234, 238, 242, 246,
    250, 255
};
// LUT for number of dither steps
// 31 / run_length for each run
static const uint8_t run_inc[32] = {
      6,  10,  15,  20,  21,  26,  28,  32,  32,  36,
     36,  36,  42,  42,  42,  42,  51,  51,  51,  51,
     51,  64,  51,  64,  51,  64,  64,  64,  64,  64,
     51,  255
};
static uint8_t last_base    = 0;
static uint8_t accumulator = 0;
static uint8_t dither_bump = 0;

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
      
    // Timer1 enable async external crystal, 1:1 prescalar
    T1CON = 0b00001110; // 2 sec ticks
    // no preload = 2sec interupt
    TMR1H = 0x80;     // 1sec overflow
    //TMR1H = 0xC0;     // 0.5sec overflow
    //TMR1H = 0xE0;     // 0.25sec overflow
    //TMR1H = 0xF0;     // 0.125sec overflow
            
    // Analogue channel setup
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
    uint8_t pwm_duty_dithered = 0;
    static uint8_t adc_divider = 0;
    while(!state_change_flag) {
        // phase A
        while(!state_change_flag) {
            ++pwm_counter;
            LED_A = 1;
            //LED_B = (pwm_counter > pwm_duty);
            LED_B = (pwm_counter > pwm_duty_dithered);
            if (pwm_counter >= PWM_STEPS) {
                pwm_counter = 0;
                phase ^= 1;
                pwm_duty = get_pwm_duty();   // update duty, dither_bump set inside
                pwm_duty_dithered = pwm_duty + dither_bump;  // computed once per period
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
    }
    if(next_status != STATE_MANUAL){
        next_status = STATE_OFF;  // move to next state
    }
}

void run_bright(void) {
    // run PWM at max until timer elapsed -no need to read ADC
    // constantly testing pwm_counter is inefficient = ~500Hz
    // but acceptable in this use - compare vs run_dim
    pwm_duty = PWM_STEPS;  // full on
    while(!state_change_flag) {
        if (++pwm_counter > PWM_STEPS) {
                pwm_counter -= PWM_STEPS;  // 30us Cheaper than = 0 with modulo
                phase ^= 1;
        }
        // LED update
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
                /*tick_lo = 0xD4;   // 38100 = 0x94D4
                tick_hi = 0x94;*/
                tick_lo = 0x0a;
                tick_hi = 0x00;
                TMR1ON = 1;
                run_bright();
                break;
            case STATE_DIM:
                //start timer for dim state 1635-2300
                // 6 hours 25 minutes = 23,100 seconds
                /*tick_lo = 0x3C;   // 23100 = 0x5A3C
                tick_hi = 0x5A;*/
                tick_lo = 0x0a;
                tick_hi = 0x00;
                run_dim();
                break;
            default:
                //start timer for off state
                // 7 hours 0 minutes = 25,200 seconds
                /*tick_lo = 0x70;    // 25200 = 0x6270
                tick_hi = 0x62; */
                tick_lo = 0x0a;
                tick_hi = 0x00;
                lights_off();
                break;
        }
    }
}
