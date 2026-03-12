/*
 * main.c  —  Dithered complementary PWM dimmer
 * Target:    PIC12F675
 * Toolchain: MPLAB X / XC8
 *
 * Overview
 * --------
 * Two complementary PWM outputs (GP0 / GP1) are generated in software.
 * A 256-entry gamma-correction LUT maps the 8-bit ADC reading to
 * one of 32 coarse duty-cycle steps.  Temporal dithering then interpolates
 * between steps, smoothing changes in brightness levels at a
 * PWM frequency of ~595 Hz (each step ≈ 22 µs).
 *
 * State machine
 * -------------
 *   STATE_BRIGHT  — full PWM, timed  (≈10h 35m)
 *   STATE_DIM     — dithered PWM, timed  (≈6h 25m)
 *   STATE_OFF     — outputs dark, CPU sleeping  (≈7h 00m)
 *   STATE_MANUAL  — dithered PWM, timer disabled (switch override)
 *
 * Timing note
 * -----------
 * frequency = 1 / (2 * PWM_STEPS * 22µs  +  75µs + 35µs)
 * ADC acquisition + processing ≈ 75 µs; code overhead ≈ 35 µs.
 *
 * File layout
 * -----------
 *   config.h      — fuse bits, pin defines, state constants, tick preloads
 *   pwm_tables.h  — extern declarations for LUTs
 *   pwm_tables.c  — GAMMA_LUT256, run_start[], run_inc[] (flash, page 2)
 *   main.c        — this file: ISR, dither logic, state functions, main()
 */

#include "config.h"
#include "pwm_tables.h"
#include "pwm_tables.c"

/* ==========================================================================
 * Module-level variables
 * ========================================================================== */

/* --- Dithering state ------------------------------------------------------ */
static uint8_t last_base   = 0;   // Previous coarse duty, detects level change
static uint8_t accumulator = 0;   // Running fractional accumulator
static uint8_t dither_bump = 0;   // Set to 1 when accumulator overflows

/* --- PWM state ------------------------------------------------------------ */
volatile uint8_t pwm_counter = 0;
volatile uint8_t pwm_duty    = 1;   // Current coarse duty step (1..PWM_STEPS)
volatile uint8_t pwm_active;        // Scratch flag used in run_bright()
volatile uint8_t phase       = 0;   // Alternates which output leads each period

/* --- ADC result (latched inside ISR, consumed in main context) ------------ */
uint8_t myint_ADRESH = 1;

/* --- Timer / state-machine ------------------------------------------------ */
volatile uint8_t tick_lo          = 0x14;  // 16-bit 1 Hz down-counter (lo byte)
volatile uint8_t tick_hi          = 0x00;  //                           (hi byte)
volatile uint8_t state_change_flag = 0;    // Set by ISR; cleared by main loop
volatile uint8_t next_status      = STATE_BRIGHT;

/* --- GPIO edge detection -------------------------------------------------- */
uint8_t gp3_last_state;

/* ==========================================================================
 * Hardware initialisation
 * ========================================================================== */
void setup(void)
{
    OSCCAL = __osccal_val();          // Apply factory oscillator calibration

    /* Timer1 — async external 32.768 kHz crystal, 1:1 prescaler
     * TMR1H = 0x80 → overflows every 32768 counts = 1 second             */
    T1CON = 0b00001110;
    TMR1H = 0x80;

    /* ADC — left-justified, Vdd ref, AN2 selected, 2 µs conversion clock  */
    ANSEL  = 0b00010100;              // bits 6:4 = 001 (2 µs); AN2 analogue
    ADCON0 = 0b00001001;              // left-justify, Vdd ref, AN2, ADC ON
    __delay_us(20);                   // Allow ADC input to settle

    /* GPIO direction — GP0 & GP1 outputs; GP2 (AN2) & GP3 (switch) inputs */
    TRISIO = 0b00001100;
    GPIO   = 0x00;

    /* Comparator off (lowest power) */
    CMCON = 0b00000111;

    /* GPIO interrupt on GP3 (manual-override switch) */
    IOC = 0b00001000;

    /* Interrupts */
    PIE1bits.ADIE   = 1;             // ADC complete
    PIE1bits.TMR1IE = 1;             // Timer1 overflow (1 Hz tick)
    INTCONbits.PEIE = 1;             // Peripheral interrupts
    INTCONbits.GIE  = 1;             // Global interrupts
    INTCONbits.GPIE = 1;             // GPIO pin-change interrupt

    /* Watchdog — prescaler assigned to WDT, 1:128 → ~2.3 s timeout */
    OPTION_REGbits.PSA = 1;
    OPTION_REGbits.PS  = 0b111;

    /* Initialise GP3 edge-detection and select starting state */
    gp3_last_state = GPIObits.GP3;
    next_status = GPIObits.GP3 ? STATE_MANUAL : STATE_BRIGHT;
}

/* ==========================================================================
 * Interrupt service routine
 * ========================================================================== */
void __interrupt() ISR(void)
{
    /* -- ADC conversion complete ------------------------------------------ */
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF  = 0;
        myint_ADRESH   = ADRESH;      // Latch result; consumed by get_pwm_duty()
    }

    /* -- GP3 pin-change (manual-override switch) --------------------------- */
    else if (INTCONbits.GPIF) {
        uint8_t gp3 = GPIObits.GP3;  // Latch pin before clearing flag
        INTCONbits.GPIF = 0;
        if (gp3 != gp3_last_state) {
            gp3_last_state     = gp3;
            state_change_flag  = 1;
            next_status = gp3 ? STATE_MANUAL : STATE_BRIGHT;
        }
    }

    /* -- Timer1 overflow (1 Hz tick, used for state-duration countdown) ---- */
    else if (PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;
        TMR1H = 0x80;                 // Reload for next 1-second period
        CLRWDT();

        if (tick_lo || tick_hi) {
            if (--tick_lo == 0xFF && tick_hi) {   // Borrow from high byte
                tick_hi--;
            }
            if (!(tick_lo | tick_hi)) {           // Counter reached zero
                state_change_flag = 1;
            }
        }
    }
}

/* ==========================================================================
 * Dithering  —  called once per PWM period inside run_dim() / manual_override()
 *
 * Returns the coarse (un-bumped) duty step.
 * Sets the module-level dither_bump flag when the fractional accumulator
 * overflows, so the caller can add 1 to the duty for that single period.
 * ========================================================================== */
uint8_t get_pwm_duty(void)
{
    uint8_t base = GAMMA_LUT[myint_ADRESH];
    dither_bump  = 0;

    if (myint_ADRESH == 0) {
        /* Minimum ADC reading — suppress all dithering at blackout level */
        accumulator = 0;
    }
    else if (base < PWM_STEPS) {
        /* Mid-range: apply temporal dithering between coarse steps */
        if (base != last_base) {
            accumulator = 0;          // New brightness level — reset accumulator
            last_base   = base;
        }
        uint8_t idx      = base - 1;
        uint8_t position = myint_ADRESH - run_start[idx];
        uint8_t fraction = position * run_inc[idx];
        uint8_t old_acc  = accumulator;
        accumulator      = old_acc + fraction;
        if (accumulator < old_acc) {
            dither_bump = 1;          // Overflow → bump duty by 1 this period
        }
    }
    else {
        /* Maximum brightness — no dithering needed */
        accumulator = 0;
        last_base   = 0;
    }

    return base;                      // Always return the un-bumped base value
}

/* ==========================================================================
 * State: BRIGHT  —  full PWM output until timer expires
 *
 * pwm_duty is pinned to PWM_STEPS (100 % on-time).
 * Phase alternates each period so both outputs share the load equally.
 * No ADC reads needed in this state.
 * ========================================================================== */
void run_bright(void)
{
    pwm_duty = PWM_STEPS;

    while (!state_change_flag) {
        if (++pwm_counter > PWM_STEPS) {
            pwm_counter -= PWM_STEPS;   // Cheaper than = 0 with modulo
            phase ^= 1;
        }
        pwm_active = (pwm_counter > pwm_duty);
        if (phase) {
            LED_A = 1;
            LED_B = pwm_active;
        } else {
            LED_B = 1;
            LED_A = pwm_active;
        }
    }

    if (next_status != STATE_MANUAL) {
        next_status = STATE_DIM;
    }
}

/* ==========================================================================
 * State: DIM  —  dithered PWM with ADC brightness control, timed
 *
 * Phase A and phase B each occupy PWM_STEPS counts.
 * ADC conversion is triggered near the end of phase B (always in the
 * off-period, as long as pwm_duty < PWM_STEPS - 10).
 * ========================================================================== */
void run_dim(void)
{
    uint8_t pwm_duty_dithered = 0;

    while (!state_change_flag) {

        /* --- Phase A: LED_A on, LED_B carries the duty-cycle edge --- */
        while (!state_change_flag) {
            ++pwm_counter;
            LED_A = 1;
            LED_B = (pwm_counter > pwm_duty_dithered);
            if (pwm_counter >= PWM_STEPS) {
                pwm_counter       = 0;
                phase            ^= 1;
                pwm_duty          = get_pwm_duty();          // Updates dither_bump
                pwm_duty_dithered = pwm_duty + dither_bump;  // Computed once per period
                break;
            }
        }

        /* --- Phase B: LED_B on, LED_A carries the duty-cycle edge --- */
        while (!state_change_flag) {
            ++pwm_counter;
            LED_B = 1;
            LED_A = (pwm_counter > pwm_duty_dithered);
            if (pwm_counter == PWM_STEPS - 10) {
                ADCON0bits.GO_nDONE = 1;    // Start ADC in guaranteed off-time
            }
            if (pwm_counter >= PWM_STEPS) {
                pwm_counter = 0;
                phase      ^= 1;
                break;
            }
        }
    }

    if (next_status != STATE_MANUAL) {
        next_status = STATE_OFF;
    }
}

/* ==========================================================================
 * State: OFF  —  both outputs dark, CPU sleeping between timer ticks
 * ========================================================================== */
void lights_off(void)
{
    LED_A = 1;
    LED_B = 1;

    while (!state_change_flag) {
        SLEEP();
        NOP();
    }

    if (next_status != STATE_MANUAL) {
        next_status = STATE_BRIGHT;
    }
}

/* ==========================================================================
 * State: MANUAL  —  dithered PWM with ADC control, no timer, WDT kept alive
 *
 * Identical to run_dim() except:
 *  - TMR1 is stopped before entry (see main)
 *  - CLRWDT() is called once per outer loop iteration
 * ========================================================================== */
void manual_override(void)
{
    uint8_t pwm_duty_dithered = 0;

    while (!state_change_flag) {

        /* --- Phase A --- */
        while (!state_change_flag) {
            ++pwm_counter;
            LED_A = 1;
            LED_B = (pwm_counter > pwm_duty_dithered);
            if (pwm_counter >= PWM_STEPS) {
                pwm_counter       = 0;
                phase            ^= 1;
                pwm_duty          = get_pwm_duty();
                pwm_duty_dithered = pwm_duty + dither_bump;
                break;
            }
        }

        /* --- Phase B --- */
        while (!state_change_flag) {
            ++pwm_counter;
            LED_B = 1;
            LED_A = (pwm_counter > pwm_duty_dithered);
            if (pwm_counter == PWM_STEPS - 10) {
                ADCON0bits.GO_nDONE = 1;
            }
            if (pwm_counter >= PWM_STEPS) {
                pwm_counter = 0;
                phase      ^= 1;
                break;
            }
        }

        CLRWDT();
    }

    if (next_status != STATE_MANUAL) {
        next_status = STATE_BRIGHT;
    }
}

/* ==========================================================================
 * main  —  hardware init, then state machine dispatch loop
 * ========================================================================== */
void main(void)
{
    setup();

    while (1) {
        state_change_flag = 0;

        switch (next_status) {

            case STATE_MANUAL:
                TMR1ON = 0;           // Timer not needed in manual mode
                manual_override();
                break;

            case STATE_BRIGHT:
                tick_lo = TICK_BRIGHT_LO;
                tick_hi = TICK_BRIGHT_HI;
                TMR1ON  = 1;
                run_bright();
                break;

            case STATE_DIM:
                tick_lo = TICK_DIM_LO;
                tick_hi = TICK_DIM_HI;
                /* TMR1 already running from STATE_BRIGHT */
                run_dim();
                break;

            default:                  /* STATE_OFF */
                tick_lo = TICK_OFF_LO;
                tick_hi = TICK_OFF_HI;
                lights_off();
                break;
        }
    }
}
