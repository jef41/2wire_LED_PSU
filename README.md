# 2 wire LED string power supply & dimmer
logic control for an H-bridge based PSU. Esssentially 2 complementary PWM signals that never go above 50% duty cycle. Intended to drive &amp; PWM dim a string of festive LED's

Using a PIC12F675 and a BD62110 Motor Driver IC to create a  +/-24V PWM signal. Operates at 1-100% duty cycle on a ~160Hz period.

A string of 150 Christmas fairy lights (pulling unknown current) on 2 wires, supplied with a CZJUTAI power supply (model JT-EL/FC249"-G4). The PSU output is rated between 24 & 26V, 9W (so somewhere in the region of 360mA rated output). The power supply has a number of flashing modes/patterns and a auto-off timer (6 hours on 18 off?). The steady-on mode works fine, but the lights are too bright of an evening. I do not want any of the flashing modes, but I would like to be able to dim the lights.

The power supply seems to contain an H-bridge which takes 25V DC and outputs 50V peak to peak sqaure wave at about 242Hz. It may be possible to deconstruct the PSU & replace the microcontroller, but I thought it neater to start from scratch. PIC12F675 is an 8pin 4MHz MCU, operating at 1us per instruction. The BD62110 is an H-bridge with some additional features like MOSFET driver circuit & crossover protection, it will handle from 8V to 28V input and drive up to 1Amp continuous current.

On the PIC, two output pins will drive the BD62110, one ADC input pin will read the potentiometer voltage to change the PWM duty cycle. On the BD62110 we will feed in 24V DC and the input logic. I believe the LED string has its own current limiting circuitry, so the output will go directly to the LED string.

![PCB schematic generated from KiCad](./24V_PWM_schematic.png)

at 1% duty cycle the on period for each LED is 32us;

![screen capture from oscilloscope showing 1% duty cycle waveform and timing](./WA000013.png)

at 100% the duty cycle is in reality 50% on, 50% off;

![screen capture from oscilloscope showing 100% duty cycle waveform](./WA000012.png)

and at some intermediate duty cycle;

![screen capture from oscilloscope showing a small duty cycle waveform](./WA000010.png)

These examples are using a 7bit ADC mapped to 100 levels of brightness, so each duty cycle consists of 
(32us × 100 possibly high according to duty cycle) + (32us × 100 definitely low) ≈ 164Hz

easily fits a 66 × 32 mm board

![render of the populated PCB](./PCB.png)
