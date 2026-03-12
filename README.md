# 2 wire LED string power supply & dimmer
logic control for an H-bridge based PSU. Esssentially 2 complementary PWM signals that never go above 50% duty cycle. Intended to drive &amp; PWM dim a string of festive LED's

Using a PIC12F675 and a BD62110 Motor Driver IC to create a  +/-24V PWM signal. The PWM is software (rather than hardware) based and operates on a ~150Hz period.

A string of 150 Christmas fairy lights (drawing ~100mA) on 2 wires, supplied with a CZJUTAI power supply (model JT-EL/FC249"-G4). The PSU output is rated between 24 & 26V, 9W (so somewhere in the region of 360mA rated output). The power supply has a number of flashing modes/patterns and a auto-off timer (6 hours on 18 off). The steady-on mode works fine, but the lights are too bright of an evening. I do not want any of the flashing modes, but I would like to be able to dim the lights.

The power supply seems to contain an H-bridge which takes 25V DC and outputs 2 complementary sqaure waves at about 242Hz. It may be perfectly possible to deconstruct the PSU & replace the microcontroller, but I thought it more of an interesting exercise to start from scratch. PIC12F675 is an 8pin 4MHz MCU, operating at 1us per instruction. The BD62110 is an H-bridge with some additional features like MOSFET driver circuit & crossover protection, it will handle from 8V to 28V input and drive up to 1Amp continuous current.

On the PIC, two output pins will drive the BD62110, one ADC input pin will read the potentiometer voltage to change the PWM duty cycle. On the BD62110 we will feed in 24V DC and the input logic. The LED string has its own current limiting circuitry, so the output will go directly to the LED string.

It is amazing how much learning can be achieved by delving into a simple device. This branch adds dithering to the timer controlled PWM method. The number of PWM steps is determined by the maximum value in the lookup table. A range of 128 allowsvalues allows for plenty of discrtete levels of brightness, the table is gamma corrected at 2.2 to roughly approximate human perception of light increase. With this correction the first 21 entries are PWM duty of 1, the first 10 PWM steps consume almost 1/3 or the table. Additionally the eye is most sensitive to these changes - going from 1 to 2 duty cycles represents a 50% brightness increase. The result was that you would turn the potentiometer for some distance with no change, then a sudden visible step change happens. The first 8 or 10 steps are clearly visible. Applying dithering at 150 to 200Hz did not work, creating a visible flickering. In this bracnch I have kept the LUT to a length of 256, but the maximum value is now 32. Having only 32 PWM steps, at which point  significantly improves PWM frequency (along with some code improvements) to about 650Hz at which point dithering becomes effective. I did try a gamma of 1.8 to try and smooth out step changes at brighter levels, but reverted to gamma 2.2.

![PCB schematic generated from KiCad](./24V_PWM_schematic.png)

at minimum duty cycle the on period for each LED is 32us;

![screen capture from oscilloscope showing 1% duty cycle waveform and timing](./WA000013.png)

at 100% the duty cycle is in reality 50% on, 50% off;

![screen capture from oscilloscope showing 100% duty cycle waveform](./WA000012.png)

and at some intermediate duty cycle;

![screen capture from oscilloscope showing a small duty cycle waveform](./WA000010.png)

These examples are using a 7bit ADC mapped to 100 levels of brightness, so each duty cycle consists of 
(32us × 100 possibly high according to duty cycle) + (32us × 100 definitely low) ≈ 164Hz

easily fits a 66 × 32 mm board

![render of the populated PCB](./PCB.png)
