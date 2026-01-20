# 2wire_LED_PSU
logic control for an H-bridge based PSU. Intended to drive &amp; PWM dim a string of festive LED's

Using a PIC12F675 and a BD62110 Motor Driver IC to create a  +/-24V PWM signal. Operates at 1-100% duty cycle on a ~127Hz period.

A string of 150 Christmas fairy lights (pulling unknown current) on 2 wires, supplied with a CZJUTAI power supply (model JT-EL/FC249"-G4). The PSU output is rated between 24& 26V, 9W. The power supply has a number of flashing modes/patterns and a auto-off timer (6hours on 18 off?). The steady-on mode works fine, but the lights are too bright of an evening. I do not want any of the flashing modes, but I would like to be able to dim the lights.

The power supply seems to contain an H-bridge which takes 25V DC and outputs 50V peak to peak sqaure wave at about 242Hz. It may be possible to deconstruct the PSU & replace the microcontroller, but I thought it neater to start from scratch. PIC12F675 is an 8pin 4MHz MCU, operating at 1us per instruction. The BD62110 is an H-bridge with some additional features like MOSFET driver circuit & crossover protection, it will take up 8V to 28V input and drive up to 1Amp continuous current.

On the PIC two output pins will drive the BD62110, one ADC input pin will read the potentiometer voltage to change the PWM duty cycle. On the BD62110 we will feed in 24V DC and the input logic. I believe the LED string has its own current limiting circuitry, so the output will go directly to the LED string.

