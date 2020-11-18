# Tiny RC control - Control leds or loads with your RC transmitter/receiver using an ATtiny85

Control leds or loads with your RC receiver, using a 3 way switch. This project is based on the ATtiny85. The initial setup is for rc-airplane lights, but the program is highly custamisable with settings at the start of the program file.

Several functions implemented:
* Switch position 1: All off.
* Switch position 2: Pulsating, flashing and/or constant on leds/loads.
* Switch position 3: Switches on extra constant on leds/loads.

LEDS / LOADS can be driven by:

* Directly driven from attiny's output using a calculated series resistor

* Switch load (in 0V line) with an AO3400 mosfet, when load is powered by external
     power source (flight battery). Open mosfet through 1K resistor from attiny.  
* Switch load (in VCC line) with an AO3401 mosfet, when load is powered by the same 
     power source as the attiny (through servo cable). Open mosfet through 1K resistor
     from attiny. invert the output using '| INVERT'.  
* Drive leds with the CN5611 (or CN5612). They can be driven directly with the calculated
     resistor value from the attiny to the led driver. The pinout of the CN5611 is also 
     very convenient when using several chips on a small board.  
* Drive the input of a PT4115 with a 1K series resistor. This chip can be found in some 
     automobile constant current led lights.

LED brightnes is corrected according to CIE 1931, as pwm is not linear with eye perception.

The Attimy85 needs no futher external parts except a 0.1uF decoupling capacitor. It can be programmed with a cheap usbasp. the DIP version ATtiny85 is used in this project. One can add a 10 to 6 pin adapter to the usbasp and connect the adapter directly to a attiny85 DIP version development board. The pins match exactly. All these parts are very cheap. Program it as a Digispark (8Mhz - No USB).


