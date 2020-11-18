#include <Arduino.h>

/*
 * tiny85rccontrol - Tiny 85 RC Control
 *
 * Copyright (C) 2020      Eric Woudstra
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License v2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

// Definitions not to be changed

#define MEASUREPIN       0x0A00
#define PULSATING_LED    0x0B00
#define FLASHING_LED     0x0C00
#define ONOFF1_LED       0x0D00
#define ONOFF2_LED       0x0E00

#define DIMMASK          0x00FF 
#define FUNCTIONMASK     0x0F00 
#define INVERT           0x8000 

///////////////////////////
// Custom settings begin //
///////////////////////////

// PIN definitions: pinfunction | dimvalue
static const unsigned int pinfunction[] = {
  MEASUREPIN             |   0,      // PB0
  PULSATING_LED          |   0,      // PB1
  ONOFF2_LED             |   0,      // PB2
  ONOFF1_LED             |   0,      // PB3
  FLASHING_LED           |   0,      // PB4
};

// Only PB0, PB1 and PB4 can be a pulsating led
// 1 < dimvalue < 255 enables  dim
//     dimvalue = 0   disables dim
// Only PB0, PB1 and PB4 can have a dim value other then 0

/*
 * More examples:
static const byte pinfunction[] = {
  ONOFF1_LED,            |   0,      // PB0
  ONOFF1_LED | INVERT,   |   0,      // PB1
  ONOFF2_LED,            |   0,      // PB2
  ONOFF2_LED | INVERT,   |   0,      // PB3
  MEASUREPIN,            |   0,      // PB4
};
static const byte pinfunction[] = {
  MEASUREPIN,            |   0,      // PB0
  FLASHING_LED | INVERT, |   0,      // PB1
  PULSATING_LED,         |   0,      // PB2
  ONOFF1_LED,            |   0,      // PB3
  ONOFF2_LED,            | 128,      // PB4
};
*/

// Definitions about output timing
#define CYCLEFREQ 50          // in Hertz, minimal 50 recommended

#define FLASHINGPERIOD     50 // in (1/CYCLEFREQ) seconds
#define FLASHINGTIME        1 // in (1/CYCLEFREQ) seconds
#define PULSATINGPERIOD   150 // in (1/CYCLEFREQ) seconds

// Definitions about the switch
#define SWITCHOFF      1 // swap 1 and 3 when the switch switches up side down
#define SWITCHON       2 // 2 = middle position
#define SWITCHONPLUS   3 // 0 = no known position

#define SWITCHNOSIGNAL SWITCHONPLUS // when no valid signal received, 
                                    // run program in this switch position

// Definitions about the input pulse in microseconds
#define MINPERIOD   18000
#define REALPERIOD  20000
#define MAXPERIOD   22000
#define MINPULSE      800
#define S1PULSE      1100
#define S2PULSE      1500
#define S3PULSE      1900
#define MAXPULSE     2200
#define NOSIGNAL  1000000

/////////////////////////
// Custom settings end //
/////////////////////////

// CIE 1931 definitions, for linear brightness correction
#define CIE_ARRAY_SIZE 101
#define CIE_RANGE 255
#define CIE(X) ( ( X<(8.0*(CIE_ARRAY_SIZE-1)/100) )? \
   0.5+  X*100.0*CIE_RANGE/((CIE_ARRAY_SIZE-1)*902.3) : \
   0.5+ ((X*100.0/(CIE_ARRAY_SIZE-1)+16.0)/116.0) * \
        ((X*100.0/(CIE_ARRAY_SIZE-1)+16.0)/116.0) * \
        ((X*100.0/(CIE_ARRAY_SIZE-1)+16.0)/116.0) * CIE_RANGE \
)
static const PROGMEM byte cie_table[CIE_ARRAY_SIZE] = { 0,
  CIE( 1),CIE( 2),CIE( 3),CIE( 4),CIE( 5),CIE( 6),CIE( 7),CIE( 8),CIE( 9),CIE(10),
  CIE(11),CIE(12),CIE(13),CIE(14),CIE(15),CIE(16),CIE(17),CIE(18),CIE(19),CIE(20),
  CIE(21),CIE(22),CIE(23),CIE(24),CIE(25),CIE(26),CIE(27),CIE(28),CIE(29),CIE(30),
  CIE(31),CIE(32),CIE(33),CIE(34),CIE(35),CIE(36),CIE(37),CIE(38),CIE(39),CIE(40),
  CIE(41),CIE(42),CIE(43),CIE(44),CIE(45),CIE(46),CIE(47),CIE(48),CIE(49),CIE(50),
  CIE(51),CIE(52),CIE(53),CIE(54),CIE(55),CIE(56),CIE(57),CIE(58),CIE(59),CIE(60),
  CIE(61),CIE(62),CIE(63),CIE(64),CIE(65),CIE(66),CIE(67),CIE(68),CIE(69),CIE(70),
  CIE(71),CIE(72),CIE(73),CIE(74),CIE(75),CIE(76),CIE(77),CIE(78),CIE(79),CIE(80),
  CIE(81),CIE(82),CIE(83),CIE(84),CIE(85),CIE(86),CIE(87),CIE(88),CIE(89),CIE(90),
  CIE(91),CIE(92),CIE(93),CIE(94),CIE(95),CIE(96),CIE(97),CIE(98),CIE(99),CIE(100),
};
#define cie(X) pgm_read_byte(cie_table + (((long)(CIE_ARRAY_SIZE-1)*X)/255) );

#include <avr/power.h>

// Global variables used in interrupt
volatile static unsigned long lastrising = -NOSIGNAL;
volatile static long inputperiod = REALPERIOD;
volatile static long inputpulse = 0;
volatile static byte swi = 0;
volatile static byte measurepin = 0;
volatile static byte sw[4] = {0,0,0,0};

//pin change interrupt 0
ISR(PCINT0_vect)
{
  unsigned long now = micros();
  if (digitalRead(measurepin)==HIGH) {
    inputperiod = now - lastrising;
    lastrising = now;
  }
  else {
    if (  (inputperiod <= (long)MINPERIOD) || (inputperiod >= (long)MAXPERIOD)) {
      sw[swi] = 0;
      inputperiod = REALPERIOD;
    }
    else {
      inputpulse = now - lastrising;   
      inputpulse = (inputpulse * (long)(REALPERIOD>>2)) / (inputperiod>>2);
      if ((inputpulse  <= (long)MINPULSE ) || (inputpulse  >= (long)MAXPULSE))  sw[swi] = 0;
      else {
        if      (inputpulse < (((long)S1PULSE+(long)S2PULSE)/2)) sw[swi] = 1;
        else if (inputpulse < (((long)S2PULSE+(long)S3PULSE)/2)) sw[swi] = 2;
        else                                                     sw[swi] = 3;
      }
    }
    swi++; swi &= 3;
  }
}

static void digitalWrites(unsigned int function, byte state) {
  for (byte pin = 0 ; pin < sizeof(pinfunction) ; pin++) {
    if (function == (pinfunction[pin] & FUNCTIONMASK)) {
      int dim=pinfunction[pin] & DIMMASK;
      if (dim) {
        dim=cie(dim);
        if ((pinfunction[pin] & INVERT) == 0) analogWrite(pin, dim);
        else analogWrite(pin, 255 - dim);
      } else {
        if ((pinfunction[pin] & INVERT) == 0) digitalWrite(pin, state);
        else digitalWrite(pin, LOW + HIGH - state);
      }
    }
  }
}
static void analogWrites(unsigned int function, byte value) {
  for (byte pin = 0 ; pin < sizeof(pinfunction) ; pin++) {
    if (function == (pinfunction[pin] & FUNCTIONMASK)) {
      int dim=pinfunction[pin] & DIMMASK;
      if (dim) value=(value*dim)>>8;
      value=cie(value);
      if ((pinfunction[pin] & INVERT) == 0) analogWrite(pin, value);
      else analogWrite(pin, 255 - value);
    }
  }
}

// the setup function runs once when you press reset or power the board
void setup()
{
//  OSCCAL += 3; // correct calibration for 5V operation

  if(F_CPU == 8000000) clock_prescale_set(clock_div_1);
  if(F_CPU == 4000000) clock_prescale_set(clock_div_2);
  if(F_CPU == 2000000) clock_prescale_set(clock_div_4);
  if(F_CPU == 1000000) clock_prescale_set(clock_div_8);

  for (byte pin = 0 ; pin < sizeof(pinfunction) ; pin++) {
    if (MEASUREPIN == (pinfunction[pin] & FUNCTIONMASK)) {
      measurepin = pin;
      pinMode(pin, INPUT);
    }
    else {
      pinMode(pin,  OUTPUT);
      if ((pinfunction[pin] & INVERT) == 0) digitalWrite(pin, LOW);
      else digitalWrite(pin, HIGH);
    }
  }
  // This part may be different on avrs other then attiny85
  PCMSK |= (1 << measurepin); // pin change interrupt enabled for measurepin
  GIMSK |= (1 << PCIE);       // pin change interrupt enable
}

// the loop function runs over and over again forever
void loop() {
  static int remoteswitchpos = -1;             // always different from sw[x] at first time 
  static int remoteswitchdo = SWITCHNOSIGNAL; // position, including fake position when no signal
  static byte sw0;
  static int flashingcnt = 0;
  static int pulsatingcnt = 0, pulsatingdir = 1;
  static unsigned long loopstartmicros=0;
  static long loopperiod, microsleft;
  
  if ((micros() - lastrising) > ((long)NOSIGNAL) ) {  // no signal for specified time
    sw[0]=0; sw[1]=0; sw[2]=0; sw[3]=0; 
  }
  sw0 = sw[0];
  if ( (remoteswitchpos!=sw0) && (sw0==sw[1]) && (sw[1]==sw[2]) && (sw[2]==sw[3]) ) {
    remoteswitchpos = sw0;
    if (sw0 != 0) remoteswitchdo = sw0; else remoteswitchdo = SWITCHNOSIGNAL;  
    switch (remoteswitchdo) {
      case SWITCHON:
        // Switch switched to position ON
        digitalWrites(ONOFF1_LED, HIGH);
        digitalWrites(ONOFF2_LED, LOW);
        break;
      case SWITCHONPLUS:
        // Switch switched to position ONPLUS
        digitalWrites(ONOFF1_LED, HIGH);
        digitalWrites(ONOFF2_LED, HIGH);
        break;
      case SWITCHOFF:
      default:
        // Switch moved in position 1
        digitalWrites(ONOFF1_LED, LOW);
        digitalWrites(ONOFF2_LED, LOW);
        break;
    }
  }

  if ((remoteswitchdo == SWITCHON) || (remoteswitchdo == SWITCHONPLUS)) {
    // SWitch in position SWITCHON or SWITCHONPLUS   
    flashingcnt++;
    if (flashingcnt>=(FLASHINGPERIOD)) flashingcnt=0;
    if (flashingcnt<FLASHINGTIME) digitalWrites(FLASHING_LED, HIGH);
    else                          digitalWrites(FLASHING_LED, LOW);
    pulsatingcnt += pulsatingdir;
    if ((pulsatingcnt<=0) || (pulsatingcnt>=((PULSATINGPERIOD)/2))) 
         pulsatingdir = -pulsatingdir;
    analogWrites(PULSATING_LED, (((long)255*pulsatingcnt*2))/PULSATINGPERIOD  );
  }
  else {
    digitalWrites(FLASHING_LED,  LOW);
    digitalWrites(PULSATING_LED, LOW);
  }
  
  loopperiod = ((1000000/CYCLEFREQ)*(inputperiod>>2))/(REALPERIOD>>2);
  microsleft = loopperiod - (micros()-loopstartmicros);
  if (microsleft > 4) delayMicroseconds(microsleft); // delay cycleperiod - loop execution time
  loopstartmicros += loopperiod;
}
