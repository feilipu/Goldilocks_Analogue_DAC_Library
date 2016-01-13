
#include <Arduino.h>

#include <DAC.h>

uint16_t aValue; // where the value to go to DAC is stored.
uint16_t bValue;

void setup() {
  // put your setup code here, to run once:

  DAC_init(TRUE); // set single DAC value (DC) with the post write latch set TRUE.
}

void loop() {
  // put your main code here, to run repeatedly:

  static uint16_t t = 0;

  DAC_out( &aValue, &bValue ); // write both channels to the DAC

  t += 0x10;  // increment through staircase values. 0x10 is LSB for 12 bit output.

  aValue =  t;   // ramp the a channel up
  bValue = -t;   // ramp the b channel down.
}
