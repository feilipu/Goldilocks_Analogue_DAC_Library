This library implements firmware for the MCP4822 12-bit, dual channel, SPI interface, DAC integrated into the Goldilocks Analogue, a ATmega1284p MCU classic Arduino board.

Timing to generate a regular sample is provided by Timer 3.
This means that Arduino functions (for servos and analogwrite() depending on Timer 3 are disabled for that timer.

IIR filtering functions with (high, band, and low) pass configuration options are available and can be used prior to sampling.


## General



## Reference Manual



## Compatibility

  * ATmega1284p @ 24.576MHz : Seeed Studio Goldilocks Analogue

## Files & Configuration

* DAC.h : contains the definitions for all functions.
* 

Example code for outputting a simple DC DAC value below.

```

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
```
An example using the timer sample output functionality to produce an audio wave.

```
#include <Arduino.h>
#include <DAC.h>

uint16_t aValue; // Left Channel: where the value to go to DAC is stored.
uint16_t bValue; // Right Channel

/*
   Example for a simple audio DSP function.
   Needs to at least provide *ch_A and *ch_B
   Runs within Timer3 interrupt routine - time critical.
   Keep it very short and punchy!
   Use the DEBUG_PING #define to check if you think you're using too much time.
*/
void audio_sample_processing ( uint16_t * ch_A,  uint16_t * ch_B ) // this is a standard function.
{
  static uint16_t t = 0;

  *ch_A = t;
  *ch_B = -t;

  t += 0x10;  // increment through staircase values. 0x10 is LSB for 12 bit output.
}

void setup() {
  // put your setup code here, to run once:

  DAC_init(FALSE); // set timed DAC value (AC with the post-write latch set FALSE (reduces timing jitter by first latching out the previously buffered sample).

  // Actual DAC output is done by the Timer 3 interrupt. So this is how we configure everything, before we set it running.
  DAC_setHandler( audio_sample_processing, &aValue, &bValue);   // set the handler for the audio, and also the two DAC values for the handler to modify.
  DAC_Timer3_init( 48000 ); // Set & start the reproduction timer to the required sampling rate 16kHz, 44.1kHz, 48kHz or similar.
}

void loop() {
  // put your main code here, to run repeatedly:

  // nothing here... becuase we use the Timer 3 to generate audio at the sampling rate.
}
```
