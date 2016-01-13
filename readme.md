This library implements firmware for the MCP4822 12-bit, dual channel, SPI interface, DAC integrated into the Goldilocks Analogue, a ATmega1284p MCU classic Arduino board.

Timing to generate a regular sample is provided by Timer 3.
This means that Arduino functions (for servos and analogwrite() depending on Timer 3 are disabled for that timer.

IIR filtering functions with (high, band, and low) pass configuration options are available and can be used prior to sampling.

## General

The DAC is configured to output values from 0V to 4.096V.
The Op Amp buffers these values.
The Headphone Amp applies a -6dB attenuator prior to amplification, providing a 1Vpp input signal for full volume output biased around 0V. The lower corner frequency is 6Hz. The useful upper frequency depends on the sampling rate.

## Reference Manual

Initialise the USART 1 MSPIM bus specifically for DAC use.
The LDAC latch can be triggered after buffering the new values (TRUE) for value setting (DC values). Or it can be latched first (FALSE) to latch the previous values loaded into the DAC buffers, useful for audio where the minimum sampling jitter is desired.
```
void DAC_init(uint8_t const post_latch);
```

Set up the sampling Timer3, which runs at audio sampling rate in Hz.
We use a 16 bit timer, which is useful for any sampling rate, including 44,100 Hz, 22,050 Hz, 11,025 Hz and any above rates too.
```
void DAC_Timer3_init(uint16_t const samplesSecond);	
void DAC_Timer3_end(void); // stop the sampling reconstruction Timer3.
```

The function preparing the samples needs to follow this prototype, and then be registered by setting this handler
Set up the DSP processing to prepare the samples.
```
void audio_sample_processing ( uint16_t * ch_A,  uint16_t * ch_B ) // this is a standard function.
void DAC_setHandler(analogue_DSP * const handler, uint16_t * const ch_A, uint16_t * const ch_B);	
```

Dual channel left justified 16 bit values are output each time the DAC_out function is called.
The DAC output values are called by reference.
```
//	if (something) {
//		uint16_t j = 0x1234;
//		uint16_t k = 0x5678;
//		DAC_out(NULL, NULL); 	// the ch_A and ch_B are turned off by NULL, or
//		DAC_out(&j, &k);		// output the j and k values by reference
//	}
void DAC_out(const uint16_t * const ch_A, const uint16_t * const ch_B);
```


Second order IIR -- "Direct Form I Transposed"

a(0)*y(n) = b(0)*x(n) + b(1)*x(n-1) +  b(2)*x(n-2) - a(1)*y(n-1) -  a(2)*y(n-2)

assumes a(0) = IIRSCALEFACTOR = 32

Compute filter function coefficients
* http://en.wikipedia.org/wiki/Digital_biquad_filter
* https://www.hackster.io/bruceland/dsp-on-8-bit-microcontroller
* http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
```
void setIIRFilterLPF( filter_t * const filter ); // Low Pass
void setIIRFilterBPF( filter_t * const filter ); // Band Pass
void setIIRFilterHPF( filter_t * const filter ); // High Pass
```
Returns y(n) filtered by the biquad IIR process in place of x(n)
```
void IIRFilter( filter_t * const filter, int16_t * const xn );
```

## Compatibility

  * ATmega1284p @ 24.576MHz : Seeed Studio Goldilocks Analogue
  * Other MCP4822 DAC platforms.

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

  // nothing here... because we use the Timer 3 to generate audio at the sampling rate.
}
```
