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

Example code for basic functions below.

```
#include <Arduino.h>
#include <time.h>

void setup() {
  // put your setup code here, to run once:
  
  setup_RTC_interrupt(); // initialise the RTC.
  
  Serial.begin(38400);   // open the serial port at 38400 bps.

}

void loop() {
  // put your main code here, to run repeatedly:

  delay(2000);

}
```


