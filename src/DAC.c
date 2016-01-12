

#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "mult16x16.h"
#include "mult16x8.h"
#include "mult32x16.h"

#include "DAC.h"
/*--------------------------------------------------*/
/*--------------------Local Variables---------------*/
/*--------------------------------------------------*/

// DSP function callback pointer.
static analogue_DSP * audioHandler;

 // pointers to storage for the values to be written to MCP4822
static uint16_t* ch_A_ptr;
static uint16_t* ch_B_ptr;

static uint8_t DAC_post_latch = FALSE;

/*--------------------------------------------------*/
/*---------------Public Functions-------------------*/
/*--------------------------------------------------*/

void
DAC_init(uint8_t const post_latch)
{

	/* For Goldilocks Analogue we are using USART1 in MSPIM mode, rather than using the overloaded SPI interface, to drive the DAC.
	 * The SPI SCK is connected to XCK1, and the SPI MOSI is connected to TX1. SPI MISO is not connected, as it is not needed.
	 */
	 DAC_post_latch = post_latch;			// configure whether the DAC latches outputs before or after setting buffers.

	SPI_PORT_DIR_SS_DAC |= SPI_BIT_SS_DAC;	// Set the _SS as output pin.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;		// Pull the _SS high to deselect the Goldilocks Analogue DAC.

	DAC_PORT_DIR_LDAC |= DAC_BIT_LDAC;		// Set MCP4822 _LDAC as output pin.
	DAC_PORT_LCAC |= DAC_BIT_LDAC;			// Pull MCP4822 _LDAC high to disable the DAC latch. No output will be latched.

	UBRR1 = 0x0000;							// Set Baud Rate to maximum, to speed XCK1 setting time.

	DDRD |= _BV(PD4);						// Setting the XCK1 port pin as output, enables USART master SPI mode.
	UCSR1C = _BV(UMSEL11) | _BV(UMSEL10);	// Set USART MSPI mode of operation using SPI data mode 0,0. UCPHA1 = UCSZ10
	UCSR1B = _BV(TXEN1);					// Enable transmitter. Enable the TX1 (don't enable the RX1, and the rest of the interrupt enable bits leave set to 0 too).

	/* Set baud rate.  IMPORTANT: The actual Baud Rate must be set after the transmitter is enabled */
	UBRR1 = 0x0000;							// FCPU/2 = 0x0000
}


/*
 * Timer_n / Counter_n Prescale
 * CSn2 CSn1 CSn0 Description
 * 0    0    0    No clock source (Timer/Counter stopped).
 * 0    0    1    clk I/O /1    (No prescaling).
 * 0    1    0    clk I/O /8    (From prescaler).
 * 0    1    1    clk I/O /64   (From prescaler).
 * 1    0    0    clk I/O /256  (From prescaler).
 * 1    0    1    clk I/O /1024 (From prescaler).
 * 1    1    0    External clock source on Tn pin. Clock on falling edge.
 * 1    1    1    External clock source on Tn pin. Clock on rising edge.
 */

void
DAC_Timer3_init(uint16_t const samplesSecond) // set up the sampling reconstruction Timer3, runs at audio sampling rate in Hz.
{

#if defined(DEBUG_PING)
	DDRD |= _BV(DDD7);					// set the debugging ping
	PORTD &= ~_BV(PORTD7);
#endif

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// setup Timer3 for codec clock division
		TIMSK3  = 0x00;					// clear all Timer 3 interrupts.
		TIFR3 = _BV(ICF3) | _BV(OCF3B) | _BV(OCF3A) | _BV(TOV3);  // clear all Timer 3 flags.
		TCCR3A = 0x00;					// set to CTC Mode 12.
		TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30); // CTC Mode 12. TOP = ICR3. No prescaling.
		TCCR3C = 0x00;					// not used

		TCNT3 =  0x0000;				// clear the counter, high byte first for 16bit writes. gcc compiler knows how to handle this.
		ICR3 = (F_CPU / ((uint32_t)samplesSecond)) -1; // Set ICR3 to be the TOP value.
	}
	TIMSK3 = _BV(ICIE3);				// turn on Input Capture interrupt (used for overflow interrupt when ICR3 is TOP.)
}

void
DAC_Timer3_end(void) // stop the sampling reconstruction Timer3.
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TIMSK3  = 0x00;               // clear all Timer 3 interrupts.
		TIFR3 = _BV(ICF3) | _BV(OCF3B) | _BV(OCF3A) | _BV(TOV3);  // clear all Timer 3 flags.
	}
}

/**
 * Sets the DAC audio handler function.
 * This function will be called to process the audio at each sample period.
 *
 */
void
DAC_setHandler(analogue_DSP * const handler, uint16_t * const ch_A, uint16_t * const ch_B)		// set up the DSP processing to prepare the samples.
{
	if (handler) audioHandler = handler;
	if (ch_A) ch_A_ptr = ch_A;
	if (ch_B) ch_B_ptr = ch_B;
}


/*--------------------------------------------------*/
/*-----------Time Critical Functions----------------*/
/*--------------------------------------------------*/

void
DAC_out(const uint16_t * const ch_A, const uint16_t * const ch_B)
{
	DAC_command_t write;

    if (DAC_post_latch == FALSE)
    {
    	DAC_PORT_LCAC &= ~DAC_BIT_LDAC;	// Pull MCP4822 _LDAC low to enable the DAC output to latch the previous sample input buffer values.
									    // We do this first to minimise the sample jitter when latching output values.
    }
    
	if (ch_A != NULL)
	{
		write.value.u16 = (*ch_A) >> 4;
		write.value.u8[1] |= CH_A_OUT;
	}
	else								// ch_A is NULL so we turn off the DAC
	{
		write.value.u8[1] = CH_A_OFF;
	}

	DAC_PORT_LCAC |= DAC_BIT_LDAC;		// Pull MCP4822 _LDAC high to disable the DAC latch.
										// Values we write into the DAC buffer this time, will be latched into the DAC output next interrupt.
	
	UCSR1A = _BV(TXC1);					// Clear the Transmit complete flag.
	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC;	// Pull _SS low to select the Goldilocks Analogue DAC
	UDR1 = write.value.u8[1];			// Begin transmission ch_A.
	UDR1 = write.value.u8[0];			// Continue transmission ch_A. UDR1 is double buffered.

	if (ch_B != NULL)					// start processing ch_B while we're doing the ch_A transmission
	{
		write.value.u16 = (*ch_B) >> 4;
		write.value.u8[1] |= CH_B_OUT;
	}
	else								// ch_B is NULL so we turn off the DAC
	{
		write.value.u8[1] = CH_B_OFF;
	}

	while ( !(UCSR1A & _BV(TXC1)) );	// Check we've finished ch_A.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull _SS high to deselect the Goldilocks Analogue DAC.

	UCSR1A = _BV(TXC1);					// Clear the Transmit complete flag.
	SPI_PORT_SS_DAC &= ~SPI_BIT_SS_DAC; // Pull _SS low to select the Goldilocks Analogue DAC.
	UDR1 = write.value.u8[1];			// Begin transmission ch_B.
	UDR1 = write.value.u8[0];			// Continue transmission ch_B.
	while ( !(UCSR1A & _BV(TXC1)) );	// Check we've finished ch_B.
	SPI_PORT_SS_DAC |= SPI_BIT_SS_DAC;	// Pull _SS high to deselect the Goldilocks Analogue DAC.
	
	if (DAC_post_latch != FALSE)
    {
    	DAC_PORT_LCAC &= ~DAC_BIT_LDAC;	// Pull MCP4822 _LDAC low to enable the DAC output to latch the previous sample input buffer values.
										// We do this last when latching single (or DC) output values.
    }
}


/*--------------------------------------------------*/
/*-------Interrupt (Loop at Sample Rate)------------*/
/*--------------------------------------------------*/

// This is an example interrupt process that will output values at the sample rate defined.

ISR(TIMER3_CAPT_vect) __attribute__ ((hot, flatten));
ISR(TIMER3_CAPT_vect)
{
#if defined(DEBUG_PING)
	// start mark - check for start of interrupt - for debugging only
	PORTD |=  _BV(PORTD7);				// Ping IO line.
#endif

	// MCP4822 data transfer routine
	// move data to the MCP4822 - done first for regularity (reduced jitter).
	DAC_out  (ch_A_ptr, ch_B_ptr);

	// audio processing routine - do whatever processing on input is required - prepare output for next sample.
	// Fire the global audio handler.
	if (audioHandler!=NULL)
		audioHandler(ch_A_ptr, ch_B_ptr);

#if defined(DEBUG_PING)
	// end mark - check for end of interrupt - for debugging only
	PORTD &= ~_BV(PORTD7);
#endif
}


//========================================================
//********************* IIR Filter *********************//
//========================================================
// second order IIR -- "Direct Form I Transposed"
//  a(0)*y(n) = b(0)*x(n) + b(1)*x(n-1) +  b(2)*x(n-2)
//                   - a(1)*y(n-1) -  a(2)*y(n-2)
// assumes a(0) = IIRSCALEFACTOR = 32

// Compute filter function coefficients
// http://en.wikipedia.org/wiki/Digital_biquad_filter
// https://www.hackster.io/bruceland/dsp-on-8-bit-microcontroller
// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt

void
setIIRFilterLPF( filter_t * const filter ) // Low Pass Filter Setting
{
	if ( !(filter->sample_rate) )
		filter->sample_rate = SAMPLE_RATE;

	if ( !(filter->cutoff) )
		filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

	if ( !(filter->peak) )
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate>>1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

	filter->b0	= \
	filter->b2	= float2int( ((1.0 - cosW0) / 2.0) * scale );
	filter->b1	= float2int(  (1.0 - cosW0) * scale );

	filter->a1	= float2int( (-2.0 * cosW0) * scale );
	filter->a2	= float2int( (1.0 - alpha) * scale );
}

void
setIIRFilterHPF( filter_t * const filter ) // High Pass Filter Setting
{
	if ( !(filter->sample_rate) )
		filter->sample_rate = SAMPLE_RATE;

	if ( !(filter->cutoff) )
		filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

	if ( !(filter->peak) )
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate>>1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

	filter->b0	= float2int( ((1.0 + cosW0) / 2.0) * scale );
	filter->b1	= float2int( -(1.0 + cosW0) * scale );
	filter->b2	= float2int( ((1.0 + cosW0) / 2.0) * scale );

	filter->a1	= float2int( (-2.0 * cosW0) * scale );
	filter->a2	= float2int( (1.0 - alpha) * scale );
}

void
setIIRFilterBPF( filter_t * const filter ) // Band Pass Filter Setting
{
	if ( !(filter->sample_rate) )
		filter->sample_rate = SAMPLE_RATE;

	if ( !(filter->cutoff) )
		filter->cutoff = UINT16_MAX >> 1; // 1/4 of sample rate = filter->sample_rate>>2

	if ( !(filter->peak) )
		filter->peak =  (uint16_t)(M_SQRT1_2 * UINT16_MAX / Q_MAXIMUM); // 1/sqrt(2) effectively

	double frequency = ((double)filter->cutoff * (filter->sample_rate>>1)) / UINT16_MAX;
	double q = (double)filter->peak * Q_MAXIMUM / UINT16_MAX;
	double w0 = (2.0 * M_PI * frequency) / filter->sample_rate;
	double sinW0 = sin(w0);
	double cosW0 = cos(w0);
	double alpha = sinW0 / (q * 2.0f);
	double scale = IIRSCALEFACTOR / (1 + alpha); // a0 = 1 + alpha

	filter->b0	= float2int( alpha * scale );
	filter->b1	= 0;
	filter->b2	= float2int( -alpha * scale );

	filter->a1	= float2int( (-2.0 * cosW0) * scale );
	filter->a2	= float2int( (1.0 - alpha) * scale );
}

// Coefficients in 8.8 format
// interim values in 24.8 format
// returns y(n) in place of x(n)
void
IIRFilter( filter_t * const filter, int16_t * const xn )
{
    int32_t yn;			// current output
    int32_t  accum;		// temporary accumulator

    // sum the 5 terms of the biquad IIR filter
	// and update the state variables
	// as soon as possible
    MultiS16X16to32(yn,filter->xn_2,filter->b2);
    filter->xn_2 = filter->xn_1;

    MultiS16X16to32(accum,filter->xn_1,filter->b1);
    yn += accum;
    filter->xn_1 = *xn;

    MultiS16X16to32(accum,*xn,filter->b0);
    yn += accum;

    MultiS16X16to32(accum,filter->yn_2,filter->a2);
    yn -= accum;
    filter->yn_2 = filter->yn_1;

    MultiS16X16to32(accum,filter->yn_1,filter->a1);
    yn -= accum;

    filter->yn_1 = yn >> (IIRSCALEFACTORSHIFT + 8); // divide by a(0) = 32 & shift to 16.0 bit outcome from 24.8 interim steps

    *xn = filter->yn_1; // being 16 bit yn, so that's what we return.
}


