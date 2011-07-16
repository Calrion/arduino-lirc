/**
 * Arduino IR receiver to lirc.
 *
 * This is a IR receiver that outputs over the serial port pulse timmings.
 * Must of the code in this module is borrowed from the avrlirc code at
 * http://www.foxharp.boston.ma.us/avrlirc/ website.  I have changed the code
 * to fit inside the arduino architecture running on an Arduino nano v3
 * hardware.  I have not made many changes to the code and it basically runs
 * the same way on this processor.  Just compiled it against the Ardunio
 * libraries for serial port comms.
 */
/* License
 *
 * Copyright 2002 Karl Bongers (karl@turbobit.com)
 * Copyright 2007 Paul Fox (pgf@foxharp.boston.ma.us)
 * Copyright 2011 Rod Boyce (developer@boyce.net.nz)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this program; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include "WProgram.h" //Include arduino headers
#include <avr/sleep.h>


// values for TCCR1B
#define CLKDIV_8    2
#define CLKDIV_64   3
#define CLKDIV_256  4
#define CLKDIV_1024 5

#define SCALE_DENOM(fosc) ((fosc / 256) / 4)


///CONSTANTS///
const int irPin    = 8;
const int ledPin   = 13;
const int debugPin = 12;

volatile uint8_t  hadOverflow;
volatile bool     pulseIsHigh;
volatile uint16_t pulseLength;

static inline void Led1Off( void )
{
    digitalWrite( ledPin, LOW );
}

static inline void Led1On( void )
{
    digitalWrite( ledPin, HIGH );
}

static inline bool IrHigh( void )
{
    return HIGH == digitalRead( irPin );
}

static inline void debugToggle( void )
{
    if( HIGH == digitalRead( debugPin ))
    {
        digitalWrite( debugPin, LOW );
    }
    else
    {
        digitalWrite( debugPin, HIGH );
    }
}

/**
 * timer1 overflow interrupt handler.
 * if we hit the overflow without getting a transition on the IR
 * line, then we're certainly "between" IR packets.  we save the
 * overflow indication until just before the next "real" pulse --
 * lirc wants to see it then (just before the "real" data),
 * rather than at the end.
 */
ISR( TIMER1_OVF_vect, ISR_NOBLOCK )
{
    uint8_t tmp;

    if( IrHigh())
    {
        tmp = 0xff; // high byte of eventual dummy pulselen
    }
    else
    {
        tmp = 0x7f;
    }
    hadOverflow = tmp;
    debugToggle();
}

/**
 * timer1 compare match interrupt handler.  this is simply a way
 * of turning off the "IR message received" LED sooner.
 * otherwise we could do it in the overflow handler.  this has no
 * affect on the timing protocol, but doing it here makes the LED
 * behaviour match the user's button presses a little more
 * closely.
 */
ISR( TIMER1_COMPA_vect, ISR_NOBLOCK )
{
    Led1Off();
}

/**
 * input capture event handler
 * the "event" is a transition on the IR line.  we save the
 * captured count, and restart the timer from zero again.
 */
ISR( TIMER1_CAPT_vect, ISR_NOBLOCK )
{

    // read the event
    pulseLength = ICR1;
    // and save the new state of the IR line.
    pulseIsHigh = IrHigh();

    // restart the timer
    TCNT1 = 0;

    // change detection edge, and clear interrupt flag -- it's
    // set as result of detection edge change
    cli();
    TCCR1B ^= bit(ICES1);
    TIFR1 &= ~bit(ICF1);
    sei();

}

/**
 * Wiggling light pattern, to show life at startup.  very useful
 * for visually detecting watchdog or crash.
 */
void blinky( void )
{
    uint8_t i;

    for( i = 0; i < 6; i++ )
    {
        delay( 250 );
        if( i & 1 )
        {
            Led1Off();
        }
        else
        {
            Led1On();
        }
    }
}

/**
 * send 16 bits, little-endian
 */
void TxWord( uint16_t t )
{
    Serial.write( t & 0xff );
    Serial.write(( t >> 8 ) & 0xff );
}

/**
 *  we want the timer overflow to be (a lot) longer than the
 *  longest interval we need to record using ICR1, which is
 *  something like .25 sec.  we also need to convert from
 *  timer count intervals to 16384'ths of a second.
 *
 *  16.0000Mhz
 *     16000000 counts/sec, prescaled by 256, gives 62500 counts/sec,
 *     or 16.00usec/count, times 65536 gives overflow at 1.04sec.  good.
 *     want 16384'this:  scale count by 16384 / 62500. ( 4096 / 15625 )
 *
 *  14.7456Mhz
 *     14745600 counts/sec, prescaled by 256, gives 57600 counts/sec,
 *     or 17.36usec/count, times 65536 gives overflow at 1.14sec.  good.
 *     want 16384'this:  scale count by 16384 / 57600. ( 4096 / 14400 )
 *
 *  12.0000Mhz
 *     12000000 counts/sec, prescaled by 256, gives 46875 counts/sec,
 *     or 21.33usec/count, times 65536 gives overflow at 1.40sec.  good.
 *     want 16384'ths:  scale count by 16384 / 46875. ( 4096 / 11719 )
 *
 *  11.0592
 *     11059200 counts/sec, prescaled by 256, gives 43200 counts/sec,
 *     or 23.15usec/count, times 65536 gives overflow at 1.51sec.  good.
 *     want 16384'ths:  scale count by 16384 / 43200. ( 4096 / 10800 )
 *
 *  8.0000Mhz
 *     8000000 counts/sec, prescaled by 256, gives 31250 counts/sec,
 *     or 32usec/count, times 65536 gives overflow at 2.09sec.  good.
 *     want 16384'ths:  scale count by 16384 / 31250. ( 4096 / 7812 )
 *
 *  3.6864Mhz
 *     3686400/256 --> 14400, so scale by 16384 / 14400 --> 4096 / 3600
 *
 */
void EmitPulseData( void )
{
    uint16_t len;
    uint8_t high;
    uint8_t overflow;

    while( pulseLength )
    {
        cli();
        len = pulseLength;
        high = pulseIsHigh;
        overflow = hadOverflow;

        pulseLength = 0;
        hadOverflow = 0;

        sei();

        Led1On();
        if( overflow )
        {
            // if we had an overflow, then the current pulse_length
            // is meaningless -- it's just the last remnant of a
            // long gap.  just send the previously recorded
            // overflow value to indicate that gap.  this is
            // effectively the start of a "packet".
            TxWord(( overflow << 8 ) | 0xff );
        }
        else
        {
            uint32_t l;

            /* do long arithmetic.  expensive, but we have time. */
            l = (uint32_t) len * 4096 / SCALE_DENOM( 16000000 );

            if( l > 0x7fff ) // limit range.
            {
                len = 0x7fff;
            }
            else
            {
                len = l;
            }

            if( 0 == len ) // pulse length never zero.
            {
                len++;
            }

            if( !high ) // report the state we transitioned out of
            {
                len |= 0x8000;
            }

            TxWord( len );
        }
    }
}

/**
 * Program entry point
 *
 * @return nothing
 */
int main( void )
{
    // Initialise the Arduino library.
    init();

    // Initialise the serial port.
    Serial.begin( 115200 );

    //Configure ledPin as an output
    pinMode( debugPin, OUTPUT);
    pinMode( ledPin, OUTPUT);
    pinMode( irPin, INPUT );

    TCCR1A = 0; // undo what init() above has done
    TCCR1B = bit(ICNC1) | CLKDIV_256;   // see comments at EmitPulseData()
    // timer1 overflow int enable, and input capture event int enable.
    TIMSK1 = bit(TOIE1) | bit(OCIE1A) | bit(ICIE1);

    // we use the output compare interrupt to turn off the
    // "activity" LED.  this value is around 1/20th of a
    // second for all the "interesting" clock rates (see comments
    // at emit_pulse_data(), below
    OCR1A = 3000;

    blinky();

    while( true )
    {
        cli();
        if( !pulseLength )
        {
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        }
        sei();
        EmitPulseData();
    }

    return 0;
}
