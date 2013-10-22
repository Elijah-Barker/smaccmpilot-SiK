// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2011 Michael Smith, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

///
/// @file	serial.c
///
/// MCS51 Serial port driver with flow control and AT command
/// parser integration.
///

#include "serial.h"
#include "packet.h"
#include "hxstream.h"


// flag indicating the transmitter is idle
volatile bool			tx_idle;

static void			_serial_write(register uint8_t c);
static void serial_device_set_speed(register uint8_t speed);

// save and restore serial interrupt. We use this rather than
// __critical to ensure we don't disturb the timer interrupt at all.
// minimal tick drift is critical for TDM
#define ES0_SAVE_DISABLE __bit ES_saved = ES0; ES0 = 0
#define ES0_RESTORE ES0 = ES_saved

// threshold for considering the rx buffer full
#define SERIAL_CTS_THRESHOLD_LOW  17
#define SERIAL_CTS_THRESHOLD_HIGH 34

void
serial_interrupt(void) __interrupt(INTERRUPT_UART0)
{
	register uint8_t	c;

	// check for received byte first
	if (RI0) {
		// acknowledge interrupt and fetch the byte immediately
		RI0 = 0;
		c = SBUF0;

		// if AT mode is active, the AT processor owns the byte
		if (at_mode_active) {
			// If an AT command is ready/being processed, we would ignore this byte
			if (!at_cmd_ready) {
				at_input(c);
			}
		} else {
			// run the byte past the +++ detector
			at_plus_detector(c);

			// and queue it for general reception
			hxstream_rx_handler(c);
		}
	}

	// check for anything to transmit
	if (TI0) {
		// acknowledge the interrupt
		TI0 = 0;

		// look for another byte we can send
		if (hxstream_tx_handler()) {
			// tx_handler returns true when no byte has been transmitted.
			// note that the transmitter requires a kick to restart it
			tx_idle = true;
		}
	}
}


void
serial_init(register uint8_t speed)
{
	// disable UART interrupts
	ES0 = 0;

	// reset buffer state, discard all data
	hxstream_init();
	tx_idle = true;

	// configure timer 1 for bit clock generation
	TR1 	= 0;				// timer off
	TMOD	= (TMOD & ~0xf0) | 0x20;	// 8-bit free-running auto-reload mode
	serial_device_set_speed(speed);		// device-specific clocking setup
	TR1	= 1;				// timer on

	// configure the serial port
	SCON0	= 0x10;				// enable receiver, clear interrupts

#ifdef SERIAL_CTS
	// setting SERIAL_CTS low tells the other end that we have
	// buffer space available
	SERIAL_CTS = false;
#endif

	// re-enable UART interrupts
	ES0 = 1;
}


void
serial_restart(void)
{
#ifdef SERIAL_RTS
	if (feature_rtscts && SERIAL_RTS && !at_mode_active) {
		// the line is blocked by hardware flow control
		return;
	}
#endif
	// generate a transmit-done interrupt to force the handler to send another byte
	tx_idle = false;
	TI0 = 1;
}


///
/// Table of supported serial speed settings.
/// the table is looked up based on the 'one byte'
/// serial rate scheme that APM uses. If an unsupported
/// rate is chosen then 57600 is used
///
static const __code struct {
	uint8_t rate;
	uint8_t th1;
	uint8_t ckcon;
} serial_rates[] = {
	{1,   0x2C, 0x02}, // 1200
	{2,   0x96, 0x02}, // 2400
	{4,   0x2C, 0x00}, // 4800
	{9,   0x96, 0x00}, // 9600
	{19,  0x60, 0x01}, // 19200
	{38,  0xb0, 0x01}, // 38400
	{57,  0x2b, 0x08}, // 57600 - default
	{115, 0x96, 0x08}, // 115200
	{230, 0xcb, 0x08}, // 230400
};

//
// check if a serial speed is valid
//
bool 
serial_device_valid_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++) {
		if (speed == serial_rates[i].rate) {
			return true;
		}
	}
	return false;
}

static 
void serial_device_set_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++) {
		if (speed == serial_rates[i].rate) {
			break;
		}
	}
	if (i == num_rates) {
		i = 3; // 57600 default
	}

	// set the rates in the UART
	TH1 = serial_rates[i].th1;
	CKCON = (CKCON & ~0x0b) | serial_rates[i].ckcon;

	// tell the packet layer how fast the serial link is. This is
	// needed for packet framing timeouts
	packet_set_serial_speed(speed*125UL);	
}

