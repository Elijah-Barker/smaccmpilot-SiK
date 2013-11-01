// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2011 Michael Smith, All Rights Reserved
// Copyright (c) 2013 Pat Hickey, Galois Inc, All Rights Reserved
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

#include <string.h>
#include "board.h"
#include "serial.h"
#include "hxstream.h"

// Permits a 96 byte data frame with four extra bytes available
// for future tag expansion.
#define HX_FRAMELEN  100

#define HX_FBO 0x7e // '~'
#define HX_CEO 0x7c // '|'
#define HX_ESC(_x) (_x^0x20)

#define HX_STATE_IDLE      0
#define HX_STATE_FSTART    1
#define HX_STATE_DATA      2
#define HX_STATE_ESC       3
#define HX_STATE_NOFRAMING 4

#define TERM_STATE_IDLE  0
#define TERM_STATE_WRITE 1
#define TERM_STATE_READY 2
#define TERM_STATE_TXING 3


// FIFO status
#define BUF_FULL(_which) (((_which##_insert + 1) & _which##_mask) == (_which##_remove))
#define BUF_NOT_FULL(_which) (((_which##_insert + 1) & _which##_mask) != (_which##_remove))
#define BUF_EMPTY(_which) (_which##_insert == _which##_remove)
#define BUF_NOT_EMPTY(_which) (_which##_insert != _which##_remove)
#define BUF_USED(_which) ((_which##_insert - _which##_remove) & _which##_mask)
#define BUF_FREE(_which) ((_which##_remove - _which##_insert - 1) & _which##_mask)

// FIFO insert/remove operations
//
// Note that these are nominally interrupt-safe as only one of each
// buffer's end pointer is adjusted by either of interrupt or regular
// mode code.  This is violated if printing from interrupt context,
// which should generally be avoided when possible.
//
#define BUF_INSERT(_which) do {  \
        _which##_insert = ((_which##_insert+1) & _which##_mask); } while(0)
#define BUF_AT_INSERT(_which) &((_which##_buf)[_which##_insert])
#define BUF_REMOVE(_which) do { \
        _which##_remove = ((_which##_remove+1) & _which##_mask); } while(0)
#define BUF_AT_REMOVE(_which) &((_which##_buf)[_which##_remove])

struct frame {
	uint8_t len;
	uint8_t fdata[HX_FRAMELEN];
};

struct frame_builder {
    uint8_t state;
    uint8_t id;
    uint8_t offs;
};

#define INIT_FRAME_BUILDER(_fb) do { \
	(_fb)->state = HX_STATE_IDLE; \
	(_fb)->id = 0; \
	(_fb)->offs = 0; \
	} while (0)

#define TX_PUT(c) do { SBUF0 = c; } while (0)

//----------------------------------------------------------------------------

// NUM_xx_FRAMES must be a power of 2 so that masking works:
#define NUM_RX_FRAMES 16
#define NUM_TX_FRAMES 4

__xdata static struct frame_builder rx_fbuilder;
__xdata static struct frame         rx_buf[NUM_RX_FRAMES];
__pdata static const  uint8_t       rx_mask = NUM_RX_FRAMES - 1;
 __data        volatile uint8_t     rx_insert, rx_remove;

__xdata static struct frame_builder tx_fbuilder;
__xdata static struct frame         tx_buf[NUM_TX_FRAMES];
__pdata static const  uint8_t       tx_mask = NUM_TX_FRAMES - 1;
 __data        volatile uint8_t     tx_insert, tx_remove;

__xdata static struct frame         tx_term;
__xdata static uint8_t              tx_term_state;
__xdata static struct frame         rx_term;

static bool frame_rx (uint8_t c,
		__xdata struct frame* __xdata f,
		__xdata struct frame_builder* __xdata fb);
static bool frame_tx (__xdata struct frame * __xdata f,
		__xdata struct frame_builder * __xdata fb);

void hxstream_init (void) {
	INIT_FRAME_BUILDER(&rx_fbuilder);
	rx_insert = 0;
	rx_remove = 0;

	INIT_FRAME_BUILDER(&tx_fbuilder);
	tx_insert = 0;
	tx_remove = 0;

	tx_term_state = TERM_STATE_IDLE;
}

// Return value: true if the byte was dropped from processing due to
//  full buffers.
#pragma save
#pragma nooverlay
bool hxstream_rx_handler(uint8_t c) {
	__xdata struct frame *rx_frame;
	__xdata uint8_t i;
	if (rx_fbuilder.state < HX_STATE_DATA) {
		// Frame not written to until we're in data mode. Until then
		// we use frame_rx to detect a frame's start and id byte
		frame_rx(c, NULL, &rx_fbuilder);
	} else {
		// id 0 is for frames to be sent through the radio
		if (rx_fbuilder.id == 0) {
			if (BUF_NOT_FULL(rx)) {
				rx_frame = BUF_AT_INSERT(rx);
				if (frame_rx(c, rx_frame, &rx_fbuilder)) {
					BUF_INSERT(rx);
					errors.serial_rx_ok++;
				}
			} else {
				rx_fbuilder.state = HX_STATE_IDLE;
				if (errors.serial_rx_overflow != 0xFFFF) {
					errors.serial_rx_overflow++;
				}
			}
		}
		// id 1 is for control frames
		if (rx_fbuilder.id == 1) {
			rx_frame = &rx_term;
			if (frame_rx(c, rx_frame, &rx_fbuilder)) {
				for (i = 0; i < rx_term.len; i++) {
					at_input_aux(rx_term.fdata[i]);
				}
			}
		}
	}
	return false;
}
#pragma restore

static bool frame_rx (uint8_t c,
		__xdata struct frame* __xdata f,
		__xdata struct frame_builder* __xdata fb)
{
	__xdata uint8_t off;
	switch (fb->state) {
		case HX_STATE_IDLE:
			if (c == HX_FBO) {
				fb->state = HX_STATE_FSTART;
			}
			break;

		case HX_STATE_FSTART:
			if (c == 0 || c == 1) {
				fb->state = HX_STATE_DATA;
				fb->offs = 0;
				fb->id   = c;
			} else if (c == HX_FBO) {
				// Getting multiple FBOs in a row
				// keeps us at this state.
				fb->state = HX_STATE_FSTART;
			} else {
				// Error, we don't handle that frame type
				fb->state = HX_STATE_IDLE;
			};
			break;

		case HX_STATE_DATA:
			if (c == HX_CEO) {
				// Next byte will be escaped:
				fb->state = HX_STATE_ESC;
			} else if (c == HX_FBO) {
				// Complete Frame
				f->len = fb->offs;
				// Next byte will be start of next frame:
				fb->state = HX_STATE_FSTART;
				return true;
			} else {
				off = fb->offs;
				if (off < HX_FRAMELEN) {
					// Ordinary byte onto the frame:
					f->fdata[off] = c;
					fb->offs = off + 1;
				} else {
					// Error, no more room to store data.
					// Wait at idle for an FBO before continuing.
					fb->state = HX_STATE_IDLE;
				}
			}
			break;

		case HX_STATE_ESC:
			if (c == HX_FBO) {
				// Unexpected frame ending
				f->len = 0;
				fb->state = HX_STATE_FSTART;
			} else {
				off = fb->offs;
				if (off < HX_FRAMELEN) {
					// Escaped byte onto the frame:
					f->fdata[off] = HX_ESC(c);
					fb->offs = off + 1;
					fb->state = HX_STATE_DATA;
				} else {
					// Error, no more room to store data.
					// Wait at idle for an FBO before continuing.
					fb->state = HX_STATE_IDLE;
				}
			}
			break;

		default:
			fb->state = HX_STATE_IDLE;
	}
	return false;
}

// Return value: true when nothing has been transmitted
bool hxstream_tx_handler() {
	__xdata struct frame * __xdata f;
	bool complete;
	bool transmitted;

	complete = true;
	transmitted = false;
	if (tx_term_state == TERM_STATE_TXING) {
		f = &tx_term;
		complete = frame_tx(f, &tx_fbuilder);
		transmitted = true;
		if (complete) {
			tx_term_state = TERM_STATE_IDLE;
			INIT_FRAME_BUILDER(&tx_fbuilder);
		}
	} else if (BUF_NOT_EMPTY(tx)) {
		f = BUF_AT_REMOVE(tx);
		if (f->len <= HX_FRAMELEN) {
			complete = frame_tx(f, &tx_fbuilder);
			transmitted = true;
		} else {
			complete = true;
		}
		if (complete) {
			BUF_REMOVE(tx);
			INIT_FRAME_BUILDER(&tx_fbuilder);
		}
	}
	if ((transmitted == false) && tx_term_state == TERM_STATE_READY) {
		tx_term_state = TERM_STATE_TXING;
		tx_fbuilder.id = 1;
		f = &tx_term;
		if (at_resp_noframing) {
			tx_fbuilder.state = HX_STATE_NOFRAMING;
		}
		complete = frame_tx(f, &tx_fbuilder);
		transmitted = true;
	}
	return (transmitted)?false:true;
}

static bool frame_tx (__xdata struct frame * __xdata f,
		__xdata struct frame_builder * __xdata fb)
{
	__xdata uint8_t o,d;
	switch (fb->state) {
		case HX_STATE_IDLE:
			TX_PUT(HX_FBO);
			fb->state = HX_STATE_FSTART;
			break;
		case HX_STATE_FSTART:
			TX_PUT(fb->id);
			fb->state = HX_STATE_DATA;
			break;
		case HX_STATE_DATA:
			if (fb->offs >= f->len) {
				TX_PUT(HX_FBO);
				return true;
			}
			o = fb->offs;
			fb->offs = o + 1;
			d = f->fdata[o];
			if (d == HX_FBO || d == HX_CEO) {
				TX_PUT(HX_CEO);
				fb->state = HX_STATE_ESC;
			} else {
				TX_PUT(d);
			}
			break;
		case HX_STATE_ESC:
			o = fb->offs;
			d = f->fdata[o-1];
			TX_PUT(HX_ESC(d));
			fb->state = HX_STATE_DATA;
			break;
		case HX_STATE_NOFRAMING:
			if (fb->offs >= f->len) {
				TX_PUT(0);
				return true;
			}
			o = fb->offs;
			fb->offs = o + 1;
			d = f->fdata[o];
			TX_PUT(d);
	}
	return false;
}

void hxstream_write_frame (__xdata uint8_t* __data buf, __pdata uint8_t count) {
	// write frame to the hxstream transmit buffer
	if (BUF_NOT_FULL(tx) && count <= HX_FRAMELEN) {
		memcpy((BUF_AT_INSERT(tx))->fdata, buf, count);
		(BUF_AT_INSERT(tx))->len = count;
		BUF_INSERT(tx);
		errors.serial_tx_ok++;
	} else {
		if (errors.serial_tx_overflow != 0xFFFF) {
			errors.serial_tx_overflow++;
		}
	}
}

bool hxstream_read_frame (__xdata uint8_t * __data buf, __pdata uint8_t count) {
	// read a frame from the hxstream recieve buffer.
	if (hxstream_read_available() == count) {
		memcpy(buf, (BUF_AT_REMOVE(rx))->fdata, count);
		BUF_REMOVE(rx);
		return true;
	}
	return false;
}

uint8_t hxstream_read_available (void) {
	if (BUF_NOT_EMPTY(rx)) {
		return (BUF_AT_REMOVE(rx))->len;
	}
	return 0;
}

void hxstream_term_begin_frame (void) {
	tx_term.len = 0;
	tx_term_state = TERM_STATE_WRITE;
}

void hxstream_term_end_frame (void) {
	if (tx_term_state == TERM_STATE_WRITE) {
		if (tx_term.len > 0) {
			tx_term_state = TERM_STATE_READY;
			if (tx_idle) {
				serial_restart();
			}
		} else {
			tx_term_state = TERM_STATE_IDLE;
		}
	}
}

void hxstream_term_putchar(char c) {
	if (tx_term_state == TERM_STATE_WRITE) {
		if (tx_term.len < HX_FRAMELEN) {
			tx_term.fdata[tx_term.len] = c;
			tx_term.len++;
		}
	}
}


#pragma save
#pragma nooverlay
void hxstream_check_rts(void) {
	if ((BUF_NOT_EMPTY(tx) || tx_term_state == TERM_STATE_READY) && tx_idle) {
		serial_restart();
	}
}
#pragma restore

