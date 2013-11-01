// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2013 Pat Hickey, Galois Inc. All Rights Reserved.
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

#ifndef __HXSTREAM_H__
#define __HXSTREAM_H__

#include <stdint.h>
#include <stdbool.h>

bool hxstream_rx_handler (uint8_t c);
bool hxstream_tx_handler (void);
void hxstream_init       (void);

void hxstream_write_frame (__xdata uint8_t* __data buf, __pdata uint8_t count);
bool hxstream_read_frame  (__xdata uint8_t* __data buf, __pdata uint8_t count);
uint8_t hxstream_read_available (void);

void hxstream_term_begin_frame (void);
void hxstream_term_end_frame (void);
void hxstream_term_putchar(char c);

void hxstream_check_rts(void);

// XXX DEBUG


 __data extern volatile uint8_t     rx_insert, rx_remove;
 __data extern volatile uint8_t     tx_insert, tx_remove;

#endif // __HXSTREAM_H__
