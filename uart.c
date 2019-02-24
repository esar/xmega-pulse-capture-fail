/*

   Copyright (C) 2019 Stephen Robinson
  
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 2 of the License, or
   (at your option) any later version.
  
   vivepos is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this code (see the file names COPING).  
   If not, see <http://www.gnu.org/licenses/>.
  
*/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// TX_BUF_SIZE must be power of 2
#define TX_BUF_SIZE    512

typedef uint16_t index_t;
static uint8_t g_txBuf[TX_BUF_SIZE];
static index_t g_txReadPos = 0;
static index_t g_txWritePos = 0;

ISR(USARTE0_DRE_vect)
{
	if(g_txReadPos != g_txWritePos)
	{
		// buffer is not empty, transmit next byte
		USARTE0.DATA = g_txBuf[g_txReadPos];
		g_txReadPos = (g_txReadPos + 1) & (TX_BUF_SIZE - 1);
	}
	else
	{
		// buffer is empty, turn off interrupt
		USARTE0.CTRLA &= ~USART_DREINTLVL_LO_gc;
	}
}

int uartPutChar(char c, FILE* stream)
{
	index_t pos = (g_txWritePos + 1) & (TX_BUF_SIZE - 1);

	// discard is tx buffer is full
	if(pos == g_txReadPos)
		return 0;

	// add data to buffer and enable interrupt to start sending
	g_txBuf[g_txWritePos] = c;
	g_txWritePos = pos;
	USARTE0.CTRLA |= USART_DREINTLVL_LO_gc;

	return 0;
}

int uartGetChar(FILE* stream)
{
	return -1;
}

static FILE uartOutput = FDEV_SETUP_STREAM(uartPutChar, NULL, 
                                           _FDEV_SETUP_WRITE);
static FILE uartInput  = FDEV_SETUP_STREAM(NULL, uartGetChar, 
                                           _FDEV_SETUP_READ);

void uartInit()
{
	PORTE.OUT = 1 << 3;    // set TX pin high
	PORTE.DIR |= 1 << 3;    // set TX pin as output

	// 115200
	USARTE0.BAUDCTRLA = 2094 & 0xFF;
	USARTE0.BAUDCTRLB = (9 << 4) | (2094 >> 8);

	USARTE0.CTRLB |= (1 << USART_RXEN_bp) | (1 << USART_TXEN_bp);

	stdout = &uartOutput;
	stdin  = &uartInput;
}

