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

#include <string.h>
#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uart.h"


#define POS_MSG_LEN         8
#define POS_MSG_LEN_BITS    ((POS_MSG_LEN)*8)
#define POS_MSG_LEN_SPI     ((POS_MSG_LEN)*4)

#define NUM_MSGS            16


uint8_t posOutBuffer[NUM_MSGS][POS_MSG_LEN_SPI];


void clockInit()
{
	OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;  // Enable the internal 32MHz & 32KHz oscillators
	while(!(OSC.STATUS & OSC_RC32KRDY_bm));       // Wait for 32Khz oscillator to stabilize
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));       // Wait for 32MHz oscillator to stabilize
	DFLLRC32M.CTRL = DFLL_ENABLE_bm ;             // Enable DFLL, calibrate against internal 32Khz clock
	CCP = CCP_IOREG_gc;                           // Disable register security for clock update
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;              // Switch to 32MHz clock
	OSC.CTRL &= ~OSC_RC2MEN_bm;                   // Disable 2Mhz oscillator
}

void posOutInit()
{
	PORTC.OUT &= ~(1 << 3);
	PORTC.OUT =  1 << 3;    // set TX pin high
	PORTC.DIR |= 1 << 3;    // set TX pin as output
	PORTC.PIN3CTRL |= PORT_INVEN_bm;

	// set baud to 3.2MHz
	USARTC0.BAUDCTRLA = 128;              // BSEL = 128
	USARTC0.BAUDCTRLB = 11 << 4;          // BSCALE = -5
	USARTC0.CTRLC = USART_CMODE_MSPI_gc;  // set mode to SPI
	USARTC0.CTRLB = USART_TXEN_bm;        // enable transmitter

	DMA.CTRL |= DMA_ENABLE_bm;
	DMA.CH2.CTRLA |= DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	DMA.CH2.ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc  | 
	                   DMA_CH_SRCDIR_INC_gc    |
	                   DMA_CH_DESTRELOAD_NONE_gc |
	                   DMA_CH_DESTDIR_FIXED_gc;
	DMA.CH2.TRIGSRC = DMA_CH_TRIGSRC_USARTC0_DRE_gc;
	DMA.CH2.DESTADDR0 = ((uint16_t)&USARTC0.DATA >> 0*8) & 0xFF;
	DMA.CH2.DESTADDR1 = ((uint16_t)&USARTC0.DATA >> 1*8) & 0xFF;
	DMA.CH2.DESTADDR2 = 0;
	DMA.CH2.SRCADDR0 = ((uint16_t)posOutBuffer[0] >> 0*8) & 0xFF;
	DMA.CH2.SRCADDR1 = ((uint16_t)posOutBuffer[0] >> 1*8) & 0xFF;
	DMA.CH2.SRCADDR2 = 0;
	DMA.CH2.TRFCNT = POS_MSG_LEN_SPI;
}

void posOutStart(uint8_t bufferIndex)
{
	DMA.CH2.CTRLB |= DMA_CH_TRNIF_bm;
	DMA.CH2.SRCADDR0 = ((uint16_t)posOutBuffer[bufferIndex] >> 0*8) & 0xFF;
	DMA.CH2.SRCADDR1 = ((uint16_t)posOutBuffer[bufferIndex] >> 1*8) & 0xFF;
	DMA.CH2.CTRLA |= DMA_CH_ENABLE_bm;
}

void posOutWait()
{
	while(!(DMA.CH2.CTRLB & DMA_CH_TRNIF_bm))
		;
	DMA.CH2.CTRLB |= DMA_CH_TRNIF_bm;
}

void setBuffer(uint8_t* buffer, uint16_t val)
{
	uint8_t i;

	for(i = 0; i < 16; i+=2)
	{
		if(val & 0x8000)
		{
			if(val & 0x4000)
				*buffer++ = 0x33;
			else
				*buffer++ = 0x37;
		}
		else
		{
			if(val & 0x4000)
				*buffer++ = 0x73;
			else
				*buffer++ = 0x77;
		}
		val <<= 2;
	}
}

void main()
{
	uint8_t i, j;

	clockInit();

	uartInit();
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();
	printf("hello\r\n");

	for(i = 0; i < NUM_MSGS; ++i)
	{
		for(j = 0; j < 4; ++j)
		{
			setBuffer(posOutBuffer[i] + 0, i+1);
			setBuffer(posOutBuffer[i] + 8, i+1);
			setBuffer(posOutBuffer[i] + 16, i+1);
			setBuffer(posOutBuffer[i] + 24, i+1);
		}
	}

	printf("posOutInit\r\n");
	posOutInit();

	printf("loop\r\n");
	for(;;)
	{
		printf("sending...\r\n");
		for(i = 0; i < NUM_MSGS; ++i)
		{
			posOutStart(i);
			posOutWait();
		}

		printf("sleeping...\r\n");
		_delay_ms(1000);
	}
}

