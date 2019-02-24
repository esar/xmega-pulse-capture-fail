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
#define POS_MSG_LEN_SPI     ((POS_MSG_LEN)*4)    /* 1 nibble per bit */

#define STATE_IDLE            0
#define STATE_RECV_STARTED    1
#define STATE_SEND_STARTED    2

typedef struct  __attribute__((packed))
{
	union
	{
		struct
		{
			uint16_t count1;
			uint16_t count2;
			uint16_t count3;
			uint16_t count4;
		} counts;
		uint8_t bytes[8];
	};

} pos_msg_t;


#define LOGLEN    15
volatile pos_msg_t msgs[LOGLEN];
volatile uint8_t msgcount = 0;
volatile uint8_t errors[LOGLEN];
volatile uint8_t errflag = 0;


volatile uint8_t state = STATE_IDLE;

uint8_t posRecvBuffer[2][POS_MSG_LEN_BITS];
uint8_t posSendBuffer[4][POS_MSG_LEN_SPI];
volatile uint8_t posSendBufferIndex = 0;
volatile uint8_t posRecvBufferIndex = 1;
volatile uint8_t posSendBufferReady = 0;

void posInReset();
void posOutStart(uint8_t bufferIndex);


void encodeReceivedMessageToSendBuffer(uint8_t* in, uint8_t* out)
{
	uint8_t i = 0;
uint8_t val = 0;
uint8_t msgbyte = 0;

	for(i = 0; i < POS_MSG_LEN_BITS; i += 2)
	{
		if(in[i] < 15)
		{
val = (val << 1) | 0;
			if(in[i+1] < 15)
			{
val = (val << 1) | 0;
				*out++ = 0x77;
			}
			else
			{
val = (val << 1) | 1;
				*out++ = 0x73;
			}
		}
		else
		{
val = (val << 1) | 1;
			if(in[i+1] < 15)
			{
val = (val << 1) | 0;
				*out++ = 0x37;
			}
			else
			{
val = (val << 1) | 1;
				*out++ = 0x33;
			}
		}

if(i != 0 && (i & 7) == 6 && msgcount < LOGLEN)
{
	msgs[msgcount].bytes[7 - (msgbyte++)] = val;
	val = 0;
}
	}


if(msgcount < LOGLEN)
{
	errors[msgcount] = 0;
	//msgs[msgcount].bytes[3 - (msgbyte++)] = val;
	if(msgs[msgcount].counts.count1 != msgcount + 1 ||
	   msgs[msgcount].counts.count2 != msgcount + 1 ||
	   msgs[msgcount].counts.count3 != msgcount + 1 ||
	   msgs[msgcount].counts.count4 != msgcount + 1)
	{
	PORTC.OUT |= (1 << 5);
	PORTC.OUT &= ~(1 << 5);
	errors[msgcount] = 1;
	}
	msgcount++;
}

}

void posInProcessMessage(uint8_t recvBufferIndex)
{
PORTC.OUT |= (1 << 4);
/*
	if(state == STATE_RECV_STARTED)
		PORTC.INTCTRL |= PORT_INT0LVL_LO_gc;
*/
	// If this is the first message then start transmitting
	if(state == STATE_RECV_STARTED)
	{
		// start transmitting
		state = STATE_SEND_STARTED;
		posOutStart(posSendBufferIndex);
		posSendBufferIndex = (posSendBufferIndex + 1) & 3;
	}

	// Queue the message for transmission
	encodeReceivedMessageToSendBuffer(posRecvBuffer[recvBufferIndex], 
	                                  posSendBuffer[posRecvBufferIndex]);
	posRecvBufferIndex = (posRecvBufferIndex + 1) & 3;
	++posSendBufferReady;
	if(posSendBufferReady > 3)
		errflag = 1;
	//	printf("OOPS\r\n");
PORTC.OUT &= ~(1 << 4);
}

// Pin change interrupt for incoming data pin
// This interrupt is only used so that we don't have to have the timer overflow
// interrupt enabled all the time, so we don't have to waste loads of CPU time
// handling idle interrupts every 50us
ISR(PORTC_INT0_vect)
{
	// If this was a rising edge then disable this interrupt and enable
	// the timer overflow interrupt to detect the input going idle
	if(PORTC.IN & 1)
	{
		if(state == STATE_IDLE)
		{
			state = STATE_RECV_STARTED;
			TCC0.INTCTRLA |= TC_OVFINTLVL_LO_gc;
		}
		else if(state == STATE_RECV_STARTED)
		{
			state = STATE_SEND_STARTED;
			posOutStart(posSendBufferIndex);
			posSendBufferIndex = (posSendBufferIndex + 1) & 3;
		}
		PORTC.INTCTRL &= ~PORT_INT0LVL_LO_gc;  // disable portc.int0

	//	printf("start\r\n");
	}
	PORTC.INTFLAGS |= PORT_INT0IF_bm;
}


// Timer overflow for pulse width capture timer
// Called when there have been no incoming pulses for 50us
ISR(TCC0_OVF_vect)
{
	// If we've finished sending everything (send DMA is idle, no msg waiting)
	// then we can reset everything ready for the next stream of data
	// If we haven't then we'll be interrupted again in another 50uS
	if(!(DMA.CH2.CTRLB & DMA_CH_CHBUSY_bm) && 
	   !posSendBufferReady)
	{
		state = STATE_IDLE;

		// Disable this interrupt and re-enable the pin-change
		// interrupt to detect the start of incoming data
		PORTC.INTCTRL |= PORT_INT0LVL_LO_gc;
		TCC0.INTCTRLA &= ~TC_OVFINTLVL_LO_gc;

		// Reset the receive DMA channels in case they had 
		// received some partial data
		posInReset();

		// Reset the send buffers and copy our own position to
		// the first buffer so it's ready to be sent
		posSendBufferIndex = 0;
		posRecvBufferIndex = 1;
		// TODO: encode our own position to the send buffer
		memcpy(posSendBuffer[0], "\x33\x33\x33\x33\x33\x33\x33\x33\x33\x33\x33\x33\x33\x33\x33\x37\x33\x33\x33\x33\x33\x33\x33\x73\x33\x33\x33\x33\x33\x33\x33\x77", 32);
	}
	
	//printf("stop\r\n");

	TCC0.INTFLAGS |= TC0_OVFIF_bm;
}


// Incoming message ready interrupt from DMA channel 0
ISR(DMA_CH0_vect)
{
	// Queue the message for transmission
	posInProcessMessage(0);
	DMA.CH0.CTRLB |= DMA_CH_TRNIF_bm;
}


// Incoming message ready interrupt from DMA channel 1
ISR(DMA_CH1_vect)
{
	// Queue the message for transmission
	posInProcessMessage(1);
	DMA.CH1.CTRLB |= DMA_CH_TRNIF_bm;
}


// Message send complete interrupt from DMA channel
ISR(DMA_CH2_vect)
{
	// Message transmission complete, start the next one
	if(posSendBufferReady)
	{
		posOutStart(posSendBufferIndex);
		posSendBufferIndex = (posSendBufferIndex + 1) & 3;
		--posSendBufferReady;
	}

	DMA.CH2.CTRLB |= DMA_CH_TRNIF_bm;
}

ISR(TCC0_ERR_vect)
{
	printf("TCC0 ERR\r\n");
}

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

void posInInit()
{
	PORTC.INT0MASK = 1;                  // pin0 -> portc.int0
	PORTC.INTCTRL = PORT_INT0LVL_LO_gc;  // enable portc.int0

	EVSYS.CH0MUX = 32+64;           // PORTC,PIN0 -> event 0

	TCC0.PER = 0xFFFF;
	TCC0.CTRLD = TC_EVSEL_CH0_gc | TC_EVACT_PW_gc;
	TCC0.CTRLB = 16;                // CCA enabled
	TCC0.PER = 1600;                // overflow after 50us to detect idle
	//TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;
	TCC0.INTCTRLA |= TC_ERRINTLVL_LO_gc;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc; // Enable, no divider

	DMA.CTRL |= DMA_ENABLE_bm | DMA_DBUFMODE_CH01_gc | DMA_PRIMODE_CH01RR23_gc;

	DMA.CH0.CTRLA = DMA_CH_REPEAT_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	DMA.CH0.CTRLB = DMA_CH_TRNINTLVL_LO_gc;
	DMA.CH0.ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc  | 
	                   DMA_CH_SRCDIR_FIXED_gc    |
	                   DMA_CH_DESTRELOAD_TRANSACTION_gc |
	                   DMA_CH_DESTDIR_INC_gc;
	DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCA_gc;
	DMA.CH0.DESTADDR0 = ((uint16_t)posRecvBuffer[0] >> 0*8) & 0xFF;
	DMA.CH0.DESTADDR1 = ((uint16_t)posRecvBuffer[0] >> 1*8) & 0xFF;
	DMA.CH0.DESTADDR2 = 0;
	DMA.CH0.SRCADDR0 = ((uint16_t)&TCC0.CCAL >> 0*8) & 0xFF;
	DMA.CH0.SRCADDR1 = ((uint16_t)&TCC0.CCAL >> 1*8) & 0xFF;
	DMA.CH0.SRCADDR2 = 0;
	DMA.CH0.TRFCNT = POS_MSG_LEN_BITS;
	DMA.CH0.REPCNT = 0;

	DMA.CH1.CTRLA = DMA_CH_REPEAT_bm | DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	DMA.CH1.CTRLB = DMA_CH_TRNINTLVL_LO_gc;
	DMA.CH1.ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc  | 
	                   DMA_CH_SRCDIR_FIXED_gc    |
	                   DMA_CH_DESTRELOAD_TRANSACTION_gc |
	                   DMA_CH_DESTDIR_INC_gc;
	DMA.CH1.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCA_gc;
	DMA.CH1.DESTADDR0 = ((uint16_t)posRecvBuffer[1] >> 0*8) & 0xFF;
	DMA.CH1.DESTADDR1 = ((uint16_t)posRecvBuffer[1] >> 1*8) & 0xFF;
	DMA.CH1.DESTADDR2 = 0;
	DMA.CH1.SRCADDR0 = ((uint16_t)&TCC0.CCAL >> 0*8) & 0xFF;
	DMA.CH1.SRCADDR1 = ((uint16_t)&TCC0.CCAL >> 1*8) & 0xFF;
	DMA.CH1.SRCADDR2 = 0;
	DMA.CH1.TRFCNT = POS_MSG_LEN_BITS;
	DMA.CH1.REPCNT = 0;

	DMA.CH0.CTRLA |= DMA_CH_ENABLE_bm;
	//DMA.CH0.CTRLA |= DMA_CH_TRFREQ_bm;
}

void posInReset()
{
	DMA.CH0.CTRLA &= ~DMA_CH_ENABLE_bm;
	DMA.CH1.CTRLA &= ~DMA_CH_ENABLE_bm;
	DMA.CH0.CTRLA |= DMA_CH_RESET_bm;
	DMA.CH1.CTRLA |= DMA_CH_RESET_bm;
	posInInit();
}

void posOutInit()
{
	PORTC.OUT &= ~(1 << 3);    // set TX pin high
	PORTC.OUT |=  1 << 3;    // set TX pin high
	PORTC.DIR |= 1 << 3;    // set TX pin as output
	PORTC.PIN3CTRL |= PORT_INVEN_bm;

	// set baud to 3.2MHz
	USARTC0.BAUDCTRLA = 128;              // BSEL = 128
	USARTC0.BAUDCTRLB = 11 << 4;          // BSCALE = -5
	USARTC0.CTRLC = USART_CMODE_MSPI_gc;  // set mode to SPI
	USARTC0.CTRLB = USART_TXEN_bm;        // enable transmitter

	DMA.CTRL |= DMA_ENABLE_bm;
	DMA.CH2.CTRLA |= /*DMA_CH_REPEAT_bm |*/ DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;
	DMA.CH2.CTRLB = DMA_CH_TRNINTLVL_LO_gc;
	DMA.CH2.ADDRCTRL = DMA_CH_SRCRELOAD_NONE_gc  | 
	                   DMA_CH_SRCDIR_INC_gc    |
	                   DMA_CH_DESTRELOAD_NONE_gc |
	                   DMA_CH_DESTDIR_FIXED_gc;
	DMA.CH2.TRIGSRC = DMA_CH_TRIGSRC_USARTC0_DRE_gc;
	DMA.CH2.DESTADDR0 = ((uint16_t)&USARTC0.DATA >> 0*8) & 0xFF;
	DMA.CH2.DESTADDR1 = ((uint16_t)&USARTC0.DATA >> 1*8) & 0xFF;
	DMA.CH2.DESTADDR2 = 0;
	DMA.CH2.SRCADDR0 = ((uint16_t)posSendBuffer[0] >> 0*8) & 0xFF;
	DMA.CH2.SRCADDR1 = ((uint16_t)posSendBuffer[0] >> 1*8) & 0xFF;
	DMA.CH2.SRCADDR2 = 0;
	DMA.CH2.TRFCNT = POS_MSG_LEN_SPI;
}

void posOutStart(uint8_t bufferIndex)
{
	DMA.CH2.SRCADDR0 = ((uint16_t)posSendBuffer[bufferIndex] >> 0*8) & 0xFF;
	DMA.CH2.SRCADDR1 = ((uint16_t)posSendBuffer[bufferIndex] >> 1*8) & 0xFF;
	DMA.CH2.CTRLA |= DMA_CH_ENABLE_bm;
	//DMA.CH2.CTRLA |= DMA_CH_TRFREQ_bm;
}

void posOutWait()
{
	while(!(DMA.CH2.CTRLB & DMA_CH_TRNIF_bm))
		;
	DMA.CH2.CTRLB |= DMA_CH_TRNIF_bm;
}

void main()
{
	clockInit();

	uartInit();
	sei();
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	printf("hello\r\n");

	PORTC.OUT |=  1 << 4;
	PORTC.DIR |= 1 << 4;
	PORTC.OUT |=  1 << 5;
	PORTC.DIR |= 1 << 5;

	//PORTC.DIRSET = (1 << 7);									
	//PORTCFG.CLKEVOUT |= PORTCFG_CLKOUT_PC7_gc;

	//EVSYS.CH1MUX = 0b11000100;
	//PORTC.DIRSET = (1 << 7);									
	//PORTCFG.EVOUTSEL = PORTCFG_EVOUTSEL_1_gc;
	//PORTCFG.CLKEVOUT |= PORTCFG_EVOUT_PC7_gc;

	memcpy(posSendBuffer[0], "\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x73\x77\x77\x77\x77\x77\x77\x77\x37\x77\x77\x77\x77\x77\x77\x77\x33", 32);
	memcpy(posSendBuffer[1], "\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x77\x73\x77\x77\x77\x77\x77\x77\x77\x37\x77\x77\x77\x77\x77\x77\x77\x33", 32);

	printf("posOutInit\r\n");
	posOutInit();

	printf("posInInit\r\n");
	posInInit();

	printf("loop\r\n");
	for(;;)
	{
/*
	// If this is the first message then start transmitting
	if(state == STATE_RECV_STARTED)
	{
		// start transmitting
		state = STATE_SEND_STARTED;
		posOutStart(posSendBufferIndex);
		posSendBufferIndex = (posSendBufferIndex + 1) & 3;
	}
*/
		if(TCC0.INTFLAGS & TC0_ERRIF_bm)
		{
			printf("ERR OVERFLOW\r\n");
			//TCC0.INTFLAGS |= TC0_ERRIF_bm;
		}
		if(msgcount == LOGLEN)
		{
			uint8_t i;

			_delay_us(100);

			for(i = 0; i < LOGLEN; ++i)
			{
				//if(errors[i])
				//	break;
				printf("msg %u: %04x %04x %04x %04x %s\r\n", i, msgs[i].counts.count1, msgs[i].counts.count2, msgs[i].counts.count3, msgs[i].counts.count4, errors[i] ? "***" : "");
			}
			msgcount = 0;

			if(i != LOGLEN)
				printf("ERR MISSED\r\n");

			if(TCC0.INTFLAGS & TC0_ERRIF_bm)
			{
				printf("ERR OVERFLOW\r\n");
				//TCC0.INTFLAGS |= TC0_ERRIF_bm;
			}
			if(errflag)
			{
				printf("ERR BUFS\r\n");
				errflag = 0;
			}
		}
	}
}

