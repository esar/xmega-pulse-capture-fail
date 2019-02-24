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

void test()
{
	uint8_t i;

	for(i = 0; i < 4; ++i)
	{
		printf("Flags: %s %s\r\n", (TCC0.INTFLAGS & TC0_CCAIF_bm) ? "CCAIF" : "",
		                           (TCC0.INTFLAGS & TC0_ERRIF_bm) ? "ERRIF" : "");

		printf("rise\r\n");
		EVSYS.DATA = 1;
		EVSYS.STROBE = 1;
		_delay_us(10);

		printf("Flags: %s %s\r\n", (TCC0.INTFLAGS & TC0_CCAIF_bm) ? "CCAIF" : "",
		                           (TCC0.INTFLAGS & TC0_ERRIF_bm) ? "ERRIF" : "");

		printf("fall\r\n");
		EVSYS.DATA = 0;
		EVSYS.STROBE = 1;
		_delay_us(10);
	}

	printf("Flags: %s %s\r\n", (TCC0.INTFLAGS & TC0_CCAIF_bm) ? "CCAIF" : "",
	                           (TCC0.INTFLAGS & TC0_ERRIF_bm) ? "ERRIF" : "");

	printf("restart\r\n");
	TCC0.CTRLFSET |= TC_CMD_RESTART_gc;
	_delay_us(10);

	printf("Flags: %s %s\r\n", (TCC0.INTFLAGS & TC0_CCAIF_bm) ? "CCAIF" : "",
	                           (TCC0.INTFLAGS & TC0_ERRIF_bm) ? "ERRIF" : "");

	printf("restart\r\n");
	TCC0.CTRLFSET |= TC_CMD_RESTART_gc;
	_delay_us(10);

	printf("Flags: %s %s\r\n", (TCC0.INTFLAGS & TC0_CCAIF_bm) ? "CCAIF" : "",
	                           (TCC0.INTFLAGS & TC0_ERRIF_bm) ? "ERRIF" : "");

	printf("\r\n");
}

void main()
{
	clockInit();

	uartInit();
	sei();
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	printf("CAPT mode\r\n");
	printf("=========\r\n");
	TCC0.CTRLA = TC_CLKSEL_OFF_gc;
	TCC0.CTRLFSET |= TC_CMD_RESET_gc;
	TCC0.PER = 0xFFFF;
	TCC0.CTRLD = TC_EVSEL_CH0_gc | TC_EVACT_CAPT_gc;
	TCC0.CTRLB = TC0_CCAEN_bm;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	test();
	_delay_ms(100);

	printf("PW mode\r\n");
	printf("=========\r\n");
	TCC0.CTRLA = TC_CLKSEL_OFF_gc;
	TCC0.CTRLFSET |= TC_CMD_RESET_gc;
	TCC0.PER = 0xFFFF;
	TCC0.CTRLD = TC_EVSEL_CH0_gc | TC_EVACT_PW_gc;
	TCC0.CTRLB = TC0_CCAEN_bm;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	test();
	_delay_ms(100);

	printf("FRQ mode\r\n");
	printf("=========\r\n");
	TCC0.CTRLA = TC_CLKSEL_OFF_gc;
	TCC0.CTRLFSET |= TC_CMD_RESET_gc;
	TCC0.PER = 0xFFFF;
	TCC0.CTRLD = TC_EVSEL_CH0_gc | TC_EVACT_FRQ_gc;
	TCC0.CTRLB = TC0_CCAEN_bm;
	TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
	test();

	for(;;);
}

