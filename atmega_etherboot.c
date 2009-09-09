/**********************************************************/
/* Ethernet Bootloader for Atmel megaAVR Controllers      */
/*                                                        */
/* tested with ATmega328                                  */
/* should work with other mega's, see code for details    */
/*                                                        */
/* atmega_etherboot.c                                     */
/*                                                        */
/*                                                        */
/* 20090308: integrated Mega changes into main bootloader */
/*           source by D. Mellis                          */
/* 20080930: hacked for Arduino Mega (with the 1280       */
/*           processor, backwards compatible)             */
/*           by D. Cuartielles                            */
/* 20070626: hacked for Arduino Diecimila (which auto-    */
/*           resets when a USB connection is made to it)  */
/*           by D. Mellis                                 */
/* 20060802: hacked for Arduino by D. Cuartielles         */
/*           based on a previous hack by D. Mellis        */
/*           and D. Cuartielles                           */
/*                                                        */
/* Monitor and debug functions were added to the original */
/* code by Dr. Erik Lins, chip45.com. (See below)         */
/*                                                        */
/* Thanks to Karl Pitrich for fixing a bootloader pin     */
/* problem and more informative LED blinking!             */
/*                                                        */
/* For the latest version see:                            */
/* http://www.chip45.com/                                 */
/*                                                        */
/* ------------------------------------------------------ */
/*                                                        */
/* based on stk500boot.c                                  */
/* Copyright (c) 2003, Jason P. Kyle                      */
/* All rights reserved.                                   */
/* see avr1.org for original file and information         */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/* Target = Atmel AVR m128,m64,m32,m16,m8,m162,m163,m169, */
/* m8515,m8535. ATmega161 has a very small boot block so  */
/* isn't supported.                                       */
/*                                                        */
/* Tested with m168                                       */
/**********************************************************/

/* $Id$ */

/* some includes */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "W5100.h"

#define Sn_RX_MASK (0x0800-1)

#define S0_RX_BASE (RX_BASE+(Sn_RX_MASK+1)*0)
#define S1_RX_BASE (RX_BASE+(Sn_RX_MASK+1)*1)
#define S2_RX_BASE (RX_BASE+(Sn_RX_MASK+1)*2)
#define S3_RX_BASE (RX_BASE+(Sn_RX_MASK+1)*3)

#define Sn_TX_MASK (0x0800-1)

#define S0_TX_BASE (TX_BASE+(Sn_TX_MASK+1)*0)
#define S1_TX_BASE (TX_BASE+(Sn_TX_MASK+1)*1)
#define S2_TX_BASE (TX_BASE+(Sn_TX_MASK+1)*2)
#define S3_TX_BASE (TX_BASE+(Sn_TX_MASK+1)*3)

/* the current avr-libc eeprom functions do not support the ATmega168 */
/* own eeprom write/read functions are used instead */
#if !defined(__AVR_ATmega168__) || !defined(__AVR_ATmega328P__)
#include <avr/eeprom.h>
#endif

/* Use the F_CPU defined in Makefile */

/* 20060803: hacked by DojoCorp */
/* 20070626: hacked by David A. Mellis to decrease waiting time for auto-reset */
/* set the waiting time for the bootloader */
/* get this from the Makefile instead */
/* #define MAX_TIME_COUNT (F_CPU>>4) */

/* 20070707: hacked by David A. Mellis - after this many errors give up and launch application */
#define MAX_ERROR_COUNT 5


/* SW_MAJOR and MINOR needs to be updated from time to time to avoid warning message from AVR Studio */
/* never allow AVR Studio to do an update !!!! */
#define HW_VER	 0x02
#define SW_MAJOR 0x01
#define SW_MINOR 0x10



/* monitor functions will only be compiled when using ATmega128, due to bootblock size constraints */
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
#define MONITOR 1
#endif


/* define various device id's */
/* manufacturer byte is always the same */
#define SIG1	0x1E	// Yep, Atmel is the only manufacturer of AVR micros.  Single source :(

#if defined __AVR_ATmega1280__
#define SIG2	0x97
#define SIG3	0x03
#define PAGE_SIZE	0x80U	//128 words

#elif defined __AVR_ATmega1281__
#define SIG2	0x97
#define SIG3	0x04
#define PAGE_SIZE	0x80U	//128 words

#elif defined __AVR_ATmega128__
#define SIG2	0x97
#define SIG3	0x02
#define PAGE_SIZE	0x80U	//128 words

#elif defined __AVR_ATmega64__
#define SIG2	0x96
#define SIG3	0x02
#define PAGE_SIZE	0x80U	//128 words

#elif defined __AVR_ATmega32__
#define SIG2	0x95
#define SIG3	0x02
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega16__
#define SIG2	0x94
#define SIG3	0x03
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega8__
#define SIG2	0x93
#define SIG3	0x07
#define PAGE_SIZE	0x20U	//32 words

#elif defined __AVR_ATmega88__
#define SIG2	0x93
#define SIG3	0x0a
#define PAGE_SIZE	0x20U	//32 words

#elif defined __AVR_ATmega168__
#define SIG2	0x94
#define SIG3	0x06
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega328P__
#define SIG2	0x95
#define SIG3	0x0F
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega162__
#define SIG2	0x94
#define SIG3	0x04
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega163__
#define SIG2	0x94
#define SIG3	0x02
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega169__
#define SIG2	0x94
#define SIG3	0x05
#define PAGE_SIZE	0x40U	//64 words

#elif defined __AVR_ATmega8515__
#define SIG2	0x93
#define SIG3	0x06
#define PAGE_SIZE	0x20U	//32 words

#elif defined __AVR_ATmega8535__
#define SIG2	0x93
#define SIG3	0x08
#define PAGE_SIZE	0x20U	//32 words
#endif


/* function prototypes */
void putch(char);
char getch(void);
void getNch(uint8_t);
void byte_response(uint8_t);
void nothing_response(void);
char gethex(void);
void puthex(char);
uint8_t spi(uint8_t data); //Basic 8-Bit SPI exchange
uint8_t spi_W5100(uint8_t opcode, uint16_t address, uint8_t data); //Send a full 32-Bit Command to the W5100 chip
void sock_close();

/* some variables */
union address_union {
	uint16_t word;
	uint8_t  byte[2];
} address;

union length_union {
	uint16_t word;
	uint8_t  byte[2];
} length;

struct flags_struct {
	unsigned eeprom : 1;
	unsigned rampz  : 1;
} flags;

uint8_t buff[256];
uint8_t address_high;

uint8_t pagesz=0x80;

uint8_t i;

uint8_t error_count = 0;

void (*app_start)(void) = 0x0000;

/* main program starts here */
int main(void)
{
	uint8_t ch,ch2;
	uint16_t w;

#ifdef WATCHDOG_MODS
	ch = MCUSR;
	MCUSR = 0;

	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = 0;

	// Check if the WDT was used to reset, in which case we dont bootload and skip straight to the code. woot.
	if (! (ch &  _BV(EXTRF))) // if its a not an external reset...
		app_start();  // skip bootloader
#else
	asm volatile("nop\n\t");
#endif


	/* Ethernet stuff */
	DDRB |= (1 << DDB2) | //SS
		(1 << DDB3) | //MOSI
		(1 << DDB5); //SCK

	//Initial Pin States
	PORTB |= (1 << PINB2); //SS High

	/*  SPI Control Register */
	SPCR = (0 << SPIE) | //SPI Interrupt Enable (active high)
		(1 << SPE)  | //SPI Enable (active high)
		(0 << DORD) | //Data Order (MSB first low; LSB first high)
		(1 << MSTR) | //Master/Slave Select (Master high; Slave low)
		(0 << CPOL) | //Clock Polarity
		(0 << CPHA) | //Clock Phase
		(0 << SPR1) | //SPI Clock Rate Select 1
		(0 << SPR0); //SPI Clock Rate Select 0
	
	/* SPI Status Register */
	SPSR = (0 << SPIF) | //SPI Interrupt Flag (active high)
		(0 << WCOL) | //Write COLlision Flag (active high)
		(0 << SPI2X); //Double SPI Speed Bit (active high)

	
	//Set Gateway Address: 169.254.183.177
	spi_W5100(1, GAR2, 0xA9);
	spi_W5100(1, GAR1, 0xFE);
	spi_W5100(1, GAR2, 0xB7);
	spi_W5100(1, GAR3, 0x00);

	//Set Subnet Mask: 255.255.255.0
	spi_W5100(1, SUBR0, 0xFF);
	spi_W5100(1, SUBR1, 0xFF);
	spi_W5100(1, SUBR2, 0xFF);
	spi_W5100(1, SUBR3, 0x00);

	//Set Device MAC: 00:08:DC:01:02:03
	spi_W5100(1, SHAR0, 0x00);
	spi_W5100(1, SHAR1, 0x08);
	spi_W5100(1, SHAR2, 0xDC);
	spi_W5100(1, SHAR3, 0x01);
	spi_W5100(1, SHAR4, 0x02);
	spi_W5100(1, SHAR5, 0x03);

	//Set Device IP: 169.254.183.177
	spi_W5100(1, SIPR0, 0xA9);
	spi_W5100(1, SIPR1, 0xFE);
	spi_W5100(1, SIPR2, 0XB7);
	spi_W5100(1, SIPR3, 0xB1);
	
	//Assign 2K Memory per socket for RX and TX
	spi_W5100(1, RMSR, 0x55);
	spi_W5100(1, TMSR, 0x55);
	
	while(1) {
		spi_W5100(1, S0_MR, 0x01); //Set TCP Mode on Socket 0

		//Use port 23
		spi_W5100(1, S0_PORT0, 0x00);
		spi_W5100(1, S0_PORT1, 0x17);

		//Set Socket Open
		spi_W5100(1, S0_CR, Sn_CR_OPEN);

		//Check if Socket has opened if not try again
		if(spi_W5100(0, S0_SR, 0x00) != Sn_SR_SOCK_INIT) {
			spi_W5100(1, S0_CR, Sn_CR_CLOSE);
			continue;
		}

		//Set Socket to Listen
		spi_W5100(1, S0_CR, Sn_CR_LISTEN);

		//Check if Socket is listening if not try again
		if(spi_W5100(0, S0_SR, 0x00) != Sn_SR_SOCK_LISTEN) {
			spi_W5100(1, S0_CR, Sn_CR_CLOSE);
			continue;
		}
		break;
	}
	
	uint32_t count = 0;
	
	while(1) {
		if(spi_W5100(0, S0_SR, 0x00) == Sn_SR_SOCK_ESTABLISHED) {
			uint16_t rx_size = (spi_W5100(0, S0_RX_RSR0, 0x00) << 8) | spi_W5100(0, S0_RX_RSR1, 0x00);
			uint16_t rx_pointer = (spi_W5100(0, S0_RX_RD0, 0x00) << 8) | spi_W5100(0, S0_RX_RD1, 0x00);
			spi_W5100(1, S0_RX_RD0, ((rx_pointer+rx_size) & 0xFF00) >> 8);
			spi_W5100(1, S0_RX_RD1, (rx_pointer+rx_size) & 0x00FF);
			spi_W5100(1, S0_CR, Sn_CR_RECV);
		
			break;
		}
		count++;
		if (count > MAX_TIME_COUNT) 
			app_start();
	}
	



	/* 20050803: by DojoCorp, this is one of the parts provoking the
		 system to stop listening, cancelled from the original */
	//putch('\0');

	/* forever loop */
	for (;;) {

	/* get character from UART */
	ch = getch();

	/* A bunch of if...else if... gives smaller code than switch...case ! */

	/* Hello is anyone home ? */ 
	if(ch=='0') {
		nothing_response();
	}


	/* Request programmer ID */
	/* Not using PROGMEM string due to boot block in m128 being beyond 64kB boundry  */
	/* Would need to selectively manipulate RAMPZ, and it's only 9 characters anyway so who cares.  */
	else if(ch=='1') {
		if (getch() == ' ') {
			putch(0x14);
			putch('A');
			putch('V');
			putch('R');
			putch(' ');
			putch('I');
			putch('S');
			putch('P');
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}
	}


	/* AVR ISP/STK500 board commands  DON'T CARE so default nothing_response */
	else if(ch=='@') {
		ch2 = getch();
		if (ch2>0x85) getch();
		nothing_response();
	}


	/* AVR ISP/STK500 board requests */
	else if(ch=='A') {
		ch2 = getch();
		if(ch2==0x80) byte_response(HW_VER);		// Hardware version
		else if(ch2==0x81) byte_response(SW_MAJOR);	// Software major version
		else if(ch2==0x82) byte_response(SW_MINOR);	// Software minor version
		else if(ch2==0x98) byte_response(0x03);		// Unknown but seems to be required by avr studio 3.56
		else byte_response(0x00);				// Covers various unnecessary responses we don't care about
	}


	/* Device Parameters  DON'T CARE, DEVICE IS FIXED  */
	else if(ch=='B') {
		getNch(20);
		nothing_response();
	}


	/* Parallel programming stuff  DON'T CARE  */
	else if(ch=='E') {
		getNch(5);
		nothing_response();
	}


	/* P: Enter programming mode  */
	/* R: Erase device, don't care as we will erase one page at a time anyway.  */
	else if(ch=='P' || ch=='R') {
		nothing_response();
	}


	/* Leave programming mode  */
	else if(ch=='Q') {
		nothing_response();
#ifdef WATCHDOG_MODS
		// autoreset via watchdog (sneaky!)
		WDTCSR = _BV(WDE);
		while (1); // 16 ms
#endif
	}


	/* Set address, little endian. EEPROM in bytes, FLASH in words  */
	/* Perhaps extra address bytes may be added in future to support > 128kB FLASH.  */
	/* This might explain why little endian was used here, big endian used everywhere else.  */
	else if(ch=='U') {
		address.byte[0] = getch();
		address.byte[1] = getch();
		nothing_response();
	}


	/* Universal SPI programming command, disabled.  Would be used for fuses and lock bits.  */
	else if(ch=='V') {
		if (getch() == 0x30) {
			getch();
			ch = getch();
			getch();
			if (ch == 0) {
				byte_response(SIG1);
			} else if (ch == 1) {
				byte_response(SIG2); 
			} else {
				byte_response(SIG3);
			} 
		} else {
			getNch(3);
			byte_response(0x00);
		}
	}


	/* Write memory, length is big endian and is in bytes  */
	else if(ch=='d') {
		length.byte[1] = getch();
		length.byte[0] = getch();
		flags.eeprom = 0;
		if (getch() == 'E') flags.eeprom = 1;
		for (w=0;w<length.word;w++) {
			buff[w] = getch();	                        // Store data in buffer, can't keep up with serial data stream whilst programming pages
		}
		if (getch() == ' ') {
			if (flags.eeprom) {		                //Write to EEPROM one byte at a time
				address.word <<= 1;
				for(w=0;w<length.word;w++) {
#if defined(__AVR_ATmega168__)  || defined(__AVR_ATmega328P__)
					while(EECR & (1<<EEPE));
					EEAR = (uint16_t)(void *)address.word;
					EEDR = buff[w];
					EECR |= (1<<EEMPE);
					EECR |= (1<<EEPE);
#else
					eeprom_write_byte((void *)address.word,buff[w]);
#endif
					address.word++;
				}			
			}
			else {					        //Write to FLASH one page at a time
				if (address.byte[1]>127) address_high = 0x01;	//Only possible with m128, m256 will need 3rd address byte. FIXME
				else address_high = 0x00;
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__)
				RAMPZ = address_high;
#endif
				address.word = address.word << 1;	        //address * 2 -> byte location
				/* if ((length.byte[0] & 0x01) == 0x01) length.word++;	//Even up an odd number of bytes */
				if ((length.byte[0] & 0x01)) length.word++;	//Even up an odd number of bytes
				cli();					//Disable interrupts, just to be sure
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__)
				while(bit_is_set(EECR,EEPE));			//Wait for previous EEPROM writes to complete
#else
				while(bit_is_set(EECR,EEWE));			//Wait for previous EEPROM writes to complete
#endif
				asm volatile(
					 "clr	r17		\n\t"	//page_word_count
					 "lds	r30,address	\n\t"	//Address of FLASH location (in bytes)
					 "lds	r31,address+1	\n\t"
					 "ldi	r28,lo8(buff)	\n\t"	//Start of buffer array in RAM
					 "ldi	r29,hi8(buff)	\n\t"
					 "lds	r24,length	\n\t"	//Length of data to be written (in bytes)
					 "lds	r25,length+1	\n\t"
					 "length_loop:		\n\t"	//Main loop, repeat for number of words in block							 							 
					 "cpi	r17,0x00	\n\t"	//If page_word_count=0 then erase page
					 "brne	no_page_erase	\n\t"						 
					 "wait_spm1:		\n\t"
					 "lds	r16,%0		\n\t"	//Wait for previous spm to complete
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm1       \n\t"
					 "ldi	r16,0x03	\n\t"	//Erase page pointed to by Z
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"							 
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "wait_spm2:		\n\t"
					 "lds	r16,%0		\n\t"	//Wait for previous spm to complete
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm2       \n\t"									 

					 "ldi	r16,0x11	\n\t"	//Re-enable RWW section
					 "sts	%0,r16		\n\t"						 			 
					 "spm			\n\t"
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "no_page_erase:		\n\t"							 
					 "ld	r0,Y+		\n\t"	//Write 2 bytes into page buffer
					 "ld	r1,Y+		\n\t"							 
								 
					 "wait_spm3:		\n\t"
					 "lds	r16,%0		\n\t"	//Wait for previous spm to complete
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm3       \n\t"
					 "ldi	r16,0x01	\n\t"	//Load r0,r1 into FLASH page buffer
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
								 
					 "inc	r17		\n\t"	//page_word_count++
					 "cpi r17,%1	        \n\t"
					 "brlo	same_page	\n\t"	//Still same page in FLASH
					 "write_page:		\n\t"
					 "clr	r17		\n\t"	//New page, write current one first
					 "wait_spm4:		\n\t"
					 "lds	r16,%0		\n\t"	//Wait for previous spm to complete
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm4       \n\t"
#ifdef __AVR_ATmega163__
					 "andi	r30,0x80	\n\t"	// m163 requires Z6:Z1 to be zero during page write
#endif							 							 
					 "ldi	r16,0x05	\n\t"	//Write page pointed to by Z
					 "sts	%0,r16		\n\t"
					 "spm			\n\t"
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
					 "ori	r30,0x7E	\n\t"	// recover Z6:Z1 state after page write (had to be zero during write)
#endif
					 "wait_spm5:		\n\t"
					 "lds	r16,%0		\n\t"	//Wait for previous spm to complete
					 "andi	r16,1           \n\t"
					 "cpi	r16,1           \n\t"
					 "breq	wait_spm5       \n\t"									 
					 "ldi	r16,0x11	\n\t"	//Re-enable RWW section
					 "sts	%0,r16		\n\t"						 			 
					 "spm			\n\t"					 		 
#ifdef __AVR_ATmega163__
					 ".word 0xFFFF		\n\t"
					 "nop			\n\t"
#endif
					 "same_page:		\n\t"							 
					 "adiw	r30,2		\n\t"	//Next word in FLASH
					 "sbiw	r24,2		\n\t"	//length-2
					 "breq	final_write	\n\t"	//Finished
					 "rjmp	length_loop	\n\t"
					 "final_write:		\n\t"
					 "cpi	r17,0		\n\t"
					 "breq	block_done	\n\t"
					 "adiw	r24,2		\n\t"	//length+2, fool above check on length after short page write
					 "rjmp	write_page	\n\t"
					 "block_done:		\n\t"
					 "clr	__zero_reg__	\n\t"	//restore zero register
#if defined __AVR_ATmega168__  || __AVR_ATmega328P__ || __AVR_ATmega128__ || __AVR_ATmega1280__ || __AVR_ATmega1281__ 
					 : "=m" (SPMCSR) : "M" (PAGE_SIZE) : "r0","r16","r17","r24","r25","r28","r29","r30","r31"
#else
					 : "=m" (SPMCR) : "M" (PAGE_SIZE) : "r0","r16","r17","r24","r25","r28","r29","r30","r31"
#endif
					 );
				/* Should really add a wait for RWW section to be enabled, don't actually need it since we never */
				/* exit the bootloader without a power cycle anyhow */
			}
			putch(0x14);
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}		
	}


	/* Read memory block mode, length is big endian.  */
	else if(ch=='t') {
		length.byte[1] = getch();
		length.byte[0] = getch();
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
		if (address.word>0x7FFF) flags.rampz = 1;		// No go with m256, FIXME
		else flags.rampz = 0;
#endif
		address.word = address.word << 1;	        // address * 2 -> byte location
		if (getch() == 'E') flags.eeprom = 1;
		else flags.eeprom = 0;
		if (getch() == ' ') {		                // Command terminator
			putch(0x14);
			for (w=0;w < length.word;w++) {		        // Can handle odd and even lengths okay
				if (flags.eeprom) {	                        // Byte access EEPROM read
#if defined(__AVR_ATmega168__)  || defined(__AVR_ATmega328P__)
					while(EECR & (1<<EEPE));
					EEAR = (uint16_t)(void *)address.word;
					EECR |= (1<<EERE);
					putch(EEDR);
#else
					putch(eeprom_read_byte((void *)address.word));
#endif
					address.word++;
				}
				else {

					if (!flags.rampz) putch(pgm_read_byte_near(address.word));
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
					else putch(pgm_read_byte_far(address.word + 0x10000));
					// Hmmmm, yuck  FIXME when m256 arrvies
#endif
					address.word++;
				}
			}
			putch(0x10);
		}
	}


	/* Get device signature bytes  */
	else if(ch=='u') {
		if (getch() == ' ') {
			putch(0x14);
			putch(SIG1);
			putch(SIG2);
			putch(SIG3);
			putch(0x10);
		} else {
			if (++error_count == MAX_ERROR_COUNT)
				app_start();
		}
	}


	/* Read oscillator calibration byte */
	else if(ch=='v') {
		byte_response(0x00);
	}


#if defined MONITOR 

	/* here come the extended monitor commands by Erik Lins */

	/* check for three times exclamation mark pressed */
	else if(ch=='!') {
		ch = getch();
		if(ch=='!') {
		ch = getch();
		if(ch=='!') {
			PGM_P welcome = "";
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
			uint16_t extaddr;
#endif
			uint8_t addrl, addrh;

#ifdef CRUMB128
			welcome = "ATmegaBOOT / Crumb128 - (C) J.P.Kyle, E.Lins - 050815\n\r";
#elif defined PROBOMEGA128
			welcome = "ATmegaBOOT / PROBOmega128 - (C) J.P.Kyle, E.Lins - 050815\n\r";
#elif defined SAVVY128
			welcome = "ATmegaBOOT / Savvy128 - (C) J.P.Kyle, E.Lins - 050815\n\r";
#elif defined __AVR_ATmega1280__ 
			welcome = "ATmegaBOOT / Arduino Mega - (C) Arduino LLC - 090930\n\r";
#endif

		
			/* print a welcome message and command overview */
			for(i=0; welcome[i] != '\0'; ++i) {
				putch(welcome[i]);
			}

			/* test for valid commands */
			for(;;) {
				putch('\n');
				putch('\r');
				putch(':');
				putch(' ');

				ch = getch();
				putch(ch);


				/* read byte from address */
				else if(ch == 'r') {
					ch = getch(); putch(ch);
					addrh = gethex();
					addrl = gethex();
					putch('=');
					ch = *(uint8_t *)((addrh << 8) + addrl);
					puthex(ch);
				}

				/* write a byte to address  */
				else if(ch == 'w') {
					ch = getch(); putch(ch);
					addrh = gethex();
					addrl = gethex();
					ch = getch(); putch(ch);
					ch = gethex();
					*(uint8_t *)((addrh << 8) + addrl) = ch;
				}

				/* read from uart and echo back */
				else if(ch == 'u') {
					for(;;) {
						putch(getch());
					}
				}
#if defined(__AVR_ATmega128__) || defined(__AVR_ATmega1280__)
				/* external bus loop  */
				else if(ch == 'b') {
					putch('b');
					putch('u');
					putch('s');
					MCUCR = 0x80;
					XMCRA = 0;
					XMCRB = 0;
					extaddr = 0x1100;
					for(;;) {
						ch = *(volatile uint8_t *)extaddr;
						if(++extaddr == 0) {
							extaddr = 0x1100;
						}
					}
				}
#endif

				else if(ch == 'j') {
					app_start();
				}

			} /* end of monitor functions */

		}
		}
	}
	/* end of monitor */
#endif
	else if (++error_count == MAX_ERROR_COUNT) {
		app_start();
	}
	} /* end of forever loop */

}


char gethexnib(void) {
	char a;
	a = getch(); putch(a);
	if(a >= 'a') {
		return (a - 'a' + 0x0a);
	} else if(a >= '0') {
		return(a - '0');
	}
	return a;
}


char gethex(void) {
	return (gethexnib() << 4) + gethexnib();
}


void puthex(char ch) {
	char ah;

	ah = ch >> 4;
	if(ah >= 0x0a) {
		ah = ah - 0x0a + 'a';
	} else {
		ah += '0';
	}
	
	ch &= 0x0f;
	if(ch >= 0x0a) {
		ch = ch - 0x0a + 'a';
	} else {
		ch += '0';
	}
	
	putch(ah);
	putch(ch);
}




void putch(char ch)
{
	sock_close();
	uint16_t tx_write_pointer = spi_W5100(0, S0_TX_WR0, 0x00);
	tx_write_pointer = ((tx_write_pointer & 0xFF) << 8) + spi_W5100(0, S0_TX_WR1, 0x00);	
	uint16_t tx_offset = tx_write_pointer & Sn_TX_MASK;
	uint16_t tx_address = S0_TX_BASE + tx_offset;

	if((tx_offset+1) > Sn_TX_MASK) tx_address = S0_TX_BASE;
	
	spi_W5100(1, tx_address, ch);

	spi_W5100(1, S0_TX_WR0, ((tx_write_pointer+1) & 0xFF00) >> 8);
	spi_W5100(1, S0_TX_WR1, (tx_write_pointer+1) & 0xFF);
	spi_W5100(1, S0_CR, Sn_CR_SEND);
}


char getch(void)
{
	uint16_t rx_size = 0x00;
	
	uint32_t count = 0;
	while(rx_size < 0x01) {
		sock_close();
		rx_size = (spi_W5100(0, S0_RX_RSR0, 0x00) << 8) | spi_W5100(0, S0_RX_RSR1, 0x00);		
		count++;
		if (count > MAX_TIME_COUNT)
			app_start();
	}
	
	uint8_t ch = 0;

	uint16_t rx_pointer = (spi_W5100(0, S0_RX_RD0, 0x00) << 8) | spi_W5100(0, S0_RX_RD1, 0x00);
	uint16_t rx_offset = rx_pointer & Sn_RX_MASK; //Calculate offset
	uint16_t rx_address = S0_RX_BASE+rx_offset; //Calculate physical start address
	
	if((rx_offset+1) > Sn_RX_MASK) rx_address = S0_RX_BASE;
	
	ch = spi_W5100(0, rx_address, 0x00);

	spi_W5100(1, S0_RX_RD0, ((rx_pointer+1) & 0xFF00) >> 8);
	spi_W5100(1, S0_RX_RD1, (rx_pointer+1) & 0x00FF);
	spi_W5100(1, S0_CR, Sn_CR_RECV);
	return ch;
}


void getNch(uint8_t count)
{
	while(count--) {
		getch();
	}
}


void byte_response(uint8_t val)
{
	if (getch() == ' ') {
		putch(0x14);
		putch(val);
		putch(0x10);
	} else {
		if (++error_count == MAX_ERROR_COUNT)
			app_start();
	}
}


void nothing_response(void)
{
	if (getch() == ' ') {
		putch(0x14);
		putch(0x10);
	} else {
		if (++error_count == MAX_ERROR_COUNT)
			app_start();
	}
}

uint8_t spi_W5100(uint8_t opcode, uint16_t address, uint8_t data) {
	PORTB = PORTB & ~(1 << PINB2); //Make SS Low

	if(opcode) spi(0xF0); //write
	else spi(0x0F); //read
	
	spi((address&0xFF00) >> 8); //Address 1
	spi(address&0xFF); //Address 2
	
	uint8_t response = spi(data); //Send data
	
	PORTB |= (1 << PINB2); //Make SS High
	
	return response;
}

uint8_t spi(uint8_t data) {
	/* Start transmission */
	SPDR = data;
	/* Wait for transmission complete */
	while(!(SPSR & (1 << SPIF)));
	/* Return received data */
	return SPDR;
}

void sock_close() {
	if(spi_W5100(0, S0_SR, 0x00) == Sn_SR_SOCK_CLOSE_WAIT) {
		spi_W5100(1, S0_CR, Sn_CR_CLOSE);
		app_start();
	}
	if(spi_W5100(0, S0_SR, 0x00) == Sn_SR_SOCK_CLOSED) {
		spi_W5100(1, S0_CR, Sn_CR_CLOSE);
		app_start();
	}
}


/* end of file ATmegaBOOT.c */
