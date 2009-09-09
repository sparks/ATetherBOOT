/**********************************************************/
/* Serial Bootloader for Atmel megaAVR Controllers        */
/*                                                        */
/* tested with ATmega8, ATmega128 and ATmega168           */
/* should work with other mega's, see code for details    */
/*                                                        */
/* ATmegaBOOT.c                                           */
/*                                                        */
/* 20090201: hacked for 1k bootloader for ATmega168/328   */
/*           swapped bulk of assembler for examples at:   */
/* http://www.stderr.org/doc/avr-libc/avr-libc-user-manual/group__avr__boot.html   */
/*           by D. Brink, dcb AT opengauge.org            */
/*  with updated compiler options by S. Abeyasinghe spoink AT gmail.com */
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
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/boot.h>
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
#if !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega328P__)
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


/* onboard LED is used to indicate, that the bootloader was entered (3x flashing) */
/* if monitor functions are included, LED goes on after monitor was entered */
#ifdef __AVR_ATmega128__
/* Onboard LED is connected to pin PB7 (e.g. Crumb128, PROBOmega128, Savvy128) */
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
#define LED      PINB7
#else
/* Onboard LED is connected to pin PB2 (e.g. Crumb8, Crumb168) */
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PINB
/* 20060803: hacked by DojoCorp, LED pin is B5 in Arduino */
/* #define LED      PINB2 */
#define LED      PINB0
#endif


/* monitor functions will only be compiled when using ATmega128, due to bootblock size constraints */
#ifdef __AVR_ATmega128__
#define MONITOR
#endif


/* define various device id's */
/* manufacturer byte is always the same */
#define SIG1	0x1E	// Yep, Atmel is the only manufacturer of AVR micros.  Single source :(

#if defined __AVR_ATmega128__
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


/* define null response */
#define NULL_RESPONSE 0xFA


// ladyada mods - start after programming done & on power cycle!
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#define QUICKSTART_MODS 1
#endif


/* function prototypes */
void putch(char);
char getch(void);
static inline void getNch(uint8_t); // static inline saves a few bytes
void byte_response(uint8_t);
static inline void nothing_response(void);
char gethex(void);
void puthex(char);
void flash_led(uint8_t);
static inline void boot_program_page(uint32_t, uint8_t*);
int main (void) __attribute__ ((naked,section (".init9")));
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

struct flags_struct { // changed from a packed struct to save some bytes
	uint8_t eeprom;
	uint8_t rampz;
} flags;

uint8_t buff[256];
uint8_t address_high;

uint8_t pagesz=0x80;

uint8_t i;

uint8_t error_count = 0;
uint8_t firstchar = 0;
unsigned int pagenumber = 0;

void (*app_start)(void) = 0x0000;


void boot_program_page (uint32_t page, uint8_t *buf)
{
	uint16_t i;

	eeprom_busy_wait ();

	boot_page_erase (page);
	boot_spm_busy_wait ();      // Wait until the memory is erased.

	for (i=0; i<SPM_PAGESIZE; i+=2)
	{
		// Set up little-endian word.

		uint16_t w = *buf++;
		w += (*buf++) << 8;

		boot_page_fill (page + i, w);
	}

	boot_page_write (page);     // Store buffer in flash page.
	boot_spm_busy_wait();       // Wait until the memory is written.

	// Reenable RWW-section again. We need this if we want to jump back
	// to the application after bootloading.

	boot_rww_enable ();
}


/* main program starts here */
int main(void)
{
	asm volatile ( "clr __zero_reg__" );
	SP=RAMEND;

	uint8_t ch,ch2;
	uint16_t w;

#ifdef QUICKSTART_MODS
	//ch = MCUSR;
	//MCUSR = 0;
	WDTCSR |= _BV(WDCE) | _BV(WDE);
	WDTCSR = 0;
#endif

	/* set pin direction for bootloader pin and enable pullup */
	/* for ATmega128, two pins need to be initialized */
#ifdef __AVR_ATmega128__
	BL_DDR &= ~_BV(BL0);
	BL_DDR &= ~_BV(BL1);
	BL_PORT |= _BV(BL0);
	BL_PORT |= _BV(BL1);
#else
	/* We run the bootloader regardless of the state of this pin.  Thus, don't
	put it in a different state than the other pins.  --DAM, 070709
	BL_DDR &= ~_BV(BL);
	BL_PORT |= _BV(BL);
	*/
#endif


	/* set LED pin as output */
	LED_DDR |= _BV(LED);

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
	

	flash_led(NUM_LED_FLASHES * 2);

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
		firstchar = 1;       // we got an appropriate bootloader instr.
		nothing_response();
	} else if (firstchar == 0) {
		app_start();
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
#ifdef QUICKSTART_MODS
		flash_led(2);
		// start immediately -ada
		app_start();
#endif
	}


	/* Set address, little endian. EEPROM in bytes, FLASH in words  */
	/* Perhaps extra address bytes may be added in future to support > 128kB FLASH.  */
	/* This might explain why little endian was used here, big endian used everywhere else.  */
	else if(ch=='U') {
		address.byte[0] = getch();
		address.byte[1] = getch();
#if defined __AVR_ATmega128__
		if (address.word>0x7FFF) flags.rampz = 1;		// No go with m256, FIXME
		else flags.rampz = 0;
#endif
		address.word = address.word << 1;	        // address * 2 -> byte location
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
				boot_program_page(pagenumber,(uint8_t *)&buff);
				pagenumber+=SPM_PAGESIZE;
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
		flags.eeprom = 0;
		if (getch() == 'E') flags.eeprom = 1;
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

#if defined __AVR_ATmega128__
					if (!flags.rampz) putch(pgm_read_byte_near(address.word));
					else putch(pgm_read_byte_far(address.word + 0x10000));
					// Hmmmm, yuck  FIXME when m256 arrvies
#else
					putch(pgm_read_byte_near(address.word));
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


#ifdef MONITOR

	/* here come the extended monitor commands by Erik Lins */

	/* check for three times exclamation mark pressed */
	else if(ch=='!') {
		ch = getch();
		if(ch=='!') {
		ch = getch();
		if(ch=='!') {

#ifdef __AVR_ATmega128__
			uint16_t extaddr;
#endif
			uint8_t addrl, addrh;

#ifdef CRUMB128
			PGM_P welcome = {"ATmegaBOOT / Crumb128 - (C) J.P.Kyle, E.Lins - 050815\n\r"};
#elif defined PROBOMEGA128
			PGM_P welcome = {"ATmegaBOOT / PROBOmega128 - (C) J.P.Kyle, E.Lins - 050815\n\r"};
#elif defined SAVVY128
			PGM_P welcome = {"ATmegaBOOT / Savvy128 - (C) J.P.Kyle, E.Lins - 050815\n\r"};
#endif

			/* turn on LED */
			LED_DDR |= _BV(LED);
			LED_PORT &= ~_BV(LED);

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

				/* toggle LED */
				if(ch == 't') {
					if(bit_is_set(LED_PIN,LED)) {
						LED_PORT &= ~_BV(LED);
						putch('1');
					} else {
						LED_PORT |= _BV(LED);
						putch('0');
					}
				} 

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
#ifdef __AVR_ATmega128__
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
		if (val != NULL_RESPONSE)
			putch(val);
		putch(0x10);
	} else {
		if (++error_count == MAX_ERROR_COUNT)
			app_start();
	}
}


void nothing_response(void)
{
	byte_response(NULL_RESPONSE);
}


void flash_led(uint8_t count)
{
	while (count--) {
		LED_PORT ^= _BV(LED);       // toggle!
		_delay_ms(100);
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
