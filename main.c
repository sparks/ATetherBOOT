#include "main.h"

/* Initialisation Code */
void init(void) {
	/* Set Status LED, SS, MOSI, SCK output, all others input */
	DDRB |= (1 << DDB0) | //Status LED
		(1 << DDB2) | //SS
		(1 << DDB3) | //MOSI
		(1 << DDB5); //SCK

	//Initial Pin States
	PORTB = (1 << PINB0) | //Status Off
		(1 << PINB2); //SS High

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
	
	//Interupt on Pin B1
	PCICR = (1 << PCIE0);
	PCMSK0 |= (1 << PCINT1);
	
	//Set Gateway Address: 169.254.183.177
	spi_W5100('w', GAR2, 0xA9);
	spi_W5100('w', GAR1, 0xFE);
	spi_W5100('w', GAR2, 0xB7);
	spi_W5100('w', GAR3, 0x00);

	//Set Subnet Mask: 255.255.255.0
	spi_W5100('w', SUBR0, 0xFF);
	spi_W5100('w', SUBR1, 0xFF);
	spi_W5100('w', SUBR2, 0xFF);
	spi_W5100('w', SUBR3, 0x00);

	//Set Device MAC: 00:08:DC:01:02:03
	spi_W5100('w', SHAR0, 0x00);
	spi_W5100('w', SHAR1, 0x08);
	spi_W5100('w', SHAR2, 0xDC);
	spi_W5100('w', SHAR3, 0x01);
	spi_W5100('w', SHAR4, 0x02);
	spi_W5100('w', SHAR5, 0x03);

	//Set Device IP: 169.254.183.177
	spi_W5100('w', SIPR0, 0xA9);
	spi_W5100('w', SIPR1, 0xFE);
	spi_W5100('w', SIPR2, 0XB7);
	spi_W5100('w', SIPR3, 0xB1);
	
	//Assign 2K Memory per socket for RX and TX
	spi_W5100('w', RMSR, 0x55);
	spi_W5100('w', TMSR, 0x55);
	
	sei(); //Turn on interrupts
}

void init_S0_TCP() {
	spi_W5100('w', S0_MR, 0x01); //Set TCP Mode on Socket 0
	
	//Use port 23
	spi_W5100('w', S0_PORT0, 0x00);
	spi_W5100('w', S0_PORT1, 0x17);
	
	//Set Socket Open
	spi_W5100('w', S0_CR, Sn_CR_OPEN);
	
	//Check if Socket has opened if not try again
	if(spi_W5100('r', S0_SR, 0x00) != Sn_SR_SOCK_INIT) {
		spi_W5100('w', S0_CR, Sn_CR_CLOSE);
		init_S0_TCP();
		return;
	}
	
	//Set Socket to Listen
	spi_W5100('w', S0_CR, Sn_CR_LISTEN);
	
	//Check if Socket is listening if not try again
	if(spi_W5100('r', S0_SR, 0x00) != Sn_SR_SOCK_LISTEN) {
		spi_W5100('w', S0_CR, Sn_CR_CLOSE);
		init_S0_TCP();
		return;
	}
	
}

int main(void) {
	_delay_ms(500);
	
	init(); //Run initialisation
	
	
	while(1) {
		init_S0_TCP(); //Setup Socket 0 as TCP
		
		while(1) {
			if(spi_W5100('r', S0_SR, 0x00) == Sn_SR_SOCK_ESTABLISHED) {
				uint16_t rx_size = (spi_W5100('r', S0_RX_RSR0, 0x00) << 8) | spi_W5100('r', S0_RX_RSR1, 0x00);
				uint16_t rx_pointer = (spi_W5100('r', S0_RX_RD0, 0x00) << 8) | spi_W5100('r', S0_RX_RD1, 0x00);
				spi_W5100('w', S0_RX_RD0, ((rx_pointer+rx_size) & 0xFF00) >> 8);
				spi_W5100('w', S0_RX_RD1, (rx_pointer+rx_size) & 0x00FF);
				spi_W5100('w', S0_CR, Sn_CR_RECV);
				
				break;
			}
		}
	
		while(1) {
			if(spi_W5100('r', S0_SR, 0x00) == Sn_SR_SOCK_CLOSE_WAIT) {
				spi_W5100('w', S0_CR, Sn_CR_CLOSE);
				break;
			}
			
			if(spi_W5100('r', S0_SR, 0x00) == Sn_SR_SOCK_CLOSED) {
				spi_W5100('w', S0_CR, Sn_CR_CLOSE);
				break;
			}
			
			uint16_t rx_size = (spi_W5100('r', S0_RX_RSR0, 0x00) << 8) | spi_W5100('r', S0_RX_RSR1, 0x00);
			if(rx_size >= 0x01) {
				//RX
			
				uint8_t incoming = 0;

				uint16_t rx_pointer = (spi_W5100('r', S0_RX_RD0, 0x00) << 8) | spi_W5100('r', S0_RX_RD1, 0x00);
				uint16_t rx_offset = rx_pointer & Sn_RX_MASK; //Calculate offset
				uint16_t rx_address = S0_RX_BASE+rx_offset; //Calculate physical start address
				
				if((rx_offset+1) > Sn_RX_MASK) rx_address = S0_RX_BASE;
				
				incoming = spi_W5100('r', rx_address, 0x00);

				spi_W5100('w', S0_RX_RD0, ((rx_pointer+1) & 0xFF00) >> 8);
				spi_W5100('w', S0_RX_RD1, (rx_pointer+1) & 0x00FF);
				spi_W5100('w', S0_CR, Sn_CR_RECV);
				

				//TX
				
				uint16_t tx_write_pointer = spi_W5100('r', S0_TX_WR0, 0x00);
				tx_write_pointer = ((tx_write_pointer & 0xFF) << 8) + spi_W5100('r', S0_TX_WR1, 0x00); //Get received size	
				uint16_t tx_offset = tx_write_pointer & Sn_TX_MASK;
				uint16_t tx_address = S0_TX_BASE + tx_offset;

				if((tx_offset+1) > Sn_TX_MASK) tx_address = S0_TX_BASE;
				
				spi_W5100('w', tx_address, incoming);

				spi_W5100('w', S0_TX_WR0, ((tx_write_pointer+1) & 0xFF00) >> 8);
				spi_W5100('w', S0_TX_WR1, (tx_write_pointer+1) & 0xFF);
				spi_W5100('w', S0_CR, Sn_CR_SEND);
			}
		}
	}
	
	return 0;
}

void error(uint8_t code) {
	while(1) { //Continuously flash error code
		uint8_t i;
		for(i = 0;i < code+1;i++) {
			PORTB = PORTB & ~(1 << PINB0); //Status LED On
			_delay_ms(500); //Delay approx 1sec
			PORTB |= (1 << PINB0); //Status LED Off
			_delay_ms(500); //Delay approx 0.5sec
		}
		
		_delay_ms(5000); //Delay approx 7sec
	}
}

uint8_t spi_W5100(uint8_t opcode, uint16_t address, uint8_t data) {
	PORTB = PORTB & ~(1 << PINB2); //Make SS Low

	if(opcode == 'r' || opcode == 0x0F) {
		if(spi(0x0F) != 0x00) error(1); //read
	} else if(opcode == 'w' || opcode == 0xF0) {
		if(spi(0xF0) != 0x00) error(1); //write
	}
	
	if(spi((address&0xFF00) >> 8) != 0x01) error(2); //Address 1
	if(spi(address&0x00FF) != 0x02) error(3); //Address 2
	
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

ISR (PCINT0_vect) {
	//Interrupt Request from W5100
}
