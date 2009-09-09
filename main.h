/* AVR-libC Includes */
#include <avr/io.h> // Definition of interrupt names
#include <avr/interrupt.h> // ISR interrupt service routine
#include <util/delay.h> //Delay stuff

/* Other Includes */
#include "W5100.h" //W5100 Registers

/* Memory Definitions */
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

/* Function Prototypes */
uint8_t spi(uint8_t data); //Basic 8-Bit SPI exchange
uint8_t spi_W5100(uint8_t opcode, uint16_t address, uint8_t data); //Send a full 32-Bit Command to the W5100 chip
void error(uint8_t code); //Display Errors on Status LED
void init();
void init_S0_TCP();
