//
//  spi_master.h
//  AVR_Starter
//
//  Created by Ruedi Heimlicher on 19.10.2014.
//
//

#ifndef AVR_Starter_spi_master_h
#define AVR_Starter_spi_master_h

//#define SPI_BUFFERSIZE 8


#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB

#define SPI_CS       1
#define SPI_SS       2

#define SPI_MOSI     3
#define SPI_MISO     4
#define SPI_SCK      5


volatile uint8_t spi_rxbuffer[SPI_BUFFERSIZE];
volatile uint8_t spi_txbuffer[SPI_BUFFERSIZE];

unsigned char SPI_get_put_char(uint8_t cData);

#endif
