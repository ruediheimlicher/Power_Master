//
//  dac.c
//  Power_Master
//
//  Created by Ruedi Heimlicher on 10.11.2014.
//
//

#include "dac.h"
#include "defines.h"
#include <avr/io.h>



extern volatile uint8_t spi_txbuffer[SPI_BUFFERSIZE];

#define DAC_PORT  PORTC
#define DAC_DDR  DDRC

#define DAC_CS  2

void initDAC(void)
{
   DAC_DDR |= (1<<DAC_CS); // Ausgang fuer CS DAC
   DAC_PORT |= (1<<DAC_CS); // HI
}


void setDAC(void)
{
   uint16_t spiwaitcounter = WHILEMAX; // 5 ms
   int32_t tmpDAC=0;
   uint8_t dummy =0;
   cli();
   DAC_PORT &= ~(1<<DAC_CS); // CS DAC Lo
   _delay_us(10);
   SPDR = spi_txbuffer[3]; //HI-Byte
   while(!(SPSR & (1<<SPIF)) && spiwaitcounter < WHILEMAX)
   {
      // spiwaitcounter++;
   }
   
   dummy = SPDR;
   
   _delay_us(10);
   SPDR = spi_txbuffer[2]; //LO-Byte
   while(!(SPSR & (1<<SPIF)) && spiwaitcounter < WHILEMAX)
   {
      // spiwaitcounter++;
   }
   
   dummy = SPDR;

   
   _delay_us(10);
   DAC_PORT |= (1<<DAC_CS); // HI
}// end setDAC