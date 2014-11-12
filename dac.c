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


#define DAC_CS  2

void dac_init(void)
{
   SOFT_SPI_DDR |= (1<<SOFT_CS); // Ausgang fuer CS DAC
   SOFT_SPI_PORT |= (1<<SOFT_CS); // HI
   SOFT_SPI_DDR |= (1<<SOFT_SCK); // Ausgang fuer SCK DAC
   SOFT_SPI_PORT |= (1<<SOFT_SCK); // HI
   SOFT_SPI_DDR |= (1<<SOFT_MOSI); // Ausgang fuer MOSI DAC
   SOFT_SPI_PORT |= (1<<SOFT_MOSI); // HI

}




uint8_t spi_out(uint8_t dataout) // von LCD_DOG_Graph
{
   //cli();
   
   // OSZI_B_LO;
   CS_LO; // Chip enable
   uint8_t datain=0xFF;
   _delay_us(1);
   uint8_t pos=0;
   SCL_LO; // SCL LO
   _delay_us(1);
   uint8_t tempdata=dataout;
   
   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         //DATA_HI;
         SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         //DATA_LO;
         SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
       _delay_us(1);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<SOFT_SCK);
      _delay_us(1);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   //  OSZI_B_HI;
   
   CS_HI;// Chip disable
   
   sei();
   return datain;
}
uint8_t spi_out16(uint8_t dataHI, uint8_t dataLO) // von LCD_DOG_Graph
{
   //cli();
   
   // OSZI_B_LO;
   CS_LO; // Chip enable
   uint8_t datain=0xFF;
   //_delay_us(1);
   uint8_t pos=0;
   SCL_LO; // SCL LO
   _delay_us(1);
   uint8_t tempdata=dataHI;
   
   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         DATA_HI;
         //SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         DATA_LO;
         //SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
      _delay_us(1);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<SOFT_SCK);
      //_delay_us(1);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   
   
   tempdata=dataLO;

   for (pos=8;pos>0;pos--)
   {
      
      if (tempdata & 0x80)
      {
         DATA_HI;
         //SOFT_SPI_PORT |= (1<<SOFT_MOSI);
         
      }
      else
      {
         DATA_LO;
         //SOFT_SPI_PORT &= ~(1<<SOFT_MOSI);
      }
      tempdata<<= 1;
      _delay_us(1);
      //SCL_HI;
      SOFT_SPI_PORT |= (1<<SOFT_SCK);
      //_delay_us(1);
      //SCL_LO;
      SOFT_SPI_PORT &= ~(1<<SOFT_SCK);
      
   }
   
  
   
   //  OSZI_B_HI;
   
   CS_HI;// Chip disable
   
   sei();
   return datain;
}


//##############################################################################################
//Writes one byte to data or cmd register
//
//##############################################################################################
void display_write_byte(unsigned cmd_data, unsigned char data)
{
   spi_out(data);
   
}

//##############################################################################################


void setDAC_soft(void)
{
   uint8_t i=0;
   DAC_PORT &= ~(1<<DAC_CS); // CS DAC Lo
   _delay_us(1);

   for (i=0;i<0x0f;i++)
   {
      DAC_PORT ^= (1<<3);
   }

   _delay_us(1);
   DAC_PORT |= (1<<DAC_CS); // HI

}

void setDAC(void)
{
   // data ausgeben an DAC
   uint16_t spiwaitcounter = WHILEMAX; // 5 ms
   int32_t tmpDAC=0;
   uint8_t dummy =0;
   uint8_t i=0;
   //cli();
//   DAC_PORT &= ~(1<<DAC_CS); // CS DAC Lo
//   _delay_us(10);
   spi_out16(spi_txbuffer[3],spi_txbuffer[2]);
   //spi_out16(0x44,0x88);
   _delay_us(1);
//   DAC_PORT |= (1<<DAC_CS); // HI
   sei();
}// end setDAC