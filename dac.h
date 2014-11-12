//
//  dac.h
//  Power_Master
//
//  Created by Ruedi Heimlicher on 10.11.2014.
//
//

#ifndef __Power_Master__dac__
#define __Power_Master__dac__

#include <stdio.h>

#endif /* defined(__Power_Master__dac__) */

// Hardware PINs DAC

#define DAC_PORT  PORTC
#define DAC_DDR   DDRC







#define SOFT_CS       2

#define SOFT_SCK      3
#define SOFT_MOSI     4
#define SOFT_MISO     5

#define MAXSPANNUNG  756
#define MINSPANNUNG  500


// SOFT-SPI defines

#define SOFT_SPI_PORT   PORTC
#define SOFT_SPI_DDR    DDRC


#define CS_HI        SOFT_SPI_PORT |= (1<<SOFT_CS)
#define CS_LO        SOFT_SPI_PORT &= ~(1<<SOFT_CS)

#define SCL_HI       SOFT_SPI_PORT |= (1<<SOFT_SCK)
#define SCL_LO       SOFT_SPI_PORT &= ~(1<<SOFT_SCK)

#define DATA_HI      SOFT_SPI_PORT |= (1<<SOFT_MOSI)
#define DATA_LO      SOFT_SPI_PORT &= ~(1<<SOFT_MOSI)
