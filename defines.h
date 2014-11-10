/*
 * ----------------------------------------------------------------------------
 * defines
 */

#define SPI_BUFFERSIZE 8
#define WHILEMAX 0xFFFF // Wartezeit in while-Schleife : 5 ms

#define LOOPLED_PORT	PORTD
#define LOOPLED_DDR	DDRD
#define LOOPLED_PIN	4


#define TASTATUR_ON            0

#define MANUELL_PORT          PORTD
#define MANUELL_DDR           DDRD
#define MANUELL_PIN           PIND

#define MANUELL               7	// Bit 7 von Status
#define MANUELLPIN            6	// Pin 6 von PORT D fuer Anzeige Manuell
#define MANUELLNEU            7	// Pin 7 von Status. Gesetzt wenn neue Schalterposition eingestellt
#define MANUELLTIMEOUT        100 // Loopled-counts bis Manuell zurueckgesetzt wird. 02FF: ca. 100 s

#define ROTARY_PORT          PORTD
#define ROTARY_DDR           DDRD
#define ROTARY_PIN            PIND
#define ROTARY_PIN0           0
#define ROTARY_PIN1           1




// OSZI
#define OSZIPORT           PORTB
#define OSZIPORTDDR        DDRB
#define OSZIPORTPIN        PINB
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1

#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZI_A_TOGG OSZIPORT ^= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)




