//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
//#define F_CPU 4000000UL  // 4 MHz
#include <avr/delay.h>
#include "AVR_Starter.h"
#include "lcd.c"
#include "adc.c"
#include "utils.c"
#include "slaves.c"
#include "spi_master.c"

#include "defines.h"

uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;


//#define TWI_PORT		PORTC
//#define TWI_PIN		PINC
///#define TWI_DDR		DDRC


//#define SDAPIN		4
//#define SCLPIN		5

#define EXT_ADRESSE				0x62
#define TWI_ERR_BIT				7
#define EXT		1

volatile uint8_t					Programmstatus=0x00;
uint8_t Tastenwert=0;
uint8_t TastaturCount=0;
volatile uint16_t					Manuellcounter=0; // Counter fuer Timeout
uint16_t TastenStatus=0;
uint16_t Tastencount=0;
uint16_t Tastenprellen=0x01F;
//volatile uint8_t		TWI_Flag=0;

// TWI
static volatile uint8_t Write_Err;
static volatile uint8_t Read_Err;
static volatile uint8_t EEPROM_Err;

// SPI
volatile uint8_t data;

volatile uint8_t textpos=0;
volatile uint8_t inpos=0;

volatile uint8_t data_array[8];
volatile uint8_t arraypos=0;

volatile uint8_t inbuffer[16];
volatile uint8_t outbuffer[16];


volatile uint8_t rxdata =0;
uint8_t erfolg =0;
volatile uint8_t			twistatus=0;
extern volatile uint8_t rxbuffer[TWI_BUFFERSIZE];
extern volatile uint8_t txbuffer[TWI_BUFFERSIZE];
volatile uint8_t twi_Call_count0=0;	//	Anzahl TWI-Calls in einer Schleife
volatile uint8_t twi_Reply_count0=0;	//	Anzahl TWI-Replies in einer Schleife
volatile uint8_t twi_Stat_count=0;	//	Anzahl Resets nach erfolglosen TWI-Aufrufen


// Rotary
volatile uint16_t rot_eingang_plus=0x00;
volatile uint16_t rot_eingang_A=0x00;
volatile uint16_t rot_count_A=0x00;
volatile uint16_t rot_loopcount_A=0x00;

volatile uint8_t old_rot_pin=1;
volatile uint8_t new_rot_pin=1;
volatile uint8_t akt_rot_pin=1;
volatile uint8_t rot_changecount=0;

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

// SPI
volatile char incoming=0;
volatile uint8_t outcounter=0;
extern volatile uint8_t spi_rxbuffer[SPI_BUFFERSIZE];
extern volatile uint8_t spi_txbuffer[SPI_BUFFERSIZE];

//volatile uint8_t out[8] = {'H','e','l','l','o',' ',' ',' '};
volatile uint8_t out[8][8] ={
{'H','a','l','l','o',' ',' ',' '},
{'W','i','r',' ',' ',' ',' ',' '},
{'g','e','h','e','n',' ',' ',' '},
{'a','u','f',' ','d','e','n',' '},
{'B','a','c','h','t','e','l',' '},
{'u','n','d',' ','a','u','c','h'},
{'a','u','f',' ','d','e','n',' '},
{'T','u','r','m',' ',' ',' ','*'}};

void delay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms)
   {
		_delay_ms(0.96);
		ms--;
	}
}

// TWI


// http://www.mikrocontroller.net/attachment/211211/Drehgeber_per_interrupt.c
//#define DEF_BAUDRATE    9600
//#define F_CPU           16000000
/*
// funktionen zur ser. Datenausgabe
void init_serio0(void)
{
   uint8_t temp;
   UCSR0B = 0x0;                               // alles abschalten
   UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);         // default wie nach reset 8 Bit, async, no par.
  //UBRR0=((F_CPU/16L/(DEF_BAUDRATE/10L)+5L)/10L) - 1;    // passend gerundet
   UBRR0=((F_CPU/16L/(DEF_BAUDRATE/10L)+5L)/10L) - 1;    // passend gerundet
   UCSR0B |= (1<<RXEN0) | (1<<TXEN0);          // RX + TX freigeben
   temp = UDR0;
}

void ser_putchar(char c)
{
   while(!(UCSR0A & (1<<UDRE0)));
   UDR0 = c;
}

void ser_string(char *s)
{
   while(*s) ser_putchar(*s++);
   ser_putchar(13);
   ser_putchar(10);
}
 
 // ausgeben
 ser_string(ergebnis_str);               // und per UART ausgeben
*/



void device_init(void)
{
   //Loop LED
   
   LOOPLED_DDR	|= (1<<LOOPLED_PIN);
   //LOOPLED_PORT	PORTD
   //LOOPLED_PIN	4
   
   
	//LCD
	/* INITIALIZE */
	LCD_DDR |=(1<<LCD_RSDS_PIN);
	LCD_DDR |=(1<<LCD_ENABLE_PIN);
	LCD_DDR |=(1<<LCD_CLOCK_PIN);
   
   OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI
	
   /*
   // TWI vorbereiten
	TWI_DDR |= (1<<SDA_PIN);//Bit 4 von PORT C als Ausgang für SDA
	TWI_PORT |= (1<<SDA_PIN); // HI
	
	TWI_DDR |= (1<<SCL_PIN);//Bit 5 von PORT C als Ausgang für SCL
	TWI_PORT |= (1<<SCL_PIN); // HI
   */
   
}

void rotary_init(void)
{
   ROTARY_DDR &= ~(1<<ROTARY_PIN0);   // Eingang 0
   ROTARY_PORT |= (1<<ROTARY_PIN0); // HI
   ROTARY_DDR &= ~(1<<ROTARY_PIN0); //Eingang 1
   ROTARY_PORT |= (1<<ROTARY_PIN1);
   
   // PCINT16/RXD PD0
   PCICR |= (1<<PCIE2);
   PCIFR |= (1<<PCIF2);
   PCMSK2|= (1<<PCINT16);
   //PCMSK2|= (1<<PCINT17);
}



ISR(PCINT2_vect)
{
   //rot_eingang0++;
   //OSZI_A_LO;
   
   uint8_t rot_pin0 = ROTARY_PIN & 0x01; //Eingang 0
   
   uint8_t rot_pin1 = (ROTARY_PIN & 0x02); //Eingang 1
   
   
   new_rot_pin = ROTARY_PIN & 0x02;
   
   akt_rot_pin = new_rot_pin ^ old_rot_pin;
   uint8_t deltaA=20;
   if (rot_count_A > 0x5)
   {
      deltaA = 1;
   }
   //deltaA = 10;
   rot_count_A=0;
   
  if ((rot_pin0==1) && (rot_pin1 == 0))
   {
      if (0xFFFF - rot_eingang_A > deltaA)
      {
         rot_eingang_A += deltaA;
      }
      else
      {
         rot_eingang_A = 0xFFFF;
      }
     
   }
 else if ((rot_pin0==0) && (rot_pin1 == 0))
   {
      if (rot_eingang_A > deltaA)
      {
         rot_eingang_A -= deltaA;
      }
      else
      {
         rot_eingang_A = 0;
      }
      
   }
   spi_txbuffer[2] = (rot_eingang_A & 0x00FF);
   spi_txbuffer[3] = ((rot_eingang_A & 0xFF00)>>8);
    //OSZI_A_HI;
}

int main (void)
{
	device_init();
	
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	delay_ms(1000);
	lcd_cls();
	lcd_puts("READY\0");
	
	delay_ms(1000);
	uint8_t i=0;
   
   rotary_init();
   sei();
   i2c_init();
   spi_master_init();
   uint16_t spiloop =0;
   uint8_t twiloop=0;
   //DDRB |=(1<<0);
   device_init();
#pragma mark while
	while (1) 
	{
		
		loopCount0 ++;
		//_delay_ms(2);
		
		if (loopCount0 >=0xFFFF)
		{
			//OSZI_A_LO;
			//LOOPLED_PORT ^= (1<<LOOPLED_PIN);
         
			loopCount1++;
         rot_loopcount_A++;
         
			if (rot_loopcount_A > 0xFF)
         {
            rot_count_A++;
            rot_loopcount_A=0;
         }
         //OSZI_A_HI;
         
         
			if ((loopCount1 >0xFFF0) )
			{
            LOOPLED_PORT ^= (1<<LOOPLED_PIN);
            loopCount1=0;

            // SPI
            if ((twiloop)==3)
            {
               twiloop=0;
               //OSZI_A_LO;
               
               //_delay_us(100);
               uint8_t outindex=0;
               textpos=0;
               inbuffer[0] = '\0';
               //outbuffer[0] = '\0';
               spiloop++;
               //lcd_gotoxy(0,1);
               spi_txbuffer[0]= '$';
               spi_txbuffer[1]= 0x27;
               
               SPI_PORT |=  (1<<SPI_CS); // CS LO, Start, Slave soll erstes Byte laden
               _delay_us(1);
               outindex=0;
              
               SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start, Slave soll erstes Byte laden
               _delay_us(1);
               
               //PORTB &= ~(1<<0);
               for (outindex=0;outindex < SPI_BUFFERSIZE;outindex++)
               //for (outindex=0;outindex < 4;outindex++)
               {
                  OSZI_A_LO;
                  //SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start, Slave soll erstes Byte laden
                  _delay_us(2);
                  
                  SPDR = spi_txbuffer[outindex];
                  
                  while(!(SPSR & (1<<SPIF)) && spiwaitcounter < WHILEMAX)
                  {
                    // spiwaitcounter++;
                  }
                  
                  spi_rxbuffer[outindex] = SPDR;
                  spiwaitcounter=0;
                  _delay_us(1);
                  //SPI_PORT |= (1<<SPI_SS); // SS HI End, Slave soll  Byte-Zähler zurücksetzen
                  OSZI_A_HI;
               }
               //PORTB |=(1<<0);
               arraypos++;
               arraypos &= 0x07;
               //spi_rxbuffer[outindex] = '\0';
               //outbuffer[outindex] = '\0';
               //char rest = SPDR;
               
               // wichtig
               _delay_us(5);
               
               SPI_PORT |= (1<<SPI_SS); // SS HI End, Slave soll  Byte-Zähler zurücksetzen
               SPI_PORT &= ~(1<<SPI_CS); // CS HI End, Slave soll  Byte-Zähler zurücksetzen
               //OSZI_A_HI;
               
               lcd_gotoxy(0,0);
               lcd_puts("in");
               //lcd_gotoxy(12,1);
               lcd_puts("  ");
               //lcd_gotoxy(0,0);
               lcd_puthex(spi_rxbuffer[0]);
               lcd_putc(' ');
               lcd_puthex(spi_rxbuffer[1]);
               lcd_putc(' ');
               lcd_puthex(spi_rxbuffer[2]);
               lcd_putc(' ');
               lcd_puthex(spi_rxbuffer[3]);
               lcd_putc(' ');
               lcd_puthex(spi_rxbuffer[4]);
               lcd_putc('*');
              
               for (i=0;i < 8;i++)
               {
                //  lcd_puthex(spi_rxbuffer[i]);

               }
               //outcounter++;
               if (outcounter > 9)
               {
                  //outcounter =0;
               }
            }
            
            // TWI
				{
               twiloop++;
               lcd_gotoxy(8,1);
               lcd_puts(" EA:");
               //lcd_puthex((rot_eingang_A & 0xFF00)>>8);
               
               lcd_putint16((rot_eingang_A));
                //txbuffer[0]=0;
               //txbuffer[1]=0;

			}

			}
			
			loopCount0 =0;
		}
		
      
      
#pragma mark Tastatur 
		/* ******************** */
      if (TASTATUR_ON)
      {
		initADC(TASTATURPIN);
		Tastenwert=(readKanal(TASTATURPIN)>>2);
		
//		lcd_gotoxy(3,1);
//		lcd_putint(Tastenwert);
//		Tastenwert=0;
		if (Tastenwert>5)
		{
			/*
			 0:											1	2	3
			 1:											4	5	6
			 2:											7	8	9
			 3:											x	0	y
			 4: Schalterpos -
			 5: Manuell ein
			 6: Schalterpos +
			 7: 
			 8: 
			 9: 
			 
			 12: Manuell aus
			 */
			 
			TastaturCount++;
			if (TastaturCount>=200)
			{
				
				 
				 lcd_gotoxy(17,1);
				 lcd_puts("T:  \0");
				 //lcd_putint(Tastenwert);
				 
				uint8_t Taste=Tastenwahl(Tastenwert);
				//Taste=0;
				 lcd_gotoxy(19,1);
				 lcd_putint1(Taste);
				 //delay_ms(600);
				// lcd_clr_line(1);
				 

				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
//				lcd_gotoxy(18,1);
//				lcd_putint2(Taste);
				
				switch (Taste)
				{
					case 0:// Schalter auf Null-Position
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
							Manuellcounter=0;
							Programmstatus |= (1<<MANUELLNEU);
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SI:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SP\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							*/
						}
						
					}break;
						
					case 1:	//	
					{ 
					if (Programmstatus & (1<<MANUELL))
						{
						Manuellcounter=0;
						
						}
					}break;
						
					case 2://
					{ 
					
						if (Programmstatus & (1<<MANUELL))
						{
						Manuellcounter=0;
						
						
						}
						
					}break;
						
					case 3: //	Uhr aus
					{ 
						if (Programmstatus & (1<<MANUELL))
						{
						uint8_t i=0;
						Manuellcounter=0;
						

						}
					}break;
						
					case 4://
					{ 
                  uint8_t i=0;

					}break;
						
					case 5://
					{ 
						Programmstatus |= (1<<MANUELL);	// MANUELL ON
						Manuellcounter=0;
						MANUELL_PORT |= (1<<MANUELLPIN);
						Programmstatus |= (1<<MANUELLNEU);
						//lcd_clr_line(1);
						/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
						*/
					}break;
						
					case 6://
					{ 
					
					}break;
						
					case 7:// Schalter rückwaerts
					{ 
						if ((Programmstatus & (1<<MANUELL)) )
						{
							Manuellcounter=0;
							Programmstatus |= (1<<MANUELLNEU);
							//OSZIALO;
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							//OSZIAHI;
							*/
						}
						else 
						{
							
						}
	
					}break;
						
					case 8://
					{ 
						
					}break;
						
					case 9:// Schalter vorwaerts
					{ 
						Manuellcounter=0;
						if ((Programmstatus & (1<<MANUELL)) )
						{
							//OSZIALO;
							Programmstatus |= (1<<MANUELLNEU);
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer

							//OSZIAHI;
							*/
							
						}
						else 
						{
							//lcd_gotoxy(10,0);
							//lcd_puts("S:!\0");
						}
					

					}break;

					case 10:// *
					{ 
						
					}break;

					case 11://
					{ 
						
					}break;
						
					case 12: // # Normalbetrieb einschalten
					{
						Programmstatus &= ~(1<<MANUELL); // MANUELL OFF
						Programmstatus &= ~(1<<MANUELLNEU);
						MANUELL_PORT &= ~(1<<MANUELLPIN);
					}
						
				}//switch Tastatur
				
//				delay_ms(400);
//				lcd_gotoxy(18,1);
//				lcd_puts("  ");		// Tastenanzeige loeschen

			}//if TastaturCount	
			
		}
      } // if TASTATUR_ON
	}
	
	
	return 0;
}
