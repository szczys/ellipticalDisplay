/********************************
* SloJak                    	*
* MIT License               	*
* Copyright 2016 - Mike Szczys  *
* http://jumptuck.com 	    	*
*				                *
********************************/

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define     BUFLEN      20
volatile uint8_t SShighFlag = 1;
volatile uint8_t bufferIDX = 0;
volatile uint8_t spiRxBuffer[BUFLEN] = { 0 };

/**************** Prototypes *************************************/
void init_IO(void);
void init_interrupts(void);
int main(void);
/**************** End Prototypes *********************************/

void init_IO(void){
    DDRB |= 1<<PB0 | 1<<PB1;
    PORTB |= 1<<PB0 | 1<<PB1;
}

void init_SPI(void){
    //Slave using interrupts
    SPCR |= (1<<SPE) | (1<<SPIE) | (1<<CPOL) | (1<<CPHA);
}

void init_interrupts(void) {
    //Pin change interrupts for the rotary encoder
    /*
    PCICR |= 1<<PCIE0;      //enable PCINT0_vect  (PCINT0..7 pins)
    PCMSK0 |= 1<<PCINT6;    //interrupt on PCINT6 pin
    PCMSK0 |= 1<<PCINT7;    //interrupt on PCINT7 pin

    //Timer overflow for debounce and systick
    TCCR0B |= 1<<CS02 | 1<<CS00;		//Divide by 1024
    TIMSK0 |= 1<<TOIE0;			//enable timer overflow interrupt
    */
    sei();

}

int main(void)
{
    uint8_t validMsg[BUFLEN] = { 0 };
    validMsg[0] = 0xA0;
    
    init_IO();
    init_SPI();
    init_interrupts();
    
    while(1)
    {
        if (SShighFlag == 0) {
            if (PINB & (1<<PB2)) {
                //Interrupt routine set the flag low but SS is now high (SPI complete)
                //Let's validate message and get ready for next message
                cli();      //clear interrupts so we don't overwrite message
                
                if (spiRxBuffer[0] == 0xA0) {
                    //message valid. Copy to persistent array
                    for (uint8_t i=0; i<BUFLEN; i++) {
                        validMsg[i] = spiRxBuffer[i];
                    }
                }
                //Get values ready for next message
                SShighFlag == 1;
                bufferIDX = 0;
                
                sei();      //enable interrupts
            }
        }
        if (validMsg[1] == 0xF0) {
            PORTB |= 1<<PB1;
        }
        else { PORTB &= ~(1<<PB1); }
    }
}

ISR(SPI_STC_vect) {
    SShighFlag = 0;
    spiRxBuffer[bufferIDX++] = SPDR;
    //TODO: Handle buffer overflow condition
}
