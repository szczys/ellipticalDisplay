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

#define     BUFLEN      20   //Message length plus terminating character?
volatile uint8_t SShighFlag = 1;
volatile uint8_t bufferIDX = 0;
volatile uint8_t spiRxBuffer[BUFLEN] = { 0 };

uint8_t outB = 0x00;
uint8_t outC = 0x00;
uint8_t outD = 0x00;

#define     PORTB_MASK  (1<<PB0) | (1<<PB1) | (1<<PB6) | (1<<PB7)
#define     PORTD_MASK  (1<<PD0) | (1<<PD1) | (1<<PD2) | (1<<PD3) | (1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7)
#define     PORTC_MASK  (1<<PC0) | (1<<PC1) | (1<<PC2)

/**************** Prototypes *************************************/
void init_IO(void);
void init_interrupts(void);
void decodeDigit(uint8_t byteH, uint8_t byteL, uint8_t decPlace);
void displaySegments(uint8_t decPlace, uint8_t decoded);
int main(void);
/**************** End Prototypes *********************************/

void init_IO(void){
    //7-Seg Displays
    DDRB |= PORTB_MASK;
    DDRC |= PORTC_MASK;
    DDRD |= PORTD_MASK;
    
    PORTB |= PORTB_MASK;
    PORTC |= PORTC_MASK;
    PORTD |= PORTD_MASK;
}

void decodeDigit(uint8_t byteH, uint8_t byteL, uint8_t decPlace) {
    uint8_t decoded = ((byteH & 0x07)<<5) | (byteL>>3);
    displaySegments(decPlace, decoded);
    //displaySegments(decPlace,0b11010011);
}

void displaySegments(uint8_t decPlace, uint8_t decoded) {
    if (decPlace == 0) {
      if (decoded & 1<<0) { outD |= 1<<PD6; } // 3 B 10
      if (decoded & 1<<1) { outB |= 1<<PB7; } // 4 G 7
      if (decoded & 1<<2) { outD |= 1<<PD5; } // 6 C 8
      if (decoded & 1<<4) { outD |= 1<<PD7; } // 1 A 11
      if (decoded & 1<<5) { outB |= 1<<PB0; } // 2 F 12
      if (decoded & 1<<6) { outD |= 1<<PD4; } // 5 E 5
      if (decoded & 1<<7) { outB |= 1<<PB6; } // 7 D 6
    }
    else if (decPlace == 1) {
      if (decoded & 1<<0) { outB |= 1<<PB1; } // 3 B 15
      if (decoded & 1<<1) { outC |= 1<<PC1; } // 4 G 17
      if (decoded & 1<<2) { outD |= 1<<PD3; } // 6 C 3
      if (decoded & 1<<4) { outC |= 1<<PC0; } // 1 A 16
      if (decoded & 1<<5) { outC |= 1<<PC2; } // 2 F 18
      if (decoded & 1<<6) { outD |= 1<<PD1; } // 5 E 1
      if (decoded & 1<<7) { outD |= 1<<PD2; } // 7 D 2
    }
    PORTB = outB;
    PORTC = outC;
    PORTD = outD;
}

void init_SPI(void){
    //Slave using interrupts
    SPCR |= (1<<SPE) | (1<<SPIE) | (1<<CPOL) | (1<<CPHA);
}

void init_interrupts(void) {
    sei();

}

void segOff(void) {
    

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
                SShighFlag = 1;
                bufferIDX = 0;
                
                
                
                sei();      //enable interrupts
            }
        }
        
        outB = 0;
        outC = 0;
        outD = 0;
        
        decodeDigit(validMsg[6], validMsg[7], 0);
        decodeDigit(validMsg[5], validMsg[6], 1);
        /*
        if (validMsg[1] == 0xF0) {
            PORTB |= 1<<PB1;
        }
        else { PORTB &= ~(1<<PB1); }
        */
    }
}

ISR(SPI_STC_vect) {
    if (bufferIDX >= BUFLEN) {
        //unexpected msg length; invalidate message
        spiRxBuffer[0] = 0x00;
        bufferIDX = 1;  //Prevent overflow
    }

    spiRxBuffer[bufferIDX++] = SPDR;

    SShighFlag = 0;     //Flag used by main loop for transmission complete detection
}
