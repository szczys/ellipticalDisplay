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

#include "KS0108.h"


#define     BUFLEN      20   //Message length plus terminating character?
volatile uint8_t SShighFlag = 1;
volatile uint8_t bufferIDX = 0;
volatile uint8_t spiRxBuffer[BUFLEN] = { 0 };

/**************** Display Variables*******************************/
//These should probably be a struct and maybe not global.
int8_t calHuns = 8;
int8_t calTens = 8;
int8_t calOnes = 8;
uint8_t calDP = 1;
uint8_t heart = 1;
uint8_t statusCals = 1;
uint8_t statusTime = 1;
uint8_t statusPulse = 1;
uint8_t statusScan = 1;
uint8_t statusRPM = 1;
uint8_t statusDistance = 1;
int8_t minTens = 2;
int8_t minOnes = 0;
uint8_t timeColon = 1;
int8_t secTens = 0;
int8_t secOnes = 0;
int8_t spdHuns = 0;
int8_t spdTens = 0;
int8_t spdOnes = 0;
uint8_t targetUp = 1;
uint8_t targetDn = 1;
uint8_t rpmBarGraph = 6;
uint8_t rpmTargetGraph = 6;
/*****************************************************************/

/**************** GLCD Positioning Values ************************/
#define     SCAN_LINE   0
#define     SCAN_LBL    6

#define     TIME_LINE   1
#define     TIME_LBL    6
#define     TIME_MT     41
#define     TIME_MO     46
#define     TIME_ST     51
#define     TIME_SO     56

#define     SPEED_LINE  2
#define     SPEED_LBL   6
#define     SPEED_H     47
#define     SPEED_T     53
#define     SPEED_O     59

#define     CAL_LINE    3
#define     CAL_LBL     6

#define     RPM_BAR_LINE    4
#define     RPM_BAR_LBL     6

#define     RPM_TAR_LINE    5
#define     RPM_TAR_LBL     6
/*****************************************************************/

/**************** Prototypes *************************************/
void init_IO(void);
void init_interrupts(void);
void decodeDigit(uint8_t byteH, uint8_t byteL, uint8_t decPlace);
void displaySegments(uint8_t decPlace, uint8_t decoded);
void showDisplay(void);
int main(void);
/**************** End Prototypes *********************************/

void init_IO(void){

}

void decodeDigit(uint8_t byteH, uint8_t byteL, uint8_t decPlace) {
    uint8_t decoded = ((byteH & 0x07)<<5) | (byteL>>3);
    displaySegments(decPlace, decoded);
    //displaySegments(decPlace,0b11010011);
}

void displaySegments(uint8_t decPlace, uint8_t decoded) {
/*
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
*/
}

void init_SPI(void){
    //Slave using interrupts
    SPCR |= (1<<SPE) | (1<<SPIE) | (1<<CPOL) | (1<<CPHA);
}

void init_interrupts(void) {
    sei();

}

void showDisplay(void) {
    /* Screen organization:
    Scan
    Time/Distance TimeDisplay 
    Speed SpeedVal RPMlabel
    Cal/Pulse Caloriesdigits
    rpmBarGraph targetUp TargetDn
    rmpTargetGraph
    */
    
    if (statusScan) {
        GLCD_GoTo(SCAN_LBL, SCAN_LINE);
        GLCD_WriteString("Scan");
    }
    GLCD_GoTo(TIME_LBL, TIME_LINE);
    if (statusTime) {
        GLCD_WriteString("Time: ");
    }
    else if (statusDistance) {
        GLCD_WriteString("Dist: ");
    }

    GLCD_GoTo(TIME_MT, TIME_LINE);
    GLCD_WriteChar(minTens+48);
    GLCD_WriteChar(minOnes+48);
    if (timeColon) { GLCD_WriteChar(':'); }
    GLCD_WriteChar(secTens+48);
    GLCD_WriteChar(secOnes+48);

    GLCD_GoTo(SPEED_LBL, SPEED_LINE);
    //TODO: need if statement for speed indicator
    GLCD_WriteString("Speed: ");
    GLCD_GoTo(SPEED_H, SPEED_LINE);
    GLCD_WriteChar(spdHuns+48);
    GLCD_WriteChar(spdTens+48);
    GLCD_WriteChar(spdOnes+48);
    if (statusRPM) {
        GLCD_WriteChar(' ');
        GLCD_WriteString("RPM");
    }
    
    GLCD_GoTo(CAL_LBL, CAL_LINE);
    if (statusCals) {
        GLCD_WriteString("Calories: ");
    }
    else if (statusPulse) {
        GLCD_WriteString("Pulse: ");
    }
    GLCD_WriteChar(calHuns+48);
    GLCD_WriteChar(calTens+48);
    if (calDP) { GLCD_WriteChar('.'); }
    GLCD_WriteChar(calOnes+48);
    
    GLCD_GoTo(RPM_BAR_LBL, RPM_BAR_LINE);
    GLCD_WriteString("Current: ");
    GLCD_WriteChar(rpmBarGraph+48);
    if (targetUp) {
        GLCD_WriteChar(' ');
        GLCD_WriteChar(128);
    }
    else if (targetDn) {
        GLCD_WriteChar(' ');
        GLCD_WriteChar(129);
    }

    GLCD_GoTo(RPM_TAR_LBL, RPM_TAR_LINE);
    GLCD_WriteString("Target: ");
    GLCD_WriteChar(rpmTargetGraph+48);
}

int main(void)
{
    uint8_t validMsg[BUFLEN] = { 0 };
    validMsg[0] = 0xA0;
    
    GLCD_Initalize();
    GLCD_ClearScreen();
    GLCD_WriteString("Hello World");
    
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
                //TODO: Get the last byte out of the register so we can get 5 'stranded' bits
                if (spiRxBuffer[0] == 0xA0) {
                    //message valid. Copy to persistent array
                    for (uint8_t i=0; i<BUFLEN; i++) {
                        //TODO: Check if valid message is different.
                        validMsg[i] = spiRxBuffer[i];
                    }
                }
                //Get values ready for next message
                SShighFlag = 1;
                bufferIDX = 0;
                
                //TODO: If message is different, parse values and update display here
                
                sei();      //enable interrupts
            }
        }
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
