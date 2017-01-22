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

/**************** Prototypes *************************************/
void init_IO(void);
void init_interrupts(void);
int main(void);
/**************** End Prototypes *********************************/

void init_IO(void){
    DDRB |= 1<<PB0;
    PORTB |= 1<<PB0;
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

    sei();
    */
}

int main(void)
{
    init_IO();
    init_interrupts();
    //_delay_ms(200);


    while(1)
    {
       PINB |= 1<<PB0;
      _delay_ms(200);
    }
}

/*
ISR(PCINT0_vect) {
    // Encoder service code is from Circuits@Home
    // https://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros

    static uint8_t old_AB = 3;  //lookup table index
    static int8_t encval = 0;   //encoder value
    static const int8_t enc_states [] PROGMEM = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table

    old_AB <<=2;  //remember previous state
    old_AB |= ((ENC_RD>>6) & 0x03 ); //Shift magic to get PB6 and PB7 to LSB
    encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));
    // post "Navigation forward/reverse" event
    if( encval < -3 ) {  //four steps forward
        knobChange = -1;
        encval = 0;
    }
    else if( encval > 3  ) {  //four steps backwards
        knobChange = 1;
        encval = 0;
    }
}

ISR(TIMER0_OVF_vect) {
    static unsigned char ct0, ct1;
    unsigned char i;

    TCNT0 = (unsigned char)(signed short)-(F_CPU / 1024 * 10e-3 + 0.5);   // preload for 10ms

    i = key_state ^ ~BUT_PIN;    // key changed ?
    ct0 = ~( ct0 & i );          // reset or count ct0
    ct1 = ct0 ^ (ct1 & i);       // reset or count ct1
    i &= ct0 & ct1;              // count until roll over ?
    key_state ^= i;              // then toggle debounced state
    key_press |= key_state & i;  // 0->1: key press detect
    
    static uint8_t delayCount = STD_DELAY/10;   //This is a red herring. delayCount should be exposed to game.c
    
    if (++timingDelay >= delayCount) {
        timingDelay = 0;
        ++move_tick;
    }
}
*/
