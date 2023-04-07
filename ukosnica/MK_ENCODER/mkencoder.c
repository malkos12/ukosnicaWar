    /*
 * mkencoder.c
 *
 *  Obs�uga ENKODERA z przyciskiem
 *
 *  Created on: 8 lut 2016
 *      Author: Miros�aw Karda�
 *      website: www.atnel.pl
 */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include<stdbool.h>


#include "mkencoder.h"



volatile int         enco_cnt;
volatile uint8_t     enco_dir;
volatile uint8_t     enco_flag;
volatile bool screenEncoderEnable;



// wska�nik do funkcji callback dla zdarzenia ENCODER_EVENT() do obs�ugi klawisza
static void (*enc_event_sw_callback)( void );

// funkcja do rejestracji funkcji zwrotnej w zdarzeniu ENCODER_EVENT()
void register_enc_event_sw_callback(void (*callback)(void)) {
    enc_event_sw_callback = callback;
}


// wska�nik do funkcji callback dla zdarzenia ENCODER_EVENT() do obs�ugi enkodera
static void (*enc_event_callback)( void );

// funkcja do rejestracji funkcji zwrotnej w zdarzeniu ENCODER_EVENT()
void register_enc_event_callback(void (*callback)( void )) {
    enc_event_callback = callback;
}



#if HALF_STEP == 1
// Use the half-step state table (emits a code at 00 and 11)
const uint8_t enc_tab[6][4] PROGMEM = {
  {0x3 , 0x2, 0x1,  0x0}, {0x23, 0x0, 0x1,  0x0},
  {0x13, 0x2, 0x0,  0x0}, {0x3 , 0x5, 0x4,  0x0},
  {0x3 , 0x3, 0x4, 0x10}, {0x3 , 0x5, 0x3, 0x20}
};
#else
// Use the full-step state table (emits a code at 00 only)
const uint8_t enc_tab[7][4] PROGMEM = {
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x10},
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x20},
  {0x6, 0x5, 0x4,  0x0},
};
#endif

int get_encoder( void ) {
    uint8_t sreg = SREG;
    cli();
    int res = enco_cnt;
    SREG = sreg;
    return res;
}

void set_encoder( int val ) {
    uint8_t sreg = SREG;
    cli();
    enco_cnt = val;
    SREG = sreg;
}



void encoder_proc( void ) {
    static uint8_t enc_stat;
    register uint8_t pin = ENC_AB_PIN;
    register uint8_t ABstate = ((pin&ENC_B)?2:0) | ((pin&ENC_A)?1:0);
    enc_stat = pgm_read_byte( &enc_tab[enc_stat & 0x0f][ABstate] );
    ABstate = (enc_stat & 0x30);
    if (ABstate && screenEncoderEnable) {
        enco_dir = ABstate;
        if( ABstate == ENC_RIGHT ) enco_cnt--;
        else enco_cnt++;
        enco_flag = 1;
    }
}



void mk_encoder_init( void ) {

#if USE_ENC_SWITCH == 1
    ENC_SW_PORT |= ENC_SW;
#endif

    ENC_AB_PORT |= ENC_A | ENC_B;

    encoder_proc();


#if USE_INT_IRQ    == 1


#if    ENC_INT == -1

#ifndef GICR
    EIMSK |= (1<<INT0);        /* enable INT0 */
    EICRA |= (1<<ISC00);    /* INT0 - falling & rising edge */
#else
    GICR |= (1<<INT0);        /* enable INT0 */
    MCUCR |= (1<<ISC00);    /* INT0 - falling & rising edge */
#endif

#ifndef GICR
    EIMSK |= (1<<INT1);        /* enable INT1 */
    EICRA |= (1<<ISC10);    /* INT1 - falling & rising edge */
#else
    GICR |= (1<<INT1);        /* enable INT1 */
    MCUCR |= (1<<ISC10);    /* INT1 - falling & rising edge */
#endif

#endif

#if    ENC_INT == -2
#ifndef GICR
    PCICR |= (1<<PCIE2);
#else
    GICR |= (1<<PCIE);
#endif
    PCMSK_REG |= (1<<PCINT_B)|(1<<PCINT_A);
#endif


#endif


}

void ENCODER_EVENT( void ) {

#if USE_INT_IRQ == 0
    encoder_proc();

#endif

    if( enco_flag ) {
        if( enc_event_callback ) enc_event_callback();
        enco_flag = 0;
    }




#if USE_ENC_SWITCH == 1

    static uint16_t enc_key_lock;

    if( !enc_key_lock && !(ENC_SW_PIN & ENC_SW ) ) {
        enc_key_lock = 65000;

        if(enc_event_sw_callback) enc_event_sw_callback();

    }
    else if( enc_key_lock && (ENC_SW_PIN & ENC_SW ) ) enc_key_lock++;

#endif

}

#if USE_INT_IRQ    == 1

#if    ENC_INT == -2
ISR (PCINT_IRQ_VECT) {
    encoder_proc();
}
#endif


#if    ENC_INT == -1
ISR (INT1_vect) {
    encoder_proc();
}

ISR (INT0_vect) {
    encoder_proc();
}
#endif

#endif

