/*
 * mkencoder.c
 *
 *  Obs³uga ENKODERA z przyciskiem
 *
 *  Created on: 8 lut 2016
 *      Author: Miros³aw Kardaœ
 *      website: www.atnel.pl
 */

#ifndef MKENCODER_H_
#define MKENCODER_H_

/*--------------------- inicjalizacja / obs³uga enkodera w main.c -----------------------------
    mk_encoder_init();
    register_enc_event_callback( enc_event );    // w³asna funckja obs³ugi enkodera
    register_enc_event_sw_callback( enc_sw );    // w³asna funckja obs³ugi klawisza enkodera

    // w programie g³ównym u¿ytkownik ma do dyspozycji zmienne:
     1. enco_cnt - licznik enkodera (-32768 do 32767)
     2. enco_dir - kierunek ( 1 - left, 2 - right )

    while(1) {
        ENCODER_EVENT();
    }
-----------------------------------------------------------------------------------------------
*/

//****************************** konfiguracja sprzêtowa enkodera ******************************

#define HALF_STEP        0        //     0 - fullstep encoder
//    1 - halfstep encoder

#define USE_ENC_SWITCH    1        //  0 - brak obs³ugi przycisku
//    1 - z obs³ug¹ przycisku

#define    USE_INT_IRQ        1        //  0 - obs³uga typu pooling
//    1 - obs³uga na przerwaniach


#if USE_INT_IRQ == 1

#define    ENC_INT            -1        // ( -1 - INT0 & INT1, -2 - PCINT )

#if ENC_INT == -2

#define PCINT_IRQ_VECT    PCINT2_vect    // wektor przerwania PCINT
#define PCMSK_REG        PCMSK2        // nazwa rejestru maskuj¹cego grupy przerwañ PCINT
#define PCINT_A            PCINT18        // nazwa przerwania PCINTx dla pinu A enkodera
#define PCINT_B            PCINT19        // nazwa przerwania PCINTx dla pinu B enkodera
#endif

#endif


//------------ piny A i B enkodera -----------------------
#define ENC_A        (1<<PD2)    // pin pod³¹czony do wejœcia INTx
#define ENC_B        (1<<PD3)    // pin pod³¹czony do innego wejœcia tego samego portu
#define ENC_AB_PIN    PIND
#define ENC_AB_PORT    PORTD


//------------- przycisk enkodera -------------------------
#define ENC_SW            (1<<PD4)    // pin pod³¹czony do dowolnego wejœcia
#define ENC_SW_PORT        PORTD
#define ENC_SW_PIN        PIND
#define ENC_SW_DIR        DDRD

//**************************************************** koniec konfiguracji sprzêtowej ****************


#ifdef GIMSK
#define GICR     GIMSK
#endif



#define ENC_SW_ON        (ENC_SW_PIN & ENC_SW)

#define enc_A_HI        (ENC_A_PIN & ENC_A)
#define enc_B_HI        (ENC_B_PIN & ENC_B)



#define ENC_LEFT     0x10
#define ENC_RIGHT     0x20


//extern volatile int enco_cnt;
extern volatile uint8_t enco_dir;




//--------- funkcje biblioteki enkodera -------------------
void mk_encoder_init( void );    // inicjalizacja obs³ugi enkodera
void ENCODER_EVENT( void );        // zdarzenie enkodera do pêtli g³ównej

// rejestracja w³asnej funkcji obs³ugi przycisku enkodera
void register_enc_event_sw_callback( void ( *callback )( void ) );

// rejestracja w³asnej funkcji obs³ugi enkodera
void register_enc_event_callback( void ( *callback )( void ) );


void encoder_proc( void );
int get_encoder( void );
void set_encoder( int val );



#endif /* MKENCODER_H_ */