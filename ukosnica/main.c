                                                                                                                /*
*
*  Created on: 2018-03-24
*       Autor: Mateusz Małkowski
*       Poprawki: 2023.04
*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include<stdbool.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "MK_ENCODER/mkencoder.h"
#include "LCD/lcd44780.h"
#include <stdlib.h>
#include <avr/eeprom.h>

//parametry konfiguracyjne
#define maxScreen 5  //ilosc ekranów które widzi uzytkownik, nie dotyczy ekranów wyswietlanych np podczas bazowania

uint8_t normalSpeed=15; //szybkosc przesowu
#define baseSpeed 100 // 1-255 im wiecej tym wolniej
#define minAngle 0 // jeżeli minus to od 0 do 180 czyli kąt razy 2
#define maxAngle 180 // dla 180 kat +45st dla 90 0st
#define maxSetpoint 11000 //maxSetpoint
#define minSetpoint 0

#define lcdFrequance 4000 // odswierzanie wyswietlacza

//PINOUT
//-------------------------------------------------------------------------------------------------------------

//Do sterownika silnika krokowego
#define pulsPIN (1<<PB0)
#define directionPIN (1<<PB1)
#define enablePIN (1<<PD7)

//Czujniki
//#define emergencyPIN (1<<PB3) // czujniki krańcowe
#define basePIN (1<<PB2)  // czujnik bazy INT2

//Przyciski i enkoder
//#define aEncoderPIN (1<<PD2)//sygnał A encodera    INT0
//#define bEncoderPIN (1<<PD3)//sygnał B encodera    INT1
//#define setEncoderPIN (1<<PD4)//przycisk encodera
#define keyScreenPIN (1<<PD0) // przycisk przełączania ekranów
//#define keyRightPIN (1<<PD1)

#define BUZ       (1<<PD6)

//Porty
#define encoderPort PINC //port encodera 8bit absolutny
#define lcdPort PINA     //port wyswietlacza
#define drivePort PINB   //port sterownika napędu

//---------------------------------------------------------------------------------------------------------------

//Makrodefinicje
//---------------------------------------------------------------------------------------------------------------
#define pulsInvert PORTB ^= pulsPIN
#define pulsOn PORTB |=pulsPIN
#define pulsOff PORTB &= ~pulsPIN
#define keyDownScreen !(PIND & keyScreenPIN)
#define keyDownRight !(PIND & keyRightPIN)
#define basePosition !(PINB & basePIN)
#define keyDownSetEncoder !(PIND & setEncoderPIN)
#define offEnable PORTD |= enablePIN
#define onEnable PORTD &= ~enablePIN
#define invertDirection motorDirection=!motorDirection
#define onDirectionPin PORTB |= directionPIN
#define offDirectionPin PORTB &=~directionPIN


#define screenEncoderEnableInvert screenEncoderEnable=!screenEncoderEnable
//#define currentSetInvert currentSet=!currentSet
#define invertManualEnable manualEnable=!manualEnable


#define BUZ_OFF   PORTD &= ~BUZ
#define BUZ_ON    PORTD |= BUZ
#define BEEP      BUZ_ON; _delay_ms(50); BUZ_OFF
#define BEEP1     BUZ_ON; _delay_us(500); BUZ_OFF
//---------------------------------------------------------------------------------------------------------------



//Zmienne
//------------------------------------------------------------------------------------------------------------------

//pozycja
//uint16_t turnsValue;
uint8_t encoderValue;
uint32_t aPosition=7086;
uint8_t lastEncoderAngle=0;

uint8_t actualEncoderAngle=0;
int8_t buffor=0;
uint16_t lastMemoryAngle=300;
uint32_t lastMemoryFactor=0;
uint8_t angleSet;
uint8_t i;
//uint8_t angleCurrent;
uint32_t angleFactor=171;
uint32_t lastFactor=171;
uint32_t setpoint=0;


bool screenEncoderEnable= false;

bool manualEnable=false;
bool baseAllowed=false;
bool goToPositionEnable=false;


//wyswietlacz
int currentSet=0;
uint32_t setPosition=90;;
uint16_t lcdTimer=0;
uint32_t pulsPosition;

//do sterownika silnika krokowego
uint8_t driveEnable = 1;
bool motorDirection = false; //1-do czujnika bazy 0-kierunek przeciwny
uint8_t baseActual = 1; // czy wymagane bazowanie 1 - nie wymagane; 0- wymagane

//do enkodera sterowania
uint32_t val;
int initialValue=90;

//stale
static uint8_t grayTable[256] = {127, 126, 124, 125, 121, 120, 122, 123, 115, 114, 112, 113, 117, 116, 118, 119, 103, 102, 100, 101, 97, 96, 98, 99, 107, 106, 104, 105, 109, 108, 110, 111, 79, 78, 76, 77, 73, 72, 74, 75, 67, 66, 64, 65, 69, 68, 70, 71, 87, 86, 84, 85, 81, 80, 82, 83, 91, 90, 88, 89, 93, 92, 94, 95, 31, 30, 28, 29, 25, 24, 26, 27, 19, 18, 16, 17, 21, 20, 22, 23, 7, 6, 4, 5, 1, 0, 2, 3, 11, 10, 8, 9, 13, 12, 14, 15, 47, 46, 44, 45, 41, 40, 42, 43, 35, 34, 32, 33, 37, 36, 38, 39, 55, 54, 52, 53, 49, 48, 50, 51, 59, 58, 56, 57, 61, 60, 62, 63, 191, 190, 188, 189, 185, 184, 186, 187, 179, 178, 176, 177, 181, 180, 182, 183, 167, 166, 164, 165, 161, 160, 162, 163, 171, 170, 168, 169, 173, 172, 174, 175, 143, 142, 140, 141, 137, 136, 138, 139, 131, 130, 128, 129, 133, 132, 134, 135, 151, 150, 148, 149, 145, 144, 146, 147, 155, 154, 152, 153, 157, 156, 158, 159, 223, 222, 220, 221, 217, 216, 218, 219, 211, 210, 208, 209, 213, 212, 214, 215, 199, 198, 196, 197, 193, 192, 194, 195, 203, 202, 200, 201, 205, 204, 206, 207, 239, 238, 236, 237, 233, 232, 234, 235, 227, 226, 224, 225, 229, 228, 230, 231, 247, 246, 244, 245, 241, 240, 242, 243, 251, 250, 248, 249, 253, 252, 254, 255};


//EEPROM MEMORY
//uint8_t EEMEM memoryAngle=999;
//uint_t EEMEM memoryFactor = angleFactor;


//------------------------------------------------------------------------------------------------------------------



//deklaracje funkcji
//void enkoderek (void);
// własne funkcje do obsługi enkodera - callbacki
void my_encoder( void );
void enc_switch( void );
//void direction(void);
void display_lcd( uint8_t cb ); // funkcja wyświetljąca informacje na LCD
void button(void);
void actualPosition(void);
void actualAngle(void);
void drive(void);
void saveIfFactorChange(void);
void lcdRefresh(void);
void goToPosition(void);
void baseRequire(void);
void setZero(void);
void enableDriveIfNotInPosition(void);


//---------------------------------przerwania-----------------------
//przerwanie od timera
ISR( TIMER0_COMP_vect )
{

    if((goToPositionEnable==true && currentSet==0)|| (goToPositionEnable==true && currentSet==4) || (currentSet==2 && manualEnable==true)) 	//(()||(baseActual=0)||(goEnable==true)) //(przejaz w trybie manualnym),(bazowanie),(dojazd do pozycji zadanej)
    {
       pulsInvert;
//		if(goToPositionEnable==true)
//		{
//							goToPosition(); //sprawdzenie czy zostala osiagnieta pozycja
//		}
    }
}

//---przerwanie od czujnika bazy----
//po najechaniu na czujnik weyzerój pozycje aktualną
//ISR(INT2_vect)
//{
//    aPosition=0;
//    baseActual=1;
//    motorDirection=true;
//    currentSet=0;
//	OCR0 = normalSpeed;
//}

//---------------------------------petla glówna----------------------------------------------------------
int main( void )
{
	GICR |=(1<< INT2); //rejestr przerwania zewnętrznego
	//MCUCR= (1<<ISC01) | (1<<ISC00);

     //ustawienie portów
    DDRA |= ( 1 << PA7 );    // ustawiamy kierunek linii podświetlenia LCD jako WYJŚCIE
    PORTA |= ( 1 << PA7 );    // zalaczamy podświetlenie LCD - stan wysoki

    DDRB |= pulsPIN | directionPIN;    //ustawienie pinów do sterownika silnika jako WYJSCIA
    DDRD |=enablePIN; //ustawienie pinu enable jako wyjscie
    onEnable;
    PORTB |= directionPIN; // ustawienie 1-prawo? 0-lewo po starcie w prawo
    DDRB &= ~basePIN;      //ustawienie czujników jako wejscia
    PORTB |= basePIN;      //podciagnięcie do plus

    DDRC &= ( 1 << PC0 ) | ( 1 << PC1 ) | ( 1 << PC2 ) | ( 1 << PC3 ) | ( 1 << PC4 ) | ( 1 << PC5 ) | ( 1 << PC6 ) | ( 1 << PC7 );         //port encodera pozycji ustawiony jako WEJSCIE
    PORTC = 0xff;                                                                                                                         // podciągniecie do plusa przez wewnetrzny rezystor

    DDRD &=  ~keyScreenPIN ; // ustawienie pinów przycisków i enkodera zadajacego na wejscia
    DDRD |= BUZ;  //ustawienie buzera jako wyjscie
    PORTD |= keyScreenPIN; // podciągniecie do plusa
    BUZ_OFF;

    //inicjalizacja enkodera sterowania
    mk_encoder_init();
    register_enc_event_callback( my_encoder );
    register_enc_event_sw_callback( enc_switch );
    set_encoder( initialValue );

    actualAngle();
    lastEncoderAngle=actualEncoderAngle;

//ustawienie timera do generowania impulsów
    TCCR0 |= ( 1 << WGM01 ); //tryb CTC timera
    //TCCR0 |= ( 1 << CS02 ) | ( 1 << CS00 );   //preskaler =1024
    TCCR0 |= ( 1 << CS01 ) | ( 1 << CS00 );   //preskaler =64 //dość szybko
    //TCCR0 |= ( 1 << CS02 );   //preskaler =256
    //TCCR0 |= ( 1 << CS01 );   //preskaler =8
    OCR0 = normalSpeed;
    // czestotliwośc impulsu 11 059 200/1024/39=276 Hz
    TIMSK |= ( 1 << OCIE0 ); // ZEZWOLENIE NA PRZERWANIE COMAPRE MATCH    */


sei(); // wlaczenie globalnego przerwania

//odczyt z pamieci eeprom
//do {} while (!eeprom_is_ready());
//lastFactor=eeprom_read_dword(&memoryFactor);
//do {} while (!eeprom_is_ready());
//
//do {} while (!eeprom_is_ready());
//lastMemoryAngle=eeprom_read_dword(&memoryAngle);
//do {} while (!eeprom_is_ready());

angleFactor=lastFactor; //przypisanie wartosci z pamieci do programu
//baseRequire();               //sprawdzenie czy wymagane bazowanie, jezeli tak to bazowanie
BEEP;
//inicjalizacja wyśietlacza
    lcd_init();
//ekran powitalny
    lcd_cls();
    lcd_locate( 0, 0 );
    lcd_str_P( PSTR( "Step Motor Driver" ) );
    lcd_locate( 1, 0 );
    lcd_str_P( PSTR( "ver. 0.9   by MM" ) );
    _delay_ms(100);
    display_lcd( currentSet );


//inicjalizacja enkodera
    my_encoder();



//--------------------------------------------------------pętla WHILE-------------------------------------------
    while ( 1 )
    {

    ENCODER_EVENT();
    button();
    drive();
    actualAngle();
    goToPosition();

    lcdRefresh();
   // saveIfFactorChange();


    }

//--------------------------------------------------------koniec pętli WHILE-------------------------------------------
}
//--------------------------------------------------------koniec pętli głównej----------------------------------------------------------

// obsługa zdarzenia pokręcania enkoderem
void my_encoder( void ) {
	BEEP1;

    val = get_encoder();
      if(currentSet==0)
      {
             //min max_encoderaSterowania
             if(val>=maxAngle)  val=maxAngle;
             if(val<0)     val=0;
             set_encoder( val);
             setPosition=val;



      }

      else if(currentSet==1)
      {

      angleFactor=val;
      }
      else if(currentSet==2)
      {
    	  invertDirection;
      }
      else if(currentSet==4)
      {
          //min max_encoderaSterowania
          if(val>=maxSetpoint)  val=maxSetpoint;
          if(val<minSetpoint)     val=minSetpoint;
          set_encoder( val);
          setpoint=val;
      }
      else if(currentSet==5)
      {
    	  //min max_encoderaSterowania
    	             if (normalSpeed>=255)  normalSpeed=255;
    	             if(normalSpeed<0)     normalSpeed=0;
    	             set_encoder( normalSpeed);
    	             normalSpeed=val;

      }

    // wyświetlenie informacji
     display_lcd( currentSet );
   }

// obsługa zdarzenia wciśnięcia przycisku enkodera
// każde kliknięcie powoduje cykliczne przełączanie

void enc_switch( void )
{
    BEEP;



    if(currentSet==0)
    {
        //set_encoder(val);
        screenEncoderEnableInvert;
        setpoint=(uint32_t)setPosition*10000/(uint32_t)angleFactor;
        enableDriveIfNotInPosition();

    }
    else if(currentSet==1)
    {
        set_encoder(angleFactor);
        screenEncoderEnableInvert;
    }
    else if(currentSet==2)
    {
        invertManualEnable;

	}
    else if(currentSet==3)
	{
		setZero();
	}
    else if(currentSet==4)
	{
    	screenEncoderEnableInvert; //enable adjusting setpoint
    	set_encoder(aPosition);
    	enableDriveIfNotInPosition(); //run if in not correct position

	}
    else if(currentSet==5)
	{	set_encoder(normalSpeed);
	 OCR0 = normalSpeed;
    	 screenEncoderEnableInvert;
	}

    display_lcd( currentSet );
    enco_dir = 0;


}

void button ()
{
  if(keyDownScreen)
  {
   //   BEEP;
 _delay_ms(200);
  currentSet++;
  if(currentSet>maxScreen) currentSet=0;
  if(currentSet==2) screenEncoderEnable=true;
  else {screenEncoderEnable=false;
 manualEnable=false;}
  //screenEncoderEnable=false;
  display_lcd(currentSet);
  }
//    if(keyDownRight && currentSet==2)
//  {
// _delay_ms(200);
// manualEnable=true;
//  display_lcd(currentSet);
//
//  }
//  else
//  {
//      manualEnable=false;
//}

}


void actualAngle()
{
  encoderValue = encoderPort;
          for ( i = 0; i < 256;i++ )
        {
         if ( encoderValue == grayTable[i] )
            {
            actualEncoderAngle = i;
            break;
           }
        }
}
void actualPosition ()
{
                actualAngle();



                 if ( actualEncoderAngle != lastEncoderAngle )
                  {
                      buffor=actualEncoderAngle-lastEncoderAngle;
                      aPosition=aPosition-buffor;
                      lastEncoderAngle = actualEncoderAngle;
                 }
                 else{
                    //aPosition=aPosition;
                 }

                    if(aPosition!=0)
                 {
                 pulsPosition=aPosition*256+actualEncoderAngle;
                 }
                 else
                 {
                     pulsPosition=actualEncoderAngle;
                 }

}
void drive()
{

	//Enable PIN
	if(manualEnable==true || goToPositionEnable==true){
		onEnable;
		//pulsOn;

	}
	else {
		offEnable;
		//pulsOff;
	}
	//Direction PIN
	if(motorDirection==true)
	{
		onDirectionPin;

	}
	else offDirectionPin;



}


//ZAPIS WSPOLCZYNNIKA PO ZMNIANIE
void saveIfFactorChange(void)
{

    if(angleFactor!=lastFactor && screenEncoderEnable==false)
    {
     //   eeprom_write_dword(&memoryFactor,angleFactor);
        lastFactor=angleFactor;
    }

}


//BAZOWANIE
//----------------------------------------------------------------------------

void baseRequire(void)
{
	if(actualEncoderAngle!=lastMemoryAngle)
	{
		display_lcd( 3 );

		while(baseAllowed==false)
		{
			_delay_ms(5);
		}
			display_lcd( 4 );
			OCR0 = baseSpeed;//ustaw wolna predkosc
			motorDirection=false;//ustaw kierunek w kierunku czujnika
			baseActual=0;
			baseAllowed=false;


	}
	baseActual=1;

}

//DOJAZD DO POZYCJI
//----------------------------------------------------------------------------
void goToPosition(void)
{
	actualPosition();


if(currentSet==0 || currentSet==4)
{


        int32_t temp=setpoint-aPosition;
        if(temp>0)
        {
         motorDirection=false;
        }
        else
        {
        motorDirection=true;
        }

	if(aPosition==setpoint)
	{
	 goToPositionEnable=false;
	}
}





}
//LCD
//------------------------------------------------------------------------------

void lcdRefresh()
{
    //odświerzanie LCD
    lcdTimer++; //licznik do odświerzania wyświetlacza

    if(lcdTimer>=lcdFrequance)
    {
        lcdTimer=0;
        display_lcd( currentSet );
    }
}




void display_lcd( uint8_t currentSet)
{
// ekran 1
if(currentSet==0)
{

    lcd_cls();
    lcd_locate( 0, 0 );
    lcd_str_P( PSTR( "Kat:" ) );
    lcd_locate( 0, 4 );

   if(screenEncoderEnable) {drive(); lcd_char('[');}
   uint16_t tempValue=0;
   	   if(setPosition==maxAngle/2) angleSet=0;
   	   else if(setPosition>maxAngle/2)
   		   {
   		   lcd_char('+');
   		   tempValue=setPosition-maxAngle/2;
   		   }
   	   else if(setPosition<maxAngle/2)
   	   {
   		 lcd_char('-');
   		 tempValue=maxAngle/2-setPosition;
   	   }
   		angleSet=tempValue/2;
   		   lcd_int(angleSet);
   		   if(tempValue%2)
   		   {
   			lcd_str(".5");
   		   }
     lcd_char(223);
    if(screenEncoderEnable) lcd_char(']');
   lcd_str( "" );
   lcd_str("Set:");
   lcd_int(setPosition);



    //druga linia
    lcd_locate( 1, 0 );
    lcd_str_P( PSTR( "A:" ) );
    lcd_str( " __ " );
     lcd_locate( 1, 8 );
    lcd_str_P( PSTR( "S:" ) );
    lcd_str( " " );

        lcd_locate( 1, 3 );
          //lcd_int( actualEncoderAngle);
        lcd_int( aPosition );
        lcd_str( "  " );
        lcd_locate( 1, 10 );
        lcd_int(setpoint);



        // lcd_locate( 0, 0 );
        // lcd_int( val );
        // lcd_char(' ');
 }
 //ekran 2
 if(currentSet==1)
 {

    lcd_cls();
    lcd_locate(0,0);
    lcd_str("Wspol:(kat*20 000/p)");
    lcd_char(223);
  //  lcd_str("last;");
  //  lcd_int(lastValue);
     lcd_locate(1,0);
    if(screenEncoderEnable) lcd_char('[');
    lcd_int(angleFactor);
    if(screenEncoderEnable) {lcd_char(']'); }

 }
 //ekran 3
 if(currentSet==2)
 {

    lcd_cls();
    lcd_locate(0,0);
    lcd_str("Manual:");
    if(manualEnable==true){lcd_str("jade");}
    else {lcd_str("           ");}
    //druga linia
    lcd_locate( 1, 0 );
    lcd_str_P( PSTR( "p:" ) );
        lcd_locate( 1, 3 );
        lcd_int( aPosition );
        lcd_str( "  ");
        lcd_int(actualEncoderAngle);
        lcd_str( "  ");
        if(motorDirection==true)
        {
            lcd_str("Lewo");
        }
        else
        {
            lcd_str("Prawo");
        }

 }
//ekran 4
 if(currentSet==3)
 {
    lcd_cls();
    lcd_locate(0,0);
    lcd_str("Baza dla 0"); lcd_char(223);
    //druga linia
        lcd_locate( 1, 0 );
        lcd_str_P( PSTR( "A:" ) );
        lcd_str( " __ " );
         lcd_locate( 1, 8 );
        lcd_str_P( PSTR( "P:" ) );
        lcd_str( " __ " );

            lcd_locate( 1, 3 );
              //lcd_int( actualEncoderAngle);
            lcd_int( aPosition );
            lcd_str( "  " );
            lcd_locate( 1, 10 );
           lcd_int(actualEncoderAngle);
             lcd_str( "  " );
 }

 //ekran 5
 if(currentSet==4)
 {
	    lcd_cls();
	    lcd_locate(0,0);
	    lcd_str("Setpoint: " );
	    lcd_locate( 10,0 );

	      if(screenEncoderEnable) {drive(); lcd_char('[');}

	       		   lcd_int(setpoint);

	       if(screenEncoderEnable) lcd_char(']');

	       //druga linia wyswietlacza
	       lcd_locate( 1, 0 );
	       lcd_str("Actual: ");  lcd_int( aPosition );






 }

 //ekran 6
 if(currentSet==5)
 {
    lcd_cls();
    lcd_locate(0,0);
    lcd_str("Predkosc");
    lcd_locate( 1, 0 );

      if(screenEncoderEnable) {drive(); lcd_char('[');}


       		   lcd_int(normalSpeed);

       if(screenEncoderEnable) lcd_char(']');

 }
}

void setZero ()
{
	aPosition=0;
	lastEncoderAngle=actualEncoderAngle;
}

void enableDriveIfNotInPosition()
{
    if(screenEncoderEnable==false && aPosition!=setpoint){
    	goToPositionEnable=true;
    }
}






