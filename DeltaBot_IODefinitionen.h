#include "DeltaBot.h"

#define sbi(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define cbi(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))

//Definitionen für Stepper 1:
#define S1DIRPIN PIND3
#define S1DIRPORT PORTD
#define S1DIRDDR DDRD
#define S1STEPPIN PIND4
#define S1STEPPORT PORTD
#define S1STEPDDR DDRD
#define S1ENDINPREG PINC
#define S1ENDPIN PINC5
#define S1ENDPORT PORTC
#define S1ENDDDR DDRC

//Definitionen für Stepper 2:
#define S2DIRPIN PINC3
#define S2DIRPORT PORTC
#define S2DIRDDR DDRC
#define S2STEPPIN PINC4
#define S2STEPPORT PORTC
#define S2STEPDDR DDRC
#define S2ENDINPREG PINC
#define S2ENDPIN PINC2
#define S2ENDPORT PORTC
#define S2ENDDDR DDRC

//Definitionen für Stepper 3:
#define S3DIRPIN PINC0
#define S3DIRPORT PORTC
#define S3DIRDDR DDRC
#define S3STEPPIN PINC1
#define S3STEPPORT PORTC
#define S3STEPDDR DDRC
#define S3ENDPIN				//
#define S3ENDPORT				//PROBLEM: Hängt am ADC7 Pin. Workaround muss noch gemacht werden.
#define S3ENDDDR				//

//Definitionen für Stepper 4:
#define S4DIRPIN PIND5
#define S4DIRPORT PORTD
#define S4DIRDDR DDRD
#define S4STEPPIN				//
#define S4STEPPORT				//PROBLEM: Hängt am ADC6 Pin. Verwendung nicht möglich, da nicht als Ausgang verwendbar.
#define S4STEPDDR				//


//Definitionen der Schalter
#define NOTAUSINPREG PIND
#define NOTAUSPIN PIND2
#define NOTAUSPORT PORTD
#define NOTAUSDDR DDRD

#define KONFIGINPREG PINB
#define KONFIGPIN PINB5
#define KONFIGPORT PORTB
#define KONFIGDDR DDRB

#define STARTINPREG PINB
#define STARTPIN PINB4
#define STARTPORT PORTB
#define STARTDDR DDRB

#define STOPINPREG PINB
#define STOPPPIN PINB3
#define STOPPPORT PORTB
#define STOPPDDR DDRB


//Definitionen der Schaltausgänge
#define SCHALT1PIN PINB0
#define SCHALT1PORT PORTB
#define SCHALT1DDR DDRB
#define SCHALT2PIN PINB2
#define SCHALT2PORT PORTB
#define SCHALT2DDR DDRB
#define SCHALT3PIN PINB1
#define SCHALT3PORT PORTB
#define SCHALT3DDR DDRB
#define SCHALT4PIN PIND7
#define SCHALT4PORT PORTD
#define SCHALT4DDR DDRD
#define SCHALT5PIN PIND6
#define SCHALT5PORT PORTD
#define SCHALT5DDR DDRD






//In dieser Funktion werden alle Ein- und Ausgänge gesetzt.
//Diese Funktion sollte am Anfang im Hauptprogramm aufgerufen werden.

int PinKonfiguration(void);

