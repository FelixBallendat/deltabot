#include "DeltaBot.h"

#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>

//Zeichen, welches das Ende eines gesendeten G-Code Satzes angibt

#define GCode_SatzEndeZeichen '\n'

//G-Codes für die Maschinenbefehle der Schaltausgaenge:

#define Schalt1an "M50"
#define Schalt1aus "M51"
#define Schalt2an "M52"
#define Schalt2aus "M53"
#define Schalt3an "M54"
#define Schalt3aus "M55"
#define Schalt4an "M56"
#define Schalt4aus "M57"
#define Schalt5an "M58"
#define Schalt5aus "M59"
#define Referenzieren "M60"

//Baud-Rate fuer serielle Uebertragung:

#define UART_BAUD_RATE      115200

// Funktionen implementieren

int initSeriell(void);

void uart_gets( char* Buffer, uint8_t MaxLen );

int LeseGcode(double sollPosition[]);