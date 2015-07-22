#ifndef __DeltaBot_H__
#define __DeltaBot_H__

#define F_CPU 20000000UL				//CPU Frequenz definieren, 20 MHz.

#include <avr/io.h>						//Standard Library für AVR-spezifische Funktionen.
#include <stdint.h>						//Stellt Variablentypen zur Verfügung.
#include <inttypes.h>					//Stellt Variablentypen zur Verfügung.
#include <util/delay.h>					//Stellt die _delay_ms(); Funktion zur Verfügung.
#include <avr/interrupt.h>				//Stellt Interrupt-Funktionen zur Verfügung.
#include <stdlib.h>						//Standard C Funktionen wie abs(); werden durch diese Library zur Verfügung gestellt.



#include "DeltaBot_IODefinitionen.h"	//Definitionen der Pins, etc.
#include "DeltaBot_SerielleKommunikation.h" //Alles für die Serielle Kommunikation
#include "DeltaBot_Motor.h"				//Ansteuerung der Motoren und Schaltausgänge
#include "uart.h"
#include "DeltaBot_Berechnungen.h"



#endif