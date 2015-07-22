#ifndef __DeltaBot_H__
#define __DeltaBot_H__

#define F_CPU 20000000UL				//CPU Frequenz definieren, 20 MHz.

#include <avr/io.h>						//Standard Library f�r AVR-spezifische Funktionen.
#include <stdint.h>						//Stellt Variablentypen zur Verf�gung.
#include <inttypes.h>					//Stellt Variablentypen zur Verf�gung.
#include <util/delay.h>					//Stellt die _delay_ms(); Funktion zur Verf�gung.
#include <avr/interrupt.h>				//Stellt Interrupt-Funktionen zur Verf�gung.
#include <stdlib.h>						//Standard C Funktionen wie abs(); werden durch diese Library zur Verf�gung gestellt.



#include "DeltaBot_IODefinitionen.h"	//Definitionen der Pins, etc.
#include "DeltaBot_SerielleKommunikation.h" //Alles f�r die Serielle Kommunikation
#include "DeltaBot_Motor.h"				//Ansteuerung der Motoren und Schaltausg�nge
#include "uart.h"
#include "DeltaBot_Berechnungen.h"



#endif