#include "DeltaBot.h"

#define hoch 1
#define runter 0



//Anzahl der Steps, die bei der Referenzfahrt wieder zur�ckgefahren wird, nachdem die Schalter mit hoher Geschwindigkeit angefahren wurden.
#define Rueckzug 50
//Wartezeit zwischen den Steps bei den schnellen Fahrten der Referzierung
#define RefGeschwSchnell 100 //bester Wert: 2
//Wartezeit zwischen den Steps bei den langsamen Fahrten der Referenzierung
#define RefGeschwLangsam 10000 //bester Wert: 10000
//Wartezeit zwischen den Steps beim Zur�ckziehen (Schritt 2)
#define RefGeschwRueckzug 100 //bester Wert: 100
//maximale Anzahl an Steps die gemacht werden, bis die Arme an den Endschaltern ankommen M�SSEN!
#define RefMaxSteps 8000

/**
 * Abfahren einer Geraden.
 * Die Funktion berechnet alle n�tigen Vorbereitungen, wie z.B. das Aufteilen in Zwischenschritte, und das Hochz�hlen dieser.
 * Das Verfahren der Motoren mit linearer Interpolation innerhalb der Zwischenschritte wird dann in der Funktion Zwischenschritte �bernommen.
 * \param startPosition Array mit X, Y, Z Wert
 * \param sollPosition Array mit X, Y, Z Wert
 * \param fwert ????
 * 
 * \return int
 */
int Gerade(double startPosition[], double sollPosition[], double fWert);

//Diese Funktion steuert die Motoren an, und �bernimmt die lineare interpolation innerhalb eines Zwischenschrittes.
//Die zu verfahrenden Steps werden in der globalen Variable DeltaSteps
//
//!BEMERKUNG!:
//Es sollte langfristig eine effizientere Methode gefunden werden. Die Methode mit Hilfe des KgV verursacht eine hohe Rechenlast,
//da Interrupts aufgerufen werden, in denen kein Motor verfahren wird.
//Eine m�gliche L�sung w�re es, vorrausschauend einzelne Interrupts zu �berspringen und so nur Interrupts auszul�sen, in denen auch Motoren bewegt werden.
int ZwischenschritteAusfuehren(double fWert);

//Diese Funktion kann verwendet werden, um die Motoren anzusteuern.
//�bergeben wird: Nummer des Schrittmotors (1-3) und gew�nschte Drehrichtung (0/1)
int Stepper(int stepper, int direction);

//Hiermit k�nnen die Schaltausg�nge geschaltet werden.
//�bergeben wird: Nummer des Ausgangs (1-5) und gew�nschter Zustand (0/1)
int Schalt(int ausgang, int anaus);

//Hiermit kann der Status der Endschalter �berpr�ft werden.
//Vereinfacht den Workaround f�r den Fehler der Platine, dass der dritte Endschalter nur via ADC abgefragt werden kann, da dies dann nicht mehr seperat ber�cksichtigt werden muss.
//Liefert 1 bzw. 0 zur�ck.
int Endschalter(int schalter);

//F�hrt eine Referenzfahrt aus, und setzt an den Endschaltern alle IstPositionen der Stepper auf Null.
//Liefert eine eins zur�ck, wenn erfolgreich referenziert wurde.
int Referenzfahrt(void);

//Startet und konfiguriert den ADC.
int ADCKonfiguration(void);


//Lie�t den ADC7 Pin aus und gibt die 8 hochwertigsten Bits zur�ck.
int ADC7Read(void);