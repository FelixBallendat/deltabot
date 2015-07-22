#include "DeltaBot.h"

#define hoch 1
#define runter 0



//Anzahl der Steps, die bei der Referenzfahrt wieder zurückgefahren wird, nachdem die Schalter mit hoher Geschwindigkeit angefahren wurden.
#define Rueckzug 50
//Wartezeit zwischen den Steps bei den schnellen Fahrten der Referzierung
#define RefGeschwSchnell 100 //bester Wert: 2
//Wartezeit zwischen den Steps bei den langsamen Fahrten der Referenzierung
#define RefGeschwLangsam 10000 //bester Wert: 10000
//Wartezeit zwischen den Steps beim Zurückziehen (Schritt 2)
#define RefGeschwRueckzug 100 //bester Wert: 100
//maximale Anzahl an Steps die gemacht werden, bis die Arme an den Endschaltern ankommen MÜSSEN!
#define RefMaxSteps 8000

/**
 * Abfahren einer Geraden.
 * Die Funktion berechnet alle nötigen Vorbereitungen, wie z.B. das Aufteilen in Zwischenschritte, und das Hochzählen dieser.
 * Das Verfahren der Motoren mit linearer Interpolation innerhalb der Zwischenschritte wird dann in der Funktion Zwischenschritte übernommen.
 * \param startPosition Array mit X, Y, Z Wert
 * \param sollPosition Array mit X, Y, Z Wert
 * \param fwert ????
 * 
 * \return int
 */
int Gerade(double startPosition[], double sollPosition[], double fWert);

//Diese Funktion steuert die Motoren an, und übernimmt die lineare interpolation innerhalb eines Zwischenschrittes.
//Die zu verfahrenden Steps werden in der globalen Variable DeltaSteps
//
//!BEMERKUNG!:
//Es sollte langfristig eine effizientere Methode gefunden werden. Die Methode mit Hilfe des KgV verursacht eine hohe Rechenlast,
//da Interrupts aufgerufen werden, in denen kein Motor verfahren wird.
//Eine mögliche Lösung wäre es, vorrausschauend einzelne Interrupts zu überspringen und so nur Interrupts auszulösen, in denen auch Motoren bewegt werden.
int ZwischenschritteAusfuehren(double fWert);

//Diese Funktion kann verwendet werden, um die Motoren anzusteuern.
//Übergeben wird: Nummer des Schrittmotors (1-3) und gewünschte Drehrichtung (0/1)
int Stepper(int stepper, int direction);

//Hiermit können die Schaltausgänge geschaltet werden.
//Übergeben wird: Nummer des Ausgangs (1-5) und gewünschter Zustand (0/1)
int Schalt(int ausgang, int anaus);

//Hiermit kann der Status der Endschalter überprüft werden.
//Vereinfacht den Workaround für den Fehler der Platine, dass der dritte Endschalter nur via ADC abgefragt werden kann, da dies dann nicht mehr seperat berücksichtigt werden muss.
//Liefert 1 bzw. 0 zurück.
int Endschalter(int schalter);

//Führt eine Referenzfahrt aus, und setzt an den Endschaltern alle IstPositionen der Stepper auf Null.
//Liefert eine eins zurück, wenn erfolgreich referenziert wurde.
int Referenzfahrt(void);

//Startet und konfiguriert den ADC.
int ADCKonfiguration(void);


//Ließt den ADC7 Pin aus und gibt die 8 hochwertigsten Bits zurück.
int ADC7Read(void);