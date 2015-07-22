#include "DeltaBot_IODefinitionen.h"


//In dieser Funktion werden alle Ein- und Ausgänge gesetzt.
//Diese Funktion sollte am Anfang im Hauptprogramm aufgerufen werden.
int PinKonfiguration(void)
{
	//Ausgänge konfigurieren
	sbi(S1STEPDDR, S1STEPPIN);
	sbi(S2STEPDDR, S2STEPPIN);
	sbi(S3STEPDDR, S3STEPPIN);
	
	sbi(S1DIRDDR, S1DIRPIN);
	sbi(S2DIRDDR, S2DIRPIN);
	sbi(S3DIRDDR, S3DIRPIN);
	
	sbi(SCHALT1DDR, SCHALT1PIN);
	sbi(SCHALT2DDR, SCHALT2PIN);
	sbi(SCHALT3DDR, SCHALT3PIN);
	sbi(SCHALT4DDR, SCHALT4PIN);
	sbi(SCHALT5DDR, SCHALT5PIN);
	
	
	//Eingänge Konfigurieren
	cbi(NOTAUSDDR, NOTAUSPIN);
	cbi(KONFIGDDR, KONFIGPIN);
	cbi(STARTDDR, STARTPIN);
	cbi(STOPPDDR, STOPPPIN);
	cbi(S1ENDDDR, S1ENDPIN);
	cbi(S2ENDDDR, S2ENDPIN);
	//HIER FEHLT NOCH DER DRITTE ENDSCHALTER. DIESER KANN NICHT ALS EINGANG KONFIGURIERT WERDEN, DA NUR ADC MESSUNG MÖGLICH.
	
	return 1;
}