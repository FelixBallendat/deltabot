/***************************************************************************

Der "DeltaBot" ist eine Projektarbeit an der Hochschule München im Studiengang Maschinenbau mit Schwerpunkt Mechatronik.
Sommersemester 2015, Fakultät 03

Die Projektarbeit wurde erstellt von:
Felix Ballendat
Jakob Karbaumer
Markus Birg
Markus Moser

Die Projektarbeit wurde betreut von:
Hr. Prof. Höcht
Hr. Dipl. Ing Pleger



***************************************************************************/


/***************************************************************************

Allgemeine ToDo-Liste;

Alle
- Dokumentation schreiben!

Mechanik
- Beugewinkel der Kugelköpfe erhöhen. Vorr
- Platinenhalterung. Oben drauf, am besten mit Plexiglasscheibe
- Halterung für Pumpe, Netzteil und Ventil ==> Aluplatte? Edelstahl?
- Ringlicht abschleifen/Streuscheibe
- Oberfläche Mattieren ==>Papier? Mattlack?
- Neuer Schlauch. Aus dem Labor?

Elektronik
- Neue Treiber testen
- Hitzeproblem am 7805: Ursache? Kühlkörper?
- Magnetventil anschließen, Freilaufdiode!
- Verkabelung ==> LED-Ring
- Netzteil bestellen

Software:
- Geradeninterpolation
- IPK Fehler beseitigen
- GCode Kommunikation Fertig und einfügen
- Hauptprogramm schreiben
- Optimierung der Bilderkennung



***************************************************************************/
#include "DeltaBot.h"

//Nur zum debuggen
char debugbuffer[65];


//Globale Variablen. Diese sind extern und werden aus der Motor.c geholt.
//Speichert die aktuelle Sollposition sowie den Sollvorschub in [mm] bzw. [mm/min]. Endpunkt der Geraden.
//Muster: XSollPos, YSollPos, ZSollPos, SollFeed
//Diese Variable wird durch die UART Eingabe gesetzt.
//extern volatile double SollPosition [3];

//Speichert die Startposition. Entspricht der Istposition des Greifers in [mm].
//Startposition der Geraden.
//extern volatile double StartPosition [3];


void Nullpunktverschiebung(double SollPosition[])
{
	SollPosition[0] -= 150; 
	SollPosition[1] -= 150;
	SollPosition[2] -= 250;		
}

int main(void)
{
	uart_puts_P("start");
	
	//Interrupts erlauben
	sei();
	
	//Als erstes werden mit dieser Funktion alle Ein- und Ausgänge gesetzt.
	PinKonfiguration();
	
	//ADC EInschalten. Wird für den dritten Endschalter gebraucht.
	ADCKonfiguration();

	//Serielle Schnittstelle initialisieren um G-Code empfangen und verarbeiten zu koennen
	initSeriell();

	//Referenzfahrt ausführen, um die Position der Arme zu definieren.
	Referenzfahrt();
	

		extern uint8_t Koordinaten_veraendert;	// wird 1 gesetzt, sobald neue anzufahrende Koordinaten vorliegen
		extern uint8_t M_Befehl_ausgefuehrt;	// wird 1 gesetzt, sobald sobald ein M-Befehl ausgefuehrt wurde
		
		extern uint8_t GeradeAusgefuert;
		
		//TestlaufAusfuehren();
		
		//Speichert die Startposition. Entspricht der Istposition des Greifers in [mm].
		//Startposition der Geraden.
		//Startposition initial setzen, dies entspricht der Lage an den Endschaltern!
		double StartPosition [3] = {0, 0, -154 }; //Exakter Wert: -153.815; aktuelle Kinematik funktioniert erst ab -172mm.
		
		
		//Speichert die aktuelle Sollposition sowie den Sollvorschub in [mm] bzw. [mm/min]. Endpunkt der Geraden.
		//Muster: XSollPos, YSollPos, ZSollPos, SollFeed
		//Diese Variable wird durch die UART Eingabe gesetzt.
		double SollPosition[4] = {StartPosition[0], StartPosition[1], StartPosition[2], 10};
			
		
		#pragma region testlauf

		//Teste inverse Kinematik
		int32_t resultSchritte[] = {0,0,0};		
		InverseKinematik(50, 50, -200, resultSchritte);
		if(resultSchritte[0] == 1345 && resultSchritte[1] == 2922 &&resultSchritte[2] == 1785)
		{
			uart_puts_P("Kinematik korrekt");
		}
		else
		{
			uart_puts_P("Kinematik falsch");
		}
		
		double testSollPosition[4] = {200, 200, 50, 10};
		Nullpunktverschiebung(testSollPosition);
		Gerade(StartPosition, testSollPosition, 10);
					
		#pragma endregion testlauf
			
			
		while(1)
		{

			LeseGcode(SollPosition);		
			
			if (Koordinaten_veraendert == 1)
			{	
				Nullpunktverschiebung(SollPosition);
				
				Gerade(StartPosition, SollPosition, SollPosition[3]);
				
				for (int8_t i = 0; i < 3; i++)
				{
					StartPosition[i] = SollPosition[i];
				}
				
				Koordinaten_veraendert = 0;
			}	
			
			if (M_Befehl_ausgefuehrt ==1 || GeradeAusgefuert ==1)
			{
				uart_puts_P("ok");
			}
		}
			/*
	for(;;)
	{
	LeseGcode();

	//	if Koordinaten_veraendert == 1
	//	{
	//		Motoren_verfahren();
	//		Koordinaten_veraendert = 0;
	//	}
	//
	//	if M_Befehl_ausgefuehrt ODER Motoren-wurden-erfolgreich-verfahren == 1
	//	{
	//		uart_puts_P("ok");	//fordert nächsten Befehl an
	//	}
	}*/
		
// 		SollPosition[0] = 0;
// 		SollPosition[1] = -100;
// 		SollPosition[2] = -180;
// 		Gerade();
		
// 		SollPosition[0] = 0;
// 		SollPosition[1] = 0;
// 		SollPosition[2] = -156;
// 		Gerade();


		

		
		
		
		//extern volatile int32_t ABSschritte[3];
		// 		   uart_puts_P("\n\n\nAbsolut Steps: ");
		// 		   for(int n = 0; n < 3; n++)
		// 		   {
		// 			   dtostrf(ABSschritte[n], 8, 4, debugbuffer);
		// 			   uart_puts(debugbuffer);
		// 			   uart_puts_P("\n");
		// 		   }

		
		
		
		
		
		//Anzahl der zu verfahrenden Schritte pro zwischenschritt, pro Motor. Vorzeichen gibt die Richtung an.
		//Wird in der Funktion Gerade berechnet und dann in der Funktion Zwischenschritte verwendet.
		// 		  extern volatile int16_t DeltaSteps [3];
		//
		// 			  DeltaSteps[0] = 500;
		// 			  DeltaSteps[1] = 500;
		// 			  DeltaSteps[2] = 500;
		//
		// 			  ZwischenschritteAusfuehren();
		
		
		//Globale Variablen Extern:
		//Hier werden die Absolutpositionen der Stepper gespeichert. Ausgangspunkt ist der Endschalter. Variable kommt aus der Berechnung.c
		// 		  extern volatile int32_t ABSschritte[3];
		//
		//
	}
	
	//void TestlaufAusfuehren()
	//{
		//
		//Gerade(100, 100, 100, 0);
	//}
