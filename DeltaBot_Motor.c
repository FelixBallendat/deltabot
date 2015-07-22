#include "DeltaBot_Motor.h"

//Globale Variablen Extern:
//Hier werden die Absolutpositionen der Stepper gespeichert. Ausgangspunkt ist der Endschalter. Variable kommt aus der Berechnung.c
//extern volatile int32_t ABSschritte[3];

//Nur zum debuggen, Puffer f�r die serielle �bertragung.
//Wird aus der DeltaBot.c eingebunden.
extern char debugbuffer[16];

//Globale Variablen Intern:
//Speichert die aktuelle Position der Steppermotoren in [Steps]. Nullpunkt ist der Endschalter.
volatile double StepperPosition [3] = {0, 0, 0};

//Speichert die aktuelle Sollposition sowie den Sollvorschub in [mm] bzw. [mm/min]. Endpunkt der Geraden.
//Muster: XSollPos, YSollPos, ZSollPos, SollFeed
//Diese Variable wird durch die UART Eingabe gesetzt.
//volatile double SollPosition[4] = {0, 0, 0, 0};

//Punkt der in den Zwischenschritten als n�chstes angefahren wird.
volatile double AktuellePosition[3] = {0, 0, 0};

//Speichert die gew�nschte Schrittweite, in die die Gerade zerlegt wird.
//Erster Wert zum probieren: 1mm.
double Schrittweite = 1;

//Speichert die Zeit, die f�r einen Zwischenschritt zur Verf�gung steht.
//Die Dauer wird nicht in Sekunden, sondern in CPU Takten angegeben.
//Wird in der Gerade Funktion aus dem Vorschub und der Schrittweite berechnet.
volatile int16_t TakteZwischenInterrupt = 0;

//Schaltet den Zwischenschritt-Interrupt ein.
//Ein == Eingeschaltet.
volatile uint8_t ZwischenschrittInterruptEnable = 0;

//Anzahl der zu verfahrenden Schritte pro zwischenschritt, pro Motor. Vorzeichen gibt die Richtung an.
//Wird in der Funktion Gerade berechnet und dann in der Funktion Zwischenschritte verwendet.
volatile int16_t DeltaSteps [3] = {0, 0, 0};
	
//L�nge der Geraden
double LaengeGerade = 0;
//Einheitsvektor der geplanten Geraden. Gibt die Richtung vor.
double Einheitsvektor[3] = {0, 0, 0};
//L�ngederGeraden/Schrittweite gibt die Anzahl der Zwischenschritte an.
uint16_t Zwischenschritte = 0;



volatile uint8_t gw;
volatile uint8_t kw1;
volatile uint8_t kw2;
volatile uint8_t vielfaches1=1;
volatile uint8_t vielfaches2=1;
volatile double teiler1 = 0;
volatile double teiler2 = 0;
volatile uint8_t GeradeAusgefuert = 0;


volatile uint8_t Vorzeichen[3] = {0, 0, 0};

volatile uint8_t InterruptZaehler = 1;


/**
 * \brief 
 * Abfahren einer Geraden.
 * Die Funktion berechnet alle n�tigen Vorbereitungen, wie z.B. das Aufteilen in Zwischenschritte, und das Hochz�hlen dieser.
 * Das Verfahren der Motoren mit linearer Interpolation innerhalb der Zwischenschritte wird dann in der Funktion Zwischenschritte �bernommen.
 * \param startPosition Array mit X, Y, Z Wert
 * \param sollPosition Array mit X, Y, Z Wert
 * \param fwert ????
 * 
 * \return int
 */
int Gerade(double startPosition[], double sollPosition[], double fWert)
{		
	
	/* Grober Ablauf:
	1. L�nge der Geraden bestimmen.
	2. Einheitsvektor bestimmen.
	3. Anzahl der Zwischenschritte bestimmen.
	4. Schrittfrequenz bestimmen und Timer einstellen.
	loop
	5. Zwischenschritt errechnen, Delta der Steps ausrechnen, �ber Kinematikfunktion und Istposition der Arme und Funktion "Zwischenschritte" aufrufen.
	6. Zwischenschrittz�hler aktualisieren
	/loop
	7. XYZ-Positionen aktualisieren und Funktion verlassen.
	*/
	

	LaengeGerade=0;
	//uart_puts_P("Geradenfunktion aufgerufen!\n");
	
	//Als erstes wird die L�nge der Geraden berechnet.
	//L�nge der Geraden: sqrt(a^2+b^2+c^2)
	for (uint8_t i = 0; i < 3; i++)
	{
		LaengeGerade = LaengeGerade + ((sollPosition[i] - startPosition[i]) * (sollPosition[i] - startPosition[i]));
	}
	LaengeGerade = sqrt(LaengeGerade);

	//Bestimmen des Einheitsvektors:
	//(Sollpos - Istpos)/geradenl�nge
	for (uint8_t i = 0; i < 3; i++)
	{
		Einheitsvektor[i] = ((sollPosition[i] - startPosition[i]) / LaengeGerade);
	}
	
	//Nun wird die Anzahl der Zwischenschritte bestimmt.
	//Da nur "ganze" Zwischenschritte ausgef�hrt werden, wird aufgerundet. Der letzte Zwischenschritt kann dann k�rzer sein.
	//Zum aufrunden werden erst die Nachkommastellen mit einem (int) Cast abgeschnitten, dann eins dazugez�hlt.
	Zwischenschritte = LaengeGerade / Schrittweite;
	Zwischenschritte = (uint32_t) Zwischenschritte;
	Zwischenschritte++;
	

	//Der n�chste Punkt ist der Aktuelle Punkt plus Einheitsvektor.
	//Anschlie�end wird die Position der Stepper aus der Kinematik bestimmt und das delta errechnet.
	for(uint8_t n = 0; n < 3; n++)
	{
		AktuellePosition[n] = startPosition[n] + (Schrittweite * Einheitsvektor[n]);
	}
			
	int32_t aBSschritte[3] = {0, 0, 0};
		
	InverseKinematik(AktuellePosition[0], AktuellePosition[1], AktuellePosition[2], aBSschritte);
			
	for (uint8_t n = 0; n < 3; n++)
	{
		DeltaSteps[n] = aBSschritte[n] - StepperPosition[n];
	}
	
	
	for (uint16_t i = 0; i < Zwischenschritte; i++)
	{
		ZwischenschritteAusfuehren(fWert);
	}
	
	for (int8_t i = 0; i < 3; i++)
	{
		startPosition[i] = sollPosition[i];
	}
		
	GeradeAusgefuert = 1;
	return 0;
}

//Diese Funktion steuert die Motoren an, und �bernimmt die lineare interpolation innerhalb eines Zwischenschrittes.
//Die zu verfahrenden Steps werden in der globalen Variable DeltaSteps gespeichert.
//
//!BEMERKUNG!:
//Es sollte langfristig eine effizientere Methode gefunden werden. Die Methode mit Hilfe des KgV verursacht eine hohe Rechenlast,
//da Interrupts aufgerufen werden, in denen kein Motor verfahren wird.
//Eine m�gliche L�sung w�re es, vorrausschauend einzelne Interrupts zu �berspringen und so nur Interrupts auszul�sen, in denen auch Motoren bewegt werden.
int ZwischenschritteAusfuehren(double fWert)
{
	/* Grober Ablauf:
	1. Kleinstes gemeinsames Vielfaches der Steps bestimmen.
	2. Zeit zwischen den Interrupts bestimmen. Periodendauer = Zeit zwischen Schritten/KgV.
	loop
	3. KgVCounter eins hochz�hlen, aber mit 1 beginnen!!! Weil sonst bei Null alle Motoren einen Step ausf�hren, deshalb ein Schritt pro Motor zu viel.
	4. DeltaSteps%kgvCounter = 0, dann betreffenden Motor einen Step ausf�hren lassen.
	/loop
	5. Funktion beenden.
	*/
	

	//uart_puts_P("Zwischenschritte Ausgef�hrt!\n");
	uint16_t AnzahlDerInterrupts = 0;
	

	
	//Da bei NUll die Rechnung versagt, wird immer ein STep gemacht. Da wir Mikrostepping verwenden ist ein Step nicht relevant.
	for (int8_t i = 0; i < 3; i++)
	{
		if (DeltaSteps[i] == 0)
		{
			DeltaSteps[i] = 1;
		}
	}
	
		//Vorzeichen bestimmen, damit sp�ter in der ISR die Richtung festgelegt werden kann.
		for(uint8_t i = 0; i < 3; i++)
		{
			if (DeltaSteps[i] > 0)
			Vorzeichen[i] = 0;
			else
			Vorzeichen[i] = 1;
			
			DeltaSteps[i] = abs(DeltaSteps[i]);
		}
	
	/*
	1. Gr��ten Delta Step bestimmen ==> Bestimmt die Anzahl der Interrupts, muss deswegen bei jedem Interrupt aufgerufen werden...
	2. Den gr��ten durch die beiden anderen teilen
	3. Compare Register Wert berechnen
	4. Im Interrupt den gr��ten Motor jedes mal eins weiter drehen.
	5. die anderen(int) drehen, wenn aktueller wert dem COunter entspricht. Dann (gr��ter/kleinerer Motor) hinzuf�gen.
	6. wenn COunter voll, Interrupt sperren.
	
	*/
	
	

	
	//gr��ten Wert bestimmen.
	if(DeltaSteps[0] > DeltaSteps[1])
	{
		if (DeltaSteps[0] > DeltaSteps[2])
		{
			gw = 0;
			kw1= 2;
			kw2=1;
		}
		else
		{
			gw= 2;
			kw1=0;
			kw2=1;
		}
	}
	else
	{
		if (DeltaSteps[1] > DeltaSteps[2])
		{
			gw= 1;
			kw1=0;
			kw2=2;
		}
		else
		{
			gw= 2;
			kw1=0;
			kw2=1;
		}
	}

	
	//Teiler der Motoren bestimmen, die nicht bei jedem Step bewegt werden.
	teiler1 = DeltaSteps[gw]/DeltaSteps[kw1];
	teiler2 = DeltaSteps[gw]/DeltaSteps[kw2];
	AnzahlDerInterrupts = abs(DeltaSteps[gw]);
	
		
	//Vorteiler setzen
	uint16_t Vorteiler = 64;
	sbi(TCCR1B, CS11);
	sbi(TCCR1B, CS10);
	//Timer konfigurieren
	//Betriebsmodus des Timers
	sbi(TCCR1B, WGM12);
	//Compare match Interrupt einschalten.
	sbi(TIMSK1, OCIE1A);
	
	if (fWert == 0)
	{
	 fWert = 0.1;
	 uart_puts_P("Wir haben nicht ewig Zeit! ==> Vorschub darf nicht null sein!");
	}
	 

	//OCR1A = ((F_CPU/Vorteiler)*(Schrittweite/fWert))/(DeltaSteps[gw]);
	OCR1A = 1000;

	
	//Starten der Zwischenschritt Interrupts
	ZwischenschrittInterruptEnable = 1;
	//uart_puts_P("Interrupt enable gesetzt!\n");

	
	//Der n�chste Punkt ist der Aktuelle Punkt plus Einheitsvektor.
	//Anschlie�end wird die Position der Stepper aus der Kinematik bestimmt und das delta errechnet.
	for(uint8_t n = 0; n < 3; n++)
	{
		AktuellePosition[n] = AktuellePosition[n] + Schrittweite * Einheitsvektor[n];
	}
		
	int32_t aBSschritte[3] = {0, 0, 0};
					
	InverseKinematik(AktuellePosition[0], AktuellePosition[1], AktuellePosition[2], aBSschritte);
			

	
	while (InterruptZaehler <= AnzahlDerInterrupts)
	{
	//Warten auf das beenden der Interrupts	
	}
	
	ZwischenschrittInterruptEnable = 0;
	
	for (uint8_t n = 0; n < 3; n++)
	{
			DeltaSteps[n] = aBSschritte[n] - StepperPosition[n];
	}
	
	InterruptZaehler = 1;
	vielfaches1 = 1;
	vielfaches2 = 1;
	teiler1 = 0;
	teiler2 = 0;
	
	return 1;
}

ISR (TIMER1_COMPA_vect)
{
	//uart_puts_P("Interrupt!\n");

	
	//Als erstes wird gepr�ft, ob �berhaupt gefahren werden soll.
	if (ZwischenschrittInterruptEnable == 1)
	{
		//uart_puts_P("Enable\n");
		//Hier den Motor mit den meisten Steps als erstes fahren lassen.
		Stepper((gw + 1), Vorzeichen[gw]);
		
		//Stepper((kw2 + 1), Vorzeichen[kw2]);
		//Stepper((kw1 + 1), Vorzeichen[kw1]);
		
		if (InterruptZaehler == (int) (teiler1 * vielfaches1))
		{
			Stepper((kw1 + 1), Vorzeichen[kw1]);
			vielfaches1++;
		}
		if (InterruptZaehler == (int) (teiler2 * vielfaches2))
		{
			Stepper((kw2 + 1), Vorzeichen[kw2]);
			vielfaches2++;
		}

		InterruptZaehler++;
	}
}


//Diese Funktion kann verwendet werden, um die Motoren anzusteuern.
//�bergeben wird: Nummer des Schrittmotors (1-3) und gew�nschte Drehrichtung (0/1) 1=hoch, 0=runter
int Stepper(int stepper, int direction)
{
	//Es wird nach Steppern sortiert.
	switch(stepper)
	{
		case 1:
		//Je nach gew�nschter Richtung wird der Direction Pin auf High oder low gesetzt.
		if (direction == runter)
		{
			cbi(S1DIRPORT, S1DIRPIN);
			StepperPosition[stepper - 1]++;
		}
		else
		{
			sbi(S1DIRPORT, S1DIRPIN);
			StepperPosition[stepper - 1]--;
		}
		//Strobe am Steppin l�st den Schritt aus.
		sbi(S1STEPPORT, S1STEPPIN);
		cbi(S1STEPPORT, S1STEPPIN);
		//Erfolgreich ausgef�hrt, eins wird zur�ckgegeben.
		return 1;
		break;
		
		
		//F�r die restlichen Stepper wird genauso verfahren.
		case 2:
		if (direction == runter)
		{
			cbi(S2DIRPORT, S2DIRPIN);
			StepperPosition[stepper - 1]++;
		}
		else
		{
			sbi(S2DIRPORT, S2DIRPIN);
			StepperPosition[stepper - 1]--;
		}
		sbi(S2STEPPORT, S2STEPPIN);
		cbi(S2STEPPORT, S2STEPPIN);
		return 1;
		break;
		
		
		
		case 3:
		if (direction == runter)
		{
			cbi(S3DIRPORT, S3DIRPIN);
			StepperPosition[stepper - 1]++;
		}
		else
		{
			sbi(S3DIRPORT, S3DIRPIN);
			StepperPosition[stepper - 1]--;
		}
		sbi(S3STEPPORT, S3STEPPIN);
		cbi(S3STEPPORT, S3STEPPIN);
		return 1;
		break;
		
		//Im Fehlerfalle (nicht vorhandener Stepper wird aufgerufen) wird eine Null zur�ckgegeben.
		default:
		return 0;
		break;
	}
}


//Hiermit k�nnen die Schaltausg�nge geschaltet werden.
//�bergeben wird: Nummer des Ausgangs (1-5) und gew�nschter Zustand (0/1)
int Schalt(int ausgang, int anaus)
{
	switch (ausgang)
	{
		case 1:
		if (anaus == 1)
		sbi (SCHALT1PORT, SCHALT1PIN);
		if (anaus == 0)
		cbi (SCHALT1PORT, SCHALT1PIN);
		break;
		
		case 2:
		if (anaus == 1)
		sbi (SCHALT2PORT, SCHALT2PIN);
		if (anaus == 0)
		cbi (SCHALT2PORT, SCHALT2PIN);
		break;
		
		case 3:
		if (anaus == 1)
		sbi (SCHALT3PORT, SCHALT3PIN);
		if (anaus == 0)
		cbi (SCHALT3PORT, SCHALT3PIN);
		break;
		
		case 4:
		if (anaus == 1)
		sbi (SCHALT4PORT, SCHALT4PIN);
		if (anaus == 0)
		cbi (SCHALT4PORT, SCHALT4PIN);
		break;
		
		case 5:
		if (anaus == 1)
		sbi (SCHALT5PORT, SCHALT5PIN);
		if (anaus == 0)
		cbi (SCHALT5PORT, SCHALT5PIN);
		break;
		
		
	}
	return 1;
}


//Hiermit kann der Status der Endschalter �berpr�ft werden.
//Vereinfacht den Workaround f�r den Fehler der Platine, dass der dritte Endschalter nur via ADC abgefragt werden kann, da dies dann nicht mehr seperat ber�cksichtigt werden muss.
//Liefert 1 wenn gedr�ckt bzw. 0 wenn nicht gedr�ckt zur�ck.
int Endschalter(int schalter)
{
	if (schalter == 1)
	{
		if (S1ENDINPREG & (1<<S1ENDPIN))
		return 0;
		else
		return 1;
	}
	
	if (schalter == 2)
	{
		if (S2ENDINPREG & (1<<S2ENDPIN))
		return 0;
		else
		return 1;
	}
	
	//Workaround f�r den Endschalter 3
	if (schalter == 3)
	{
		if (ADC7Read() > 220)
		return 0;
		else
		return 1;
	}
	
	return 0;
}


//Startet und konfiguriert den ADC f�r ADC7 Pin.
int ADCKonfiguration(void)
{
	//ADC Multiplexer auf ADC7 pin einstellen
	sbi(ADMUX, MUX2);
	sbi(ADMUX, MUX1);
	sbi(ADMUX, MUX0);
	
	//ADLAR im ADMUX Register auf 1 setzen. Dadurch wird das 10bittige Ergebnis linksb�ndig in den Registern ADCH und ADCL gespeichert.
	//Vorteil dieser Methode: Bei reduzierter Genauigkeit reicht es, ADCH auszuwerten, da in ADCL lediglich die niederwertigsten Bits gespeichert sind.
	//F�r das Abfragen des Endschalters v�llig ausreichend.
	sbi(ADMUX, ADLAR);
	
	//Prescaler auf 128
	sbi(ADCSRA, ADPS2);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
	
	//Vergleichsspannung auf AVCC. Spule nicht vergessen!
	sbi(ADMUX, REFS0);
	
	//ADC einschalten
	sbi(ADCSRA, ADEN);
	
	return 1;
}


//Lie�t den ADC7 Pin aus und gibt die 8 hochwertigsten Bits zur�ck.
int ADC7Read(void)
{
	//ADC Conversion starten:
	sbi(ADCSRA, ADSC);
	
	//Warten bis ADWandlung fertig.
	while (ADCSRA & (1<<ADSC));
	
	//Eingelesenen Wert zur�ckgeben.
	return ADCH;
}


//F�hrt eine Referenzfahrt aus, und setzt an den Endschaltern alle IstPositionen der Stepper auf Null.
//Liefert eine eins zur�ck, wenn erfolgreich referenziert wurde.
int Referenzfahrt(void)
{
	uint16_t TimeoutCounter = 0;
	
	//1.: Alle Arme gleichzeitig schnell nach oben fahren lassen, bis alle am Endschalter anschlagen
	//2.: Alle Arme mehrere Steps (genaue Anzahl als rueckzug definiert!) zur�ckfahren
	//3.: jeden Arm einzeln LANGSAM auf die Endschalter fahren lassen, dann die Variable StepperPosition[x] auf NULL setzen.
	
	//Schritt 1:
	while (!(Endschalter(1) && Endschalter(2) && Endschalter(3)))
	{
		//Wenn das Timeout noch nicht �berschritten ist:
		if (TimeoutCounter < RefMaxSteps)
		{
			TimeoutCounter++;
			//Alle Arme werden um eins weiter hoch gefahren, solange der Endschalter nicht bet�tigt ist.
			for (int i = 1; i < 4; i++)
			{
				if (Endschalter(i) == 0)
				Stepper(i, 1);
			}
			_delay_us(RefGeschwSchnell);
		}
		else
		{
			//Wenn Timeout �berschritten ist, Fehler ausgeben!
			return 0;
		}
	}
	//Zur Sicherheit nochmal warten
	_delay_ms(200);
	
	
	//Schritt 2:
	for (int i = 0; i < Rueckzug; i++)
	{
		Stepper(1, 0);
		Stepper(2, 0);
		Stepper(3, 0);
		_delay_us(RefGeschwRueckzug);
	}
	
	
	//Schritt 3: Wird f�r jeden Arm einzeln ausgef�hrt, deswegen dreimal widerholen!
	for (uint8_t i = 1; i < 4; i++)
	{
		while (!Endschalter(i))
		{
			Stepper(i, 1);
			_delay_us(RefGeschwLangsam);
		}
		
		StepperPosition[i-1] = 0;

	}
	
	
// 	Kurze UART Ausgabe mit Debug Informationen, dass die Referenzfahrt ausgef�hrt wurde!
// 		uart_puts_P("Referenzfahrt ausgef�hrt!\n");
// 		uart_puts_P("Stepper Positionen:\n");
	
// 	Zum Abschluss wird die Stepper Position aller Motoren auf Null gesetzt.
// 		for(uint8_t i = 0; i < 3; i++)
// 		{
// 			itoa(StepperPosition[i], debugbuffer, 10);
// 			uart_puts(debugbuffer);
// 			uart_puts_P(", ");
// 		}
// 		uart_puts_P("\n");


	return 1;
}