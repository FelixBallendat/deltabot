/* wichtige Anmerkung vorab:
   "UART_RX_BUFFER_SIZE" in uart.h von P.Fleury musste von 32 auf 64 erhoeht werden
   um ausreichend lange strings empfangen zu koennen!
*/


#include "DeltaBot_SerielleKommunikation.h"

/***************************************************************
  Folgende Variablen erhalten die Parameter aus G-Code.
  Global definiert und somit durch andere Funktionen ebenfalls
  aufrufbar.
***************************************************************/  

double Gwert = 0;

uint8_t Koordinaten_veraendert = 0;	// wird 1 gesetzt, sobald neue anzufahrende Koordinaten vorliegen
uint8_t M_Befehl_ausgefuehrt = 0;	// wird 1 gesetzt, sobald sobald ein M-Befehl ausgefuehrt wurde


/***************************************************************************
  Alle Initialisierungsfunktionen für die Serielle Schnittstelle abarbeiten
  um im Hauptprogramm nur eine Funktion einbinden zu müssen
****************************************************************************/

 int initSeriell(void)
 {
	 /*
     *  Initialize UART library, pass baudrate and AVR cpu clock
     *  with the macro 
     *  UART_BAUD_SELECT() (normal speed mode )
     *  or 
     *  UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
     */
    uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	
	return 1;
 }
 

/*********************************************************************
  In dieses Array-String wird die empfangene G-Code-Zeile geschrieben
  Die Groeße muss ggf. angepasst werden !
**********************************************************************/
char Line[63];	// String mit maximal 63 zeichen


/**************************************************
  Funktion zum Einlesen von strings als char-Array
  
  Diese Funktion beruht auf Teilen der 
  - Mikrocontroller.net-Wiki
  - UART Library von P.Fleury
  - eigenen Erweiterungen
***************************************************/

void uart_gets( char* Buffer, uint8_t MaxLen )
{
	//fuer NextChar darf nicht uint8_t verwendet werden!
	//uart_getc liefert naemlich ein low-Byte und high-Byte (=16 Bit gesamt)
	//im high-Byte stehen evtl. entdeckte UART-Fehler, wie z.B. NO_DATA
	//die nachfolgenden Abfragen wuerden dann nicht mehr funktionieren
	
	unsigned int NextChar;
	uint8_t StringLen = 0;
	
	NextChar = uart_getc();         // Warte auf und empfange das nächste Zeichen

	// Sammle solange Zeichen, bis:
	// * entweder das String Ende Zeichen ('\n') kam
	// * oder das aufnehmende Array voll ist
	while( NextChar != GCode_SatzEndeZeichen && StringLen < MaxLen - 1 ) 
	{	
		if ( NextChar & UART_NO_DATA )
        {
            /* 
             * no data available from UART
			 * mache nichts
             */
			
			//nur zum Debuggen:
			//uart_puts_P("No Data on UART\n");
        }
		
		else
        {
            /*
             * new data available from UART
             * check for Frame or Overrun error
             */
            if ( NextChar & UART_FRAME_ERROR )
            {
                /* Framing Error detected, i.e no stop bit detected */
                uart_puts_P("UART Frame Error: ");
            }
            if ( NextChar & UART_OVERRUN_ERROR )
            {
                /* 
                 * Overrun, a character already present in the UART UDR register was 
                 * not read by the interrupt handler before the next character arrived,
                 * one or more received characters have been dropped
                 */
                uart_puts_P("UART Overrun Error: ");
            }
            if ( NextChar & UART_BUFFER_OVERFLOW )
            {
                /* 
                 * We are not reading the receive buffer fast enough,
                 * one or more received character have been dropped 
                 */
                uart_puts_P("Buffer overflow error: ");
            }

			*Buffer++ = NextChar;
			StringLen++;
			
			if (StringLen == MaxLen -1 )
			{uart_puts_P("empfangener String ist zu lang \n");}
			
		}
		NextChar = uart_getc();
	}
	
	// Noch ein '\0' anhängen um einen Standard
	// C-String daraus zu machen
	*Buffer = '\0';
	
}


/************************************************************
  Funktion zum Empfangen und Zerteilen der G-Code-Zeile
  - Maschinenbefehle werden direkt umgesetzt
  - Empfangene Koordinaten und Parameter werden in Variablen
    geschrieben
  - Beim Senden von Koordinaten an die Platine ist als
	Kommazeichen ein Punkt . zu verwenden	
*************************************************************/

int LeseGcode(double sollPosition[])
{	
	uart_gets( Line, sizeof( Line ) );
		
	// Debug only:
	// uart_puts_P("Das erstes Zeichen: ");
	// uart_putc(Line[0]); uart_puts_P("\n");
		
	const char trennzeichen[] = " "; //bei diesem Trennzeichen soll der string in Einzelteile geteilt werden (Standard: Leerzeichen)
	char *ptr;
		
	ptr = strtok(Line, trennzeichen); //Funktion strtok trennt Strings bei gegebenem Trennzeichen
		
	while(ptr != NULL)
	{
		//Debug only:
		//uart_puts_P("Dies ist der naechste Teil bis zum Trennzeichen (Leerzeichen): ");
		//uart_puts(ptr); uart_puts_P("\n");
			
		switch (*ptr)
		{
			case 'G' :
				ptr++; // Den Pointer eine Stelle weiter zum Zahlenwert
				Gwert = atof(ptr); //atof = "asci-to-float"
				//uart_puts_P("Der Teil ist ein G-Teil, mit dem Wert: ");
				//uart_puts(ptr); uart_puts_P("\n");
			break;

			case 'X' :
				ptr++; // Den Pointer eine Stelle weiter zum Zahlenwert
				sollPosition[0] = atof(ptr);
				//uart_puts_P("Der Teil ist ein X-Teil, mit dem Wert: ");
				//uart_puts(ptr); uart_puts_P("\n");
				Koordinaten_veraendert = 1;
			break;
				
			case 'Y' :
				ptr++; // Den Pointer eine Stelle weiter zum Zahlenwert
				sollPosition[1] = atof(ptr);
				//uart_puts_P("Der Teil ist ein Y-Teil, mit dem Wert: ");
				//uart_puts(ptr); uart_puts_P("\n");
				Koordinaten_veraendert = 1;
			break;
				
			case 'Z' :
				ptr++; // Den Pointer eine Stelle weiter zum Zahlenwert
				sollPosition[2] = atof(ptr);
				//uart_puts_P("Der Teil ist ein Z-Teil, mit dem Wert: ");
				//uart_puts(ptr); uart_puts_P("\n");
				Koordinaten_veraendert = 1;
			break;
				
			case 'F' :
				ptr++; // Den Pointer eine Stelle weiter zum Zahlenwert
				sollPosition[3] = atof(ptr);
				//uart_puts_P("Der Teil ist ein F-Teil, mit dem Wert: ");
				//uart_puts(ptr); uart_puts_P("\n");
			break;
			
			case 'M' :
				
				// "strmcp" vergleicht zwei strings ("stringcompare")
				// der String mit dem verglichen wird (z.B. Schalt1an) ist als define hinterlegt
				
				if (strcmp(ptr, Schalt1an)==0) 
				{
					Schalt(1,1);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt1aus)==0)
				{
					Schalt(1,0);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt2an)==0)
				{
					Schalt(2,1);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt2aus)==0)
				{
					Schalt(2,0);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt3an)==0)
				{
					Schalt(3,1);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt3aus)==0)
				{
					Schalt(3,0);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt4an)==0)
				{
					Schalt(4,1);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt4aus)==0)
				{
					Schalt(4,0);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt5an)==0)
				{
					Schalt(5,1);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Schalt5aus)==0)
				{
					Schalt(5,0);
					M_Befehl_ausgefuehrt = 1;
				}
				
				else if(strcmp(ptr, Referenzieren)==0)
				{
					Referenzfahrt();
					M_Befehl_ausgefuehrt = 1;
				}
				
				else 
				{
					uart_puts_P("Dieser M-Befehl ist nicht bekannt: ");
					uart_puts(Line); uart_puts_P("\n");
				}
				
				break;
							
			default :
				uart_puts_P("Dieser Befehl ist nicht bekannt: ");
				uart_puts(ptr); uart_puts_P("\n");
			break;
				
		}			
		
		// naechsten String-Abschnitt erstellen
		ptr = strtok(NULL, trennzeichen);
	}
		
	// Der nachfolgende Code dient in erster Linie zum Debuggen.
	// Hiermit ist es möglich den Inhalt der globablen Double-Variablen anzuzeigen.
	// Da die Umwandlung von Doubles zu Strings (um es über UART senden zu können)
	// nicht ganz trivial ist, möchte ich diesen Teil im Code behalten, um somit
	// bei Problemen die korrekte Übergabe der Doubles schnell ueberpruefen zu koennen.
	//
	/*	Zur Funktion dtostrf(Param1, Param2, Param3, Param4):
		Parameter 1: Eine Variable vom Typ float oder double die in einen string gewandelt werden soll.
		Parameter 2: Gesamte Länge des umgewandelten Strings incl. Vor-, Nachkommastellen, Dezimaltrennzeichen
						Falls umgewandelter String kürzer als Parameter 2, wird links mit Leerzeichen aufgefüllt
						Falls notwendige Stringläne größer als Parameter 2, wird String trotzdem in voller Länge erzeugt
		Parameter 3: Anzahl der Zeichen nach dem Komma
						enthält die float/double mehr Nachkommastellen wird auf die angegebenen Nachkommastellen gerundet
		Parameter 4: char-Array in welches das Ergebnis der Umwandlung geschrieben wird
		
	*/
		
	/*	
	char ascibuffer[9];
		
	dtostrf(Gwert, 8, 4, ascibuffer);
	uart_puts_P("Inhalt in G: "); uart_puts(ascibuffer); uart_puts("\n");
		
	dtostrf(Xwert, 8, 4, ascibuffer);
	uart_puts_P("Inhalt in X: "); uart_puts(ascibuffer); uart_puts("\n");
		
	dtostrf(Ywert, 8, 4, ascibuffer);
	uart_puts_P("Inhalt in Y: "); uart_puts(ascibuffer); uart_puts("\n");
		
	dtostrf(Zwert, 8, 4, ascibuffer);
	uart_puts_P("Inhalt in Z: "); uart_puts(ascibuffer); uart_puts("\n");
		
	dtostrf(Fwert, 8, 4, ascibuffer);
	uart_puts_P("Inhalt in F: "); uart_puts(ascibuffer); uart_puts("\n");
	*/
			
	return 1;
}