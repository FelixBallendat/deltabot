#include "DeltaBot_Motor.h"
#include "DeltaBot_Berechnungen.h"

//Nur zum debuggen, Puffer für die serielle Übertragung.
//Wird aus der DeltaBot.c eingebunden.
extern char debugbuffer[16];

// Array aus winkel in 0,01 argument schritten für tangens (0 bis 0,7 bzw 0 bis 35 Grad)
double tanwerte[71]= {
	0,
	0.5729387,
	1.14576284,
	1.718358,
	2.29061004,
	2.86240523,
	3.43363036,
	4.00417294,
	4.57392126,
	5.14276456,
	5.71059314,
	6.27729849,
	6.84277341,
	7.40691213,
	7.96961039,
	8.53076561,
	9.09027692,
	9.64804532,
	10.2039737,
	10.7579671,
	11.3099325,
	11.8597791,
	12.4074185,
	12.9527645,
	13.4957333,
	14.0362435,
	14.5742162,
	15.1095751,
	15.6422465,
	16.172159,
	16.6992442,
	17.2234362,
	17.7446716,
	18.2628899,
	18.7780332,
	19.2900462,
	19.7988764,
	20.3044737,
	20.806791,
	21.3057836,
	21.8014095,
	22.2936292,
	22.7824057,
	23.2677048,
	23.7494945,
	24.2277453,
	24.7024302,
	25.1735245,
	25.6410058,
	26.104854,
	26.5650512,
	27.0215816,
	27.4744316,
	27.9235897,
	28.3690463,
	28.8107937,
	29.2488263,
	29.6831402,
	30.1137332,
	30.5406048,
	30.9637565,
	31.3831911,
	31.7989128,
	32.2109277,
	32.6192431,
	33.0238676,
	33.4248112,
	33.8220852,
	34.2157021,
	34.6056756,
	34.9920202
};

// Array aus winkel in 0,01 argument schritten für cosinus (0 bis 0,9 bzw 90 bis25,8 Grad)
double coswerte0bis09 [91]=
{
	90,
	89.4270327,
	88.854008,
	88.2808687,
	87.7075572,
	87.134016,
	86.5601872,
	85.9860128,
	85.4114343,
	84.8363929,
	84.2608295,
	83.6846844,
	83.1078974,
	82.5304077,
	81.9521538,
	81.3730734,
	80.7931038,
	80.2121809,
	79.6302402,
	79.0472158,
	78.463041,
	77.8776478,
	77.290967,
	76.7029283,
	76.1134596,
	75.5224878,
	74.9299379,
	74.3357331,
	73.7397953,
	73.142044,
	72.5423969,
	71.9407695,
	71.3370751,
	70.7312245,
	70.1231259,
	69.5126849,
	68.899804,
	68.2843827,
	67.6663173,
	67.0455006,
	66.4218215,
	65.7951652,
	65.1654125,
	64.5324399,
	63.8961189,
	63.256316,
	62.6128925,
	61.9657035,
	61.314598,
	60.6594184,
	60,
	59.3361703,
	58.6677485,
	57.9945452,
	57.3163612,
	56.632987,
	55.9442023,
	55.2497742,
	54.5494574,
	53.8429918,
	53.1301024,
	52.410497,
	51.6838655,
	50.9498775,
	50.2081805,
	49.4583981,
	48.7001272,
	47.9329352,
	47.156357,
	46.3698911,
	45.572996,
	44.7650847,
	43.9455196,
	43.1136059,
	42.2685844,
	41.4096221,
	40.5358021,
	39.6461111,
	38.7394246,
	37.8144885,
	36.8698976,
	35.9040686,
	34.9152062,
	33.901262,
	32.8598804,
	31.7883306,
	30.6834171,
	29.5413605,
	28.3576366,
	27.1267531,
	25.8419328,
};

// Array aus winkel in 0,002 argument schritten für cosinus (0,9 bis 1 bzw 25,8 bis 0 Grad)
double coswerte09bis1 [91]=
{
	25.8419328,
	25.5777835,
	25.3110652,
	25.0416952,
	24.7695868,
	24.4946485,
	24.2167834,
	23.9358894,
	23.6518582,
	23.364575,
	23.0739181,
	22.7797579,
	22.4819566,
	22.1803672,
	21.8748327,
	21.565185,
	21.251244,
	20.9328161,
	20.6096929,
	20.2816498,
	19.9484436,
	19.6098107,
	19.2654646,
	18.9150928,
	18.5583537,
	18.1948723,
	17.8242358,
	17.4459877,
	17.0596213,
	16.6645714,
	16.2602047,
	15.8458075,
	15.4205708,
	14.9835711,
	14.5337469,
	14.0698677,
	13.590494,
	13.0939233,
	12.5781187,
	12.0406077,
	11.478341,
	10.8874829,
	10.2630959,
	9.59863838,
	8.88512427,
	8.10961446,
	7.25224687,
	6.27958064,
	5.12640008,
	3.62430749,
	0,
};



		//Hier wird die Soll Position von X Y Z in Winkel der Hebel umgerechnet, die Winkel werden von der vertikale nach oben gemessen
		//Übergeben werden die soll XYZ koordinaten in Bezug auf den Kinematischen Nullpunkt
int InverseKinematik(double XSoll, double YSoll, double  ZSoll, int32_t resultSchritte[])
{
		/*uart_puts_P("\n\n\n");
					uart_puts_P("XSoll: ");
					dtostrf(XSoll, 8, 4, debugbuffer);
					uart_puts(debugbuffer);
					uart_puts_P("   ");
					
					uart_puts_P("YSoll: ");
					dtostrf(YSoll, 8, 4, debugbuffer);
					uart_puts(debugbuffer);
					uart_puts_P("   ");
					
					uart_puts_P("ZSoll: ");
					dtostrf(ZSoll, 8, 4, debugbuffer);
					uart_puts(debugbuffer);
					uart_puts_P("   ");*/
	

	for (int i=0; i<3; i++)
	{
		double PSi = sqrt(P*P-YSoll*YSoll);										// PSi ist die in die i Ebene projezierte Länge der Pendelstüze, mit i 0,120,240 Grad
		double a = sqrt((XSoll+SWp-SWb)*(XSoll+SWp-SWb)+ZSoll*ZSoll);			// a ist der Abstand in x richtung von der Hebelachse zum Pendelstüzengelen der Platform
		
		//double beta= arctan((XSoll+SWp-SWb)/-ZSoll	);							// beta ist der winkel an der Hebelachse von der vertikalen zur Px
		//double alfa= arccos((a*a+H*H-PSi*PSi)/(2*a*H));                           // alfa ist der winkel zwischen a und Px
		
		double beta= atan((XSoll+SWp-SWb)/-ZSoll) * 180.0 / M_PI;				// Winkelberechnung mit der math.h funktion anstelle von lookup tabellen
		double alfa= acos((a*a+H*H-PSi*PSi)/(2*a*H)) * 180.0 / M_PI;			//

		resultSchritte[i] = ((WinkelZumEndschlater - (alfa + beta)) * (SchritteProUmdrehung / 360.0) );
		/*
				//debug
				

				
				
// 				uart_puts_P("arctan Argument: ");
// 				dtostrf(((XSoll+SWp-SWb)/-ZSoll	), 8, 4, debugbuffer);
// 				uart_puts(debugbuffer);
// 				uart_puts_P("   ");
// 				
// 				uart_puts_P("arccos Argument: ");
// 				dtostrf(((a*a+H*H-PSi*PSi)/(2*a*H)), 8, 4, debugbuffer);
// 				uart_puts(debugbuffer);
// 				uart_puts_P("\n");
// 				
// 				uart_puts_P("PSI: ");
// 				dtostrf(PSi, 8, 4, debugbuffer);
// 				uart_puts(debugbuffer);
// 				uart_puts_P("\n");
// 				
// 				uart_puts_P("a: ");
// 				dtostrf(a, 8, 4, debugbuffer);
// 				uart_puts(debugbuffer);
// 				uart_puts_P("\n");
// 				
// 				uart_puts_P("alpha: ");
// 				dtostrf(alfa, 8, 4, debugbuffer);
// 				uart_puts(debugbuffer);
// 				uart_puts_P("\n");
// 				
// 				uart_puts_P("beta: ");
// 				dtostrf(beta, 8, 4, debugbuffer);
// 				uart_puts(debugbuffer);
// 				uart_puts_P("\n");
// 				
// 				uart_puts_P("alpha plus beta: ");
// 				dtostrf((alfa + beta), 8, 4, debugbuffer);
// 				uart_puts(debugbuffer);
// 				uart_puts_P("\n");
// 				
// 				uart_puts_P("Absolut Schritte: ");
// 				dtostrf(ABSschritte[i], 8, 4, debugbuffer);
// 				uart_puts(debugbuffer);
// 				uart_puts_P("\n");
				
				//debug
				*/
		
		double XRot=(-0.5* XSoll) - ((0.86602540) * YSoll);						// drehung der xsoll koordinate um 120grad. alternativ kann für sqrt(3)/2 auch 0,86602540398443864676372317075294 eingesetzt werden
		double YRot=0.86602540*XSoll-0.5*YSoll;							// drehung der ysoll koordinate um 120grad. alternativ kann für sqrt(3)/2 auch 0,86602540398443864676372317075294 eingesetzt werden
	
		XSoll=XRot;											//zuweisung der verdrehten koordinaten zu xsoll für die Berechnung des nächsten winkels
		YSoll=YRot;											//zuweisung der verdrehten koordinaten zu ysoll für die Berechnung des nächsten winkels
	}
	
	//debug
	//uart_puts_P("\n");
	//debug
	
	return 1;							
}

		//aus einem Array wird hier der Tangens eines Arguments als interpolierter winkel zurückgegeben.
		//gilt für Argumente von -0,7 bis +0,7
double arctan(double argument){
	
	//debug
	//uart_puts_P("arctan aufgerufen!\nArgument: ");
	//dtostrf(argument, 16, 8, debugbuffer);
	//uart_puts(debugbuffer);
	//debug
	
	double vorzeichen=0;
	if (argument<0)											// dier Bereich prüft in welchem quadranten sich der Winkel befindet
	{														//
		argument=-1*argument;								//
		vorzeichen=1;										//
	}	
	
	int arraystelle=argument*100;
	double zkw = tanwerte[arraystelle];				//speichert den eins zkw (zu kleiner Winkel) aus der tabelle in eine variable
	double zgw = tanwerte[arraystelle+1];			//speichert den eins zgw (zu grossen Winkel) aus der tabelle in eine variable
	
	//debug
// 	uart_puts_P("\nzkw:");
// 	dtostrf(zkw, 8, 4, debugbuffer);
// 	uart_puts(debugbuffer);
// 	
// 	uart_puts_P("\nzgw:");
// 	dtostrf(zgw, 8, 4, debugbuffer);
// 	uart_puts(debugbuffer);
// 	uart_puts("\n");
// 	
// 	uart_puts("Vorzeichen: ");
// 	dtostrf(vorzeichen, 8, 4, debugbuffer);
// 	uart_puts(debugbuffer);
// 	uart_puts("\n");
	//debug
	
	double stelle=arraystelle;  // arrastelle in einen double umwandeln damit am damit weiter gerechne werden kann
	
	//uart_puts("Winkel vom Tangens: ");
	if (vorzeichen==0)
	{
//	dtostrf((zkw+((zgw-zkw)/0.01)*(argument-(stelle/100))), 8, 4, debugbuffer);
//	uart_puts(debugbuffer);	
		return (zkw+((zgw-zkw)/0.01)*(argument-(stelle/100)));			 // interpolierter winkel für positive argumente wird zurückgeben
	}
	else
	{
//		dtostrf((-1*(zkw+(((zgw-zkw)/0.01)*(argument-(stelle/100))))), 8, 4, debugbuffer);
//		uart_puts(debugbuffer);
		return (-1*(zkw+(((zgw-zkw)/0.01)*(argument-(stelle/100)))));		// interpolierter Winkel für negative argumente wird zurückgeben
	}
	
}

		//aus zwei Arraies wird hier der cosinus eines Arguments als interpolierter winkel zurückgegeben.
		//gilt für Argumente von -1 bis +1
double arccos(double argument){
	double vorzeichen=0;
	if (argument<0)											// diers Bereich prüft in welchem quadranten sich der Winkel befindet
	{														//
		argument=-1*argument;								//
		vorzeichen=1;										//
	}														//
	
	if (argument<0.9)
	{
		int arraystelle=argument*100;						//in diesem Bereich wird auf die cosinustabelle mit einem abstand von 0,01 zugegriffen
		double zgw = coswerte0bis09[arraystelle];			//speichert den eins zkw (zu kleiner Winkel) aus der tabelle in eine variable
		double zkw = coswerte0bis09[arraystelle+1];			//speichert den eins zgw (zu grossen Winkel) aus der tabelle in eine variable
		double stelle=arraystelle;
		if (vorzeichen==0)
		{
			return (zgw-((zgw-zkw)/0.01)*(argument-(stelle/100)));			 // interpolierter winkel für positive argumente wird zurückgeben
		}
		else
		{
			return (180-(zgw+((zgw-zkw)/0.01)*(argument-(stelle/100))));		// interpolierter Winkel für negative argumente wird zurückgeben
		}
	}
	else
	{
		int arraystelle=(argument-0.9)*500;					//in diesem Bereich wird auf die cosinustabelle mit einem abstand von 0,01 zugegriffen
		double zgw = coswerte09bis1[arraystelle];			//speichert den eins zkw (zu kleiner Winkel) aus der tabelle in eine variable
		double zkw = coswerte09bis1[arraystelle+1];			//speichert den eins zgw (zu grossen Winkel) aus der tabelle in eine variable
		
		double stelle=arraystelle;
		if (vorzeichen==0)
		{
			return (zgw+((zgw-zkw)/0.002)*(argument-(stelle/500)));			 // interpolierter winkel für positive argumente wird zurückgeben
		}
		else
		{
			return (180-((zgw+((zgw-zkw)/0.002)*(argument-(stelle/500)))));		// interpolierter Winkel für negative argumente wird zurückgeben
		}		
	}	
}











