#include "DeltaBot.h"

//alle variablen f�r die Berechnungen
#define H 100.0  // l�nge der 3d gedruckten Hebel in mm
#define P 255.0  // l�nge der Kohlefaser Pendelst�zen in mm
#define SWb 59.91  // halbe schl�sselweite der Basis, also der Abstand com Mittelpunkt zur Hebelachse
#define SWp 25.0  // halbe schl�sselweite der Platform, also der Abstand von der Werkzeugachse zur Ache der Pendeslst�ze


#define Mikrosteps 16.0   
#define Uebersetzung 4.0							// �bersetzung von motor zum hebel
#define SchritteProUmdrehung 200.0 * Uebersetzung * Mikrosteps
#define WinkelZumEndschlater 150.0								 //  150 ist dieser Winkel in grad
#define StepsZumEndschlater (WinkelZumEndschlater/360.0)*SchritteProUmdrehung		 // der winkel von der Vertikalen zum endschalter in steps



//Hier wird die Soll Position von X Y Z in Winkel der Hebel umgerechnet
//�bergeben werden die soll XYZ koordinaten in Bezug auf den Kinematischen Nullpunkt
int InverseKinematik(double XSoll, double YSoll, double ZSoll, int32_t resultSchritte[]);

double arctan(double argument);
double arccos(double argument);

