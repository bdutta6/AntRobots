/**
	This class enables an analog pin to read voltage of a power source
	Arduino Fio v2 or Arduino Due
	
	1/22/2015  removed pin storage, replaced with macro definition
*/

#ifndef Voltmeter_h
#define Voltmeter_h
#include "Arduino.h"
#include "RobotSelector.h"

class Voltmeter{
 public:
 // Voltmeter(int analogPin);
 Voltmeter();
 float Read();
 float GrabMin(int Samples=20);
 float GrabAvg(int Samples=20);
 private:
 const float Vcc=3.3; //ADC pin voltage
 // int _analogPin;
 int _Samples;
 float _SampleMin=0; //initiate 
};


#endif