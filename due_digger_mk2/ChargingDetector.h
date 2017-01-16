/* voltage divider circuit with 3.9k resistors  */
#include "Arduino.h"
#include "RobotSelector.h"

extern bool goingCharging;
extern void ChargingMode();

void initiateChargingDetector(){
	pinMode(ChargingDetectorPin, INPUT); //set pin as input
	// digitalWrite(ChargingDetectorPin,HIGH); //turn on pullup resistor
}

bool isChargerDetected(){
	//returns 1 if voltage is detected on the pin, 0 if not
	// bool val=digitalRead(ChargingDetectorPin);
	return (digitalRead(ChargingDetectorPin));
}
