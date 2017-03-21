

#include "MotorBoard.h"   //loads a custom library to set up drive motors
#include "driveMethods.h"  //contains locomotion drive functions
#include "RobotSelector.h"  //contains locomotion drive functions


MotorBoard Drive; //sets up motor drive, calling class MotorBoard to create an object "Drive"


void setup(){
	Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm clas
	Serial.println("setup() complete");
		Serial.println(enable1);
				Serial.println(phase1);
		Serial.println(enable2);

						Serial.println(phase2);


}


void loop(){
		Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement

}
