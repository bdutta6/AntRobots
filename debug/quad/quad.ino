

#include "MotorBoard.h"   //loads a custom library to set up drive motors
#include "driveMethods.h"  //contains locomotion drive functions
#include "RobotSelector.h"  //contains locomotion drive functions
#include "DiggerArm.h"     //loads a custom library to provide easy arm control command
#include <Servo.h>         //loads a servo library, also included inside DiggerArm, here as a reminder. HAS A BUGGED read() method as of 8/11/2014


MotorBoard Drive; //sets up motor drive, calling class MotorBoard to create an object "Drive"
DiggerArm Arm;   //sets up arm control, calling class DiggerArm  to create an object "Arm"


void setup(){
	Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm clas
	Serial.println("setup() complete");
	Serial.println(enable1);
	Serial.println(phase1);
	Serial.println(enable2);
	Serial.println(phase2);
	Arm.Attach(); //hook up servos to pwm pins

}


void loop(){
		Arm.PitchGo(HIGH_ROW_ANGLE);
		Arm.GripperGo(CLOSED_POS);
		delay(5000);
		// while(1){
			// Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
		// }
		Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement



		
		// while(1){
			// WDT_Restart(WDT);
			// Serial.println(F("Opening gripper"));
			// Arm.GripperGo(OPEN_POS);
			// delay(1000);
			// Serial.println(F("Closing gripper"));
			// Arm.GripperGo(MID_POS);
			// delay(1000);
			// Arm.GripperGo(CLOSED_POS);
			// delay(1000);
			// Serial.println(F("Raising arm"));
			// WDT_Restart(WDT);
			// Arm.PitchGo(HIGH_ROW_ANGLE);
			// delay(1500);
			// Serial.println(F("Lowering arm"));
			// Arm.PitchGo(LOW_ROW_ANGLE);
			// delay(1500); //Arm.PitchGo(100); //JSP Delete
		// }
}
