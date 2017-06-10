

#include "MotorBoard.h"   //loads a custom library to set up drive motors
#include "DiggerArm.h"     //loads a custom library to provide easy arm control command

#include <Wire.h>          //loads external arduino library to set up I2C communication protocol. Used to talk to Pixy camera
#include <PixyUART.h>       //loads external arduino library to set up CM Pixy Camera Sensor
#include "myPID.h"
#include <Servo.h>         //loads a servo library, also included inside DiggerArm, here as a reminder. HAS A BUGGED read() method as of 8/11/2014


#include "driveMethods.h"  //contains locomotion drive functions
#include "RobotSelector.h"  //contains locomotion drive functions
#include "visionMethods.h"
#include "Voltmeter.h"
#include "CurrentSensor.h"

#define CURRENT_SAMPLE_SIZE 100 //sets a number of samples to be used for reading current averages
#define VOLTAGE_SAMPLE_SIZE 100

#include "hardSerLCD.h"
hardSerLCD lcd;

CurrentSensor Current;
Voltmeter Voltage;

// --- Actuator setup stuff
MotorBoard Drive; //sets up motor drive, calling class MotorBoard to create an object "Drive"
DiggerArm Arm;   //sets up arm control, calling class DiggerArm  to create an object "Arm"

// --- camera sensor setup
/* vision stuff */
// PixyI2C pixy;       //sets up pixy camera sensors, calling class PixyI2c to create an object "pixy"
PixyUART pixy;       //sets up pixy camera sensors, calling class PixyUART to create an object "pixy"
uint16_t blocks;    //store number of blocks detected when frames are sampled
//RIGHT NOW, THERE ARE ONLY 3 VISION ITEMS
bool Object[5]; //array to hold boolean variables to tell whether or not the object of interest has been detected
uint16_t x1, xc, x7, xt, xCharging; //declare storage variables  x1=pheromone trail, xc= charging pheromone, x7 and xt are cotton stuff
uint16_t Area1, Areac, Area7, Areat, AreaCharging; //declare storage variables;

const int relayPin = 7;

double Setpoint, Input, Output; //define vars
double KP=Kp;
double KI=KI;
double KD=KD;
myPID PD(&Input, &Output, &Setpoint, KP,KI,KD); //PID control
char lastDriveState=0; //used to keep track of Forward/Backward/Right/Left/Stop commands
bool goingIn=true;

void printFresh(String lcddata){
    lcd.clear();
    lcd.setBrightness(30);
    lcd.print(lcddata);
}


void setup(){
	Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm clas
	Serial.println("setup() complete");
	Serial.println(enable1);
	Serial.println(phase1);
	Serial.println(enable2);
	Serial.println(phase2);
	Arm.Attach(); //hook up servos to pwm pins
  Arm.PitchGo(HIGH_ROW_ANGLE);
	delay(1000);
	pixy.init();        //Starts I2C communication with a camera
	delay(1000);
	Serial.println("Setting up PixyCam...done");
  Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm class
  WDT_Restart(WDT);

  // initiateSwitches();
  delay(500); //ensure a power cycle after a watchdog reset //used to be 3s
  WDT_Restart(WDT);
  
  lcd.begin(&Serial2, 9600);

    printFresh("hey                     ,");
    delay(500);
    printFresh("I'm about to break");
    delay(500);
    printFresh("I work!");
    delay(1000);
	
	pinMode(relayPin, OUTPUT);
	digitalWrite(relayPin, HIGH);
	
		/* set up PD or PID control */
	PD.SetMode(AUTOMATIC);
	PD.SetSampleTime(PD_SAMPLE_TIME); //sets sample time. default is 100ms
	PD.SetOutputLimits(-PV_adjmax,PV_adjmax); //clamp limits of PD controller feedback
	Setpoint = 160; //x coordinate of the center of the camera, 160=320/2
 
  printFresh("Setting up power sensing...");
  delay(500);
  Current.setPin(currentSensorPin);
  Voltage.setPin(voltage_pin);
  printFresh("Setting up power sensing...done");
  delay(500);
	
}

void loop(){
  // FollowLane();
//  TestCamera();
while(1){
  TestPower();
}


//		Arm.PitchGo(LOW_ROW_ANGLE);
//	  Arm.GripperGo(OPEN_POS);
//	  delay(1500);

    Forward(BASE_SPEED);
    delay(7000);
    Stop();
    
		Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
//    delay(7000);
//    Stop();

//    Arm.GripperGo(CLOSED_POS);
//    Arm.PitchGo(MID_ROW_ANGLE);
    delay(1000);
    Backward(BASE_SPEED); // Drive backward for the duration of the heading-check statement
    delay(7000);
    Stop();
    delay(1000);
//    Right(BASE_SPEED);
//    delay(2500);
//    Stop();
//    Arm.PitchGo(LOW_ROW_ANGLE);
//    Arm.GripperGo(OPEN_POS);
//    delay(1500);
//    Arm.PitchGo(HIGH_ROW_ANGLE);
//    Left(BASE_SPEED);
//    delay(4000);
//    Stop();
//    delay(1000);
   
//	//	Backward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
//		Arm.PitchGo(HIGH_ROW_ANGLE);
//		Arm.GripperGo(CLOSED_POS);
//		delay(5000);
//		// while(1){
//			// Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
//		// }
//		Forward(BASE_SPEED); // Drive forward for the duration of the heading-check statement
		
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



void TestCamera(){
	Serial.println("In TestCamera()...");
	uint16_t signature; 
	while(1){
		GetDetectedSigs(); //poll camera
		Serial.print("Pink: "); Serial.println(Area1);

		Serial.print("Green: "); Serial.println(Areat);
		Serial.print("COTTON: "); Serial.println(COTTON);
		delay(1000);

	}
}


void TestPower(){
  WDT_Restart(WDT);
  float V=Voltage.Read();
  float C=Current.Read();
  float P=V*C;
  printFresh("C =" + String(C) + "; V =" + String(V));
  delay(1500);
  printFresh("P =" + String(P));
  delay(1500);
//  Serial.print(F("Current \t"));
//  Serial.print(C);
//  Serial.print(F('\t'));
//  Serial.print(F("Voltage \t"));
//  Serial.println(V);
}
