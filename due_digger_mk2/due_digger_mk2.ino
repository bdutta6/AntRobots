/**

Author : Vadim Linevich
email  : vadim.linevich@gmail.com

Modified by : Jungsoo Park
email : ryanryan0906@hotmail.com
Last Modified Date: 05/26/2016

Modified by : Ross Warkentin - testing_branch personal
email: ross.warkentin@gmail.com
Last Modified Date: 10/14/2016

Property of CRABLAB, Georgia Institute of Technology, School of Physics
[2013]-[2014]

This is the main program, written in embedded C++, to be compiled and uploaded
to Arduino family MCU's using ARDUINO IDE (works with v1.5.7)

11/13/2014: the code is specific to Arduino Due

//WATCHDOG RESET IS ENABLED
//read this forum http://forum.arduino.cc/index.php?topic=314647.0

*/

#define FIO_LINK 1

// ********** BEGIN {SET BEHAVIOR} **********
//--comment things out if unwanted 
//lorenz stuff
#define PROBABILITY_DIG 0 //0 for active, 1 for lorenz
#define RESTING_TIME 20000// number of seconds before rerolling probabilty in lorenz mode 
//useless run stuff turn back if it did not reach the face
#define ALLOW_USELESS_RUNS 1// 1 is allow
#define USELESS_RUN_THRESH 75000 //used to be 7500,90 seconds. double the amount of needed 100 seconds now


//dont worry about this stuff
#define ALLOW_CHARGING_ON_REST 1
#define ALLOW_POWER_SAVINGS 0
//VADIM. FIND A TIMER FROMA PREVIOUS CODE> THIS WAS BROKEN 
#define BACKWARDS_KICK_TIME 1000   //every so often the robot will drive back for this many seconds. needed for avoiding getting stuck 
//run trhesh: 40s too short, 120s too long
// **********  END   {SET BEHAVIOR} ---------

// 2 ants: P1=11, P2=89
// 3 ants: P1=4, P2=21, P3=75
// 4 ants: P1=1.80,  P2=8.56, P3=25.29, P4=64.35


// #define MANUAL_ON 1
// ********** BEGIN {LIBRARY IMPORT} **********
/* note: included libraries with "" must be in the same folder with .ino sketch, while external <> libraries must be in the arduino library folder (either documents>arduino or program files>arduino */
// #include "RobotSelector.h" //load appropriate robot specific definitions and constants 
#include "MotorBoard.h"   //loads a custom library to set up drive motors
//power relay, switches, drive methods, vision methods have def.h files
#include "DiggerArm.h"     //loads a custom library to provide easy arm control command
#include "CurrentSensor.h" //loads a custom library to read data from current sensors
//#include "AntComm.h"       //loads a custom library to set up a wireless communication (Xbee Series 2 Radio)
#include "GripperSensor.h" //loads a custom library to set up a reflective gripper sensor#include <Wire.h>
#include <Wire.h>          //loads external arduino library to set up I2C communication protocol. Used to talk to Pixy camera
// #include <PixyI2C.h>       //loads external arduino library to set up CM Pixy Camera Sensor
#include <PixyUART.h>       //loads external arduino library to set up CM Pixy Camera Sensor
#include <SPI.h>           //loads external arduino library. This library is not used but required by SFE_LSM9DS0 library
#include <SFE_LSM9DS0.h>   //loads an IMU library written by a sensor manufacturer and modified by me
#include "PowerRelay.h"    //loads a custom library to set up a relay to turn power on and off
#include "Voltmeter.h"     //loads a custom library to set up an analog pin to read voltage on a source battery
#include "myPID.h"         //loads an open source library to set up PID/PD control. 
#include <Servo.h>         //loads a servo library, also included inside DiggerArm, here as a reminder. HAS A BUGGED read() method as of 8/11/2014

//#include "OpticalEncoder.h"//loads a custom library to read encoders using interrupt routine 
//#include "Encoders.h"      //sets up ISR interrupt functions. Should merge optical encoders into here
//#include "Switches.h"   //methods containing ISR interrupt  functions used to keep track of which switches are pressed
#include "driveMethods.h"  //contains locomotion drive functions
#include "visionMethods.h" //contains functions setting up Pixy camera
//#include "LED.h" //loads a custom library that turns LED on and off, used for debugging purposes
#include "ChargingDetector.h" //load a file to set up charging dection 
//#include "IRsensor.h"  //load a custom library to set up IR distance sensors
//#include "DigitalProximity.h" //loads a custom library to set up a digital proximity sensor 
//#include "AntBuzzer.h" //loads a custom library to set up a speaker/buzzer for debugging purposes
#include "enableModes.h" //sets up mode changing 

#include "LinkArduinosI2c.h" //loads a header file with high level I2C communication commands to link main board and radio MCUs
#include "MasterSlaveProtocol.h" //loads custom protocol used to communicate between two boards
#include "WireReset.h" //enables arduino to self reset iself
#include "RandomGenerator.h" //generates random numbers to enable probabilistic behaviour.
#include "mpr121.h" //for capacitive sensor
#include "CapacitiveSensor.h"
#include "MAG3110.h"


// #include <wdt.h>
// **********  END   {LIBRARY IMPORT} ---------

// ********** BEGIN {GLOBAL VARIABLE DECLARATION} **********
// --- actuator setup
// MotorBoard Drive(enable1,phase1,enable2,phase2); //sets up motor drive, calling class MotorBoard to create an object "Drive"
MotorBoard Drive; //sets up motor drive, calling class MotorBoard to create an object "Drive"
DiggerArm Arm;   //sets up arm control, calling class DiggerArm  to create an object "Arm"
// --- current sensor setup
CurrentSensor Current; //sets up a current sensor. CATION: removed constructor input
#define CURRENT_SAMPLE_SIZE 50 //sets a number of samples to be used for reading current averages
// --- gripper sensor variables
//sensor threshold is defined inside GripperSensor.h 
//GripperSensor HeadSensor; 
//sets up proximity sensors used to detect granular media //JSP
GripperSensor LUSensor(LUGripperPin);
GripperSensor LDSensor(LDGripperPin);
GripperSensor RUSensor(RUGripperPin);
GripperSensor RDSensor(RDGripperPin);
MAG3110 FSensor;

CapacitiveSensor CapSensor(CapacitiveSensorPin);
// --- IMU stuff
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM); //sets up IMU (gyrsocope, magnetometer, accelerometer)
// --- camera sensor setup
/* vision stuff */
// PixyI2C pixy;       //sets up pixy camera sensors, calling class PixyI2c to create an object "pixy"
PixyUART pixy;       //sets up pixy camera sensors, calling class PixyUART to create an object "pixy"
uint16_t blocks;    //store number of blocks detected when frames are sampled
//RIGHT NOW, THERE ARE ONLY 3 VISION ITEMS
bool Object[5]; //array to hold boolean variables to tell whether or not the object of interest has been detected
uint16_t x1, xc, x7, xt, xCharging; //declare storage variables  x1=pheromone trail, xc= charging pheromone, x7 and xt are cotton stuff
uint16_t Area1, Areac, Area7, Areat, AreaCharging; //declare storage variables;


double Setpoint, Input, Output; //define vars
double KP=Kp;
double KI=KI;
double KD=KD;
myPID PD(&Input, &Output, &Setpoint,KP,KI,KD); //PID control
char lastDriveState=0; //used to keep track of Forward/Backward/Right/Left/Stop commands


// --- switches
int switchState; //used to set up contact switches
bool disableContacts = false; //flag used to mask interrupts if the robot is stuck in a routine for too long
unsigned long whenDisabledContacts=0; //timer used to remember when contact switches were disabled
unsigned long howLongWasContact=0; //timer counter used to track duration of a contact 
unsigned long whenWasLastContact=0; //record time of last contact

int current_target_heading=IN_DIRECTION; //in charging mode, keep track of desired dir
int LFC;
int RFC;
int LSBC;
int RSBC;
int LSFC;
int RSFC;
int LBC;
int RBC;
int FS;
int LS;
int RS;
int BS;

// bool sawSomeone;
bool goingIn;
bool goingOut;
bool goingCharging;
bool chargingMode;
bool diggingMode; //used to declare that the robot is in a process of digging
bool dumpingMode; //used to declare that the robot is disposing of its payload
bool restingMode;
bool exitTunnelMode;
bool turnReversalMode;
int nextMode;
bool turn_reversal_direction; //decides which direction the robot will turn in turn reversal

bool manualMode=false; //used to mask i2c Write back, i2c appears to lock up when writing back and forth
bool justSurveying=false; //used in jamming

PowerRelay Relay; //initiating an object to turn motor and camera power on and off using 1 pin
//--- voltage tap
Voltmeter Voltage;
int checkPowerCount = 0; //variable to prevent robot from going charging due to a random voltage drop 
#define CHECK_POWER_CNT_THRESH 500

// This enumeration is used in the switch-case statements in loop() to conduct any necessary tests
enum Test {
	TEST_IMU, 
	TEST_FORCE,
	TEST_MAG,
	TEST_CAP, 					// remember that the thresholds might be different when the robot is plugged in. This will make it harder to debug issues associated with the capacitive sensors
	TEST_SWITCHES, 			// Ross: I think this testing approach is deprecated. Use TEST_CAP instead
	TEST_DRIVE_MOTORS, 
	TEST_SERVO_MOTORS, 
	TEST_CAMERA, 
	TEST_GRIPPER_SENSOR,
	TEST_POWER_SENSORS, 
	TEST_TURN_HEADING,
	TEST_NOTHING,
};

float initial_heading;
float target_heading;
float diff_heading;
bool turn_init;
bool target_reached;

//--- set up IR distance sensors
//IRsensor IRright(IRsensorRightPin);
//IRsensor IRleft(IRsensorLeftPin);
//DigitalProximity FrontBumpSensor(ProximityBumpPin);
// ********** END   {GLOBAL VARIABLE DECLARATION} ----------

// bool watchdogFlag=1; //is set high when action is performed and pulled low when code cycles. If the flag stays low for a while, robot is presumed to be stuck 
// unsigned long watchdogReset; //store time when watchdog was last reset

//quick global vars to test out some stuff

unsigned long last_dash;

unsigned long dirCheckTimer; //quick workaround to prevent instantaneous dir changing
bool dirCheckFlag;
int numOfConsequitiveBackwardKicks=0;

bool preferGyro = false; //duct tape solution to use gyroscope to do 180 degree turns 

int StartIndicatorAddr = 0;


void WDT_Setup(){ // my default time is 18 seconds
	//15 seconds - resets when exiting tunnel
  	double desiredTimeout = 18;

  	double timeout2 =(int)( desiredTimeout * 227);  //227
  
  	timeout2 = 0x0fff2000 + timeout2; // 0xfff2000 is very inportant
  	WDT_Enable(WDT,timeout2);  
  	// number of loops:
	// 0x0fff2000 0
	// 0x0fff200f 231
	// 0x0fff2fff 2981
	// WDT_Restart(WDT); //USE THIS TO RESET WATCHDOG EVERYWHERE

}    

// ********** BEGIN (SETUP SCRIPT} **********
void setup(){
	WDT_Restart(WDT);
	pullResetPinLow(); //pull reset pin low 

	#if PROBABILITY_DIG
		int someRandValue=analogRead(A4); //read ADC val from the unused pin
		randomSeed(someRandValue); //set up random generator seed ///input can be changed to analogRead(int unused analog pin);
	#endif
	
	// pullResetPinHigh(); //set reset pin to 3.3v
	// analogReadResolution(10); // Later, change argument to 12 for 12 bit resolution, recalibrate thresholds
	/* establish serial communication */
	Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm class
	WDT_Restart(WDT);

	// initiateSwitches();
	delay(500); //ensure a power cycle after a watchdog reset //used to be 3s
	WDT_Restart(WDT);

	Relay.PowerOn(); //turn the power to the robot on
	turnIMUon();
	// pixy.init();        //Starts I2C communication with a camera

	WDT_Restart(WDT);
	initiateFioSerial(); //start Fio serial on channel 2
	// Serial.println(F("Ready")); //debug line
	// watchdogReset=millis(); //initiate watchdog timer 
	//initiateSwitches(); //sets up pins for contact switches
	initiateChargingDetector(); //set up contact charger detector
	//Wire.begin(); //for capacitive sensor
	//mpr121_setup(); //for capacitive sensor

	// while(1){
	// // TestDigitalProximity();
	// TestGripperSensor();
	// }
	/* initiate servos and move them to test  */
	Arm.Attach();       //hook up servos to pwm pins
	// while(TEST_SERVO_MOTORS){
		// TestServoMotors();
	// }
	Arm.PitchGo(HIGH_ROW_ANGLE); delay(500);  //puts an arm in a default configuration. Arm is extended and parallel to ground
	//Arm.GripperGo(MID_POS); delay(500);
	//Arm.GripperGo(OPEN_POS); delay(200);
	Arm.GripperGo(CLOSED_POS);

	delay(500);        //myDelay between servo movements
	Arm.PitchGo(HIGH_ROW_ANGLE-5);    //lower the arm slightly; this shows that the robot has been reset.
	delay(500);
	Arm.PitchGo(HIGH_ROW_ANGLE);
	/* initiate various boolean sttate variables */
	// initiateSwitches(); //sets up pins for contact switches
	
	// set TEST_SWITCHES to 1 to execute this loop
	// while(TEST_SWITCHES){
		// TestSwitches();
	// }

	
	// while(TEST_DRIVE_MOTORS){
		// TestDriveMotors();
	// }
	


	// //For Force Sensitive Resistor Test and Calibration
	// while(TEST_FORCE){
		// Serial.println(analogRead(ForceSensor));	
		// if(CheckPayload()){
		// Serial.println("Payload Detected");
		// }
		// delay(200);
	// }


	//start i2c bus stuff. CALL dof.begin() LAST, because it starts actually sending data
	// initiateMaster(); //start i2c bus , probably redundant 
	WDT_Restart(WDT);

	CapSensor.setup();
	FSensor.setup();
	pixy.init();        //Starts I2C communication with a camera

	WDT_Restart(WDT);

	// Serial.println("Testing Pixy");
	// while(1){
	// TestCamera(); //
	// // GetDetectedSigs();
	// WDT_Restart(WDT);
	// }

	delay(1000);
	WDT_Restart(WDT);
	dof.begin(); //starts I2C communication with a IMU. status variable should have a hex value of 0x49D4. This should def be called, has calibration functions inside
	delay(1000);
	WDT_Restart(WDT);
	/* power electronics */


	/* set up PD or PID control */
	PD.SetMode(AUTOMATIC);
	PD.SetSampleTime(PD_SAMPLE_TIME); //sets sample time. default is 100ms
	PD.SetOutputLimits(-PV_adjmax,PV_adjmax); //clamp limits of PD controller feedback 
	Setpoint = 160; //x coordinate of the center of the camera, 160=320/2




	last_dash = millis();        //initiate timer to keep track of last robot dash forward
	//checkIMU(); //same as mashing reset button   //commented out because its redundnant

	// dof.setGyroODR(dof.G_ODR_760_BW_100);  //sets up gyro output data rate to the highest (fastest) available setting
	// dof.setMagODR(dof.M_ODR_100);          //sets up magnetometers output data rate to the highest (fastest) available setting 
	/* 
	note that the gyroscope class creates 6 global variables
	gx gy gz  //gyro         readings in x y z
	ax ay az  //accel        readings in x y z
	mx my mz  //magnetometer readings in x y z
	Also, note that accelerometer readings are not much of use. To get a good 
	reading, accelerometer need to be exposed to at least 2g's 
	 */

	/* ISR INTERUPT DECLARATIONS */
	// encoder1.Initialize();
	// attachInterrupt(EncoderPinA1, updateEncoder1, CHANGE);  
	// attachInterrupt(EncoderPinB1, updateEncoder1, CHANGE);

	// encoder2.Initialize();
	// attachInterrupt(EncoderPinA2, updateEncoder2, CHANGE);  
	// attachInterrupt(EncoderPinB2, updateEncoder2, CHANGE);

	// attachInterrupt(ChargingDetectorPin, ForceCharging, CHANGE); //doesnt quite work

	/* initiate IR sensors */
	//IRright.detectionThresh=RIGHT_IR_THRESH; //set calibration for IR sensors  //360
	//IRleft.detectionThresh=LEFT_IR_THRESH; //380  //415



	// while(1){
	// //// TestIRsensorReadings();
	 // TestIRSensors();
	 // }
		// while(TEST_POWER_SENSORS){
			// TestPowerSensors();
		// }

	// getDetectedContacts(); //poll camera for a potential fix 
	// checkCamera(); //commented out, redundant.


	// while(1){
	// GetDetectedSigs();
	// if(CHARGING_TRAIL){
	// Serial.println(Areac);
	// }
	// }
	// TestPDController();
	// while(1){
	// FollowLaneBackward();
	// }

	/*
	enable_GoingInMode();
	while (1){
		WDT_Restart(WDT);
		switchState = CapSensor.getDetectedContacts();
		RFC=switchState & FR_ANT;
	LFC=switchState & FL_ANT;
	RSBC=switchState & RSB_ANT;
	RSFC=switchState & RSF_ANT;
	LSBC=switchState & LSB_ANT;
	LSFC=switchState & LSF_ANT;
	LBC=switchState & BL_ANT;
	RBC=switchState & BR_ANT;
	FS=RFC|LFC;
	RS=RSBC|RSFC;
	LS=LSBC|LSFC;
	BS=LBC|RBC;
		Serial.print(FR_ANT);
		Serial.print(FL_ANT);
		Serial.print("Switch State ");
		Serial.print(switchState);
		Serial.print(" RFC ");
		Serial.print(RFC);
		Serial.print(" LFC ");
		Serial.print(LFC);
		Serial.print(" FS ");
		Serial.println(FS);
		//if (CONTACT){
		//handleContact();
		//}
		delay(1000);
	}
	*/
	 
	 // while(1){
	// // TestPDController();
	// GetDetectedSigs();
	// FollowLane();
	// }

	// enable_GoingCharging();

	 // if( CheckPower() ){
	 // enable_GoingCharging();
	 // }
	 // else{
	 // enable_GoingInMode();
	 // }
 
 
	/*
	#if PROBABILITY_DIG //I think that this will work :-)
	 unsigned long timeToSwitchOnResting=millis();
	 bool startWithResting=false;
	 while ( millis() - timeToSwitchOnResting < 2000){
		if(DUMPING_SWITCH){
		 startWithResting=true;
		} 
	 }
	 
		if(startWithResting){
		StartLorenzRun();
		}
		else{
		enable_GoingInMode();
		}
	 
	 // BAR
	 #else
	 // FOO
		enable_GoingInMode();
		
	#endif
	*/
	enable_GoingInMode();

	//preferGyro=false; 
	//enable_turnReversalMode(1);
		
	// while(1){
	// TestGripperSensor();
	// }
	// Backward(BASE_SPEED); //dumb fix for a rare situation when the robot is pressed against cotton and for whatever reason the robot keeps going in a reset loop
	// delay(2000); //CHECKING DIRECTION

	// while(TEST_IMU){
		// TestIMU();
	// }

	//Serial.println("Started Program");
	// enable_RestingMode();
	// while(1){
	// TestIRSensors();
	// }
	// while(1){
	// TestIRSensors();

	// }

	/*  while(1){
	 if(CHARGER){
		Serial.println("bang");
	 }
		WDT_Restart(WDT);

		} */
	 //HeadSensor.threshold=1020;// 700 //1010 //JSP
	//HERE HERE HERE HERE HERE HERE HERE HRE THIS EFFING GRIPPER SENSOR. FIX THE WIRE <---------
	 // while(1){
	 // Serial.print(HEADSENSOR);
	// Serial.print('\t');
	 // TestGripperSensor(); //wdt_restart inside
	 // WDT_Restart(WDT);
		// }
	 
	// %%
	//while(1){
	//WDT_Restart(WDT);
	//float heading;
	//unsigned long now=millis();
	//while (millis() - now < 5000){
	//uint16_t IMUstatus = dof.checkStatus(); //write to whoAmI register and see if there is response
	//Serial.print(IMUstatus,HEX);
	//Serial.print("\t");
	//dof.readMag(); //update magnetometer registers
	//heading=getHeading((float) dof.mx, (float) dof.my);
	//Serial.print(F("H  "));
	//Serial.println(heading);
	//}
	// WDT_Restart(WDT);

	// now = millis();
	// Serial.println("IMU OFF");
	// // turnIMUoff();
	// // while (millis() - now < 5000){
	// // uint16_t IMUstatus = dof.checkStatus(); //write to whoAmI register and see if there is response
	// // Serial.print(IMUstatus,HEX);
	// // Serial.print("\t");
	// // dof.readMag(); //update magnetometer registers
	// // heading=getHeading((float) dof.mx, (float) dof.my);
	// // Serial.print(F("H  "));
	// // Serial.println(heading);
	// // }
	// Serial.println("IMU ON");
	// // dof.begin(); //starts I2C communication with a IMU. status variable should have a hex value of 0x49D4. This should def be called, has calibration functions inside
	// // turnIMUon();
	// delay(1000);
	// fioWrite(RESET_REQUEST);
	// }

	// %%
	dirCheckFlag=false;
	dirCheckTimer=millis();

	/*
	 float angle=0;
	 float refRate=0;
	 unsigned long  gyroTime=millis();
	 setGyroRef(&gyroTime,&refRate);
	 unsigned long delayTimer=millis();
	 while(1){
	 WDT_Restart(WDT);
	 bool output=countGyro(&angle,&gyroTime,&refRate);
	 Serial.print("Angle \t");
	 Serial.print(angle);
	 Serial.print("Output \t");
	 Serial.println(output);
		// // if( millis() - delayTimer > 5000){
		// // delay(2500);
		// // delayTimer=millis();
		// // }
		if(output){
		setGyroRef(&gyroTime,&refRate);
		angle=0;
		}
	 }
	*/


	// //for magnetometer MAG3110 sensor test and calibration
	// while(TEST_MAG){
		// WDT_Restart(WDT);
		// Serial.println(FSensor.Detected());
	// }

	// //for capacitive sensor test and calibration
	// while(TEST_CAP){
		// int testPanel = 1; // takes on values from 0 up to and including 7
		// WDT_Restart(WDT);
		// //Serial.println(CONTACT); //print all contact
		// Serial.println(CapSensor.getOneContact(testPanel)); //print capacitive sensor value for only one pin, in this case, pin 0. Change the number to choose other pins.
		
		// fioWriteInt(CapSensor.getOneContact(testPanel)); //send the capacitive sensor value to fio.
		// //Serial.println(CapSensor.readTouch()); //print the touch status of all pins.
		// delay(1000);
		// //if (CONTACT){
		// //enable_GoingInMode();
		// //handleContact();
		// //}
	// }



	// while(TEST_TURN_HEADING){
		// WDT_Restart(WDT);
		// Forward(BASE_SPEED);
		// delay(5000);
		// TurnHeading(OUT_DIRECTION);
		// WDT_Restart(WDT);
		// Forward(BASE_SPEED);
		// delay(5000);
		// TurnHeading(IN_DIRECTION);
	// }


	/* unsigned long lastGyroRead=millis();
	float gyroZval=dof.calcGyro(dof.gz);
	unsigned long dt;
	float angle=0;
	while(1){
	WDT_Restart(WDT);
	dof.readGyro(); //update gyro registers
	float gyroZ=dof.calcGyro(dof.gz);
	Serial.print(F("G  ")); Serial.print(gyroZ);
	dt=millis()-lastGyroRead;
	lastGyroRead=millis(); //grab last time
		if( abs(gyroZval-gyroZ)>5){
		angle=angle+dt*gyroZ/1000;
		}
	Serial.print("\t angle \t");
	Serial.println(angle);
	} */

	// while(TEST_GRIPPER_SENSOR){
		// Serial.println(analogRead(FGripperPin)); // what is FGRIPPERPIN?
		// TestGripperSensor();
		// WDT_Restart(WDT);
	// }



	// #define DEBUG 1

	// #if DEBUG
	//Serial.println( F("end of a setup loop") ); //F() those strings !!! Note for JSP //bani commented
	// #endif

  // Initialise direction variables
  turn_init = false;
  target_reached = false;
	
	
}
// ********** End (SETUP SCRIPT} **********




void StartLorenzRun(){
	Relay.PowerOff();
		#ifdef FIO_LINK
			fioWrite(MASTER_RESTING);
		#endif

	bool resting=true; //flow control
	unsigned long restingStart=millis(); //timer to keep the robot waiting/resting
	
	// probability digging 
	while(resting){
		WDT_Restart(WDT);
		if( millis() - restingStart > RESTING_TIME ){ //determine if its time to roll a dice 
			if(!rollDice()){
				#ifdef FIO_LINK
					fioWrite(ROLLED_DIG);
				#endif
				Relay.PowerOn(); //can be packaged with #if
				leaveChargingStation();
				return;
			}
    
		else{
			WDT_Restart(WDT);
			restingStart=millis(); //reset timer, and wait for another roll
			#ifdef FIO_LINK
				fioWrite(ROLLED_REST); //maybe report a different byte for consecutive rests 
			#endif   
    }
   }
	}	//end of while 
} //end setup loop 


bool countGyro(float* anglePtr, unsigned long* lastTime,float* refRatePtr){
	// WDT_Restart(WDT);
	dof.readGyro(); //update gyro registers
	unsigned long now=millis();
	unsigned long dt=now-(*lastTime); //compute time
	if(dt>900){//guard against blocking delays
		(*lastTime)=now; //record time for next computation and try again
		return false; //nope
	}
	float gyroZ=dof.calcGyro(dof.gz); //compute Degrees Per Second
	#define DRIFT_THRESH 5//5
	#define GYRO_TURN_THRESH 150//150
	if( abs(gyroZ-(*refRatePtr)) > DRIFT_THRESH ){
		(*anglePtr)+=dt*gyroZ/1000; //perform integration
	} 
	(*lastTime)=now; //record for the next computation
	Serial.println(*anglePtr);
	if( abs((*anglePtr))>GYRO_TURN_THRESH ){
		return true;
	}
	else{
		return false;
	}
}

void setGyroRef(unsigned long* lastTime,float* refRatePtr){
	Stop(); delay(100);
	do{
		dof.readGyro(); //update gyro registers
		(*refRatePtr)=dof.calcGyro(dof.gz);
		(*lastTime)=millis();
  }while( abs(*refRatePtr)>10); //make sure we dont read in bad reference when the robot was bumped in
	// Serial.print("ref cal \t");
	//Serial.println(*refRatePtr);
}
 
// ********** END   {SETUP SCRIPT} ----------

// ********** BEGIN (MAIN SCRIPT} **********

void loop(){

	Serial.println("main loop");
	
	// This is advantageous in that adding a new test is as simple as adding a test state to the global enumeration and adding a test method to a separate test.cpp file (does not currently exist)
	// enumeration defined in global declaration, and TEST_CASE is in RobotSelector
	switch(TEST_CASE){
		case TEST_IMU:
			TestIMU();
			break;
		case TEST_SWITCHES:
			TestSwitches();
			break;
		case TEST_FORCE:
				//For Force Sensitive Resistor Test and Calibration
			while(1){
				Serial.println(analogRead(ForceSensor));	
				if(CheckPayload()){
					Serial.println("Payload Detected");
				}
				delay(200);
			}
			break;
		case TEST_MAG:
			//for magnetometer MAG3110 sensor test and calibration
			while(1){
				WDT_Restart(WDT);
				Serial.println(FSensor.Detected());
			}
			break;
		case TEST_CAMERA:
			Serial.println("Testing camera...");
			TestCamera();
			WDT_Restart(WDT);
			break;
		case TEST_CAP:
				//for capacitive sensor test and calibration
			while(1){
				int testPanel = 1; // takes on values from 0 up to and including 7
				WDT_Restart(WDT);
				//Serial.println(CONTACT); //print all contact
				Serial.println(CapSensor.getOneContact(testPanel)); //print capacitive sensor value for only one pin, in this case, pin 0. Change the number to choose other pins.
				
				fioWriteInt(CapSensor.getOneContact(testPanel)); //send the capacitive sensor value to fio.
				//Serial.println(CapSensor.readTouch()); //print the touch status of all pins.
				delay(1000);
				//if (CONTACT){
				//enable_GoingInMode();
				//handleContact();
				//}
				}
			break;
		case TEST_DRIVE_MOTORS:
			TestDriveMotors();		
			break;
		case TEST_SERVO_MOTORS:
			break;
		case TEST_GRIPPER_SENSOR:
			// Serial.println(analogRead(FGripperPin)); // FGripperPin not defined
			// TestGripperSensor();
			// WDT_Restart(WDT);
			break;			
		case TEST_POWER_SENSORS:
			TestPowerSensors();
			break;
		case TEST_TURN_HEADING:
			TestTurnHeading();
			break;
		case TEST_NOTHING:
			// Serial.println("Test intentionally ignored");
			break;	
		default:
			// Serial.println("Default test case. TEST_CASE not properly assigned?");
			break;
	}
		
	if(goingIn){
		Serial.println("Going in...");
		KP=Kp; //reset gain
		current_target_heading = IN_DIRECTION;
		GoingInMode();
	}

	if(diggingMode){
		Serial.println("diggingMode");
		while(diggingMode){
			DiggingMode();
			//Serial.println("Digging Mode"); // JSP //bani commented
		}
	}

	if(dumpingMode){
		Serial.println("dumpingmode");
		current_target_heading=OUT_DIRECTION;
		while(dumpingMode){
			DumpingMode();
			//Serial.println("Dumping Mode"); //JSP //bani commented
		}
	}

	if(goingOut){
		Serial.println("GoingOutMode");
		current_target_heading=OUT_DIRECTION;
		GoingOutMode();
	}

	if(goingCharging){
		GoingCharging();
	}
 
	if(chargingMode){
		ChargingMode();
	}
 
	if(restingMode){
		KP=Kp; //reset gain
		RestingMode();
	}
 
	if (exitTunnelMode){
		exitTunnel();
	}
 
	if (turnReversalMode){
		// TurnHeadingRoss(current_target_heading);
		TurnHeading(current_target_heading);

	}

}//end main loop
// ********** END   {MAIN SCRIPT} ----------

// ********** BEGIN (MODE DEFINITION} **********
//----------------------------------------------------
void GoingInMode(){
	Serial.println(F("Beginning of GoingInMode() - 1"));

	WDT_Restart(WDT);
	numOfConsequitiveBackwardKicks = 0;

	#ifdef FIO_LINK
		Serial.println(F("FIO_LINK defined"));
		fioWrite(MASTER_GOING_IN); //report over radio 
	#endif

	#if ALLOW_USELESS_RUNS
		unsigned long whenModeStart=millis(); //the robot has X seconds to get to the tunnel or it has to get out
	#endif

	//Forward(BASE_SPEED); //anti stuck VADIM. uncomment this 

	//watchdogFlag=1; //action taken
	// unsigned long lastBackup=millis(); //timer to make the robot back out slightly backward every so often 

	bool movingPitchUp = false; //remember if we are moving pitch up or down
	unsigned long whenMovedHead=millis();  //initiate timer to myDelay head shaking; //pitch head up and down to make sure that the head sensor triggers if the robot is pushing in a tunnel

	unsigned long whenIRsensorOverride=millis();

	// Arm.GripperGo(CLOSED_POS); //close the jaws
	Arm.GripperGo(CLOSED_POS); //close the jaws //JSP
	unsigned long whenForcedBackwardKick=millis();

	// TurnHeading(current_target_heading); //may need to add this for a case when the board resets
	// add head bump sensor
	//start counting IR side how long
	//unsigned long whenSawTrails()=millis();
	Serial.println(F("goingIn is..."));

	
	while(goingIn){
		Serial.println("In goingIn while-loop");


		WDT_Restart(WDT);
		Arm.PitchGo(HIGH_ROW_ANGLE);
		Arm.GripperGo(CLOSED_POS); //JSP
		
		
		if (CheckPayload()){
			Serial.println("Something found in payload");
			current_target_heading = OUT_DIRECTION;
			preferGyro = true; // 
			enable_turnReversalMode(3); // this sets the variable nextMode to 3, which corresponds to goingOut. This sets a variable that will remember which mode the robot should enter after turning around...?
			return; // if there is something in the 	 gripper, then we need to exit goingInMode
		}
 
		// Checks for Contact
		if(CONTACT){
			Serial.println("Something contacted");
			WDT_Restart(WDT);
			handleContact();
		}
		
		FollowLane(); //poll camera and call PD
		// GetDetectedSigs(); //poll camera, get latest vision info
 
		if(DUMPING_SWITCH){//BANI
			Serial.println("Dumping switch");

			//if(CHARGER ){
			Backward(BASE_SPEED); delay(1000); //move back
			//TurnHeading(IN_DIRECTION); //turn back in
			enable_turnReversalMode(1);
			FollowLane();//poll camera and call PD
			WDT_Restart(WDT);
		} 

		if(CHARGER){
			Serial.println("Charging if-statement");

			//Serial.println("charger");
			//Serial.println("Charger!!!"); // JSP //BANI
			Stop(); delay(100);
			//--- copied from chargingMode backing out routine
			unsigned long backingOutStart=millis();
			Backward(BASE_SPEED);
    
			// while(CHARGER){
			// WDT_Restart(WDT);
			// }
			bumpDelay(1000); //force backout for short time
			Stop(); delay(200); //quick stop
			//TurnHeading(IN_DIRECTION); //turn to go in a tunnel
			enable_turnReversalMode(1);
			Stop(); delay(100);
			Forward(BASE_SPEED); //start slowly driving forward

			//whenForcedBackwardKick=millis(); //the robot drove backward 
			FollowLane();//poll camera and call PD
			WDT_Restart(WDT);
		}


		//--- handle wrong way directions
		if(checkWrongDirections()){
			Serial.println("checkWrongDirections()returns true");

			//whenForcedBackwardKick=millis(); //reset timer to prevent immediate backup
		}	

		FollowLane();//poll camera and call PD
		WDT_Restart(WDT);
		// Serial.println("WDT7");

/*   else if(COTTON){ //survey on cotton IFF lane is not seen 
		if(Areac>500){
   DriveForward(x7);
   }
  } */

 // if(CheckPower()){
 // enable_GoingCharging();
 // return;
 // }
 
		if( checkHeadSensor() ){
			Serial.println("checkHeadSensor() returns true");
			return; //found soemthing, lets dig 
		}
		/* 
		#ifdef MANUAL_ON
			handleManualOverride(); 
		#endif
		*/
  
		#if ALLOW_USELESS_RUNS //prob bugged
			Serial.println("ALLOW_USELESS_RUNS is true");

			if( millis() - whenModeStart > USELESS_RUN_THRESH){
				//bool goBack = rollDiceProb(50); //%chance to roll true used to be 50
				bool goBack = true; // force the robot to go back 
				preferGyro=true; 

				if (goBack){
					Backward(BASE_SPEED);
					delay(2000);
					Stop();
					current_target_heading=OUT_DIRECTION;
					//TurnHeading(current_target_heading);
					enable_turnReversalMode(7);
					//enable_RestingMode(); //go to the charging station
					//enable_GoingOutMode(); //BANI CHANGING MODE AS RESTING IS EXCLUSIVE TO LORENZ
					return; 
				}
				//bool goBack = false;//BANI
				else{ //BANI 
					whenModeStart=millis(); //reset timer 
					WDT_Restart(WDT);
					//Serial.println("WDT8");
				}
			} 
		#endif
	}//end while(goingIn)
}
//----------------------------------------------------

void DiggingMode(){
	//no manual control, no contact support
	// #ifdef FIO_LINK//BANI LCD
	// fioWrite(DISP_DIGGING); //report over radio //BANI LCD
	// #endif//BANI LCD

	#ifdef FIO_LINK
		fioWrite(MASTER_DIGGING); //report digging mode
	#endif
	//Serial.println("Digging!!!");
	Arm.GripperGo(OPEN_POS); //open gripper up
	// Arm.PitchGo(LOW_ROW_ANGLE);//raise the arm high
	int diggingAttempts=0;
	int diggingAttemptsTotal=0; //put a limit on maximum digging attempts;
	#define EXCAVATION_PENALTY 8000
	bool newAttempt=true; 
	while(diggingMode){
		WDT_Restart(WDT);
		newAttempt=true; //this will be reset if checkHeadSensor comes on
			//if(DUMPING_SWITCH){ //digging in a wrong area. cancel this mode and go back to goingIn
			//Backward(BASE_SPEED);//back up
			//delay(1500); //backing out delay
			//enable_GoingInMode();
			//Arm.PitchGo(HIGH_ROW_ANGLE);
			//Arm.GripperGo(OPEN_POS);
			//TurnHeading(IN_DIRECTION); //reorient toward digging area
			//return;
			//}  

		//Handle Contact
		if(CONTACT){
			handleContact();
			WDT_Restart(WDT);
			// Serial.println("WDT5");
		}
			
		WDT_Restart(WDT);
		delay(2); //small delay 
		
		if (checkHeadSensor()){//check if we are getting something
			newAttempt=false;
					//Back Up from the Cohesive Material to lower the gripper system
			unsigned long BackUpStart=millis();
			unsigned long ContactStart;
			unsigned long ContactDuration;
			while( (millis() - BackUpStart) < 2000 ){ //Back up for 2 seconds, if it is interrupted by a contact, deal with contact, and then resume backing up.
				WDT_Restart(WDT); //JSP
				Backward(BASE_SPEED);
				if(CONTACT){
					WDT_Restart(WDT);
					ContactStart=millis();
					handleContact();
					ContactDuration=millis() - ContactStart; //Measure how much time the robot took to respond to contact
					BackUpStart+=ContactDuration; //Add time the robot took to respond to contact, and back up more accordingly
				}
			}
			Stop(); delay(100);
			Arm.PitchGo(MID_ROW_ANGLE);
			Arm.GripperGo(OPEN_POS);
			Forward(BASE_SPEED); delay(200);
			Dig(); //Digging Motion
			Arm.GripperGo(CLOSED_POS); //close gripper
			delay(100);
			Backward(BASE_SPEED); delay(300);Stop(); //back up a little
			Arm.PitchGo(HIGH_ROW_ANGLE);
			Arm.GripperGo(CLOSED_POS); //close gripper again
			delay(250);
			if (CheckPayload()){
				enable_exitTunnelMode();//Go into exit tunnel mode
				return;
			} 
			else{
				Arm.GripperGo(OPEN_POS);
				Arm.PitchGo(MID_ROW_ANGLE);
				Dig(); //Digging Motion, JSP
				Arm.GripperGo(CLOSED_POS); //close gripper
				delay(100);
				Backward(BASE_SPEED); delay(300);Stop(); //back up a little
				Arm.PitchGo(HIGH_ROW_ANGLE);
				Arm.GripperGo(CLOSED_POS); //close gripper again
				delay(250);
				if (CheckPayload()){
					enable_exitTunnelMode();//could be made in its own mode
					return;
				}	
			}
		}  


		Forward(BASE_SPEED); delay(100); Stop(); //step in forward 
		if(newAttempt){
			WDT_Restart(WDT);
			diggingAttempts++; //increment this number  
		}
		diggingAttemptsTotal++;
		if(diggingAttempts > 3 || diggingAttemptsTotal > 5){//check if digging was unsucessful
			WDT_Restart(WDT);
			enable_GoingInMode();
			Arm.PitchGo(HIGH_ROW_ANGLE);
			Backward(BASE_SPEED);
			delay(1000);
			Stop();
			int left_or_right = random(0,100);
			if (left_or_right >= 50){
				Left(DEFAULT_TURNING_SPEED);
			}
			else{
				Right(DEFAULT_TURNING_SPEED);
			}
			delay(200);
			Stop();
			Arm.PitchGo(HIGH_ROW_ANGLE);
			// Arm.GripperGo(CLOSED_POS);
			Arm.GripperGo(OPEN_POS);
			delay(1000);
			return;//if so, exit out
		}
		delay(100);
		Forward(BASE_SPEED); delay(100); Stop();
	}//end while(diggingMode)
}

void Dig(){
	WDT_Restart(WDT);
	//TurnHeading(IN_DIRECTION); //reorient toward digging area
	unsigned long digStart=millis();
	while (!(RUSensor.IsDetected() || LUSensor.IsDetected() || RDSensor.IsDetected() || LDSensor.IsDetected())){
		if( (millis() - digStart) > 5000 ){ //for some reason, if the hall effect sensor on gripper doesn't see anything for too long, terminate dig function
			WDT_Restart(WDT); //JSP
			return;
		}
		Forward(BASE_SPEED); // Go Forward until the hall effect sensor on gripper touches the cohesive material
	 }
	 
		if (RUSensor.IsDetected()){
			Arm.PitchGo(MID_ROW_ANGLE + 4);
		}
		else if (LUSensor.IsDetected()){
			Arm.PitchGo(MID_ROW_ANGLE + 2);
		}
		else if (RDSensor.IsDetected()){
			Arm.PitchGo(MID_ROW_ANGLE);
		}
		else{
			Arm.PitchGo(MID_ROW_ANGLE - 2);
		}

	int i = 0; //JSP
	while (i < 3){
		Arm.GripperGo(OPEN_POS);
		Forward(BASE_SPEED);
		delay(200);
		Arm.GripperGo(MID_POS);
		Forward(BASE_SPEED);
		delay(200);
		i = i + 1;
	} //JSP
}


void exitTunnel(){
	WDT_Restart(WDT);
	fioWrite(MASTER_EXIT_TUNNEL);
	Arm.PitchGo(HIGH_ROW_ANGLE);
	// Backward(255); myDelay(3000); 
	// TurnHeading(OUT_DIRECTION); //turn towards dumping area
	// Stop(); myDelay(1000); //debug myDelay
	// enable_GoingOutMode();
	// HeadSensor.threshold=940; //return threshold to its original value 
	unsigned long exitStart=millis(); //grab a time to enable timeout 
	bool inTunnel=true; //am I in the tunnel?, assume that we are to start 

	while(inTunnel){
		WDT_Restart(WDT);
		GetDetectedSigs(); //get the colors
		Backward(BASE_SPEED); //go backward
		delay(100);
	  // if(TUNNEL_START){//have I spotted the end of the tunnel marker?
	   // if(Areat > 3000){//check here if the area is larger then thresh or seen this for X amount of time //3000 too high
	   // break; //exit out of the method 
	   // }  
	  // }
		WDT_Restart(WDT); //JSP
		if( (millis() - exitStart) > 10000 ){ //temporarily timeout //used to be 10
			WDT_Restart(WDT); //JSP
	  	inTunnel = false;
			//break;
		}
	  
		if(CONTACT){
			WDT_Restart(WDT);
			handleContact();
	   	WDT_Restart(WDT);
	   	exitStart+=3500;//3000 //add time to compensate
		}
	} 
	
	// Backward(255); delay(1000); myDelay(2000); Stop();
	WDT_Restart(WDT); //JSP
	//HeadSensor.threshold=700; //return threshold to its original value. may add a second check //JSP
	current_target_heading=OUT_DIRECTION;
	preferGyro=true; 
	//TurnHeading(current_target_heading);
	enable_turnReversalMode(3); //3 denotes that the robot will go into going_out mode after turn reversal mode
	return;
}

//----------------------------------------------------
void GoingOutMode(){
	WDT_Restart(WDT);
	Arm.PitchGo(HIGH_ROW_ANGLE);
	numOfConsequitiveBackwardKicks=0;
	#ifdef FIO_LINK
		fioWrite(MASTER_GOING_OUT); //report going out
	#endif 
	//watchdogFlag=1; //action taken
	//bool IRactionHandled=false;
	//unsigned long whenIRactionHandled=millis(); //remember last IR action
	//unsigned long whenForcedBackwardKick=millis(); //remember last forced backward kick
	unsigned long whenCheckedPayload=millis(); //used to occasionally check if payload is still there 
	// bool maskHeadTrigger=false;
	// if(HEADON){
	// maskHeadTrigger=true; //if excavated load happens to block this sensor, disable it 
	// }
	while(goingOut){
		WDT_Restart(WDT);
		#ifdef MANUAL_ON
			handleManualOverride(); 
		#endif
 
 //--IR contacts 
 
 //IRactionHandled=handleIRcontacts();
 //checkIfBackwardKickNeeded(&IRactionHandled,&whenIRactionHandled,&whenForcedBackwardKick);
 //timeoutForceBackwardKick(&whenForcedBackwardKick); //force backward kick if the robot has not been backing out in a long time
 //FollowLane();//poll camera and call PD
		WDT_Restart(WDT);
  /* axed this behaviour. somehow it stopped working right anyway */
 // if( !checkPayload(&whenCheckedPayload)){//if we dropped payload. probably not working right
 // TurnHeading(IN_DIRECTION);
 // enable_GoingInMode(); 
 // return; 
 // };
 
		FollowLane();//poll camera and call PD

		//---contact handling	
		if(CONTACT){
			WDT_Restart(WDT);
			handleContact();
			FollowLane(); //poll camera and call PD
		}
		
		// if(CHARGER){ //when the charger was on the side
		// WDT_Restart(WDT);
		// Stop(); delay(100);
		// //--- copied from chargingMode backing out routine
		// // unsigned long backingOutStart=millis();
		// Backward(BASE_SPEED); bumpDelay(3000); //force backout for short time
		// Stop(); delay(200); //quick stop
		// WDT_Restart(WDT);
		// TurnHeading(OUT_DIRECTION); //turn to go in a tunnel
		// Stop(); delay(100);
		// // myDelay(1000); //debug myDelay;
		// Forward(BASE_SPEED); //start slowly driving forward
		// whenForcedBackwardKick=millis(); //note that robot drove backward
		// }
		FollowLane(); //call PD controller -- this is called immediately after it is called?
		

		//--- handle wrong way directions
		if(checkWrongDirections()){
			//whenForcedBackwardKick=millis(); //reset timer to prevent immediate backup
			FollowLane();//poll camera and call PD
		}	
		
		//if (DUMPING_SWITCH) { //VADIM SHOWED EXAMPLE
		if (CHARGER){
			Stop();
			enable_DumpingMode();
			return;
		}
	} //end while(goingOut)
}

//----------------------------------------------------
void DumpingMode(){
//renamed Deposit Mode. dumping run
WDT_Restart(WDT);
// #ifdef FIO_LINK

 fioWrite(MASTER_DUMPING); //report dumping mode
// #endif
//this mode is allocated for further development of smart media stacking and such
//added feedback dumping feature 11/10/2014 (dump untill head sensor is cleared)
 if (!justSurveying){
 DumpPayload(); //drop cotton balls
 Arm.GripperGo(CLOSED_POS);
  unsigned long modeTimeout=millis();
  while( analogRead(ForceSensor) >= ForceSensorThresh ){ // if there are still materials stuck inside grippers //JSP
  Backward(BASE_SPEED); delay(500); //back up
  Stop(); delay(100); //stop;
  Forward(BASE_SPEED); //go forward until dumping switches are compressed 
  unsigned long now=millis();
   while( (millis()-now) < 5000){//and limit forward dash for 5 seconds
    // if(DUMPING_SIGNAL){ //replace a line below with this "improvmenet"
    // if(DUMPING_SWITCH){
    if(CHARGER){ //

		WDT_Restart(WDT);
	  break;
	  }
   }
	WDT_Restart(WDT);
  Stop(); delay(100); //stop, we just hit the switches 
  DumpPayload(); //drop cotton balls again 
  //Backward(255); delay(500); //back out, and check again if there are still cotton balls. 
  //bumpDelay(500);
   if( (millis()-modeTimeout) > 15000){
   WDT_Restart(WDT);
	 DumpPayload(); //drop cotton balls
   leaveDumpingSite();
   return; //force exit 
   // break;   
   }   
  }
 } //do this untill the robot does not have anything in a jaw
 else {
 justSurveying=false; //finished surveying 
 //surveying++ or something
 } //surveying completed
 WDT_Restart(WDT);
#ifdef FIO_LINK
fioWrite(MASTER_DUMPING); //report dumping mode
#endif

 #if PROBABILITY_DIG 
 // enable_RestingMode();
  StartLorenzRun();
 return; 

 #else
 enable_GoingInMode();
 leaveDumpingSite(); //contains pre compiler directives inside 
 #endif 

 return;
} 

void leaveDumpingSite(){

 WDT_Restart(WDT);
 Backward(BASE_SPEED); 
 delay(1000); //back out
 bumpDelay(500);
 WDT_Restart(WDT);
 preferGyro=true;
 //current_target_heading=IN_DIRECTION;
 //TurnHeading(current_target_heading);
 enable_turnReversalMode(1);
 //enable_GoingInMode();//BANI //turn around to face excavation area
  if(CheckPower()){
  enable_GoingCharging();
  return;
  }

 unsigned long forcedFollowingStart =  millis();
 WDT_Restart(WDT);
 // while( (millis() - forcedFollowingStart < 700) ){
 // FollowLane(); 
 // }
 return;
}


//----------------------------------------------------

void goToDumpingSite(){
WDT_Restart(WDT);
#ifdef FIO_LINK
fioWrite(MASTER_USELESS_RUN); //report going out //replace with useless run
#endif 
current_target_heading=OUT_DIRECTION;
unsigned long whenForcedBackwardKick=millis(); //remember last forced backward kick
unsigned long whenCheckedPayload=millis(); //used to occasionally check if payload is still there 
// bool maskHeadTrigger=false;
// if(HEADON){
// maskHeadTrigger=true; //if excavated load happens to block this sensor, disable it 
// }
 while(1){
 WDT_Restart(WDT);
 #ifdef MANUAL_ON
 handleManualOverride(); 
 #endif

 //--IR contacts 
 
 //IRactionHandled=handleIRcontacts();
 //checkIfBackwardKickNeeded(&IRactionHandled,&whenIRactionHandled,&whenForcedBackwardKick);
 //timeoutForceBackwardKick(&whenForcedBackwardKick); //force backward kick if the robot has not been backing out in a long time
 FollowLane();//poll camera and call PD

  //---contact handling	
  if(CONTACT){
	WDT_Restart(WDT);
  handleContact();
  FollowLane();//poll camera and call PD
  }
	if(CHARGER){//bani
  //if(DUMPING_SWITCH){
	WDT_Restart(WDT);
  Stop();
	return;
	// Stop(); delay(100);
  // //--- copied from chargingMode backing out routine
  // // unsigned long backingOutStart=millis();
  // Backward(BASE_SPEED); bumpDelay(3000); //force backout for short time
  // Stop(); delay(200); //quick stop
  // TurnHeading(OUT_DIRECTION); //turn to go in a tunnel
  // Stop(); delay(100);
  // // myDelay(1000); //debug myDelay;
  // Forward(BASE_SPEED); //start slowly driving forward
  // whenForcedBackwardKick=millis(); //note that robot drove backward
  }
  FollowLane(); //call PD controller
  

  //--- handle wrong way directions
  if(checkWrongDirections()){
	WDT_Restart(WDT);
  //whenForcedBackwardKick=millis(); //reset timer to prevent immediate backup
  FollowLane();//poll camera and call PD
  }	
  
  // if (DUMPING_SWITCH) {
	// WDT_Restart(WDT);
  // return;
  // }
 } //end while(goingOut)
}
//--------------------
int seekCharging(){
//---
WDT_Restart(WDT);
#ifdef FIO_LINK
fioWrite(MASTER_GOING_CHARGING); //report going out
#endif 
//watchdogFlag=1; //action taken
//bool IRactionHandled=false;
//unsigned long whenIRactionHandled=millis(); //remember last IR action
//unsigned long whenForcedBackwardKick=millis(); //remember last forced backward kick
unsigned long whenCheckedPayload=millis(); //used to occasionally check if payload is still there 
// bool maskHeadTrigger=false;
// if(HEADON){
// maskHeadTrigger=true; //if excavated load happens to block this sensor, disable it 
// }
 current_target_heading=OUT_DIRECTION; 
 while(1){
 WDT_Restart(WDT);
 #ifdef MANUAL_ON
 handleManualOverride(); 
 #endif
 
 //--IR contacts 
 /*
 IRactionHandled=handleIRcontacts();
 checkIfBackwardKickNeeded(&IRactionHandled,&whenIRactionHandled,&whenForcedBackwardKick);
 timeoutForceBackwardKick(&whenForcedBackwardKick); //force backward kick if the robot has not been backing out in a long time
 FollowLane();//poll camera and call PD
 WDT_Restart(WDT);
 */ 
  //---contact handling	
  if(CONTACT){
	WDT_Restart(WDT);
  handleContact();
  FollowLane();//poll camera and call PD
  }
	
  //--- handle wrong way directions
  if(checkWrongDirections()){
  //whenForcedBackwardKick=millis(); //reset timer to prevent immediate backup
  FollowLane();//poll camera and call PD
  }	
  
  // if (CHARGER) {//bingo!
  // if (CHARGER || DUMPING_SWITCH) {//bingo! //VADIM
  if (CHARGER) { //VADIM
	Stop();
  return 1;
  }
 } //end while(1)
}

//---


void leaveChargingStation(){
//drive back and prepare for GoingIn
 WDT_Restart(WDT);
 Backward(BASE_SPEED); delay(1000);
 WDT_Restart(WDT);
 bumpDelay(3000);
 Stop(); delay(50);
 Forward(BASE_SPEED); delay(200); //nudge forward a bit in case you hit a wall or something
  Left(DEFAULT_TURNING_SPEED);
  WDT_Restart(WDT);
	delay(1000); //force overshoot 	
 current_target_heading=IN_DIRECTION;
 //TurnHeading(current_target_heading);
 enable_turnReversalMode(1); 
 //enable_GoingInMode();
 //GetDetectedSigs();
  // if(CHARGING_TRAIL){
  // }
}

void GoingCharging(){
	numOfConsequitiveBackwardKicks=0;
	WDT_Restart(WDT);
 	if ( seekCharging() ){
 		enable_ChargingMode();
 		return;
 	}
}//end going charging

//----------------------------------------------------
void ChargingMode(){
//NOT GOING TO WORK. WE CUT THE CHARGING + wire 
#ifdef FIO_LINK
fioWrite(MASTER_CHARGING);
#endif
WDT_Restart(WDT);
//need to add manual stuff. need to be able to turn relay on and off
float C; //variable for storing current 
//Serial.println("Entered Charging Mode");
Stop(); delay(1000);
//note: delay is used here instead of myDelay
//need to add a little statement that checks if the voltage is actually increasing 
WDT_Restart(WDT);
Relay.PowerOff(); delay(1000); 
float BatVoltage= Voltage.GrabAvg(25);
float previousMaxVoltage=BatVoltage;
unsigned long whenIncreasedVoltage=millis();
 while(BatVoltage < CHARGED_VOLTAGE){
 BatVoltage= Voltage.GrabAvg(25); //read current voltage
  if(BatVoltage > previousMaxVoltage){
  whenIncreasedVoltage=millis(); //reset timer
  previousMaxVoltage=BatVoltage; //record new high
  }
 WDT_Restart(WDT);
  if( (millis() - whenIncreasedVoltage) > 300000){ //5 minutes
  WDT_Restart(WDT);
	break; //havent seen voltage increase in a while 
  }

  if(!CHARGER){ //not touching charging station
  WDT_Restart(WDT);
	Relay.PowerOn(); delay(1000); //turn power back on
  // checkCamera();
  bool redock_success=redock(); //if success, should we reset voltage tracking variables? 
   if(!redock_success){
   // enable_GoingCharging(); //need to find charging beacon again 
   // return; //exit this mode   
   seekCharging(); //find charging station again 
   Relay.PowerOff();
   }   
  }
 delay(100); //small waiting delay
 // C=Current.ReadAvg(100); //debug
 }
 
//now the voltage level has jumped up, but we need to add juce
unsigned long chargingStart=millis();
//                                m  s   ms  
unsigned long desiredChargingTime=9000000; //2.5hours of charging
 while ( millis() - chargingStart < desiredChargingTime){
 WDT_Restart(WDT);
 delay(1000); //do nothing
  if(!CHARGER){ //not touching charging station
  Relay.PowerOn(); delay(1000); //turn power back on
  bool redock_success=redock();
	WDT_Restart(WDT);
   if(!redock_success){
   // enable_GoingCharging(); //need to find charging beacon again 
   // return; //exit this mode   
   seekCharging(); //find charging station again
   Relay.PowerOff();
   desiredChargingTime+=5*60*1000; //add some time to compensate 
   } 
  }
 // C=Current.ReadAvg(20);
  // if( C > -0.099 && C <=0){ //good enough
  // break; //exit this loop
  // }
 }
 Relay.PowerOn(); //turn the power to the robot on
 WDT_Restart(WDT);
 delay(3000);
 WDT_Restart(WDT);
 //checkCamera();
 // checkIMU();
 // checkCamera();
leaveChargingStation();
Forward(BASE_SPEED); //start slowly driving forward, anti stuck kick 
return;
}

void RestingMode(){
numOfConsequitiveBackwardKicks=0;
WDT_Restart(WDT);
//this mode is exclusive to probabilistic digging 
#if PROBABILITY_DIG

  #ifdef FIO_LINK
  fioWrite(MASTER_RESTING);
  #endif
 int code=seekCharging(); //get to the charging station
  if(code==2){
	return;
	}
 Stop(); //hit the breaks
  #if !ALLOW_CHARGING_ON_REST
  Backward(BASE_SPEED); //back up so that we are not charging
   while(CHARGER || DUMPING_SWITCH){
   //do nothing. Add timeout here too
   }
  delay(200);
  Stop();
  #endif
 WDT_Restart(WDT);
  //--roll a dice here if we want more digging
  if(!rollDice()){
   #ifdef FIO_LINK
   fioWrite(ROLLED_DIG);
   #endif 
  leaveChargingStation();
  return;
  }
  else{
  WDT_Restart(WDT);	
   #ifdef FIO_LINK
   fioWrite(ROLLED_REST);
   #endif
  
   #if ALLOW_POWER_SAVINGS
   Relay.PowerOff();
   #endif  
   #if ALLOW_CHARGING_ON_REST
   Relay.PowerOff();
   #endif
  }

//--- at this point we are charging if not backed out

 bool resting=true; //flow control
 unsigned long restingStart=millis(); //timer to keep the robot waiting/resting
 // probability digging 
  while(resting){
	WDT_Restart(WDT);
   if( millis() - restingStart > RESTING_TIME ){ //determine if its time to roll a dice 
    if(!rollDice()){
 	#ifdef FIO_LINK
    fioWrite(ROLLED_DIG);
    #endif
    Relay.PowerOn(); //can be packaged with #if
    leaveChargingStation();
    return;
    }
    else{
		WDT_Restart(WDT);
    restingStart=millis(); //reset timer, and wait for another roll
     #ifdef FIO_LINK
     fioWrite(ROLLED_REST); //maybe report a different byte for consecutive rests 
     #endif   
    }
   }
   #if ALLOW_CHARGING_ON_REST
  //re-dock if charging is allowed, found, and lost  
	// #ifdef FIO_LINK//bani test
    // fioWrite(CHECK_END);//bani test
    // #endif//bani test
	  if(!CHARGER){//bani
		
    //if(!CHARGER || !DUMPING_SWITCH){ //not touching charging station, redock, may need to break out of here earlier?
    WDT_Restart(WDT);
		Relay.PowerOn(); delay(1000); //turn power back on
  //this can be probably replaced with "go forward for a while, if not, do stuff below
    bool redock_success=redock(); //if success, should we reset voltage tracking variables? 
     if(!redock_success){
		 	
     code=seekCharging(); //find charging station again 
      if(code==2){return;} //
		 Relay.PowerOff();
     }   
    }	
   #endif
  
  #if !ALLOW_CHARGING_ON_REST //case where charging is NOT RFF
   
	 if(CHARGER){//bani
	 //if(CHARGER || DUMPING_SWITCH){//bani
	 WDT_Restart(WDT);
   Relay.PowerOn(); delay(1000); //turn power back on
   Backward(BASE_SPEED); delay(500); //back out, we dont want to be touching the charging station
   Stop();
    #if ALLOW_POWER_SAVINGS
    Relay.PowerOff(); delay(500); //turn power back off
    #endif
   }
  #endif
  
  if(CheckPower()){
	WDT_Restart(WDT);
  enable_GoingCharging();
  return;
  }
 }//end of while //bani removed
 return;//bani
 // #else//bani
 
 
 // seekCharging();// get to the charger//bani removed this else snippet as it screws with the rest of the code
 // delay(10000); //10 seconds//bani
 // leaveDumpingSite();//bani
 // return;//bani

 #endif

goToDumpingSite();
delay(RESTING_TIME); //wait 
leaveDumpingSite();

//enable_GoingInMode;//BANI
//return;//BANI...TO MOVE IT OUT OF MONOTONOUS LOOP
 // #ifdef ALLOW_USELESS_RUNS //and not Probability dig 
 // goToDumpingSite();
 // leaveDumpingSite();
// #endif
}

// ********** END   {MODE DEFINITION} ----------


// ********** BEGIN (SUPPORT METHODS} **********


//----------------------------------------------------
bool checkHeadSensor(){
 
 if(goingIn){
 WDT_Restart(WDT);
 /* This method will give clearnace for digging mode if head sensor detects  */
  if ( (FSensor.Detected()) ){//am I detecting something in front of me with my head sensor? Is it a false positive? //goingIn condition is necessary because it could have been changed by contact switches
  enable_DiggingMode();
	return 1; //JSP
	unsigned long sensor_timer=millis(); //make sure that we detect above threshold reading for some time to make sure its not just a noise spike
   while( FSensor.Detected() ){
   Stop(); 
	 WDT_Restart(WDT);
    if( (millis()- sensor_timer)> 1000){//we saw this sensor spike for 2 seconds at least
    enable_DiggingMode();
    // HeadSensor.threshold=1000; //weaken the threshold for digging  //1023 is nominal reading
    return 1; //exit this mode, something is detected
    break; //breaks while loop
    }
   }
  // GetDetectedSigs(); //poll camera, get latest vision info
  }
  Forward(BASE_SPEED); //proceed forward slowly 
  return 0; //nothing is detected
 }
 
 if(diggingMode){
 WDT_Restart(WDT);
 // delay(150); //wait a little to make sure that the gripper has finished closing before sampling 
  if ( FSensor.Detected() ){//am I detecting something in front of me with my head sensor? Is it a false positive? //goingIn condition is necessary because it could have been changed by contact switches
  return 1;
	unsigned long sensor_timer=millis(); //make sure that we detect above threshold reading for some time to make sure its not just a noise spike
   while( FSensor.Detected() ){
	 WDT_Restart(WDT);
   Stop(); 
    if( (millis()- sensor_timer)> 750){//we saw this sensor spike for 2 seconds at least
    return 1; //exit this mode, something is detected
    break; //breaks while loop
    }
   }
  }
  return 0; //nothing is detected
 }
 
 if(goingCharging || restingMode){
 //check if this is getting called ever
 WDT_Restart(WDT);
  if( FSensor.Detected() && !CHARGER){//am I in the excavation area and touching GM?
  unsigned long sensor_timer=millis(); //make sure that we detect above threshold reading for some time to make sure its not just a noise spike
   while( FSensor.Detected() ){
   //Stop();   
    if( (millis()- sensor_timer)> 2000){//we saw this sensor spike for 2 seconds at least
    return 1;
	break;
	}
   }
  }
 return 0;
 }
 
 if(goingOut){//payload checking
  if(FSensor.Detected()){
  return 1; //we are carrying something, no problem
  }
  else{
  unsigned long sensor_timer=millis(); //used to very that we dont have anything by continuing sampling
   while( millis() - sensor_timer < 1000){
	 WDT_Restart(WDT);
    if(FSensor.Detected()){
	  return 1; //false alarm, there is something
	  }
   }
   return 0;//looks like we lost it
  } 
 } 
}


#define DIR_CHECK_TIME 500 //2500

/**
checkWrongDirections() is a method that takes in no parameters, and returns a boolean value of 0 or 1.

0 is returned if current_target_heading is either IN_DIRECTION or OUT_DIRECTION and
the robot is facing the wrong direction and dirCheckFlag is false. A side result is that dirCheckFlag is set to true,

1 is returned if IN_DIRECTION or OUT_DIRECTION and dirCheckFlag is true

Important parameters in this method:
dirCheckFlag
dirCheckTimer

Issues with this method:

I would not be surprise if this method will likely cause a silent error. isWantedHeading has a tolerance, and you need to make sure
that all of your direction headings are at least this tolerance apart.

Why not be a void return? The only times that the return is used in a boolean statement is to immediately repeat the line before, which is likely incorrect.

**/
bool checkWrongDirections(){
	/* checks if going in a wrong directions
	adjusts if needed and returns 1 */
	WDT_Restart(WDT);
	Serial.println("checkWrongDirections");
	if(current_target_heading == IN_DIRECTION){
		if( isWantedHeading(OUT_DIRECTION) || isWantedHeading(PORT_DIRECTION) || isWantedHeading(STARBOARD_DIRECTION)  ){ //wrong way
		// if( isWantedHeading(OUT_DIRECTION) ){ //wrong way
			if(!dirCheckFlag){ //flag is not set
				dirCheckFlag=true;
				dirCheckTimer=millis(); //start timer 
				// Serial.println("flag on");
				return 0; 
			}
			if(dirCheckFlag){
				if( millis() - dirCheckTimer > DIR_CHECK_TIME ){
					// Serial.println("turn");
					Stop(); delay(100); //hit the breaks
					//TurnHeading(IN_DIRECTION);
					enable_turnReversalMode(1);
					GetDetectedSigs(); //poll camera, get latest vision info
					dirCheckFlag=false;
					return 1;	 
				}
				
				else{ // you cannot ever get here
					// Serial.println("tick tock");
					return 0;
				}		
			}	 
		}
		// Serial.println("flag reset");
		dirCheckFlag=false;	
 } //ends IN_DIRECTION
 
 
 //---
	if(current_target_heading == OUT_DIRECTION){
		if( isWantedHeading(IN_DIRECTION) || isWantedHeading(PORT_DIRECTION) || isWantedHeading(STARBOARD_DIRECTION)  ){ //wrong way
		// if( isWantedHeading(IN_DIRECTION) ){ //wrong way
			
			
			if(!dirCheckFlag){ //flag is not set
				dirCheckFlag=true;
				dirCheckTimer=millis(); //start timer 
				// Serial.println("flag on");
				return 0; 
			}
			
			
			if(dirCheckFlag){
				if( millis() - dirCheckTimer > DIR_CHECK_TIME ){
					//Serial.println("turn");
					Stop(); delay(100); //hit the breaks
					//TurnHeading(OUT_DIRECTION);
					enable_turnReversalMode(3);
					GetDetectedSigs(); //poll camera, get latest vision info
					dirCheckFlag=false;
					return 1;	 
				}
				
				else{ // you cannot ever get to this else statement
					// Serial.println("tick tock");
					return 0;
				}		
			}	 
		}
		// Serial.println("flag reset");
		dirCheckFlag=false;	
	} //ends OUT_DIRECTION
}

void turnIMUon(){
	pinMode(IMUpower,OUTPUT); //put the pin in low impeadence
	digitalWrite(IMUpower,HIGH); //turn the pullup resistor on to source voltage 
}

void turnIMUoff(){
	// pinMode(IMUpower,OUTPUT);
	digitalWrite(IMUpower,LOW);
}

// void checkIMU(){
	// //commented function calls out. Looks like it does more bad than good
	// WDT_Restart(WDT);
	// /* this method checks if there is a connection with IMU. If not,
		// this method will make the robot attempt to re-establish connection */
	// //Rewrite this. have a variable in the IMU class which starts a timer if status is bad. then if the !=0x49D4 is seen for a long time, mash reset button. No more instantaneous crap
	// uint16_t IMUstatus = dof.checkStatus(); //write to whoAmI register and see if there is response
	// if( IMUstatus == 0x49D4){
		// return; //no problem, exit this method 
	// } 
	// Stop(); //hold on a second... something is not right with IMU
	// // char powerCycles=0;
	// while(IMUstatus != 0x49D4){

		// unsigned long statusTroubleStart=millis(); //grab time when this reading started
			// while( millis() - statusTroubleStart < 2000){//wait and hope that this problem is resolved
				// IMUstatus = dof.checkStatus(); //check status again 
				// if(IMUstatus == 0x49D4){
					// Forward(BASE_SPEED); //anti stuck kick
					// WDT_Restart(WDT);
					// return; //problem solved 
				// }
			// delay(100); 
		// }
		// turnIMUoff(); //turn the pin off 
		// delay(3000);
		// fioWrite(RESET_REQUEST); //WATCHDOG DOESNT SEEM TO RESET I2C
		// delay(3000);	
		
		// while(1){
			// //watchdog timer, do your thing 
		// }
		// /*  fioWrite(RESET_REQUEST);
		 // delay(3000);
		 // fioWrite(RESET_REQUEST); //resend for redundancy to make sure that the command was recieved. 
		 // delay(3000);
		 // fioWrite(RESET_REQUEST);
		 // delay(3000); */

		 // // arduinoReset(); //situation is hopeless. Mash reset button  
			// // Relay.PowerOff(); //turn the power off 
			// // delay(1000); //wait a little
			// // Wire.endTransmission();    // stop transmitting, force the bus to relax
			// // delay(3000);
			// // Relay.PowerOn(); //turn the power back on
			// // delay(1000);
			// // IMUstatus = dof.begin(); // attempt rebooting 
			// // delay(1000);
			// // powerCycles++; //record attempt 
		// } 
	// return; 
// }

//----------------------------------------------------
//----------------------------------------------------
void DumpPayload(){
	/* this robot will make the robot drop payload and shake its head once to make sure that GM is not stuck to the gripper */
	// Serial.print(goingIn); Serial.print(diggingMode); Serial.print(goingOut); Serial.print(dumpingMode); Serial.print(goingCharging); Serial.println(chargingMode); //debug
	WDT_Restart(WDT);
	Stop(); delay(1000); //stop Note: delay is used (not myDelay). we dont want to react to bumps
	Arm.PitchGo(MID_ROW_ANGLE); delay(100); 
	Arm.GripperGo(OPEN_POS);  delay(500); //straighten the arm out and release grip
	WDT_Restart(WDT);
	Arm.PitchGo(MID_ROW_ANGLE+10); delay(200); 
	Arm.PitchGo(MID_ROW_ANGLE-5); delay(200); //shake the arm
	WDT_Restart(WDT);
	Arm.PitchGo(MID_ROW_ANGLE+10); delay(200); //shake the arm
	Arm.PitchGo(MID_ROW_ANGLE-5); delay(200); //shake the arm
	WDT_Restart(WDT);
	Arm.PitchGo(MID_ROW_ANGLE+10); delay(200); 
	Arm.PitchGo(MID_ROW_ANGLE-5); delay(200); //shake the arm
	WDT_Restart(WDT);
	Arm.PitchGo(MID_ROW_ANGLE+10); delay(200); //shake the arm
	Arm.PitchGo(MID_ROW_ANGLE-5); delay(200); //shake the arm
	WDT_Restart(WDT);
	Arm.PitchGo(MID_ROW_ANGLE+10); delay(200); 
	Arm.PitchGo(MID_ROW_ANGLE-5); delay(200); //shake the arm
	WDT_Restart(WDT);
	Arm.PitchGo(MID_ROW_ANGLE+10); delay(200); //shake the arm
	Arm.PitchGo(MID_ROW_ANGLE); delay(200); //shake the arm
	WDT_Restart(WDT);
	Arm.GripperGo(CLOSED_POS);
	
	// Ross: recursive call within the DumpingPayload may not be the best idea? while loop with an iteration count?
	if (CheckPayload()){
		DumpPayload(); // in case the cohesive material got stuck inside gripper, do DumpPayload again
	}
	//Right(255); myDelay(2000); //make a u-turn. THIS SHOULD LATER BE REPLACED WITH A WHILE LOOP with camera feedback
	//Forward(100); myDelay(200); //start slowly driving forward, camera guiding algorithm should kick in
}
//----------------------------------------------------
bool CheckPower(){
/* this method enables robot to decide whether or not it should go charge itself  */
	float BatVoltage= Voltage.GrabAvg(100); //double check;
	float C=Current.ReadAvg(100); //debug;
	WDT_Restart(WDT);
	// Serial.print("C=  "); Serial.println(C); //debug 
	// Serial.print("V=  ");Serial.print(BatVoltage); Serial.print('\t');//debug
	if (BatVoltage < LOWEST_ALLOWABLE_VOLTAGE){
		if(checkPowerCount > CHECK_POWER_CNT_THRESH){
			checkPowerCount = 0;
			return 1;
		}
		else{
			checkPowerCount++;
			return 0;
		}
	}
	else {
		checkPowerCount=0;
		return 0;	
	}
}


/** this method will make the robot turn to a given angle bearing using
		magnetometer and gyroscope as a feedback
		user also passes in 0 if we want the robot to turn left, or 1 if turn right. Default is set to left   **/
void TurnHeadingRoss(float desired_heading){

	Serial.println("In TurnHeadingRoss()");
	
	
		turnIMUoff(); //turn the pin off 
		delay(1000);
		turnIMUon();
	

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TurnHeading Setup Process Start ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	WDT_Restart(WDT);
	
	fioWrite(MASTER_TURN_REVERSAL); // Write something to the LCD
	
	unsigned long watchdog2Timer=millis(); // 2 minute time out timer, exit out of here if the robot fails to turn in 2 minutes 
		
	int instructionChanges = 0; //initiate variable to keep track of how many 
	
	// Ross: is the following line still necessary? interrupt_mask_timer is only updated, but the value is not used
	// bool turning = true; //local flag to keep turning, really not needed anymore 
	unsigned long interrupt_mask_timer=millis(); //used to make sure switches interrupts are not serviced too fast
	
	dof.readMag(); //update magnetometer registers
	float current_heading = getHeading((float) dof.mx, (float) dof.my); //get a compass direction, value returned is from 0 to 360 degrees

	//autonomously pick the best direction to initiate turning
	turn_reversal_direction = pickDirection(current_heading, desired_heading); //choose direction for turn reversal, 0 for left turn, 1 for right turn
	unsigned long time_start = millis(); // initiate timer to keep robot turning if there is a turning progress
	unsigned long action_timeout = millis(); // watchdog timer, make sure that the robot is not stuck, forcing new actions if the robot have not done anything in a while 
	Arm.PitchGo(HIGH_ROW_ANGLE); // raise the arm to decrease overall length // JSP
	WDT_Restart(WDT);

	//initiate gyro integration vars here
	float angle = 0;
	float refRate = 0;
	unsigned long  gyroTime = millis();
	setGyroRef(&gyroTime,&refRate);
	
	diff_heading = desired_heading - current_heading;
	Serial.print("Initial diff_heading = ");
	Serial.println(diff_heading);
	
	
	do {
	
		// update heading differences
		dof.readMag(); //update magnetometer registers
		current_heading = getHeading((float) dof.mx, (float) dof.my);
		Serial.println("---------------------------------------------------------");
		Serial.print("desired_heading = "); 	Serial.println(desired_heading);
		Serial.print("current_heading = "); 	Serial.println(current_heading);
		// turn_reversal_direction = pickDirection(current_heading, desired_heading); //choose direction for turn reversal, 0 for left turn, 1 for right turn

		
		// move a tiny bit in the correct direction
		if(turn_reversal_direction){
			Serial.println("Turning left");
			Drive.RightForward(255);
			Drive.LeftForward(50);
		}
		
		else{
			Serial.println("Turning right");
			Drive.LeftForward(255);
			Drive.RightForward(50);
		}
		
		if(CONTACT){//if contact switches are pressed 
			Stop(); // Stop the motors
			WDT_Restart(WDT); // Reset the WDT
			handleContact(); // call handleContact() method to determine/execute appropriate response to contact
		} 
		// delay(50); Stop();
		WDT_Restart(WDT);

	} while (!isWantedHeading(desired_heading));
	Stop();
	delay(3000);
		
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TurnHeading Setup Process End ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	// while(!isWantedHeading(desired_heading)){ // replace diff_heading > 0 with isWantedHeading()?
		// Serial.println("---------------------------------------------------------");
		// Serial.print("desired_heading = "); 	Serial.println(desired_heading);
		// Serial.print("current_heading = "); 	Serial.println(current_heading);
		// Serial.print("diff_heading = "); 	Serial.println(diff_heading);

		
		
		// delay(50); Stop();
		// Serial.println("Turn complete");


		// // update heading differences
		// dof.readMag(); //update magnetometer registers
		// current_heading = getHeading((float) dof.mx, (float) dof.my);
		// diff_heading = desired_heading - current_heading;

		// WDT_Restart(WDT);
		// // delay(10);
	// }

	Serial.println("Exiting TurnHeadingRoss");
	return;
}














//----------------------------------------------------
	
/** this method will make the robot turn to a given angle bearing using
		magnetometer and gyroscope as a feedback
		user also passes in 0 if we want the robot to turn left, or 1 if turn right. Default is set to left   **/
void TurnHeading(float desired_heading){

	// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TurnHeading Setup Process Start ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// int numOfInstances = 0; // We will is numOfInstances to keep track of the number of consecutive times we compute that we aligned with desired_heading. This is a quick and dirty method to filter out random false positives. 
	WDT_Restart(WDT);
	
	fioWrite(MASTER_TURN_REVERSAL); // Write something to the LCD
	//checkIMU(); //check we are talking to IMU //Likely to be the cause for reset at exit tunnel // removed by JSP
	
	unsigned long watchdog2Timer=millis(); // 2 minute time out timer, exit out of here if the robot fails to turn in 2 minutes 
		
	int instructionChanges = 0; //initiate variable to keep track of how many 
	
	// Ross: is the following line still necessary? interrupt_mask_timer is only updated, but the value is not used
	// bool turning = true; //local flag to keep turning, really not needed anymore 
	unsigned long interrupt_mask_timer=millis(); //used to make sure switches interrupts are not serviced too fast
	
	dof.readMag(); //update magnetometer registers
	float current_heading = getHeading((float) dof.mx, (float) dof.my); //get a compass direction, value returned is from 0 to 360 degrees

	//autonomously pick the best direction to initiate turning
	turn_reversal_direction = pickDirection(current_heading, desired_heading); //choose direction for turn reversal, 0 for left turn, 1 for right turn
	unsigned long time_start = millis(); // initiate timer to keep robot turning if there is a turning progress
	unsigned long action_timeout = millis(); // watchdog timer, make sure that the robot is not stuck, forcing new actions if the robot have not done anything in a while 
	Arm.PitchGo(HIGH_ROW_ANGLE); // raise the arm to decrease overall length // JSP
	WDT_Restart(WDT);

	//initiate gyro integration vars here
	float angle = 0;
	float refRate = 0;
	unsigned long  gyroTime = millis();
	setGyroRef(&gyroTime,&refRate);
	
		// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TurnHeading Setup Process End ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	bool turning = true; //local flag to keep turning, really not needed anymore
	int numOfInstances = 0; // We will is numOfInstances to keep track of the number of consecutive times we compute that we aligned with desired_heading. This is a quick and dirty method to filter out random false positives. 
	int requiredNumOfInstances = 1;
	
	while(turning){ // traps the execution here while turning is still true
	
		//---make sure that the gyro is still working
		WDT_Restart(WDT); // repeat this so that the board is not reset in the middle of a turn
		
		//checkIMU(); //likely to be the cause for reset at exit tunnel // removed by JSP

		//---check if we are facing correct direction
		// isWantedHeading returns true if we are indeed facing the correct direction (right?)
		if(isWantedHeading(desired_heading) && !preferGyro ){ //check if the robot is facing desired heading direction
			numOfInstances++;
			Serial.println("Not PreferGyro");
			if(numOfInstances >= requiredNumOfInstances){//5 before //make sure there is at least this many samples
				WDT_Restart(WDT);
				turning = false;
				// Stop(); delay(100); //debugging stop 
				Forward(BASE_SPEED); //anti stuck 
				Arm.PitchGo(HIGH_ROW_ANGLE); //straighten out 
				preferGyro = false; //invoke default state // not sure why this line is here, because you can't get to this line if preferGyro is true...
				break; //escape while(turning) loop
			}
		}
		
		else{
			numOfInstances = 0; // is this right? if preferGyro is true, then numOfInstances will always be set to 0
		}
 
		if(preferGyro){ //duct tape solution to do 180 degree turns
			Serial.println("PreferGyro");
			bool output = countGyro(&angle,&gyroTime,&refRate);
			Serial.println(output);
				if(output){
					WDT_Restart(WDT);
					turning = false;
					Stop(); delay(100); //debugging stop 
					Forward(BASE_SPEED); //anti stuck 
					Arm.PitchGo(HIGH_ROW_ANGLE);
					preferGyro = true; //invoke default state --- is this right? we cannot get here without preferGyro being true
					break; //escape while(turning) loop
				}
 
		}
		
		//////temporarily suppressed this
		// //---check if this method has been running for too long 
		// if( (millis() - watchdog2Timer) > 300000){ //five minute timeout 
			// fioWrite(RESET_REQUEST);
			// delay(2000);
			// arduinoReset(); //desperate measure
			// return; //and hope for the best 
		// } 

		//---exception handling for charging mode: function return 
		// if(goingCharging){ //check if charging beacon is seen, or desired charger is touched
			// if( CHARGER){//bumped into charging station by accident. potential bug if the robot ends up in a funny orientation 
				// WDT_Restart(WDT);
				// Stop(); delay(100); //hit the breaks. potentially a bug since this will not set appropriate flags in goingCharging mode. avoiding using global vars
				// //Relay.PowerOff(); //kill the power, software will eventually kick in Charing Mode
				// Arm.PitchGo(MID_ROW_ANGLE); //return arm to a horizontal position 
				// preferGyro=false; //invoke default state
				// return; //exit function   
			// }
		// }
		
		if(restingMode){
		//if(DUMPING_SWITCH){
			if(CHARGER){                  //BANI
				WDT_Restart(WDT);
				Stop(); delay(100);
				Arm.PitchGo(HIGH_ROW_ANGLE);//maintain high arm position
				preferGyro=false;
				//invoke default state
				//bool goBack = false; //BANI I THINK THIS WILL START ELSE STATEMENT IN 
				return;
			}
		}
 
	// if(goingOut){
		// if(DUMPING_SWITCH){
			// WDT_Restart(WDT);
			// Stop(); delay(100); //hit the breaks
			// Arm.PitchGo(MID_ROW_ANGLE); //straighten out 
			// preferGyro=true; //invoke default state
			// return;
		// }
	// }

		//--- execute a turning command
		switch (turn_reversal_direction){
			WDT_Restart(WDT); 
			/* right and left turns used to have 155 pwm input. changed to 255 for better traction */
			case false:
				//Left(DEFAULT_TURNING_SPEED);//try turning left
				Drive.LeftForward(255);
				Drive.RightForward(50);
				delay(500); Stop();
				break;

			case true:
				//Right(DEFAULT_TURNING_SPEED);//try turning right
				Drive.RightForward(255);
				Drive.LeftForward(50);
				delay(500); Stop();
				break;
 
		 /*
			case 4:
				Drive.RightBackward(255);
				Drive.LeftBackward(75);
				break;
		 
			case 5:
				Drive.RightForward(75);
				Drive.LeftForward(255);
				break;
		 
			case 6:
				Drive.RightBackward(75);
				Drive.LeftBackward(255);
				break;
		 
			case 7:
				Drive.RightForward(255);
				Drive.LeftForward(75);
				break;*/
		}
 
		//--- handle physical bumping
		if(CONTACT){//if contact switches are pressed 
			WDT_Restart(WDT);
	
			//if ( (millis()-interrupt_mask_timer) > 5000 ){ //if not a recent contact
			handleContact();
			instructionChanges++; //increase counter 
			interrupt_mask_timer=millis(); //reset timer
			time_start=millis(); //reset timeout timer, let the robot keep on turning
			action_timeout=millis(); //reset watchdog timer 
			// continue; //go toward next loop iteration so that new turning direction can be executed 
		//} 
		} 
		
		if(DUMPING_SWITCH){
			WDT_Restart(WDT);
			Stop(); delay(100); //pressing against something
			Backward(BASE_SPEED); delay(1500);
			bumpDelay(1000);
			turn_reversal_direction=0; //reset turning instruction  
			Left(DEFAULT_TURNING_SPEED); //and execute this turning instruction right away
			instructionChanges++; //increase counter. maybe I should be resetting here
		}
 
		//---check if the robot is making progress with gyroscope 
		dof.readGyro(); //update gyro registers
		if( abs( dof.calcGyro(dof.gz) ) >= SLOW_TURNING_RATE ){ //if robot is making progress turning
			time_start=millis(); //reset timeout timer, let the robot keep on turning
		}
		
		if( ( millis()-time_start) > TURNING_TIMEOUT ){ //gyro does not detect turning, switch turning command instruction
			instructionChanges++; //increase counter
	
/*
			switch(turning_case){
				case 0:
					turning_case=7;
					Backward(BASE_SPEED); delay(1000);
					WDT_Restart(WDT);
					break;
					case 1:
					turning_case=5;
					Backward(BASE_SPEED); delay(1000);
					WDT_Restart(WDT);
					break;
  
			case 5:
				turning_case=4;
				WDT_Restart(WDT);
				break;
				case 4:
				turning_case=5;
				Backward(BASE_SPEED); delay(1000);
				WDT_Restart(WDT);
				break;

			case 7:
				turning_case=6;
				WDT_Restart(WDT);
				break;
				case 6:
				turning_case=7;
				Backward(BASE_SPEED); delay(1000);
				WDT_Restart(WDT);
				break;  
		}*/
			time_start=millis();//reset timeout timer
			action_timeout=millis(); //reset watchdog timer 
 //continue; //go toward next loop iteration so that new turning direction can be executed
		}

		// --- enforce a change in direction at least every X seconds 
		if( (millis()-action_timeout)>10000 ){//gyro may be going of from vibrations, robot is not going anywhere
			WDT_Restart(WDT);
			instructionChanges++; //increase counter 
			turn_reversal_direction = !turn_reversal_direction; //reverse turning direction from left to right, and vice versa
  /*
			switch(turning_case){ //change turning instruction 
				case 0:
				turning_case=7;
				Backward(BASE_SPEED); delay(1000);
				WDT_Restart(WDT);
				break;
				case 1:
				turning_case=5;
				Backward(BASE_SPEED); delay(1000);
				WDT_Restart(WDT);
				break;
  
				case 5:
					turning_case=4;
					WDT_Restart(WDT);
					break;
					case 4:
					turning_case=5;
					Backward(BASE_SPEED); delay(1000);
					WDT_Restart(WDT);
					break;

				case 7:
					turning_case=6;
					WDT_Restart(WDT);
					break;
					case 6:
					turning_case=7;
					Backward(BASE_SPEED); delay(1000);
					WDT_Restart(WDT);
					break;  
				}*/
	
			time_start=millis();//reset timeout timer
			action_timeout=millis(); //reset watchdog timer 
			WDT_Restart(WDT);
		}

		//--- desperate measures 
		if(instructionChanges > 10){//10 //40 previously //the robot is probably stuck somewhere hopelessly //15
			Stop();
			while(1){
				//should trigger watchdog reset  since WDT_Restart(WDT) is not called
			}
		}
	
	WDT_Restart(WDT);
	
	}//end while(turning)

	Arm.PitchGo(HIGH_ROW_ANGLE); //maintain high angle position of the gripper 

	//Determines the next mode in which the robot will go in, and set it accordingly
	switch(nextMode){
		case 1:
			enable_GoingInMode(); //1 denotes the robot will go into going_in mode after turn reversal
			break;

		case 2:
			enable_DiggingMode();//2 denotes the robot will go into digging mode after turn reversal //never used
			break;

		case 3:
			enable_GoingOutMode();//3 denotes the robot will go into going_out mode after turn reversal
			break;

		case 4:
			enable_DumpingMode();//4 denotes the robot will go into dumping mode after turn reversal //never used
			break;

		case 5:
			enable_GoingCharging();//5 denotes the robot will go into going_charging mode after turn reversal
			break;

		case 6:
			enable_RestingMode();//6 denotes the robot will go into resting mode after turn reversal //never used
			break;

		case 7:
			enable_exitTunnelMode();//7 denotes the robot will go into exit tunnel mode after turn reversal //never used
			break;

		case 10:
			//does not change the mode
			Serial.println(F("Case 10 used - mode not updated"));
			break;

		default:
			//does not change the mode
			Serial.println(F("Default used - mode not updated"));
			break;
	}

	GetDetectedSigs(); //poll camera, get latest vision info
	FollowLane();//poll camera and call PDcheckWrongDirections
	//return;
}


//----------------------------------------------------
bool isWantedHeading(float desired_heading){
	/* this method accepts a desired heading angle and returns a boolean variable
	whether or not the robot is facing in desired angle, give or take 
	a pre-defined tolerance (+- DIRECTION_UNCERTAINTY). This method grabs a latest magnetometer data
	for comparison purposes
	IT IS ASSUMED THAT DIRECTION UNCERTAINTY IS SMALL, meaning cant have low bound <0 and high bound >360 at the same time
	tested with fake data and it works. 11/5/2014
	*/
	//get heading
	WDT_Restart(WDT);
	dof.readMag(); //update magnetometer registers
	float heading=getHeading((float) dof.mx, (float) dof.my);
	float lowBound = desired_heading - DIRECTION_UNCERTAINTY;
	float highBound = desired_heading + DIRECTION_UNCERTAINTY; 

	if (lowBound >= 0 && highBound <= 360 ){//case without wrap-around singularities
		if( heading >= lowBound && heading <= highBound){
			return true;
		}
		else{
			return false;
		} 
	}

	if (highBound > 360){ //such as desired 355+- 10 degrees
		if( heading >= lowBound && heading <= 360){
			return true;
		}
		if( heading >=0 && heading <= highBound-360){
			return true;
		}
		else{
			return false;
		}
	}

	if(lowBound < 0){ //such as 5+-10 degrees
		if( heading >=0 && heading <= highBound){
			return true;
		}
		if( heading-360 >=lowBound && heading-360 <=0) {
			return true;
		}
		else{
			return false;
		}
	}
	WDT_Restart(WDT);
}

bool pickDirection(float current_heading, float desired_heading){
	/* this method will autonomously pick the best direction to turn based 
	on angular distance */
	//flipped original 0's to 1's
	WDT_Restart(WDT);
	bool turning_case;
	if( (current_heading) < 180 ){
		if ( (desired_heading)<180 ) {
			if( desired_heading < current_heading){
				turning_case=0;
				return turning_case;
			}
			else{
				turning_case=1; return turning_case;
			}
		}
		if ( (desired_heading)>180 ) {
			if (desired_heading > (180 + current_heading) ){
				turning_case=0; return turning_case;
			}
			else{
				turning_case=1; return turning_case;
			}
		}  
	}
	if( (current_heading) > 180 ){
		if ( (desired_heading)<180 ) { //check this <
			if( (current_heading-desired_heading) >180 ){
				turning_case=1; return turning_case;
			}
			else{
				turning_case=0;
				return turning_case;
			}
		}
		if ( (desired_heading)>180 ) {
			if (desired_heading > ( current_heading) ){
				turning_case=1; return turning_case;
			}
			else{ turning_case=0;
				return turning_case;
			}
		}   
	}
}

//----------------------------------------------------
float getHeading(float hx, float hy){
/* this method grabs magnetometer data from gyroscope and returns a bearing angle
the bearing angle goes from 0 to 360 degrees and wraps around back to 0
call this method like so
 dof.readMag(); //update gyro registers
 int heading=getHeading((float) dof.mx, (float) dof.my);

note from the original library:
 [this method] only works if the sensor is flat (z-axis normal to Earth).
 Additionally, you may need to add or subtract a declination
 angle to get the heading normalized to your location.
 See: http://www.ngdc.noaa.gov/geomag/declination.shtml
  */
  float heading;
  
  if (hy > 0)
  {
    heading =  270 - (atan(hx / hy) * (180 / PI));
  
}
  else if (hy < 0)
  {
    // heading = - (atan(hx / hy) * (180 / PI));
	heading=  90  - (atan(hx / hy) * (180 / PI));
  }
  else // hy = 0
  {
    if (hx < 0) heading = 360;
    else heading = 180;
  }
  return heading;
}

//----------------------------------------------------
// float getGyroZ(){
// //pass in gyro bias and substract it
// // return filtered gyro
// //maybe even saturate to zero 
// }
//----------------------------------------------------



void handleContact(){
	WDT_Restart(WDT);
	//this is the simplified, "back out" method

	unsigned long start_of_contact=millis(); //record length of contact
	// logContacts(start_of_contact); //stick this before every return
	
	/*
	//if disable flag is set a long time ago
	if(disableContacts){
		if( millis()- whenDisabledContacts >= CONTACT_RESET_TIME){
			disableContacts=false; //reset the flag
		}
		else if (millis()- whenDisabledContacts < CONTACT_RESET_TIME){ //if set recently, take no action, switches are disabled 
			return;//exit and do nothing
		}
	}
	*/

	//Recording Contact Information to Arduino Fio
	//if(goingIn || diggingMode){ //bani THIS LOOP IS FOR SENDING BYTES TO FIO FOR LOGGING THE CONTACTS...added digging mode temporarily
	RFC=switchState & FR_ANT;
	LFC=switchState & FL_ANT;
	RSBC=switchState & RSB_ANT;
	RSFC=switchState & RSF_ANT;
	LSBC=switchState & LSB_ANT;
	LSFC=switchState & LSF_ANT;
	LBC=switchState & BL_ANT;
	RBC=switchState & BR_ANT;
	FS=RFC|LFC;
	RS=RSBC|RSFC;
	LS=LSBC|LSFC;
	BS=LBC|RBC;

	if (FS!=0b0000000000000000){
		if ((FS & SWITCH_ANT_MASK) != 0b0000000000000000){  //is the contact ant or wall?
			// the contact is ant
			#ifdef FIO_LINK
			fioWrite(FRONT_SIDE_ANT);
			#endif
		}
		
		else if ((FS & SWITCH_ANT_MASK) == 0b0000000000000000){
			// the contact is wall
			#ifdef FIO_LINK
				fioWrite(FRONT_SIDE_WALL);
			#endif
		}
	}

	if (RS!=0b0000000000000000){
		if ((RS & SWITCH_ANT_MASK) != 0b0000000000000000){  //is the contact ant or wall?
			// the contact is ant
			#ifdef FIO_LINK
				fioWrite(RIGHT_SIDE_ANT);
			#endif
		}
		
		else{
			// the contact is wall
			#ifdef FIO_LINK
				fioWrite(RIGHT_SIDE_WALL);
			#endif
		}
	}

	if (LS!=0b0000000000000000){
		if ((LS & SWITCH_ANT_MASK) != 0b0000000000000000){  //is the contact ant or wall?
			// the contact is ant
			#ifdef FIO_LINK
				fioWrite(LEFT_SIDE_ANT);
			#endif
		}
		else{
			// the contact is wall
			#ifdef FIO_LINK
				fioWrite(LEFT_SIDE_WALL);
			#endif
		}
	}

	if (BS!=0b0000000000000000){
		if ((BS & SWITCH_ANT_MASK) != 0b0000000000000000){  //is the contact ant or wall?
			// the contact is ant
			#ifdef FIO_LINK
				fioWrite(BACK_SIDE_ANT);
			#endif
		}
		else{
			// the contact is wall
			#ifdef FIO_LINK
				fioWrite(BACK_SIDE_WALL);
			#endif
		}
	}
//}


//------------------------------

	//Contact Response
	if(goingIn || goingOut || goingCharging){//if roaming around
		switch(switchState & SWITCH_WALL_MASK){
			case FL:
				//Serial.println("FL");
				Backward(BASE_SPEED);
				delay(500);
				Drive.LeftForward(255);
				Drive.RightForward(75);
				delay(500);
				//Drive.LeftForward(75);
				//Drive.RightForward(255);
				//delay(500);
				
				break;
			case FR:
				//Serial.println("FR");
				Backward(BASE_SPEED);
				delay(500);
				Drive.RightForward(255);
				Drive.LeftForward(75);
				delay(500);
				//Drive.RightForward(75);
				//Drive.LeftForward(255);
				//delay(500);
				
				break; 
			case (FL | FR):
				//Serial.println("Front Both");
				Backward(BASE_SPEED);
				delay(500);
				Drive.RightForward(255);
				Drive.LeftForward(75);
				delay(500);
				Drive.RightForward(75);
				Drive.LeftForward(255);
				delay(500);
				break;
	
			case RSF: //Right side is hit
			case RSB:
			case (RSF | RSB) :
				Drive.RightForward(255);
				Drive.LeftForward(75);
				// if(goingIn){
				// #ifdef FIO_LINK
				// fioWrite(RIGHT_SIDE); //report going out
				// #endif 
				// }
				break;
  
			case LSF: //Left side is hit
			case LSB:
			case (LSF | LSB) :
				Drive.RightForward(75);
				Drive.LeftForward(255);
				// if(goingIn){
				// #ifdef FIO_LINK
				// fioWrite(LEFT_SIDE); //report going out
				// #endif
				// }
				break;
  
			case (FL | FR | BL | BR)://robot is squished from both sides
			case (FR | BL | BR):
			case (FL | BL | BR):
			case (FL | FR | BR):
			case (FL | FR | BL):
			case (FL | BL):
			case (FL | BR):
			case (FR | BL):
			case (FR | BR):
			case (RSF | RSB | LSF | LSB): 
			case (RSF | LSF):
			case (RSB | LSB):
			case (RSF | RSB | LSF):
			case (RSF | RSB | LSB):
			case (LSF | LSB | RSF):  
			case (LSF | LSB | RSB):  
				// case 
				Stop(); delay(1000);
				WDT_Restart(WDT);
				break;
 
			case BR: //robot is bumped on the back
			case BL:
			case (BR | BL):
				Forward(BASE_SPEED+20); //kick forward
				delay(500); //kick delay
				WDT_Restart(WDT);
				// if(goingIn){
				// #ifdef FIO_LINK
				// fioWrite(BACK); //report going out
				// #endif
				// }
				break;

			default:
				//do nothing 
				break; 
		}
		logContacts(start_of_contact); return;
	}
	//------------------------------
	else if(diggingMode){
	int currentDriveState=lastDriveState; //grab current state
		switch(switchState & SWITCH_WALL_MASK){
			case BR: //back 
			case BL:
			case (BR | BL):
			case (BR | RSB): //including back side 
			case (BL | LSB):
			case (BR | BL | RSB | LSB):
				Stop(); delay(1000);//DIGGING_INTERRUPT_DELAY //this will affect lastDriveState, thats why we use another variable
  //switch(currentDriveState){
  //case drivingForward:
  //Forward(BASE_SPEED); //can add speed memory as well
  //break;
  //case drivingBackward:
  //Backward(BASE_SPEED);
  //break;
  //case turningRight:
  //Right(BASE_SPEED);
  //break;
  //case turningLeft:
  //Left(BASE_SPEED);
  //break;
  //case stopped:
  //Stop();
  //break;
  //}
			break;
		}
	logContacts(start_of_contact); return; 
	}
	//------------------------------
	else if(dumpingMode){
	// do nothing, actually its not even called 
	logContacts(start_of_contact); return;  //NEW LINE
	}
	else if(chargingMode){
	// int currentDriveState=lastDriveState; //grab current state
	 // switch(switchState){
	 // // case RB: //back 
	 // // case LB:
	 // // case (RB | LB):
	 // // case (RB | RSB): //including back side 
	 // // case (LB | LSB):
	 // // case (RB | LB | RSB | LSB):
	 // default : 
	 // Stop(); delay(CHARGINGMODE_INTERRUPT_DELAY); //this will affect lastDriveState, thats why we use another variable
		// switch(currentDriveState){
		// // case drivingForward:
		// // Forward(200); //can add speed memory as well
		// // break;
		// case drivingBackward:
		// Backward(200);
		// break;

		// case stopped:
		// Stop();
		// break;
		// }
	 // break;
	 // }
		logContacts(start_of_contact); return; 
	}

	else if(exitTunnelMode){
		switch(switchState & SWITCH_WALL_MASK){
			case FL:
				Stop(); delay(100);
				Backward(BASE_SPEED); delay(100);
				Left(DEFAULT_TURNING_SPEED); delay(100);
				Stop();
				break;
			case FR:
				Stop(); delay(100);
				Backward(BASE_SPEED); delay(100);
				Right(DEFAULT_TURNING_SPEED); delay(100);
				Stop();
				break;
			case BR: //something hit back
				Stop(); delay(100);
				Forward(BASE_SPEED); delay(100); 
				Right(DEFAULT_TURNING_SPEED); delay(100);
				Stop();
			 //TurnHeading(IN_DIRECTION); //reorient
			 Stop(); delay(3000);
			 Backward(BASE_SPEED);
			 delay(500); //force new action 
			 break;
			case BL:
				Stop(); delay(100);
				Forward(BASE_SPEED); delay(100); 
				Left(DEFAULT_TURNING_SPEED); delay(100);
				Stop();
				//TurnHeading(IN_DIRECTION); //reorient
				Stop(); delay(3000);
				Backward(BASE_SPEED);
				delay(500); //force new action
				break;
			case BR | BL:   
				Stop(); delay(100);
				Forward(BASE_SPEED); delay(100); 
				Stop();
				//TurnHeading(IN_DIRECTION); //reorient
				Stop(); delay(3000);
				Backward(BASE_SPEED);
				delay(500); //force new action 
				break;
			}
		}

		else if (turnReversalMode){
			if (!turn_reversal_direction){ //0 for left, 1 for right
				//initial turning direction is left
				switch(switchState & SWITCH_WALL_MASK){
 
					case FL:
					case FR:
					case (FL | FR):
						Backward(BASE_SPEED);
						delay(100);
						//Drive.RightBackward(75);
						Drive.LeftBackward(255);
						delay(500);
						break;
	
					case RSF: //Right side is hit
					case RSB:
					case (RSF | RSB) :
						//ignore contact
						break;
  
					case LSF: //Left side is hit
					case LSB:
					case (LSF | LSB) :
						//ignore contact
						break;
  
					case (FL | FR | BL | BR)://robot is squished from both sides
					case (FR | BL | BR):
					case (FL | BL | BR):
					case (FL | FR | BR):
					case (FL | FR | BL):
					case (FL | BL):
					case (FL | BR):
					case (FR | BL):
					case (FR | BR):
					case (RSF | RSB | LSF | LSB): 
					case (RSF | LSF):
					case (RSB | LSB):
					case (RSF | RSB | LSF):
					case (RSF | RSB | LSB):
					case (LSF | LSB | RSF):  
					case (LSF | LSB | RSB):  
						// case 
						Stop(); delay(1000);
						WDT_Restart(WDT);
						break;
 
					case BR: //robot is bumped on the back
					case BL:
					case (BR | BL):
						Forward(255); //kick forward
						delay(100); //kick delay
						Drive.RightForward(255);
						delay(500);
						//Drive.LeftForward(75);
						// if(goingIn){
						// #ifdef FIO_LINK
						// fioWrite(BACK); //report going out
						// #endif
						// }
						break;

					default:
						//do nothing 
						break; 
				}
			}
			
			
			
			else{
				//initial turning direction is right
				switch(switchState & SWITCH_WALL_MASK){
 
					case FL:
					case FR:
					case (FL | FR):
						Backward(BASE_SPEED);
						delay(100);
						Drive.RightBackward(255);
						//Drive.LeftBackward(75);
						delay(500);
						break;
	
					case RSF: //Right side is hit
					case RSB:
					case (RSF | RSB) :
						//ignore contact
						break;
  
					case LSF: //Left side is hit
					case LSB:
					case (LSF | LSB) :
						//ignore contact
						break;
		
					case (FL | FR | BL | BR)://robot is squished from both sides
					case (FR | BL | BR):
					case (FL | BL | BR):
					case (FL | FR | BR):
					case (FL | FR | BL):
					case (FL | BL):
					case (FL | BR):
					case (FR | BL):
					case (FR | BR):
					case (RSF | RSB | LSF | LSB): 
					case (RSF | LSF):
					case (RSB | LSB):
					case (RSF | RSB | LSF):
					case (RSF | RSB | LSB):
					case (LSF | LSB | RSF):  
					case (LSF | LSB | RSB):  
						// case 
						Stop(); delay(1000);
						WDT_Restart(WDT);
						break;
 
				case BR: //robot is bumped on the back
				case BL:
				case (BR | BL):
					Forward(255); //kick forward
					delay(100); //kick delay
					//Drive.RightForward(75);
					Drive.LeftForward(255);
					delay(500);
					// if(goingIn){
					// #ifdef FIO_LINK
					// fioWrite(BACK); //report going out
					// #endif
					// }
					break;

				default:
					//do nothing 
					break; 
			}
		}
	}
	logContacts(start_of_contact); 
	switchState = 0; //reset switchState
	return;  //default return
}


int handleTurningContact(int turning_case){
/* this method will change current turning_case turning direction,
log the start of the contact, */

// Serial.print("case \t");
// Serial.print(turning_case);
// Serial.print("switches \t");
// Serial.println(switchState);

/* case 0: initially started with left  turn
   case 1: initially started with right turn  
   case 3: simple backing out routine 
   
   case 4: back left
   case 5: front right
   case 6: back right
   case 7: front left 
   
   delay() should be used here instead of myDelay to avoid program getting stuck
   */
 
 
 //if left is pressed,  move 5
 //if right is pressed, move 7

 //can embed anti jamming kick in a if(!Headon), but it must not be masked

 unsigned long start_of_contact=millis(); //record length of contact
/* // logContacts(start_of_contact); //stick this before every return */
 
 if(disableContacts){//if disable flag is set. 
 if( millis()- whenDisabledContacts >= CONTACT_RESET_TIME){
 disableContacts=false; //reset the flag
 }
 else if (millis()- whenDisabledContacts < CONTACT_RESET_TIME)  {
 return turning_case;//exit and do nothing
 }
WDT_Restart(WDT);
}


 switch(turning_case){
 case 0:
 case 1:
  switch(switchState){
  case LSF: //if bumped anywhere on a left side
  case LSB:
  case(LSF | LSB):
  turning_case=5;
  logContacts(start_of_contact);
  // return turning_case;
  break;
  
  case RSF: //if bumped anywhere on a right side
  case RSB:
  case(RSF | RSB):
  turning_case=7; 
  logContacts(start_of_contact);
  // return turning_case;
  break;
  
  default:
  logContacts(start_of_contact);
  // return turning_case;
  break;
  }
 break;

 case 5:
  switch(switchState){
  case LSF: //if bumped anywhere on a left side
  case LSB:
  case(LSF | LSB):
  turning_case=5; 
  logContacts(start_of_contact);
  // return turning_case; //do nothing
  break;
  
  case RSF: //if bumped anywhere on a right side
  case RSB:
  case(RSF | RSB):
  turning_case=7;
  logContacts(start_of_contact);
  // return turning_case;
  break;

  case (BR | BL):
  turning_case=5; 
	WDT_Restart(WDT);
  Forward(255); delay(300); //anti-jamming kick
  Stop(); delay(50);
  logContacts(start_of_contact);
  // return turning_case;
  break;
  
  default:
  logContacts(start_of_contact);
  // return turning_case;
  break;
  }
 break;

 case 7:
  switch(switchState){
  case LSF: //if bumped anywhere on a left side
  case LSB:
  case(LSF | LSB):
  turning_case=5; //mistake, i think, shouldnt be 5
  logContacts(start_of_contact);
  // return turning_case;
  break;
  
  case RSF: //if bumped anywhere on a right side
  case RSB:
  case(RSF | RSB):
  turning_case=7;
  logContacts(start_of_contact);
  // return turning_case; //do nothing
  break;
  
  case (BR | BL):
  turning_case=7; 
  Forward(255); delay(300); //anti-jamming kick
  Stop(); delay(50);
  logContacts(start_of_contact);
  // return turning_case;
  break;
   
  default:
  // Stop(); delay(200);
  logContacts(start_of_contact);
  // return turning_case;
  break;
  }
 break;
//-----

 case 4:
  switch(switchState){
  case LSF: //if bumped anywhere on a left side
  case LSB:
  case(LSF | LSB):
  turning_case=5; 
  logContacts(start_of_contact);
  // return turning_case; 
  break;
  
  case RSF: //if bumped anywhere on a right side
  case RSB:
  case(RSF | RSB):
  turning_case=6; 
  logContacts(start_of_contact);
  // return turning_case;
  break;
  
  case BR:
  case BL:
  case (BR | BL):
  turning_case=5;
  Forward(255); delay(300); //anti-jamming kick
  WDT_Restart(WDT);
	Stop(); delay(50);
  logContacts(start_of_contact);
  // return turning_case;
  break;
  
  default:
  logContacts(start_of_contact);
  // return turning_case;
  break;
  }
 break;

 case 6:
  switch(switchState){
  case LSF: //if bumped anywhere on a left side
  case LSB:
  case(LSF | LSB):
  turning_case=4;
  logContacts(start_of_contact);
  // return turning_case;
  break;
  
  case RSF: //if bumped anywhere on a right side
  case RSB:
  case(RSF | RSB):
  turning_case=7;
  logContacts(start_of_contact);
  // return turning_case;
  break;
  
  case BR:
  case BL:
  case (BR | BL):
  turning_case=7;
  WDT_Restart(WDT);
	Forward(255); delay(300); //anti-jamming kick
  Stop(); delay(50);
  logContacts(start_of_contact);
  // return turning_case;
  break;
   
  default:
  logContacts(start_of_contact);
  // return turning_case;
  break;
  }
 break; 
 }
return turning_case;//exit  
}

// ********** END   {SUPPORT METHODS} ----------

// ********** BEGIN {INTERRUPT ROUTINES} **********
// ********** END   {INTERRUPT ROUTINES} ----------

// ********** BEGIN {TEST AND DEBUG} **********
//#ifdef TEST_MODE
void TestDriveMotors(){
	// captures the program here and repeats
	while(1){
		WDT_Restart(WDT);
		Serial.println(F("Forward"));
		Forward(BASE_SPEED); delay(2000);
		Stop(); delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Backward"));
		Backward(BASE_SPEED); delay(2000);
		Stop(); delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Right"));
		Right(DEFAULT_TURNING_SPEED); delay(2000);
		Stop(); delay(1000);

		WDT_Restart(WDT);
		Serial.println(F("Left"));     
		Left(DEFAULT_TURNING_SPEED); delay(2000); 
		Stop(); delay(1000);

		}
}

void TestPDController(){
	while(1){
		WDT_Restart(WDT);
		GetDetectedSigs(); //poll camera
		if(TRAIL1){
			Input=x1; //grab input
			PD.Compute(); //updates Rout
			Serial.print("Out: \t");
			Serial.println(Output);
		}
	}
}


void TestServoMotors(){
	WDT_Restart(WDT);
	Serial.println(F("Opening gripper"));
	Arm.GripperGo(OPEN_POS); delay(1000);
	Serial.println(F("Closing gripper"));
	Arm.GripperGo(MID_POS); delay(1000);
	Arm.GripperGo(CLOSED_POS);delay(1000);
	Serial.println(F("Raising arm"));
	WDT_Restart(WDT);
	Arm.PitchGo(HIGH_ROW_ANGLE); delay(1500);
	Serial.println(F("Lowering arm"));
	Arm.PitchGo(LOW_ROW_ANGLE); delay(1500); //Arm.PitchGo(100); //JSP Delete
}




void TestCamera(){
	Serial.println("In TestCamera()...");

	while(1){
		Serial.println("In while-loop");

		WDT_Restart(WDT);
		GetDetectedSigs();
		static int i = 0;
		int j;
		uint16_t blocks;
		char buf[32]; 
		
		blocks = pixy.getBlocks();

		//Serial.println(blocks);// debug
		if (blocks){
			i++;
			
			if (i%50 == 0){
				sprintf(buf, "Detected %d:\n", blocks);
				Serial.print(buf);
				for (j=0; j<blocks; j++){
					sprintf(buf, "  block %d: ", j);
					Serial.print(buf); 
					pixy.blocks[j].print();
				}
			}
		}  
	}
}


void TestIMU(){
	while(1){
		WDT_Restart(WDT);

		 //update magnetometer registers
		dof.readMag();
		float heading=getHeading((float) dof.mx, (float) dof.my);

	 // print the magnetometer data to serial
		// Serial.print(F("mx  "));
		// Serial.print(dof.mx);
		// Serial.print(F(" my "));
		// Serial.print(dof.my);
		Serial.print(F(" H "));
		Serial.println(heading);

		// update the gyro registers
		dof.readGyro();
		float gyroZ=dof.calcGyro(dof.gz);
		delay(1000);
		
		// Print the gyro data to serial
		// Serial.print(F(" G  "));
		// Serial.println(gyroZ);
	}
}
void TestGyro(){
	WDT_Restart(WDT);
	Serial.print(F("G  "));
	dof.readGyro(); //update gyro registers
	float gyroZ=dof.calcGyro(dof.gz);
	Serial.println(gyroZ);
}

void TestSwitches(){
	WDT_Restart(WDT);
	Serial.println(F("Testing Switches Cycle"));
	if (CONTACT){
		Serial.println(F("Contact: TRUE"));
		RFC = switchState & FR;
		LFC = switchState & FL;
		RSBC = switchState & RSB;
		RSFC = switchState & RSF;
		LSBC = switchState & LSB;
		LSFC = switchState & LSF;
		LBC = switchState & BL;
		RBC = switchState & BR;
		FS = RFC|LFC;
		RS = RSBC|RSFC;
		LS = LSBC|LSFC;
		BS = LBC|RBC;
		//Serial.print(switchState,HEX);//BANI COMMENTED
		//Serial.print(F("\t"));//BANI COMMENTED
		Serial.println(switchState,BIN);
		// Serial.print(F("\t"));
		// Serial.println(LSFC,BIN); //BANI LOGGING CONTACT CHECK
		// Serial.print(F("\t"));s
		// Serial.println(LSBC,BIN); //BANI ,,
		// Serial.print(F("\t"));
		// Serial.println(RSFC,BIN); //BANI ,,
		// Serial.print(F("\t"));
		// Serial.println(RSBC,BIN); //BANI ,,
		// Serial.print(F("\t"));
		// Serial.println(LBC,BIN); //BANI ,,
		// Serial.print(F("\t"));
		// Serial.println(RBC,BIN); //BANI ,,
		Serial.println(LS,BIN);//BANI
		Serial.println(RS,BIN);//BANI
		Serial.println(BS,BIN);//BANI
		
		delay(100);
	}
	
	else {
		Serial.println(F("Contact: FALSE"));
	}
	
	if(DUMPING_SWITCH){
		Serial.println(F("dumping switch!"));
	}
}

void TestPowerRelay(){
	WDT_Restart(WDT);
	Relay.PowerOff(); //turn the power to the robot on
	delay(5000);
	WDT_Restart(WDT);
	Relay.PowerOn(); //turn the power to the robot on
	delay(5000);
}

// void TestIRSensors(){
// int R=analogRead(IRsensorRightPin);
// int L=analogRead(IRsensorLeftPin);
// Serial.print(F("L   ")); Serial.print(L);
// Serial.print(F("\t"));
// Serial.print(F("R   ")); Serial.print(R); 
// Serial.print(F("\t"));

// Serial.print(IRleft.isHigh());
// Serial.print(F("\t"));
// Serial.print(IRright.isHigh());
// Serial.println();
// }

void TestGripperSensor(){
	WDT_Restart(WDT);
	//int val=HeadSensor.Read(); //JSP
	// int val=analogRead(GripperSensorPin);
	//Serial.println(val); //JSP
}

void TestTurnHeading(){
	Serial.print("Ross");
	preferGyro = false;
	restingMode = true;
	
	while(1){
			
		WDT_Restart(WDT); // prevent from resetting

		TurnHeadingRoss(IN_DIRECTION); // turn towards the bed
		// Forward(BASE_SPEED); delay(5000); Stop();	// drive forward for a total of one second
		delay(5000);
		WDT_Restart(WDT); // prevent from resetting - not sure how long TurnHeading() will take
		TurnHeadingRoss(OUT_DIRECTION); // turn towards the exit of the tunnel
		delay(5000);

		// Forward(BASE_SPEED); delay(5000); Stop();	// drive forward for a total of one second
	}
}

void TestPowerSensors(){
	WDT_Restart(WDT);
	float V=Voltage.Read();
	float C=Current.Read();
	Serial.print(F("Current \t"));
	Serial.print(C);
	Serial.print(F('\t'));
	Serial.print(F("Voltage \t"));
	Serial.println(V);
}
//#endif

void myDelay(unsigned long delayTime){
/* this function is an improvement of delay() function, but non blocking
so that interrupts can be serviced
this method is probably a few milliseconds off 
 */

unsigned long maxDelayTime=delayTime*5; //cap this 
unsigned long start=millis(); //remember time this function call started 
 while(1){
 WDT_Restart(WDT);
 // Serial.println("!");//debug
  if(CONTACT){
   if (howLongWasContact >= CONTACT_TIME_LIMIT){ //guard against getting contact stuck 
   disableContacts=true; //turn off contacts, set global flag
   howLongWasContact=0; //reset contact
   whenDisabledContacts=millis(); //record this time
   }  
  handleContact(); //need to make this is not interferers with turning
   //if contacts are not disabled, add some time to the delay to compensate for the time used in
   //servicing interrupts 
   if(!disableContacts){
   delayTime=delayTime+howLongWasContact; //compensate
   }
   /*
   if(!disableContacts && diggingMode){ 
   delayTime=delayTime+DIGGING_INTERRUPT_DELAY;
   }
   
   if(!disableContacts && goingCharging){ 
   delayTime=delayTime+GOINGCHARGING_INTERRUPT_DELAY;
   }
   
   if(!disableContacts && chargingMode){ 
   delayTime=delayTime+CHARGINGMODE_INTERRUPT_DELAY;
   }
   */
  }

 
   if(delayTime > maxDelayTime){ //protection against infinite loop. written for goingCharging bug
    if(millis()-start >= maxDelayTime){
    return;
    break;
    }    
   } 
   
  if(millis()-start >= delayTime){ //achieved desired delay time 
  return;
  break;
  } 
 }
}

void bumpDelay(unsigned long delayTime){
	/* in this method the robot will execute a non blocking delay function.
		The function will terminate if one of the contact switches is depressed  */
	unsigned long start=millis(); //record start time
	WDT_Restart(WDT);
	while(1){
		if(CONTACT){
			return; // returns and breaks?
			break; 
  }
 
		if(millis()-start >= delayTime){
			return; // returns and breaks?
			break;
		} 
	}
}


bool CheckPayload(){
	/* this method will check if the robot is still carrying payload.
		inteded to be used in goingOut mode
		need to return false */

	WDT_Restart(WDT);
	int NumCheckPayload = 0;
	while (NumCheckPayload < 3){
		if(analogRead(ForceSensor) > ForceSensorThresh){
			return true;
		}	
		NumCheckPayload++;
	}
	return false;
}


void logContacts(unsigned long start_of_contact){
	/* this method remembers how long the switches were compressed
		and sets a flag for disabling purposes if the switches were 
		compressed for too long*/
	WDT_Restart(WDT);
	if( start_of_contact - whenWasLastContact < CONTINUOUS_CONTACT){//if short duration between counters
		howLongWasContact=howLongWasContact+(millis()-start_of_contact); //add time to contact counter
			if(howLongWasContact > CONTACT_TIME_LIMIT){
				disableContacts=true; //mask them 
				whenDisabledContacts=millis(); //record time of disabling contacts 
			}
	}
	else{
		howLongWasContact=(millis()-start_of_contact); //reset and update counter
	}
	whenWasLastContact=millis(); //remember when last contact was
}

// bool checkWatchdog(){
 // if(watchdogFlag){ //everything is good)
 // //watchdogFlag=0; //reset the flag
 // watchdogReset=millis(); 
 // return 0; //return false
 // }
 // else{
  // if( millis() - watchdogReset > 7000 ){
  // return 1;
  // }
  // else{
  // return 0;
  // }
 // }
// }

// void handleTrouble(){
// Backward(BASE_SPEED); delay(1000);
// bumpDelay(1000); //back out
// TurnHeading(current_target_heading);
// watchdogReset=millis(); 
// return;
// }

bool checkCharger(){
	WDT_Restart(WDT);
	if(CHARGER && !goingCharging){
		Backward(BASE_SPEED); delay(1000);
		bumpDelay(1000);
		return true;
	}
	if(CHARGER && goingCharging){
		enable_ChargingMode();
		return true;
	}
	if(CHARGER && restingMode){
		return true;
	} 
	return false; 
}
/*
void checkCamera(){
//maybe get rid of this crap 

// poll camera, see if we get anything back. If not, something is wrong. RESTART 
 while(1){//dumb fix 
 unsigned long now = millis();
  while ( millis() - now < 16000){
  GetDetectedSigs();
   if(blocks){
   return; //everything is cool
   }
  }
 // arduinoReset();
 Backward(BASE_SPEED); delay(1000); Stop(); //drive back, maybe we are shorted at charging station
 Right(DEFAULT_TURNING_SPEED); delay(200); Stop(); 
 Relay.PowerOff();
 delay(1000);
 
 fioWrite(RESET_REQUEST); //ask a friend for a favor 
 delay(1000);
  while(1){ //keep resetting 
  fioWrite(RESET_REQUEST); //ask a friend for a favor 
  delay(5000);
  arduinoReset();// reset arduino 
  delay(5000);
  }
 }
}
*/


void handleManualOverride(){
	WDT_Restart(WDT);
  if(checkManualOverride()){
		Stop(); 
		ManualControl(); 
  }
}

bool checkManualOverride(){
	WDT_Restart(WDT);
	byte code=masterRead();
	//Serial.print(code); Serial.print('\t'); Serial.println(code,HEX); //debug 
	if(code == MANUAL_OVERRIDE_START){
		fioWrite(MANUAL_OVERRIDE_START); //send back acknowledgement
		manualMode=true;
		return true;
	}
	else{
		return false;
	}
}

void ManualControl(){
 WDT_Restart(WDT);
 while(1){
 WDT_Restart(WDT);
 byte code=masterRead();
 // Serial.print(F("loop code:   ")); Serial.println(code,HEX);
  switch(code){
  case MANUAL_OVERRIDE_END:
  fioWrite(MANUAL_OVERRIDE_END); //send back acknowledgement
  manualMode=false;
  return;
  break;

  case MANUAL_STOP:
  Stop();
  break;
  case MANUAL_GO_FORWARD:
  Forward(BASE_SPEED); delay(200);
  Stop();
  break;  
  case MANUAL_GO_BACKWARD:
  Backward(BASE_SPEED); delay(200);
  Stop();
  break;
  case MANUAL_RIGHT:
  Right(DEFAULT_TURNING_SPEED); delay(200);
  Stop();
  break;
  case MANUAL_LEFT:
  Left(DEFAULT_TURNING_SPEED); delay(200);
  Stop();
  break;
  
  //mode changing, acknowledgement is inside functions
  case MANUAL_GOING_IN:
  enable_GoingInMode();
  break;
  case MANUAL_DIGGING:
  enable_DiggingMode();
  break;
  case MANUAL_GOING_OUT:
  enable_GoingOutMode();
  break;
  case MANUAL_DUMPING:
  enable_DumpingMode();
  break;
  case MANUAL_GOING_CHARGING:
  enable_GoingCharging();
  break;
  case MANUAL_CHARGING:
  enable_ChargingMode();
  break;
  
  //power and reset
  case MANUAL_RESET:
  arduinoReset();
  break;
  case MANUAL_RELAY_ON:
  Relay.PowerOn();
  break;
  case MANUAL_RELAY_OFF:
  Relay.PowerOff();
  break; 
  
  } 
 }
}
//end for capacitive sensor



// void TestIRsensorReadings(){
// // Serial.print("R"); Serial.print("\t"); 
// // Serial.print( IRright.ReadCal() ); Serial.print("\t");
// // Serial.print("L"); Serial.print("\t"); 
// // Serial.println( IRleft.ReadCal() );
// // delay(100);
// while(1){
// //these are no longer working 
// // // Serial.print("R"); Serial.print("\t"); 
// // // Serial.print( IRright.isDetected() ); Serial.print("\t"); Serial.print( IRright.Read() ); Serial.print("\t");
// // // Serial.print("L"); Serial.print("\t"); Serial.print( IRleft.Read() ); Serial.print("\t");
// // // Serial.println( IRleft.isDetected() );
// Serial.print("R"); Serial.print("\t"); 
// Serial.print( IRright.Read() ); Serial.print("\t");
// Serial.print("L"); Serial.print("\t"); Serial.println( IRleft.Read() ); 
// // Serial.println(IRright.isBlindzone);
// }
// }
// ********** END   (TEST AND DEBUG} **********

