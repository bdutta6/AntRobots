//pin 53 is BUSTED

// ********** BEGIN (PIN ASSIGNMENT ON ARDUINO DUE} **********
// #define PHEROMONE_TRAILS 1
/* OUTPUT PINS */
// --- drive motor control
#define enable1 2     // enable  1 from motor control board. Left back motor.
#define phase1 3     // phase 1 from motor control board. Left back motor. (pwm)
#define enable2 4     // enable  2 from motor control board. Left front motor.
#define phase2 5     // phase 2 from motor control board. Left front motor. (pwm) //pin 4 is stuck in on

// MB2
#define enable3 8    // enable 3 from motor control board. Right back motor.
#define phase3 9     // phase 3 from motor control board. Right back motor. (pwm)
#define enable4 10    // enable 4 from motor control board. Right front motor.
#define phase4 11     // phase 4 from motor control board. Right front motor. (pwm)

// --- arm servo control
// --- arm servo control
#define PitchPin 12 //servo motor actuating arm
#define GripPin  13 //servo motor actuating gripper 10

// --- power switching
#define relay_pin1 7     //relay is turned on by this pin
#define IMUpower A3 //any digital pin to control the power to IMU

// #define relay_pin2 25     //relay is turned on by this pin and relay_pin1 if a two pin device is declared in PowerRelay.h file
/* INPUT PINS */
// --- sensors
#define currentSensorPin A0 //current sensor gets connected to pin A0 (analog pin)
#define voltage_pin A1        //battery voltage probe is connected to pin A1 (analog pin)
//#define GripperSensorPin A2 //analog reflective (line) sensor for gripper //JSP
#define RUGripperPin A8 //JSP
#define RDGripperPin A9 //JSP
#define LUGripperPin A6 //JSP
#define LDGripperPin A7 //JSP
//#define FGripperPin A2 //JSP
//#define IRsensorRightPin A8 //short range SHARP IR sensors
//#define IRsensorLeftPin  A9
#define ForceSensor A11 // JSP
//#define ProximityBumpPin 32
// #define SDApin 20          //A4 and A5 pins are reserved for I2C communication and are unavailable for general use
// #define SCLpin 21          //note that these last 2 define statements are not used in a code but included as a reminder that pins 4 and 5 are unavailabe
// --- others

//#define EncoderPinA1 7 //encoder 1 pin output A
//#define EncoderPinB1 8 //encoder 1 pin output B
//#define EncoderPinA2 32 //encoder 2 pin output A
//#define EncoderPinB2 33 //encoder 2 pin output B


#define CapacitiveSensorPin 22 //CapacitiveSensorPin
#define ChargeTime 0x24//0x84
#define ChargeCurrent 0x3F//0x10 //0x0F

//--- charging detector
#define ChargingDetectorPin 33 // pin 53 may be busted

#define CS 52            // Chip select

// **********  END   {PIN ASSIGNMENT ON ARDUINO FIO} ---------

//--- emergency reset
#define ResetPin 12   //digital pin which will pull reset pin LOW, restarting micro controller
// ********** BEGIN{CONVENIENCE MACROS} **********
#define TRAIL1 Object[0]     //pheromone trail 1
#define CHARGING_TRAIL Object[1]     //pheromone trail 2
#define CHARGING_GROUNDS Object[4]
// #define CHARGING_BEACON Object[3] //charging station detected //USED TO BE 6 whcih is a bug
#define COTTON Object[2] // seeing some cotton to dig
#define TUNNEL_START Object[3] //beginning of  a tunnel 

//---limit switch macros

#define SWITCH_A_MASK_WALL 	0b0000000000000001
#define SWITCH_A_MASK_ANT  	0b0000000000000011
#define SWITCH_B_MASK_WALL 	0b0000000000000100
#define SWITCH_B_MASK_ANT  	0b0000000000001100
#define SWITCH_C_MASK_WALL 	0b0000000000010000
#define SWITCH_C_MASK_ANT  	0b0000000000110000
#define SWITCH_D_MASK_WALL 	0b0000000001000000
#define SWITCH_D_MASK_ANT  	0b0000000011000000
#define SWITCH_E_MASK_WALL 	0b0000000100000000
#define SWITCH_E_MASK_ANT  	0b0000001100000000
#define SWITCH_F_MASK_WALL 	0b0000010000000000
#define SWITCH_F_MASK_ANT  	0b0000110000000000
#define SWITCH_G_MASK_WALL 	0b0001000000000000
#define SWITCH_G_MASK_ANT  	0b0011000000000000
#define SWITCH_H_MASK_WALL 	0b0100000000000000
#define SWITCH_H_MASK_ANT  	0b1100000000000000
#define SWITCH_I_MASK      	0b0000000000000000
#define SWITCH_ANT_MASK    	0b1010101010101010
#define SWITCH_WALL_MASK		0b0101010101010101


#define FL SWITCH_A_MASK_WALL		//Front Left
#define FR SWITCH_B_MASK_WALL		//Front Right
#define LSF SWITCH_C_MASK_WALL   //Left Side Front
#define RSF SWITCH_D_MASK_WALL   //Right Side Front
#define LSB SWITCH_E_MASK_WALL   //Left Side Back
#define RSB SWITCH_F_MASK_WALL   //Right Side Back
#define BL  SWITCH_G_MASK_WALL    //Back Left
#define BR  SWITCH_H_MASK_WALL   //Back Right

#define FL_ANT SWITCH_A_MASK_ANT		//Front Left, ANT
#define FR_ANT SWITCH_B_MASK_ANT		//Front Right, ANT
#define LSF_ANT SWITCH_C_MASK_ANT   //Left Side Front, ANT
#define RSF_ANT SWITCH_D_MASK_ANT   //Right Side Front, ANT
#define LSB_ANT SWITCH_E_MASK_ANT   //Left Side Back, ANT
#define RSB_ANT SWITCH_F_MASK_ANT   //Right Side Back, ANT
#define BL_ANT  SWITCH_G_MASK_ANT    //Back Left, ANT
#define BR_ANT  SWITCH_H_MASK_ANT   //Back Right, ANT

#define CONTACT (CapSensor.isDetected()) //& ~SWITCH_I_MASK) //all but switch A mask
#define DUMPING_SWITCH SWITCH_I_MASK //bit wise AND

#define HEADON FrontBumpSensor.isDetected() //check if bump is detected
#define CHARGER isChargerDetected() //found charger
#define HEADSENSOR HeadSensor.IsDetected() 
//#define DUMPING_SIGNAL	CHARGER //improvident suggestion
// **********  END   {CONVENIENCE MACROS} ---------

#define FOO  Serial.println(F("FOO"));
#define BAR  Serial.println(F("BAR"));
