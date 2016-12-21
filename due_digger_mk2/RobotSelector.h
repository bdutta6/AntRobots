//include this .h file to libraries
//this will enable to program each robot individually 
#define RobotSelector_h

// TESTING VARIABLES DEFINED HERE
#define TEST_MODE 1

// All TEST_CASE options are given below. The enumeration is defined in the the {GLOBAL VARIABLE DECLARATION} section of due_digger_mk2.ino
// TEST_IMU, 
// TEST_FORCE,
// TEST_MAG,

// remember that the thresholds might be different when the robot is plugged in.
// This will make it harder to debug issues asso	ciated with the capacitive sensors.
// Also remember that there is a parameter in the Fio software that needs to be changed in
// order to test the capacitive sensors
// TEST_CAP, 					

// TEST_SWITCHES, 			// Ross: I think this testing approach is deprecated. Use TEST_CAP instead
// TEST_DRIVE_MOTORS, 
// TEST_SERVO_MOTORS, 
// TEST_CAMERA, 
// TEST_GRIPPER_SENSOR,
// TEST_POWER_SENSORS, 
// TEST_TURN_HEADING,
// TEST_PID_CONTROLLER
// TEST_NOTHING,
#define TEST_CASE TEST_IMU
#define ROBOT_B
// Robot A is on COM 3,  radio on COM 8
// Robot B is on COM 11, radio on COM 7
// Robot C is on COM 10, radio on COM 6
// Robot D is on COM 12, radio was never setup
// Robot E is on COM 15,//BANI JSP

#ifdef ROBOT_A
#include "defA.h"        //definition macros
#include "constantsA.h"  //tunable constants
#endif

#ifdef ROBOT_B
#include "defB.h"        //definition macros
#include "constantsB.h"  //tunable constants
#endif

#ifdef ROBOT_C
#include "defC.h"        //definition macros
#include "constantsC.h"  //tunable constants
#endif

#ifdef ROBOT_D
#include "defD.h"        //definition macros
#include "constantsD.h"  //tunable constants
#endif

#ifdef ROBOT_E
#include "defE.h"        //definition macros//BANI JSP
#include "constantsE.h"  //tunable constants
#endif
// #endif