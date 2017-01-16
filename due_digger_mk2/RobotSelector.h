//include this .h file to libraries
//this will enable to program each robot individually 
#define RobotSelector_h

// TESTING VARIABLES DEFINED HERE
// #define TEST_MODE 1

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
#define TEST_CASE TEST_NOTHING
#define ROBOT_A

// ********** BEGIN {SET BEHAVIOR} **********
//--comment things out if unwanted 
//lorenz stuff
#define PROBABILITY_DIG 1 //0 for active, 1 for lorenz
#define RESTING_TIME 20000// number of seconds before rerolling probabilty in lorenz mode -- was originally 20000
#define lorenzProb 89.64 

//useless run stuff turn back if it did not reach the face
#define ALLOW_USELESS_RUNS 1 // 1 is allow
#define USELESS_RUN_THRESH 75000 //used to be 75000 

//dont worry about this stuff
#define ALLOW_CHARGING_ON_REST 0
#define ALLOW_POWER_SAVINGS 0
//VADIM. FIND A TIMER FROM A PREVIOUS CODE> THIS WAS BROKEN 
// #define BACKWARDS_KICK_TIME 1000   //every so often the robot will drive back for this many seconds. needed for avoiding getting stuck 
//run trhesh: 40s too short, 120s too long
// **********  END   {SET BEHAVIOR} ---------

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