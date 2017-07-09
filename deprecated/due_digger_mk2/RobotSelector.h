//include this .h file to libraries
//this will enable to program each robot individually 
#define RobotSelector_h

//* / testing variables defined here
// #define test_mode 1

// all test_case options are given below. the enumeration is defined in the the {global variable declaration} section of due_digger_mk2.ino
// test_imu, 
// test_force,
// test_mag,

// remember that the thresholds might be different when the robot is plugged in.
// this will make it harder to debug issues asso	ciated with the capacitive sensors.
// also remember that there is a parameter in the fio software that needs to be changed in
// order to test the capacitive sensors
// test_cap, 					

// test_switches, 			// ross: i think this testing approach is deprecated. use test_cap instead
// this enumeration is used in the switch-case statements in loop() to conduct any necessary tests
	// test_imu,
	// test_imu_cal,
	// test_force,
	// test_mag,
	// test_cap_0, 					// remember that the thresholds might be different when the robot is plugged in. this will make it harder to debug issues associated with the capacitive sensors
	// test_cap_1,
	// test_cap_2,
	// test_cap_3,
	// test_cap_4,
	// test_cap_5,
	// test_cap_6,
	// test_cap_7,
	// test_charger,
	// test_current,
	// test_switches, 			// ross: i think this testing approach is deprecated. use test_cap instead
	// test_drive_motors, 
	// test_servo_motors, 
	// test_camera,
	// test_voltage,
	// test_gripper_sensor,
	// test_power_sensors, 
	// test_turn_heading,
	// test_pick_direction,
	// test_pid_controller,
	// test_nothing, */
#define TEST_CASE TEST_SERVO_MOTORS
#define ROBOT_A

// ********** BEGIN {SET BEHAVIOR} **********
//--comment things out if unwanted 
//lorenz stuff
#define PROBABILITY_DIG 0 //0 for active, 1 for lorenz
#define RESTING_TIME 20000// number of seconds before rerolling probabilty in lorenz mode -- was originally 20000
#define lorenzProb 21.61    

//useless run stuff turn back if it did not reach the face
#define ALLOW_USELESS_RUNS 1// 1 is allow
#define USELESS_RUN_THRESH 75000 //used to be 75000 


// This will be used in the handleContact
#define REVERSE_ON_CONTACT 0
#define REVERSE_ON_CONTACT_PROB 0

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