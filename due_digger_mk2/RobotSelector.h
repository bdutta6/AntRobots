//include this .h file to libraries
//this will enable to program each robot individually 
#define RobotSelector_h

// TESTING VARIABLES DEFINED HERE
#define TEST_MODE 1

// testing = 1, running code = 0
#define TEST_IMU 0 // use this to calibrate direction headings  (377-381))
#define TEST_SWITCHES 0
#define TEST_FORCE 0
#define TEST_MAG 0
// remember that the thresholds might be different when the robot is plugged in. This will make it harder to debug issues associated with the capacitive sensors
#define TEST_CAP 0
#define TEST_SWITCHES 0 // Ross: I think this testing approach is deprecated. Use TEST_CAP instead
#define TEST_DRIVE_MOTORS 0
#define TEST_SERVO_MOTORS 0
#define TEST_CAMERA 0
#define TEST_GRIPPER_SENSOR 0
#define TEST_POWER_SENSORS 0

#define ROBOT_D
//Robot A is on COM 3,  radio on COM 8
//Robot B is on COM 11, radio on COM 7
//Robot C is on COM 10, radio on COM 6
//Robot D is on COM 12, radio was never setup
//// Robot E is on COM 15,//BANI JSP

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