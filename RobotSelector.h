//include this .h file to libraries
//this will enable to program each robot individually 
#define RobotSelector_h

#define ROBOT_C
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