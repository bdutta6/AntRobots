// ********** BEGIN {MISC TUNABLE CONSTNATS} ----------
#define GO_DIG_PROB   80 //dig 75 rest 25 
#define GO_REST_PROB  20 //unused

// #define DUMPING_TARGET_THRESH 5000 //minimum area of a dumping target to be detected  //10K works up close with the beacon turned sideways
#define CHARGING_GROUND_TRAP 100 //minimum area the robot must see
#define CHARGING_TARGET_THRESH 1000 //minimum area to start driving to a docking station
#define CHARGING_DOCKING_THRESH 20000 //area needs to be seen to initate docking  //unused i think
#define LOWEST_ALLOWABLE_VOLTAGE 3.4 //5v Power is lost when battery level reaches 3v, step up chip no longer functions  //3.3 too low
#define CHARGED_VOLTAGE 3.8 // 3.65 appears to be the max voltage  //3.79 was too high


// #define TURNING_THRESH 1000 //minimum area needed to terminate feedback turning
#define LOW_ROW_ANGLE   60 //45, JSP  //pitch servo is commanded to point toward ground
#define HIGH_ROW_ANGLE  120 //100, JSP //pitch servo is commanded to point toward the ceiling without blocking camera 
#define MID_ROW_ANGLE   60//85, JSP//pitch servo is commanded to be parallel to ground
#define TRAVEL_ANGLE    60//90, JSP     //pitch servo will be maintained around this setpoint while the robot is driving 
#define CLOSED_POS 110//179, JSP  //grip servo fully closed angle
#define OPEN_POS 55//90, JSP     //grip servo fully open angle
#define MID_POS 70 //JSP //grip servo half-open angle
#define ForceSensorThresh 50 // JSP
#define AntThresh 3 // Ant Detection Threshold for Capacitive Sensor
#define WallSingleThresh1	380 //Wall Detection, Single Contact Threshold for Capacitive Sensor
#define WallSingleThresh2 460 //Wall Detection, Single Contact Threshold for Capacitive Sensor
#define AntSingleThresh1	510
#define AntSingleThresh2	550
#define WallDoubleThresh 13//1 //for Wall Detection, two or more contact threshold for Capacitive Sensor
#define AntDoubleThresh1 15	//for Ant Detection, two or more contact threshold for Capacitive Sensor
#define AntDoubleThresh2 18

#define MINIMUM_AREA_THRESH 20   //ensure that only blocks that are at least this big are considered. Also can be configured in camera via PIXYMON

#define OUT_DIRECTION 251//62//300 // previously set to 270-
#define IN_DIRECTION  108//291//62 //  used to be 60
#define GET_BACK_DIRECTION 25 //compass direction
#define CHARGING_DIRECTION 300 //170 is at the wall, towards me
#define DIRECTION_UNCERTAINTY 10 //used to be 5. increased to 10
#define PORT_DIRECTION        340//350    //left with respect to IN_DIRECTION
#define STARBOARD_DIRECTION   193   //right with respect to IN_DIRECTION

// #define DIR_IN_R 260
// #define DIR_IN_L 122
// #define DIR_OUT_R 33
// #define DIR_OUT_L 355


#define DIGGING_INTERRUPT_DELAY 500 //used to pause robot if its bumped into while digging
#define GOINGCHARGING_INTERRUPT_DELAY 500 //used to pause robot if its bumped into while going charging
#define CHARGINGMODE_INTERRUPT_DELAY 500  //used to pause robot if its bumped into while going charging
#define CONTINUOUS_CONTACT 1000    //if two contacts occur within this time, they are treated as one
#define CONTACT_TIME_LIMIT 2000   //cap limit on contact 
#define CONTACT_RESET_TIME 5000  //used to reset contact switches if they are disabled by software
#define S AW_TRAILS_TIMEOUT 10000  //if the robot fails to see a pheromone trail, force it to back out and turn back on the trail 
#define RIGHT_IR_THRESH 850 //threshold for right IR sensor reading 
#define LEFT_IR_THRESH 850  //threshold for left IR sensor reading
#define RIGHT_IR_MAX_TIME 4000 //4 secs
#define LEFT_IR_MAX_TIME  4000
#define BOTH_IR_MAX_TIME  4000 //10 secs
#define HEAD_IR_MAX_TIME  4000
//IR tresholds: 700, Blind zone 8 seconds, min=320, max=699
#define COTTON_IR_SUPRESS 30000 //pixels squared. measured the area by printing what it is close 
#define COTTON_START_DIGGING 49000 //pixels squared measured the area by printing what it is super close
// --- PID control stuff
#define Kp 2 //2 //1.5 worked fine
#define Ki 0 //0
#define Kd 0  //.2
#define PD_SAMPLE_TIME 80 //80
#define BASE_SPEED 200 // Ross 160->200 //worked okay with 100, 200
#define PV_adjmax 205  //155
#define PD_expected_limit 600

#define drivingForward  1
#define drivingBackward 2
#define turningRight    3
#define turningLeft     4
#define stopped         0

//--- Turning stuff
#define SLOW_TURNING_RATE 11 //used to be 15, 13
#define TURNING_TIMEOUT     1500 //8 seconds timeout
#define DEFAULT_TURNING_SPEED 155 //used to be 255
// ********** END {MISC TUNABLE CONSTNATS} ----------
#define R1 3.3 //k ohms, actual resistance 
#define R2 3.3 //k omhs, two 3.3k resistors in parallel