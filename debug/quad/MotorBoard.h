/*
  This class sets up easy commands to drive dc motors
  Pass phase (enable1 and enable2) and enable (phase1 and phase2) pins to constructor
  enable 1 and phase 1 are reserved for a RIGHT wheel
  enable 2 and phase 2 are reserved for a LEFT  wheel
  
  when enable set to  LOW, the motor will be driven at full speed if phase pwm=255 and is stopped if phase pwm=0
  when enable set to HIGH, the motor will be driven at full speed if phase pwm=0   and is stopped if phase pwm=255

*/

#ifndef MotorBoard_h
#define MotorBoard_h
#include "RobotSelector.h"
#include "Arduino.h"

class MotorBoard
{
  public:
    // MotorBoard(int enable1, int phase1, int enable2, int phase2); //constructor
    MotorBoard(); //constructor
    void RightFrontForward(int pwm);//drives right front wheel forward at given pwm
    void LeftFrontForward(int pwm);//drives left front wheel forward at given pwm
    void RightBackForward(int pwm);//drives right back wheel forward at given pwm
    void LeftBackForward(int pwm);//drives left back wheel forward at given pwm
    void RightFrontBackward(int pwm);//drives right front wheel backward at given pwm
    void LeftFrontBackward(int pwm);//drives left front wheel backward at given pwm  
    void RightBackBackward(int pwm);//drives right back wheel backward at given pwm
    void LeftBackBackward(int pwm);//drives left back wheel backward at given pwm
    void RightFrontStop();//breaks right front wheel
    void LeftFrontStop();//breaks left front wheel
    void RightBackStop();//breaks right back wheel
    void LeftBackStop();//breaks left back wheel
  
  // private:
  // int _enable1;  int _phase1;  int _enable2;  int _phase2; //defining private variables
  
};



#endif
