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
#include "Arduino.h"

class MotorBoard
{
  public:
    // MotorBoard(int enable1, int phase1, int enable2, int phase2); //constructor
    MotorBoard(); //constructor   
    void RightForward(int pwm);//drives right wheels forward at given pwm
    void LeftForward(int pwm);//drives left wheels forward at given pwm
    
    void RightBackward(int pwm);//drives right wheels backward at given pwm
    void LeftBackward(int pwm);//drives left wheels backward at given pwm

    void RightStop();//stops both right wheels
    void LeftStop();//stops both left wheels
  
  // private:
  // int _enable1;  int _phase1;  int _enable2;  int _phase2; //defining private variables
  
};



#endif
