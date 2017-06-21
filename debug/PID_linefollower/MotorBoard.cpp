#include "Arduino.h"   //includes standard Arduino Library
#include "MotorBoard.h" //includes header file


#define enable1 9 // 1 equals right motor stuff
//#define rightMotor2 4
#define phase1 8
#define enable2 12 // 2 equals left motor stuff
//#define leftMotor2 13
#define phase2 11
//#define motorPower 8

// MotorBoard :: MotorBoard(int enable1, int phase1, int enable2, int phase2){
MotorBoard :: MotorBoard(){
// _phase1=phase1; _phase2=phase2; _enable1=enable1; _enable2=enable2; // storing passed values to private variables for robustness
  pinMode(phase1, OUTPUT);
  pinMode(phase2, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(enable2, OUTPUT);
}

void MotorBoard :: RightForward(int pwm){
//  Serial.println("In RightFrontForward");
  digitalWrite(enable1,LOW);
  analogWrite(phase1,pwm);  
}

void MotorBoard :: LeftForward(int pwm){
//  Serial.println("In LeftFrontForward");
  digitalWrite(enable2,LOW);
  analogWrite(phase2,pwm);  
}

void MotorBoard :: RightBackward(int pwm){
//  Serial.println("In RightFrontBackward");
  digitalWrite(enable1,HIGH);
  analogWrite(phase1,255-pwm);//note 255-pwm inversion! Nevermind. changed back to pwm  
}

void MotorBoard :: LeftBackward(int pwm){
//  Serial.println("In LeftFrontBackward");
  digitalWrite(enable2,HIGH);
  analogWrite(phase2,255-pwm);  
}

void MotorBoard :: RightStop(){
//first method is commented out cause it sets both pins to HIGH which drains a tiny amount of current
//second method is slightly better cause both pins are LOW
/*   digitalWrite(_enable1,HIGH);
  // analogWrite(_phase1,255); //THIS IS BAD. produces noise
  digitalWrite(_phase1,HIGH); */
//  Serial.println("In RightFrontStop");
  digitalWrite(enable1,LOW);
  analogWrite(phase1,0); 
}

void MotorBoard :: LeftStop(){
/*   digitalWrite(_enable2,HIGH);
  // analogWrite(_phase2,255); //THIS IS BAD. produces noise
  digitalWrite(_phase2,HIGH); */
//  Serial.println("In LeftFrontStop");
  digitalWrite(enable2,LOW);
  analogWrite(phase2,0); 
}

//-----------------------------------------------
