#include "Arduino.h"   //includes standard Arduino Library
#include "MotorBoard.h" //includes header file
#include "RobotSelector.h" //load appropriate robot specific definitions and constants 

// MotorBoard :: MotorBoard(int enable1, int phase1, int enable2, int phase2){
MotorBoard :: MotorBoard(){
// _phase1=phase1; _phase2=phase2; _enable1=enable1; _enable2=enable2; // storing passed values to private variables for robustness
	pinMode(phase1, OUTPUT);
	pinMode(phase2, OUTPUT);
  pinMode(phase3, OUTPUT);
  pinMode(phase4, OUTPUT);
	pinMode(enable1, OUTPUT);
	pinMode(enable2, OUTPUT);
  pinMode(enable3, OUTPUT);
  pinMode(enable4, OUTPUT);
}

void MotorBoard :: RightFrontForward(int pwm){
//	Serial.println("In RightFrontForward");
  digitalWrite(enable4,LOW);
  analogWrite(phase4,pwm);  
}

void MotorBoard :: LeftFrontForward(int pwm){
//  Serial.println("In LeftFrontForward");
  digitalWrite(enable2,LOW);
  analogWrite(phase2,pwm);  
}

void MotorBoard :: RightBackForward(int pwm){
//  Serial.println("In RightBackForward");
  digitalWrite(enable3,LOW);
  analogWrite(phase3,pwm);  
}

void MotorBoard :: LeftBackForward(int pwm){
//  Serial.println("In LeftBackForward");
  digitalWrite(enable1,LOW);
  analogWrite(phase1,pwm);
}

void MotorBoard :: RightFrontBackward(int pwm){
//  Serial.println("In RightFrontBackward");
  digitalWrite(enable4,HIGH);
  analogWrite(phase4,255-pwm);//note 255-pwm inversion! Nevermind. changed back to pwm  
}

void MotorBoard :: LeftFrontBackward(int pwm){
//  Serial.println("In LeftFrontBackward");
  digitalWrite(enable2,HIGH);
  analogWrite(phase2,255-pwm);  
}

void MotorBoard :: RightBackBackward(int pwm){
//  Serial.println("In RightBackBackward");
  digitalWrite(enable3,HIGH); //must change
  analogWrite(phase3, 255-pwm);  
}

void MotorBoard :: LeftBackBackward(int pwm){
//  Serial.println("In LeftBackBackward");
  digitalWrite(enable1, HIGH); //must change
  analogWrite(phase1,255-pwm);  
}

void MotorBoard :: RightFrontStop(){
//first method is commented out cause it sets both pins to HIGH which drains a tiny amount of current
//second method is slightly better cause both pins are LOW
/*   digitalWrite(_enable1,HIGH);
  // analogWrite(_phase1,255); //THIS IS BAD. produces noise
  digitalWrite(_phase1,HIGH); */
//  Serial.println("In RightFrontStop");
  digitalWrite(enable4,LOW);
  analogWrite(phase4,0); 
}

void MotorBoard :: LeftFrontStop(){
/*   digitalWrite(_enable2,HIGH);
  // analogWrite(_phase2,255); //THIS IS BAD. produces noise
  digitalWrite(_phase2,HIGH); */
//  Serial.println("In LeftFrontStop");
  digitalWrite(enable2,LOW);
  analogWrite(phase2,0); 
}

void MotorBoard :: RightBackStop(){
  /*   digitalWrite(_enable2,HIGH);
  // analogWrite(_phase2,255); //THIS IS BAD. produces noise
  digitalWrite(_phase2,HIGH); */
//  Serial.println("In RightBackStop");
  digitalWrite(enable3,LOW);
  analogWrite(phase3,0); 
}

void MotorBoard :: LeftBackStop(){
  /*   digitalWrite(_enable2,HIGH);
  // analogWrite(_phase2,255); //THIS IS BAD. produces noise
  digitalWrite(_phase2,HIGH); */
//  Serial.println("In LeftBackStop");
  digitalWrite(enable1,LOW);
  analogWrite(phase1,0); 
}

//-----------------------------------------------

void MotorBoard :: RightForward(int pwm){
  RightFrontForward(pwm);
  RightBackForward(pwm);
}

void MotorBoard :: LeftForward(int pwm){
  LeftFrontForward(pwm);
  LeftBackForward(pwm);
}

void MotorBoard :: RightBackward(int pwm){
  RightFrontBackward(pwm);
  RightBackBackward(pwm);
}

void MotorBoard :: LeftBackward(int pwm){
  LeftFrontBackward(pwm);
  LeftBackBackward(pwm);
}

void MotorBoard :: RightStop(){
  RightFrontStop();
  RightBackStop();
}

void MotorBoard :: LeftStop(){
  LeftFrontStop();
  LeftBackStop();
}
