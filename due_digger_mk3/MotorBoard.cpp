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
	Serial.println("In RightForward");
  digitalWrite(enable4,LOW);
  analogWrite(phase4,pwm);  
}

void MotorBoard :: LeftFrontForward(int pwm){
  digitalWrite(enable2,LOW);
  analogWrite(phase2,pwm);  
}

void MotorBoard :: RightBackForward(int pwm){
  digitalWrite(enable3,LOW);
  analogWrite(phase3,pwm);  
}

void MotorBoard :: LeftBackForward(int pwm){
  digitalWrite(enable1,LOW);
  analogWrite(phase1,pwm);
}

void MotorBoard :: RightFrontBackward(int pwm){
  digitalWrite(enable1,HIGH);
  analogWrite(phase1,255-pwm);//note 255-pwm inversion! Nevermind. changed back to pwm  
}

void MotorBoard :: LeftFrontBackward(int pwm){
  digitalWrite(enable2,HIGH);
  analogWrite(phase2,255-pwm);  
}

void MotorBoard :: RightBackBackward(int pwm){
  digitalWrite(enable3,HIGH); //must change
  analogWrite(phase3, 255-pwm);  
}

void MotorBoard :: LeftBackBackward(int pwm){
  digitalWrite(enable4, HIGH); //must change
  analogWrite(phase4,255-pwm);  
}

void MotorBoard :: RightFrontStop(){
//first method is commented out cause it sets both pins to HIGH which drains a tiny amount of current
//second method is slightly better cause both pins are LOW
/*   digitalWrite(_enable1,HIGH);
  // analogWrite(_phase1,255); //THIS IS BAD. produces noise
  digitalWrite(_phase1,HIGH); */
  digitalWrite(enable1,LOW);
  analogWrite(phase1,0); 
}

void MotorBoard :: LeftFrontStop(){
/*   digitalWrite(_enable2,HIGH);
  // analogWrite(_phase2,255); //THIS IS BAD. produces noise
  digitalWrite(_phase2,HIGH); */
  digitalWrite(enable2,LOW);
  analogWrite(phase2,0); 
}

void MotorBoard :: RightBackStop(){
  /*   digitalWrite(_enable2,HIGH);
  // analogWrite(_phase2,255); //THIS IS BAD. produces noise
  digitalWrite(_phase2,HIGH); */
  digitalWrite(enable3,LOW);
  analogWrite(phase3,0); 
}

void MotorBoard :: LeftBackStop(){
  /*   digitalWrite(_enable2,HIGH);
  // analogWrite(_phase2,255); //THIS IS BAD. produces noise
  digitalWrite(_phase2,HIGH); */
  digitalWrite(enable4,LOW);
  analogWrite(phase4,0); 
}



void MotorBoard :: RightForward(int pwm){
	RightBackForward(pwm);
	RightFrontForward(pwm);
}

void MotorBoard :: LeftForward(int pwm){
	LeftBackForward(pwm);
	LeftFrontForward(pwm);
}

void MotorBoard :: RightBackward(int pwm){
	RightBackBackward(pwm);
	RightFrontBackward(pwm);
}

void MotorBoard :: LeftBackward(int pwm){
	LeftBackBackward(pwm);
	LeftFrontBackward(pwm);
}

