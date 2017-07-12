/**********************************************************************************************
 * Code based on:
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * Modified by: Vadim Linevich <vadim.linevich@gmail.com>
 *
 * This Library is licensed under a GPLv3 License
 *
 **********************************************************************************************/
#include "Arduino.h"
#include "cleg.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
cleg::cleg(int motorA1, int motorA2, int motorA_PWM, int encoder1PinA, int encoder1PinB){
	
	pinMode(motorA1, OUTPUT);
	pinMode(motorA2, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinA, HIGH);
  digitalWrite(encoder1PinB, HIGH);
	
	A1_set = false;
	B1_set = false;
	encoder1Pos = 0;                  //rev counter

  attachInterrupt(digitalPinToInterrupt(2), doEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoder2, CHANGE);
	return;
}

// Interrupt on A changing state
void cleg::doEncoder1(){
  // Test transition
  A1_set = digitalRead(encoder1PinA) == HIGH;
  // and adjust encoder1Poser + if A leads B
  encoder1Pos += (A1_set != B1_set) ? +1 : -1;
	return;
}

void cleg::doEncoder2(){
  // Test transition
  A1_set = digitalRead(encoder1PinA) == HIGH;
  // and adjust encoder1Poser + if A leads B
  encoder1Pos += (A1_set != B1_set) ? +1 : -1;
	return;
}

void cleg::forward(int pwm){
	digitalWrite(motorA1, LOW);
	digitalWrite(motorA2, HIGH);
	analogWrite(motorA_PWM, pwm);
	return;
}