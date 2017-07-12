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
cleg::cleg(int pin1, int pin2, int pin3, int pin4, int pin5){
	
	motor1 = pin1;
	motor2 = pin2;
	motorPWM = pin3;
	encoderPinA = pin4;
	encoderPinB = pin5;
	
	pinMode(motor1, OUTPUT);
	pinMode(motor2, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
	
	aSet = false;
	bSet = false;
	encoderPos = 0;                  //rev counter

	return;
}

// Interrupt on A changing state
void cleg::doEncoder1(){
  // Test transition
  aSet = digitalRead(encoderPinA) == HIGH;
  // and adjust encoderPoser + if A leads B
  encoderPos += (aSet != bSet) ? +1 : -1;
	return;
}

void cleg::doEncoder2(){
  // Test transition
  aSet = digitalRead(encoderPinA) == HIGH;
  // and adjust encoderPoser + if A leads B
  encoderPos += (aSet != bSet) ? +1 : -1;
	return;
}

void cleg::rotCW(int pwm){
	digitalWrite(motor1, LOW);
	digitalWrite(motor2, HIGH);
	analogWrite(motorPWM, pwm);
	return;
}