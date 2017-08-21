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
  encoderPos = 0;

	return;
}

//// Interrupt on A changing state
//void cleg::doEncoder1(){
//  // Test transition
//  aSet = digitalRead(encoderPinA) == HIGH;
//  // and adjust encoderPoser + if A leads B
//  encoderPos += (aSet != bSet) ? +1 : -1;
//	return;
//}
//
//void cleg::doEncoder2(){
//  // Test transition
//  bSet = digitalRead(encoderPinB) == HIGH;
//  // and adjust encoderPoser + if A leads B
//  encoderPos += (aSet != bSet) ? +1 : -1;
//	return;
//}

void cleg::rotCW(int pwm){
	digitalWrite(motor1, LOW);
	digitalWrite(motor2, HIGH);
	analogWrite(motorPWM, pwm);
	return;
}

void cleg::rotCCW(int pwm){
  digitalWrite(motor1, HIGH);
  digitalWrite(motor2, LOW);
  analogWrite(motorPWM, pwm);
  return;
}

//void cleg::interupt_attach(){
//  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoder1, void (cleg::) CHANGE);
//  attachInterrupt(&digitalPinToInterrupt(encoderPinB), doEncoder2(), CHANGE);
//  return;
//}

//void cleg::checkHall(){
//  encoderPos = 0;
//  magAtHall = true;
//  detachInterrupt(digitalPinToInterrupt(HallEffect));
//}
//
//void cleg::zeroPosition(){
// attachInterrupt(digitalPinToInterrupt(HallEffect), checkHall, FALLING);
// magAtHall = false;
//// Serial.println(magAtHall);
// while(!magAtHall){
//  rotCW(50);
////  Serial.println("in start loop");
// }
// unsigned long now_A = millis();
//  while (millis() - now_A < 5000){
//    posA_set = 0;
//    posA_act = ((encoder1Pos)/(4480.0));
//    Serial.print(posA_set); Serial.print(",");
//    Serial.print(posA_act); Serial.print(",");
//    PWM_A_val= updatePid_A(PWM_A_val);
//    Serial.print(error_A);Serial.print(",");
//    Serial.println(PWM_A_val);
//    
//    if (PWM_A_val >= 0) rotCW(abs(PWM_A_val));
//    else rotCCW(abs(PWM_A_val));
//  }
//}

