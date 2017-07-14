#include "cleg.h"   //loads a custom library to set up drive motors

#define motorA1 14 //53 //A
#define motorA2 15 //51 //B
#define motorA_PWM 2
#define encoder1PinA 52
#define encoder1PinB 50

#define motorB1 16 //49 //A
#define motorB2 17 //47 //B
#define motorB_PWM 3
#define encoder2PinA 48 
#define encoder2PinB 46

#define motorC1 18 //45 //A
#define motorC2 19 //43 //B
#define motorC_PWM 4
#define encoder3PinA 44 
#define encoder3PinB 42

#define motorD1 41 //A
#define motorD2 39 //B
#define motorD_PWM 5
#define encoder4PinA 40 
#define encoder4PinB 38

cleg m1(motorA1, motorA2, motorA_PWM, encoder1PinA, encoder1PinB);
cleg m2(motorB1, motorB2, motorB_PWM, encoder2PinA, encoder2PinB);
cleg m3(motorC1, motorC2, motorC_PWM, encoder3PinA, encoder3PinB);
cleg m4(motorD1, motorD2, motorD_PWM, encoder4PinA, encoder4PinB);
void setup() {

	Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm class
	attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder2m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder1m2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoder2m2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinA), doEncoder1m3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinB), doEncoder2m3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4PinA), doEncoder1m4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4PinB), doEncoder2m4, CHANGE);

}

void loop() {
	m1.rotCW(255);
//  m2.rotCW(255);
//  m3.rotCCW(255);
//  m4.rotCCW(255);
}

void doEncoder1m1(){
	m1.doEncoder1();
}

void doEncoder2m1(){
	m1.doEncoder2();
}

void doEncoder1m2(){
  m2.doEncoder1();
}

void doEncoder2m2(){
  m2.doEncoder2();
}

void doEncoder1m3(){
  m3.doEncoder1();
}

void doEncoder2m3(){
  m3.doEncoder2();
}

void doEncoder1m4(){
  m4.doEncoder1();
}

void doEncoder2m4(){
  m4.doEncoder2();
}
