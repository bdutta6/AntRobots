#include "cleg.h"   //loads a custom library to set up drive motors

#define motorA1 53 //A
#define motorA2 51 //B
#define motorA_PWM 6
#define encoder1PinA 2
#define encoder1PinB 3

cleg m1(motorA1, motorA2, motorA_PWM, encoder1PinA, encoder1PinB);

void setup() {

	Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm class
	attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder2m1, CHANGE);

}

void loop() {
	m1.rotCW(255);
}

void doEncoder1m1(){
	m1.doEncoder1();
}

void doEncoder2m1(){
	m1.doEncoder2();
}