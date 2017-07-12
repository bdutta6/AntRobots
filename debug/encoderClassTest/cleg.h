#ifndef cleg_h
#define cleg_h
#include "Arduino.h"

class cleg{
	public:
		cleg(int, int, int, int, int);
	
		int motor1;
		int motor2;
		int motorPWM;
		int encoderPinA;
		int encoderPinB;
		
		bool aSet;
		bool bSet;
		volatile long encoderPos;                  //rev counter
		void doEncoder1();
		void doEncoder2();
		void rotCW(int);
		void rotCCW(int);

		
	private:

};

#endif