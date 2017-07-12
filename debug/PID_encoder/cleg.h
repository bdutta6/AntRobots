#ifndef cleg_h
#define cleg_h
#include "Arduino.h"

class cleg{
	public:
		cleg();
		bool A1_set;
		bool B1_set;
		volatile long encoder1Pos;                  //rev counter

		
	private:

};

#endif