#ifndef PIDcontroller_h
#define PIDcontroller_h
#include "Arduino.h"

class PIDcontroller{
	public:
	PIDcontroller(double kp, double ki, double kd, double bIn, double cIn, int sampleRate);
    int compute(double, double);
    void resetTerms();
    double getError();

		
	private:  
    double Kp;
    double Ki;
    double Kd;
    double b;
    double c;
    int sampleTime;
        
    double error;
    double errP;
    double errD;
    double iTerm;
    double dTerm;
    double lastErrD;
    unsigned long lastTime;
    int pidTerm; 
};

#endif
