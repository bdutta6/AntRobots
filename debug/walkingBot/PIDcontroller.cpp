#include "Arduino.h"
#include "PIDcontroller.h"


 /*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PIDcontroller::PIDcontroller(double kp, double ki, double kd, double bIn, double cIn, int sampleRate) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    b = bIn;
    c = cIn;
    sampleTime = sampleRate; // try 50 ms
    return;
}


int PIDcontroller::compute(double setpoint, double actual) {
    unsigned long now = millis();
    double timeChange = now - lastTime;
    if (timeChange >= sampleTime) {
        error = setpoint - actual;
        errP = b*setpoint - actual;
        errD = c*setpoint - actual;
        
        iTerm += error * sampleTime;
        dTerm = (errD - lastErrD) / sampleTime;
        
        pidTerm = Kp*errP + Kd*dTerm + Ki*iTerm;
        
        lastErrD = errD;
        lastTime = now;
    }
    return constrain(pidTerm, -255, 255);    
}

void PIDcontroller::resetTerms() {
    //when swapping between velocity and position, i need to reset some variables so that it starts from "0"
    iTerm = 0;
    lastErrD = 0;
    lastTime = millis() - sampleTime;
    return;
}

double PIDcontroller::getError() {
    return error;
}



