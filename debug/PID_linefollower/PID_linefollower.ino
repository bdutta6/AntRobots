#include <QTRSensors.h>
#include "MotorBoard.h"
#include "Arduino.h"


#define Kp .1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 20*Kp // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 

#define rightMaxSpeed 125 // max speed of the robot
#define leftMaxSpeed 125 // max speed of the robot
#define rightBaseSpeed 75 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 75  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  3     // number of sensors used
#define TIMEOUT       2000  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN     // emitter is controlled by digital pin 2

//#define rightMotor1 9
////#define rightMotor2 4
//#define rightMotorPWM 8
//#define leftMotor1 12
////#define leftMotor2 13
//#define leftMotorPWM 11
////#define motorPower 8

MotorBoard Drive;

QTRSensorsRC qtrrc((unsigned char[]) { 49, 51, 53} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  Serial.begin(9600);
//  pinMode(rightMotor1, OUTPUT); // enable 1
////  pinMode(rightMotor2, OUTPUT); // unneccessary for this type of H bridge
//  pinMode(rightMotorPWM, OUTPUT); //this is the phase pin
//  
//  pinMode(leftMotor1, OUTPUT);
////  pinMode(leftMotor2, OUTPUT); // unneccessary for this type of H bridge
//  pinMode(leftMotorPWM, OUTPUT); // phase pin
//  
////  pinMode(motorPower, OUTPUT); // standby pin which is unnecessary for this type of H bridge

// the above was rendered unnecessary with MotorBoard inclusions
  
  int i;
for (int i = 0; i < 100; i++){ // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

//  /* comment this part out for automatic calibration 
  if ( i  < 25 || i >= 75 ){ // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right();  
  }
   else {
     turn_left(); 
   }
//   */
   qtrrc.calibrate();   
   delay(10);
}
wait();  
Serial.println("calibrated");
delay(2000); // wait for 2s to position the bot before entering the main loop 
    
    /* comment out for serial printing
    
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    */
  } 

int lastError = 0;

//rightMotorSpeed and leftMotorSpeed are pwm values

void loop()
{

  unsigned int sensors[3];
  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  Serial.print(String(position)); Serial.print("  ");
  int error = 1000 - position;
  Serial.print(String(error)); Serial.print("  ");

  int motorSpeed = Kp * error + Kd * (error - lastError);
  Serial.print(String(motorSpeed)); Serial.print("  ");
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  // atm only the left side is moving and i dont know how to fix that exactly unless i know the position value
  
    if (rightMotorSpeed > rightMaxSpeed )  rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;  // keep the motor speed positive
//    else {
//      rightMotorSpeed = rightMotorSpeed;
//    }
    
  if (leftMotorSpeed > leftMaxSpeed )  leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed < 0)  leftMotorSpeed = 0; // keep the motor speed positive
//  else{
//    leftMotorSpeed = leftMotorSpeed;
//  }

  Serial.print(String(leftMotorSpeed)); Serial.print("  ");
  Serial.println(String(rightMotorSpeed));
  
////  digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
//  digitalWrite(rightMotor1, HIGH);
////  digitalWrite(rightMotor2, LOW);
//  analogWrite(rightMotorPWM, rightMotorSpeed);
////  digitalWrite(motorPower, HIGH);
//  digitalWrite(leftMotor1, HIGH);
////  digitalWrite(leftMotor2, LOW);
//  analogWrite(leftMotorPWM, leftMotorSpeed);

  Drive.RightForward(rightMotorSpeed);
  Drive.LeftForward(leftMotorSpeed);
}
  
void wait(){
//    digitalWrite(motorPower, LOW); //stops the robot 
  Drive.RightStop();
  Drive.LeftStop();
}

void turn_right(){
  Drive.RightBackward(100);
  Drive.LeftForward(100);
}
void turn_left(){
  Drive.RightForward(100);
  Drive.LeftBackward(100);
}

