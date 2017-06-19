#include "MotorBoard.h"
#include "Arduino.h"
#include "definitions.h"

MotorBoard Drive;

void setup() {
  Serial.begin(9600);
  delay(5000);
}

bool CSon_tape= false;
bool LSon_tape= false;
bool RSon_tape= false;
int LSread = 0;
int CSread = 0;
int RSread = 0;
int readings [3] = {0,0,0};
bool L = false;
bool F = false;
bool R = false;
int tL = 0;
int tR = 0;
bool C = false;

void loop() {
  
  readings[0] = readQD(left_sensor);
  readings[1] = readQD(center_sensor);
  readings[2] = readQD(right_sensor);

  TAPE_CHECK();
  Serial.print(LSon_tape); Serial.print(" ");
  Serial.print(CSon_tape); Serial.print(" ");
  Serial.print(RSon_tape); Serial.println("  in main loop");

  if (!LSon_tape && CSon_tape && !RSon_tape){
    //forward
    Drive.RightForward(BASE_SPEED_F);
    Drive.LeftForward(BASE_SPEED_F);
    F = true;
    L = false;
    R = false; 
  }
  else if (LSon_tape && CSon_tape && !RSon_tape){
    // left and center so do a curve to the left
    LEFT_CURVE();
  }
  else if (!LSon_tape && CSon_tape && RSon_tape){
    // left and center so do a curve to the left
    RIGHT_CURVE();
  }
  else if (LSon_tape && !RSon_tape){
    LEFT_TURN();
    L = true;
    F = false;
    R = false;
  }
  else if (!LSon_tape && RSon_tape){
    RIGHT_TURN();
    R = true;
    F = false;
    L = false;
  }
  else {
    // continue previous behavior
    if (F){
      Drive.RightForward(BASE_SPEED_F);
      Drive.LeftForward(BASE_SPEED_F);
    }
    else if (L){
      Drive.RightForward(BASE_SPEED_F);
      Drive.LeftBackward(BASE_SPEED_B);
    }
    else if (R){
      Drive.RightBackward(BASE_SPEED_B);
      Drive.LeftForward(BASE_SPEED_F);
    }
//    Drive.RightBackward(BASE_SPEED_F);
//    Drive.LeftBackward(BASE_SPEED_F);
  }
}



int readQD(int QRE1113_Pin){
  //Returns value from the QRE1113 
  //Lower numbers mean more refleacive
  //More than 3000 means nothing was reflected.
  pinMode( QRE1113_Pin, OUTPUT );
  digitalWrite( QRE1113_Pin, HIGH );  
  delayMicroseconds(10);
  pinMode( QRE1113_Pin, INPUT );

  long time = micros();

  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while (digitalRead(QRE1113_Pin) == HIGH && micros() - time < 3000); 
  int diff = micros() - time;

  return diff;
}

void TAPE_CHECK(){
  int LSread = readings[0];
  int CSread = readings[1];
  int RSread = readings[2];


  if ( LSread >= black_tape){
    LSon_tape= true;
  }
  else if ( LSread < black_tape){
    LSon_tape= false;
  }

  if ( RSread >= black_tape){
    RSon_tape= true;
  }
  else if ( RSread < black_tape){
    RSon_tape= false;
  }
  
  if ( CSread >= black_tape){
    CSon_tape = true;
  }
  else if ( CSread < black_tape){
    CSon_tape = false;
  }
}

void LEFT_TURN(){
  Drive.RightForward(BASE_SPEED_F);
//  Drive.LeftForward(60);
  Drive.LeftBackward(BASE_SPEED_B);
  
  while (LSon_tape && !CSon_tape){
    readings[0] = readQD(left_sensor);
    readings[1] = readQD(center_sensor);
    readings[2] = readQD(right_sensor);
    TAPE_CHECK();
    Serial.print(LSon_tape); Serial.print(" ");
    Serial.print(CSon_tape); Serial.print(" ");
    Serial.print(RSon_tape); Serial.println("  in while LSon_tape");

    if (CSon_tape && LSon_tape){
      bool oldLeft = LSon_tape; //basically it's true
      bool oldCenter = CSon_tape;
      bool oldRight = RSon_tape;
      while (oldLeft == LSon_tape && CSon_tape){
//        Drive.RightForward(60);
        Drive.RightBackward(BASE_SPEED_B);
        Drive.LeftForward(BASE_SPEED_F);
      
        readings[0] = readQD(left_sensor);
        readings[1] = readQD(center_sensor);
        readings[2] = readQD(right_sensor);
        TAPE_CHECK();
        Serial.print(LSon_tape); Serial.print(" ");
        Serial.print(CSon_tape); Serial.print(" ");
        Serial.print(RSon_tape); Serial.println("  in while oldLeft == LSon_tape");
      } //this marks when only the center sensor is on the line
      Drive.RightForward(BASE_SPEED_F);
      Drive.LeftForward(BASE_SPEED_F);
    }
  }
  Drive.RightForward(BASE_SPEED_F);
  Drive.LeftForward(BASE_SPEED_F);
}

void RIGHT_TURN(){
  Drive.RightBackward(BASE_SPEED_B);
  Drive.LeftForward(BASE_SPEED_F);
  
  while (RSon_tape && !CSon_tape){
  readings[0] = readQD(left_sensor);
  readings[1] = readQD(center_sensor);
  readings[2] = readQD(right_sensor);
  TAPE_CHECK();
  Serial.print(LSon_tape); Serial.print(" ");
  Serial.print(CSon_tape); Serial.print(" ");
  Serial.print(RSon_tape); Serial.println("  in while RSon_tape");
  
  if (CSon_tape & RSon_tape){
    bool oldLeft = LSon_tape; 
    bool oldCenter = CSon_tape;
    bool oldRight = RSon_tape; //basically it's true
    while (oldRight == RSon_tape && CSon_tape){
      Drive.RightForward(BASE_SPEED_F);
      Drive.LeftBackward(BASE_SPEED_B);
            
      readings[0] = readQD(left_sensor);
      readings[1] = readQD(center_sensor);
      readings[2] = readQD(right_sensor);
      TAPE_CHECK();
      Serial.print(LSon_tape); Serial.print(" ");
      Serial.print(CSon_tape); Serial.print(" ");
      Serial.print(RSon_tape); Serial.println("  in while oldRight == RSon_tape");
    } //this marks when only the center sensor is on the line
    Drive.RightForward(BASE_SPEED_F);
    Drive.LeftForward(BASE_SPEED_F);
  }
  }
  Drive.RightForward(BASE_SPEED_F);
  Drive.LeftForward(BASE_SPEED_F);
}

void LEFT_CURVE(){
  Drive.RightForward(BASE_SPEED_F);
  Drive.LeftForward(60);
}

void RIGHT_CURVE(){
  Drive.RightForward(60);
  Drive.LeftForward(BASE_SPEED_F);  
}

void STOP(){
  Drive.RightStop();
  Drive.LeftStop();
}

