#include "cleg.h"   //loads a custom library to set up drive motors
#include <math.h>
#include "Filter.h"
#include "PIDcontroller.h"

// swapped the A and B motors to better reflect the hildebrand model
#define motorA1 16 //A
#define motorA2 17 //B
#define motorA_PWM 4
#define encoder1PinA 44
#define encoder1PinB 42
#define hallEffect_A 13

#define motorB1 14 //A
#define motorB2 15 //B
#define motorB_PWM 2
#define encoder2PinA 40
#define encoder2PinB 38
#define hallEffect_B 10

#define motorC1 21 //A
#define motorC2 20 //B
#define motorC_PWM 5
#define encoder3PinA 52
#define encoder3PinB 50
#define hallEffect_C 12

#define motorD1 18 //A
#define motorD2 19 //B
#define motorD_PWM 3
#define encoder4PinA 48
#define encoder4PinB 46
#define hallEffect_D 11


double const pi = 3.14159;
double filterWeight = 25;
unsigned long dt = 0;

volatile long encoder1Pos = 0;                  //rev counter
volatile long encoder2Pos = 0;
volatile long encoder3Pos = 0;
volatile long encoder4Pos = 0;

bool A1_set = false;
bool B1_set = false;

bool A2_set = false;
bool B2_set = false;

bool A3_set = false;
bool B3_set = false;

bool A4_set = false;
bool B4_set = false;

unsigned long cur_time_A;
unsigned long cur_time_B;
unsigned long cur_time_C;
unsigned long cur_time_D;

double posA_set = 0.50;                               // position (Set Point) (in revolution)
double posA_act = 0.0;                                // position (actual value) (in revolution)
double velA_set = 1.0;                                // velocity (set point) (in rev per sec)
double velA_act = 0.0;                                // velocity (actual value) (in rev per sec)
int PWM_A_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posB_set = 0.0;                               // position (Set Point) (in revolution)
double posB_act = 0.0;                                // position (actual value) (in revolution)
double velB_set = 0.0;
double velB_act = 0.0;
int PWM_B_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posC_set = 0.0;                               // position (Set Point) (in revolution)
double posC_act = 0;                                // position (actual value) (in revolution)
double velC_set = 0.0;
double velC_act = 0.0;
int PWM_C_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posD_set = 0.0;                               // position (Set Point) (in revolution)
double posD_act = 0;                                // position (actual value) (in revolution)
double velD_set = 0.0;
double velD_act = 0.0;
int PWM_D_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double Kp_pos =   1015 + 50; //1390.9; //620.9375; //397.4; //655.71; //1168.8;                      // PID proportional control Gain
double Kd_pos =   65316.3 - 10000; //93886; //44353; //59610; //98357; //65746.3;                              // PID Derivitave control gain
double Ki_pos =   0.05; //7.7272; //0.6272; //1.7662; //2.9143; //5.1948;
double b_pos = 1.0;  //between 0 and 1
double c_pos = 0.25;  //between 0 and 1
int sampleRate_pos = 10; //milliseconds

double Kp_vel =  900;
double Kd_vel =  0;
double Ki_vel =  0;
double b_vel = 1;
double c_vel = 1;
int sampleRate_vel = 5; //milliseconds

float output = 0.0;

//insert PID variables here for A, B, C, & D


String magRead_A = "OFF";
String magRead_B = "OFF";
String magRead_C = "OFF";
String magRead_D = "OFF";
double standing_pos = 0.5;


float min_stance = 0.5;//0.5 // this is just an estimate that could be improved through some form of contact sensor on the cleg
float max_stance = 0.7;
float gait_period = 1.97;
float stance_duration = 0.75; // 75% stance duration (taken from the hildebrand graph)
float phase_lag = 0.25; // 25% phase lag (taken from hildebrand graph)

// modify the above variables to model different types of gaits

float stance_phase = max_stance - min_stance; // 0.2
float swing_phase = 1 - stance_phase; // 0.8

float stance_cycle = gait_period * stance_duration;
float swing_cycle = gait_period - stance_cycle;

float st_f = stance_phase / stance_cycle; // the speed of the cleg during the stance period (in rev per sec)
float sw_f = swing_phase / swing_cycle; // the speed of the cleg during the swing period 


cleg m1(motorA1, motorA2, motorA_PWM, encoder1PinA, encoder1PinB);
cleg m2(motorB1, motorB2, motorB_PWM, encoder2PinA, encoder2PinB);
cleg m3(motorC1, motorC2, motorC_PWM, encoder3PinA, encoder3PinB);
cleg m4(motorD1, motorD2, motorD_PWM, encoder4PinA, encoder4PinB);

ExponentialFilter<double> VelFilterA(filterWeight, 0);
ExponentialFilter<double> VelFilterB(filterWeight, 0);
ExponentialFilter<double> VelFilterC(filterWeight, 0);
ExponentialFilter<double> VelFilterD(filterWeight, 0);

PIDcontroller posControlA(Kp_pos, Ki_pos, Kd_pos, b_pos, c_pos, sampleRate_pos);
PIDcontroller velControlA(Kp_vel, Ki_vel, Kd_vel, b_vel, c_vel, sampleRate_vel);

PIDcontroller posControlB(Kp_pos, Ki_pos, Kd_pos, b_pos, c_pos, sampleRate_pos);
PIDcontroller velControlB(Kp_vel, Ki_vel, Kd_vel, b_vel, c_vel, sampleRate_vel);

PIDcontroller posControlC(Kp_pos, Ki_pos, Kd_pos, b_pos, c_pos, sampleRate_pos);
PIDcontroller velControlC(Kp_vel, Ki_vel, Kd_vel, b_vel, c_vel, sampleRate_vel);

PIDcontroller posControlD(Kp_pos, Ki_pos, Kd_pos, b_pos, c_pos, sampleRate_pos);
PIDcontroller velControlD(Kp_vel, Ki_vel, Kd_vel, b_vel, c_vel, sampleRate_vel);


void setup() {
  // put your setup code here, to run once:
    pinMode(encoder1PinA, INPUT);
    pinMode(encoder1PinB, INPUT);
    pinMode(hallEffect_A, INPUT);
    digitalWrite(encoder1PinA, HIGH);                      // turn on pullup resistor
    digitalWrite(encoder1PinB, HIGH);

    pinMode(encoder2PinA, INPUT);
    pinMode(encoder2PinB, INPUT);
    pinMode(hallEffect_B, INPUT);
    digitalWrite(encoder2PinA, HIGH);
    digitalWrite(encoder2PinB, HIGH);

    pinMode(encoder3PinA, INPUT);
    pinMode(encoder3PinB, INPUT);
    pinMode(hallEffect_C, INPUT);
    digitalWrite(encoder3PinA, HIGH);
    digitalWrite(encoder3PinB, HIGH);

    pinMode(encoder4PinA, INPUT);
    pinMode(encoder4PinB, INPUT);
    pinMode(hallEffect_D, INPUT);
    digitalWrite(encoder4PinA, HIGH);
    digitalWrite(encoder4PinB, HIGH);

    attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoder2B, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(encoder3PinA), doEncoder3A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder3PinB), doEncoder3B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoder4PinA), doEncoder4A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder4PinB), doEncoder4B, CHANGE);


  Serial.begin (38400); // 74880
  
  unsigned long t1 = millis();
   zeroPosition_all();
  dt = millis() - t1;
  // stand();
  // prep_for_gait();
  // set_offset_lateral_gait();
  // set_180_offset_gait();
  dt = millis() - t1;
}

unsigned long t = millis();
double old_posA_act = 0.0;
unsigned long oldTime_A = 0.0;
double timeInterval_A = 0.0;
double filtVelA_act = 0.0;
void loop() {
  cur_time_A = millis() - dt;
    Serial.print(cur_time_A); Serial.print(",");
    
    
    posA_act = ((encoder1Pos) / (4480.0)); // outputs current position in terms of revolution
    
    // velA_set = calculate_set_speed(posA_act); // changes the rotational speed modifier based on the position of the cleg
    
    // PWM_A_val = posControlA.compute(posA_set, posA_act);
    if (cur_time_A - oldTime_A >= sampleRate_vel) {
        velA_act = (posA_act - old_posA_act) / (sampleRate_vel/1000.0);
        if (isnan(velA_act)) velA_act = 0.0;
        
        PWM_A_val = velControlA.compute(velA_set, velA_act);
        old_posA_act = posA_act;
        oldTime_A = cur_time_A;
    }
    
    
    
    Serial.print(velControlA.getError());Serial.print(",");
    Serial.print(velA_set);Serial.print(",");
    Serial.println(velA_act);//Serial.print(",");
    // Serial.print(set_speed_A);Serial.print(",");
    // Serial.println(act_speed_A);
    
  
  
  
    if (PWM_A_val >= 0) {
        m1.rotCW(abs(PWM_A_val));
    }
    else {
        m1.rotCCW(abs(PWM_A_val));
    }

    if (PWM_B_val >= 0) {
        m2.rotCW(abs(PWM_B_val));
    }
    else {
        m2.rotCCW(abs(PWM_B_val));
    }

    if (PWM_C_val >= 0) {
        m3.rotCCW(abs(PWM_C_val));
    }
    else {
        m3.rotCW(abs(PWM_C_val));
    }

    if (PWM_D_val >= 0) {
        m4.rotCCW(abs(PWM_D_val));
    }
    else {
        m4.rotCW(abs(PWM_D_val));
    }
    
 if (millis() - t > 10000){
   t = millis();
   posA_set+= 0.5;
   posControlA.resetTerms();
 }
}


float pos;
float calculate_set_speed(float current_pos) { // this will only output positive numbers
  //current_pos is in revolutions but it needs to be converted to degrees (this needs to be changed now)
  // float current_pos_deg = current_pos * 360;
  pos = current_pos - floor(current_pos); // this brings it down to the basic unit circle (1 rev = 0)
  
  
  if (current_pos < max_stance) {
    output = st_f;
  } 
  else if (pos >= min_stance && pos < max_stance) {
    // signifies that it is in the stance phase
  output = st_f;
  }
  else {
    // if it is not in the stance phase, it must be in the swing phase
  output = sw_f;
  }
  return (output);
}

// Interrupt on A changing state
void doEncoder1A(){
  // Test transition
  A1_set = digitalRead(encoder1PinA) == HIGH;
  // and adjust encoder1Poser + if A leads B
  encoder1Pos += (A1_set != B1_set) ? +1 : -1;
//  if (digitalRead(hallEffect_A) == HIGH) encoder1Pos = 0;
}

// Interrupt on B changing state
void doEncoder1B(){
  // Test transition
  B1_set = digitalRead(encoder1PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder1Pos += (A1_set == B1_set) ? +1 : -1;
//  if (digitalRead(hallEffect_A) == HIGH) encoder1Pos = 0;
}

void doEncoder2A(){
  A2_set = digitalRead(encoder2PinA) == HIGH;
  encoder2Pos += (A2_set != B2_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoder2B(){
  B2_set = digitalRead(encoder2PinB) == HIGH;
  encoder2Pos += (A2_set == B2_set) ? +1 : -1;
}

void doEncoder3A(){
  A3_set = digitalRead(encoder3PinA) == HIGH; //if both encoders read the same thing, its moving CCW and if they read different values, the motor is moving CW
  encoder3Pos += (A3_set != B3_set) ? +1 : -1;
}

void doEncoder3B(){
  B3_set = digitalRead(encoder3PinB) == LOW; //HIGH
  encoder3Pos += (A3_set == B3_set) ? +1 : -1;
}

void doEncoder4A(){
  A4_set = digitalRead(encoder4PinA) == HIGH;
  encoder4Pos += (A4_set != B4_set) ? +1 : -1;
}

void doEncoder4B(){
  B4_set = digitalRead(encoder4PinB) == LOW; //HIGH
  encoder4Pos += (A4_set == B4_set) ? +1 : -1;
}


void zeroPosition_all(){
  bool magAtHall_A = false;
  bool magAtHall_B = false;
  bool magAtHall_C = false;
  bool magAtHall_D = false;
  m1.rotCW(25);
  m2.rotCW(25);
  m3.rotCCW(25);
  m4.rotCCW(25);
  while (!magAtHall_A || !magAtHall_B || !magAtHall_C || !magAtHall_D) {
  if (digitalRead(hallEffect_A) == LOW){
      magAtHall_A = true;
      encoder1Pos = 0;
      m1.rotCW(0);
    }
  if (digitalRead(hallEffect_B) == LOW){
      magAtHall_B = true;
      encoder2Pos = 0;
      m2.rotCW(0);
    }
  if (digitalRead(hallEffect_C) == LOW){
      magAtHall_C = true;
      encoder3Pos = 0;
      m3.rotCCW(0);
    }
  if (digitalRead(hallEffect_D) == LOW){
      magAtHall_D = true;
      encoder4Pos = 0;
      m4.rotCCW(0);
    }
  }
  m1.rotCW(0);
  m2.rotCW(0);
  m3.rotCCW(0);
  m4.rotCCW(0);
  return;
}


void stand(){
  unsigned long t3 = millis();
  posA_set = standing_pos;
  posB_set = standing_pos;
  posC_set = standing_pos;
  posD_set = standing_pos;
  while (millis() - t3 <= 10000){

    cur_time_A = millis() - dt;
    Serial.print(cur_time_A); Serial.print(",");

    posA_act = ((encoder1Pos) / (4480.0)); 
    PWM_A_val = posControlA.compute(posA_set, posA_act);
    Serial.print(posControlA.getError());Serial.print(",");

  
    posB_act = (encoder2Pos) / (4480.0);
    PWM_B_val = posControlB.compute(posB_set, posB_act);
    Serial.print(posControlB.getError());Serial.print(",");

    posC_act = (encoder3Pos) / (4480.0);
    PWM_C_val = posControlC.compute(posC_set, posC_act);
    Serial.print(posControlC.getError());Serial.print(",");
  
    posD_act = (encoder4Pos) / (4480.0);
    PWM_D_val = posControlD.compute(posD_set, posD_act);
    Serial.print(posControlD.getError());Serial.print(",");
  
  // these if statements make the motor stop when it is within 0.025 of the the target value 
  // if (posA_act >= standing_pos-thres_stand && posA_act <= standing_pos+thres_stand) {
    // PWM_A_val = 0;
  // }
  // if (posB_act >= standing_pos-thres_stand && posB_act <= standing_pos+thres_stand) {
    // PWM_B_val = 0;
  // }
  // if (posC_act >= standing_pos-thres_stand && posC_act <= standing_pos+thres_stand) {
    // PWM_C_val = 0;
  // }
  // if (posD_act >= standing_pos-thres_stand && posD_act <= standing_pos+thres_stand) {
    // PWM_D_val = 0;
  // }
  

    if (PWM_A_val >= 0) {
        m1.rotCW(abs(PWM_A_val));
    }
    else {
        m1.rotCCW(abs(PWM_A_val));
    }

    if (PWM_B_val >= 0) {
        m2.rotCW(abs(PWM_B_val));
    }
    else {
        m2.rotCCW(abs(PWM_B_val));
    }

    if (PWM_C_val >= 0) {
        m3.rotCCW(abs(PWM_C_val));
    }
    else {
        m3.rotCW(abs(PWM_C_val));
    }

    if (PWM_D_val >= 0) {
        m4.rotCCW(abs(PWM_D_val));
    }
    else {
        m4.rotCW(abs(PWM_D_val));
    }
  }
  return;
}
