#include "cleg.h"   //loads a custom library to set up drive motors

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

volatile long encoder1Pos = 0;                  //rev counter
volatile long encoder2Pos = 0;
volatile long encoder3Pos = 0;
volatile long encoder4Pos = 0;

boolean A1_set = false;
boolean B1_set = false;

boolean A2_set = false;
boolean B2_set = false;

boolean A3_set = false;
boolean B3_set = false;

boolean A4_set = false;
boolean B4_set = false;

double posA_set = 0.0;                               // position (Set Point) (in revolution)
double posA_act = 0.0;                                // position (actual value) (in revolution)
int PWM_A_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posB_set = 0.0;                               // position (Set Point) (in revolution)
double posB_act = 0.0;                                // position (actual value) (in revolution)
int PWM_B_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posC_set = 0.0;                               // position (Set Point) (in revolution)
double posC_act = 0;                                // position (actual value) (in revolution)
int PWM_C_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posD_set = 0.0;                               // position (Set Point) (in revolution)
double posD_act = 0;                                // position (actual value) (in revolution)
int PWM_D_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double Kp =   0.6375*3.5;//0.6375*2.5;//0.6375*2.85;//10.0//5.3  //0.85*5//2.125;//0.6375; //10 //13 //60;       //critical Kp = 1.0625                         // PID proportional control Gain
double Kd =   0.9371*9.5;//0.9371*9000;//0.9371*10;//9.6 //6.4  //1.8742*3//1.2495;//0.9371; //925 //435 //1200;                                // PID Derivitave control gain
double Ki =   0.1084*.075;//0.1084*.0001;//0.1084*.0085;//0.1626 //0.0813 // 0.0271 //0.00000000000000001;//1.5;
double Ka =   0.025;// * 5;   // integral feedback gain to prevent reset windup
double Kb =   0.4;


unsigned long lastTime_A;
int pidTerm_A = 0;                                                            // PID correction
double error_A = 0;
double last_error_A = 0;
double dErr_A = 0;
double iTerm_A = 0;
//double outMaxI_A = 0;
//double outMinI_A = 0;
double PID_feedback_A = 0.0;
int output_A = 0;
double lastInput_A = 0;
double dInput_A = 0;
bool magAtHall_A = false;
double timeChange_A = 0.0;

unsigned long lastTime_B;
int pidTerm_B = 0;                                                            // PID correction
double error_B = 0;
double last_error_B = 0;
double dErr_B = 0;
double iTerm_B = 0;
double PID_feedback_B = 0.0;
int output_B = 0;
double lastInput_B = 0;
double dInput_B = 0;
bool magAtHall_B = false;

unsigned long lastTime_C;
int pidTerm_C = 0;                                                            // PID correction
double error_C = 0;
double last_error_C = 0;
double dErr_C = 0;
double iTerm_C = 0;
double PID_feedback_C = 0.0;
int output_C = 0;
double lastInput_C = 0;
double dInput_C = 0;
bool magAtHall_C = false;

unsigned long lastTime_D;
int  pidTerm_D = 0;                                                            // PID correction
double error_D = 0;
double last_error_D = 0;
double dErr_D = 0.0;
double iTerm_D = 0;
double PID_feedback_D = 0.0;
int output_D = 0;
double lastInput_D = 0;
double dInput_D = 0;
bool magAtHall_D = false;

unsigned long cur_time_A;
unsigned long cur_time_B;
unsigned long cur_time_C;
unsigned long cur_time_D;

float output = 0.0;
float fast_val = 1.5;
float slow_val = 0.5;

float set_speed_A = 1.0;
float old_set_speed_A = 0.0;
float old_posA_set = 0.0;
float act_speed_A = 0.0;

float speed_B = 1.0;
float old_speed_B = 0.0;
float old_posB_set = 0.0;

float speed_C = 1.0;
float old_speed_C = 0.0;
float old_posC_set = 0.0;

float speed_D = 1.0;
float old_speed_D = 0.0;
float old_posD_set = 0.0;

float out = 0;
double pi = 3.14159;
unsigned long t = 0;

String magRead_A = "OFF";
String magRead_B = "OFF";
String magRead_C = "OFF";
String magRead_D = "OFF";

float f = 1.0;
float f_stand = 1.0;
unsigned long dt = 0;
float standing_pos = 0.7;//0.65
float thres_stand = 0.13;//0.1
float thres_PWM = 50;

float gait_prep_pos = 0.4;
float f_prep = 1.0;
float thres_prep = 0.05;
unsigned long t4 = 0;
unsigned long t5 = 0;
unsigned long t6 = 0;
unsigned long t7 = 0;

float intercept_A = 0;
float intercept_B = 0;
float intercept_C = 0;
float intercept_D = 0;

int pwm = 0;

float min_stance = 0.5;//0.5 // this is just an estimate that could be improved through some form of contact sensor on the cleg
float max_stance = 0.7;
float gait_period = 0.5;
float stance_duration = 0.75; // 75% stance duration (taken from the hildebrand graph)
float phase_lag = 0.25; // 25% phase lag (taken from hildebrand graph)

// modify the above variables to model different types of gaits

float stance_phase = max_stance - min_stance; // 0.2
float swing_phase = 1 - stance_phase; // 0.8

float stance_cycle = gait_period * stance_duration;
float swing_cycle = gait_period - stance_cycle;

float st_f = stance_phase / stance_cycle; // the speed of the cleg during the stance period
float sw_f = swing_phase / swing_cycle; // the speed of the cleg during the swing period 

float x = 0;
float m = (st_f-sw_f)/(swing_phase/2);//peak at sw_f    //(st_f - sw_f)*(4/swing_phase);(same integral as square)
float b = sw_f;     //2*sw_f - st_f;
float a = (st_f-sw_f)/(pow(swing_phase/2,2));     //(st_f - sw_f)* (6/(pow(swing_phase,2.0)));          //(st_f - sw_f)* (6/(swing_phase^2));     // '^' does not mean to the power in c++
float c = sw_f;     //0.5*(3*sw_f - st_f);                    //st_f - (swing_phase^2)*(a/4);
float d = (st_f-sw_f)/swing_phase;//line from swing to stance


cleg m1(motorA1, motorA2, motorA_PWM, encoder1PinA, encoder1PinB);
cleg m2(motorB1, motorB2, motorB_PWM, encoder2PinA, encoder2PinB);
cleg m3(motorC1, motorC2, motorC_PWM, encoder3PinA, encoder3PinB);
cleg m4(motorD1, motorD2, motorD_PWM, encoder4PinA, encoder4PinB);

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


  Serial.begin (9600);
  unsigned long t1 = millis();
  zeroPosition_all();
  dt = millis() - t1;
  // stand();
  // prep_for_gait();
  // set_offset_lateral_gait();
  // dt = millis() - t1;

//  while(1){
//    m1.rotCW(255);
//    m2.rotCW(255);
//    m3.rotCCW(255);
//    m4.rotCCW(255);
//  }
}


void loop() {
  
    if (t4 == 0) cur_time_A = millis() - dt;
  else cur_time_A = millis() - t4;
  Serial.print(cur_time_A); Serial.print(",");

//  posA_set = 0.0;

  posA_act = ((encoder1Pos) / (4480.0)); // outputs current position in terms of revolution


   // set_speed_A = calculate_set_speed(posA_act); // changes the rotational speed modifier based on the position of the cleg
   // if (set_speed_A != old_set_speed_A){
     // intercept_A = old_posA_set - (set_speed_A/1000.0)*cur_time_A; // works according to the equation of a line and it is necessary for a smooth change in speed
   // }
  
   posA_set = (cur_time_A)*(set_speed_A/1000.0) + intercept_A;

  ////  oldTime_A = cur_time_A;
   old_posA_set = posA_set;
   old_set_speed_A = set_speed_A;

  PWM_A_val = updatePid_A(PWM_A_val);
  // act_speed_A = dInput_A / (timeChange_A/1000.0);  // needs to be revolutions per second
  // if (PWM_A_val == 0) PWM_A_val = pwm;
  
      Serial.print(error_A);Serial.print(",");
      Serial.print(posA_set);Serial.print(",");
      Serial.println(posA_act);//Serial.print(",");
      // Serial.print(set_speed_A);Serial.print(",");
      // Serial.println(act_speed_A);

//      Serial.println(PWM_A_val);//Serial.print(",   ");

  
  // if (t5 == 0) cur_time_B = millis() - dt;
  // else cur_time_B = millis() - t5;
  // // Serial.print(cur_time_B); Serial.print(",");
// //  posB_set = 0.0;

  // posB_act = (encoder2Pos) / (4480.0);

   // speed_B = calculate_set_speed(posB_act);
   // if (speed_B != old_speed_B){
     // intercept_B = old_posB_set - (speed_B)*(cur_time_B/1000.0);
   // }
  
   // posB_set = (speed_B)*(cur_time_B/1000.0) + intercept_B;
  
  // // posB_set = f*(cur_time_B/1000.0);

   // old_posB_set = posB_set;
   // old_speed_B = speed_B;
   
   // PWM_B_val = updatePid_B(PWM_B_val);
  // if (PWM_B_val == 0) PWM_B_val = pwm;
   // Serial.print(error_B);Serial.print(",");
  
// //    Serial.print(PWM_B_val);Serial.print(",   ");




  // if (t6 == 0) cur_time_C = millis()-dt;
  // else cur_time_C = millis() - t6;
// //  Serial.print(cur_time_C); Serial.print(",");
// //  posC_set = 0.0;

    // // posC_set = f*(cur_time_C/1000.0);
// //  Serial.print(posC_set); Serial.print(",");

  // posC_act = (encoder3Pos) / (4480.0);
  
  // speed_C = calculate_set_speed(posC_act);
   // if (speed_C != old_speed_C){
     // intercept_C = old_posC_set - (speed_C/1000.0)*cur_time_C;
   // }
  
   // posC_set = (cur_time_C)* (speed_C/1000.0) + intercept_C;
// //  Serial.print(posC_act); Serial.print(",");
   // old_posC_set = posC_set;
   // old_speed_C = speed_C;
  // PWM_C_val = updatePid_C(PWM_C_val);
  // if (PWM_C_val == 0) PWM_C_val = pwm;
  // Serial.print(error_C);Serial.print(",");

// //    Serial.print(PWM_C_val);Serial.print(",   ");



  // if (t7 == 0) cur_time_D = millis()-dt;
  // else cur_time_D = millis() - t7;
// //  Serial.print(cur_time_D); Serial.print(",");
// //  posD_set = 0.0;

    // // posD_set = f*(cur_time_D/1000.0);
  
// //  Serial.print(posD_set); Serial.print(",");
  // posD_act = (encoder4Pos) / (4480.0);
// //  Serial.print(posD_act);Serial.print(",");
  
  // speed_D = calculate_set_speed(posD_act);
   // if (speed_D != old_speed_D){
     // intercept_D = old_posD_set - (speed_D/1000.0)*cur_time_D;
   // }  
  // posD_set = (cur_time_D)* (speed_D/1000.0) + intercept_D;

  // old_posD_set = posD_set;
  // old_speed_D = speed_D;
  // PWM_D_val = updatePid_D(PWM_D_val);
  // if (PWM_D_val == 0) PWM_D_val = pwm;
// //  Serial.println(PWM_D_val);

  // Serial.println(error_D);  

  

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




float calculate_set_speed(float current_pos) { // this will only output positive numbers
  //current_pos is in revolutions but it needs to be converted to degrees (this needs to be changed now)
  // float current_pos_deg = current_pos * 360;
  float pos = current_pos - floor(current_pos); // this brings it down to the basic unit circle (1 rev = 0)
  
  if (pos >= min_stance && pos < max_stance  || current_pos < max_stance) {
    // signifies that it is in the stance phase
    output = st_f;
  }
  else {
    // if it is not in the stance phase, it must be in the swing phase
  if (pos >= max_stance){
    x = pos - max_stance;
  }
  else if (pos < min_stance){
    x = pos + (1-max_stance);
  }
  // output = m*abs(x-swing_phase/2) + b; // triangular change in speed
  // output = a*(pow((x-swing_phase/2),2.0)) + c;     //a*(x-swing_phase/2)^2 + c; // parabolic change in speed
  output = d*x + sw_f; //line from swing to stance
  }
  return (output);
}



int updatePid_A(int command_A)   {            // compute PWM value
  unsigned long now_A = millis();
  timeChange_A = (double)(now_A - lastTime_A);
  error_A = posA_set - posA_act;
  //    double error_A2 = 2*pi - abs(error_A);
  //    if (error_A >= 0) error_A2 = -error_A2;
  //    if (abs(error_A2) < abs(error_A)){
  //      error_A = error_A2;
  //    }
  //    else{
  //      error_A = error_A;
  //    }
  dInput_A = posA_act - lastInput_A;
  dErr_A = dInput_A * timeChange_A;
  iTerm_A += Ki * (error_A - Ka * PID_feedback_A);
  pidTerm_A = (Kp * error_A) - (Kd * dErr_A) + iTerm_A - Kb * PID_feedback_A;
//      Serial.print(Kd*dErr_A); Serial.print(",");
//      Serial.print(iTerm_A); Serial.print(",");
//      Serial.print(Kp*error_A); Serial.print(",");
//      Serial.print(pidTerm_A);Serial.print(",");
//      Serial.print(pidTerm_A + command_A);Serial.print(",");
  last_error_A = error_A;
  lastTime_A = now_A;
  output_A = command_A + pidTerm_A;
  lastInput_A = posA_act;
  PID_feedback_A = output_A - constrain(output_A, -255, 255);
//      Serial.print(-Kb*PID_feedback_A);Serial.print(",    ");
  return constrain(output_A, -255, 255);
}

int updatePid_B(int command_B)   {             // compute PWM value
  unsigned long now_B = millis();
  double timeChange_B = (double)(now_B - lastTime_B);
  error_B = posB_set - posB_act;
  
  dInput_B = posB_act - lastInput_B;
  dErr_B = dInput_B * timeChange_B;
  iTerm_B += Ki * (error_B - Ka * PID_feedback_B);

  pidTerm_B = (Kp * error_B) - (Kd * dErr_B) + iTerm_B - Kb * PID_feedback_B;
  last_error_B = error_B;
  lastTime_B = now_B;
  output_B = command_B + pidTerm_B;
  lastInput_B = posB_act;
  PID_feedback_B = output_B - constrain(output_B, -255, 255);
  return constrain(output_B, -255, 255);
}

int updatePid_C(int command_C)   {             // compute PWM value
  unsigned long now_C = millis();
  double timeChange_C = (double)(now_C - lastTime_C);
  error_C = posC_set - posC_act;
  
  dInput_C = posC_act - lastInput_C;
  dErr_C = dInput_C * timeChange_C;
  iTerm_C += Ki * (error_C - Ka * PID_feedback_C);

  pidTerm_C = (Kp * error_C) - (Kd * dErr_C) + iTerm_C - Kb * PID_feedback_C;
  last_error_C = error_C;
  lastTime_C = now_C;
  output_C = command_C + pidTerm_C;
  lastInput_C = posC_act;
  PID_feedback_C = output_C - constrain(output_C, -255, 255);
  return constrain(output_C, -255, 255);
}

int updatePid_D(int command_D)   {             // compute PWM value
  unsigned long now_D = millis();
  double timeChange_D = (double)(now_D - lastTime_D);
  error_D = posD_set - posD_act;
  
  dInput_D = posD_act - lastInput_D;
  dErr_D = dInput_D * timeChange_D;
  iTerm_D += Ki * (error_D - Ka * PID_feedback_D);

  pidTerm_D = (Kp * error_D) - (Kd * dErr_D) + iTerm_D - Kb * PID_feedback_D;
  last_error_D = error_D;
  lastTime_D = now_D;
  output_D = command_D + pidTerm_D;
  lastInput_D = posD_act;
  PID_feedback_D = output_D - constrain(output_D, -255, 255);
  return constrain(output_D, -255, 255);
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
  magAtHall_A = false;
  magAtHall_B = false;
  magAtHall_C = false;
  magAtHall_D = false;
  m1.rotCW(25);
  m2.rotCW(25);
  m3.rotCCW(23);
  m4.rotCCW(24);
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
//  delay(1000.0);
//  unsigned long now = millis();
//  unsigned long now2 = millis();
//  while(now2 - now < t){
//    Serial.print(now2); Serial.print(",");
//    posA_act = ((encoder1Pos) / (4480.0));
//    posB_act = ((encoder2Pos) / (4480.0));
//    posC_act = ((encoder3Pos) / (4480.0));
//    posD_act = ((encoder4Pos) / (4480.0));
//    posA_set = 0; posB_set = 0; posC_set = 0; posD_set = 0;
//    PWM_A_val = updatePid_A(PWM_A_val);
//    PWM_B_val = updatePid_B(PWM_B_val);
//    PWM_C_val = updatePid_C(PWM_C_val);
//    PWM_D_val = updatePid_D(PWM_D_val);
//    Serial.print(error_A);Serial.print(",");
//    Serial.print(error_B);Serial.print(",");
//    Serial.print(error_C);Serial.print(",");
//    Serial.println(error_D);
//    now2 = millis();
//  }
}

void test_Halls(){
 while(1){
   cur_time_A = millis();
   Serial.print(cur_time_A); Serial.print(",   ");
   Serial.print("magRead_A = "); Serial.print(magRead_A); Serial.print(",  ");
   Serial.print("magRead_B = "); Serial.print(magRead_B); Serial.print(",  ");
   Serial.print("magRead_C = "); Serial.print(magRead_C); Serial.print(",  ");
   Serial.print("magRead_D = "); Serial.println(magRead_D);
   magRead_A = (digitalRead(hallEffect_A) == LOW) ? "ON" : "OFF"; 
   magRead_B = (digitalRead(hallEffect_B) == LOW) ? "ON" : "OFF"; 
   magRead_C = (digitalRead(hallEffect_C) == LOW) ? "ON" : "OFF"; 
   magRead_D = (digitalRead(hallEffect_D) == LOW) ? "ON" : "OFF"; 
   m1.rotCW(0);
   m2.rotCW(0);
   m3.rotCCW(0);
   m4.rotCCW(0);
 }
}

void stand(){
  unsigned long t3 = millis();
  float a = 1.5;
  while (millis() - t3 <= 10000){

  cur_time_B = millis() - dt;
  Serial.print(cur_time_B); Serial.print(",");

  posB_act = (encoder2Pos) / (4480.0);
  if (posB_act >= standing_pos-a*thres_stand && posB_act <= standing_pos+a*thres_stand) {
    posB_set = standing_pos;
  }
  else{
    posB_set = f_stand*(cur_time_B/1000.0);
  }
//  if (posB_set >= standing_pos) posB_set = standing_pos;

  PWM_B_val = updatePid_B(PWM_B_val);
  Serial.print(error_B);Serial.print(",");


  cur_time_C = millis()-dt;
  if (posC_act >= standing_pos-a*thres_stand && posC_act <= standing_pos+a*thres_stand) {
    posC_set = standing_pos;
  }
  else{
    posC_set = f_stand*(cur_time_C/1000.0);
  }
//  if (posC_set >= standing_pos) posC_set = standing_pos;

  posC_act = (encoder3Pos) / (4480.0);
  PWM_C_val = updatePid_C(PWM_C_val);
  Serial.print(error_C);Serial.print(",");

  
  cur_time_D = millis()-dt;

  if (posD_act >= standing_pos-a*thres_stand && posD_act <= standing_pos+a*thres_stand) {
    posD_set = standing_pos;
  }
  else{
    posD_set = f_stand*(cur_time_D/1000.0);
  }
//  if (posD_set >= standing_pos) posD_set = standing_pos;
  
  posD_act = (encoder4Pos) / (4480.0);
  PWM_D_val = updatePid_D(PWM_D_val);
  Serial.print(error_D);Serial.print(",");
  
  cur_time_A = millis() - dt;

  posA_act = ((encoder1Pos) / (4480.0)); 
  if (posA_act >= standing_pos-a*thres_stand && posA_act <= standing_pos+a*thres_stand) {
    posA_set = standing_pos;
  }
  else{
    posA_set = f_stand*(cur_time_A/1000.0);
  }
//  if (posA_set >= standing_pos) posA_set =standing_pos;

  PWM_A_val = updatePid_A(PWM_A_val);
      Serial.println(error_A);
    


  // these if statements "kick" the cleg into the target range since the PID tends to stop right at the edge of the range and 
  // gravity makes it "wobble" about that edge and the cleg ends up "jittering" 
//  if (posA_act >= standing_pos-2*thres_stand && posA_act < standing_pos) PWM_A_val = thres_PWM;
//  if (posB_act >= standing_pos-2*thres_stand && posB_act < standing_pos) PWM_B_val = thres_PWM;
//  if (posC_act >= standing_pos-2*thres_stand && posC_act < standing_pos) PWM_C_val = thres_PWM;
//  if (posD_act >= standing_pos-2*thres_stand && posD_act < standing_pos) PWM_D_val = thres_PWM;
//  
//  if (posA_act <= standing_pos+2*thres_stand && posA_act > standing_pos) PWM_A_val = -thres_PWM;
//  if (posB_act <= standing_pos+2*thres_stand && posB_act > standing_pos) PWM_B_val = -thres_PWM;
//  if (posC_act <= standing_pos+2*thres_stand && posC_act > standing_pos) PWM_C_val = -thres_PWM;
//  if (posD_act <= standing_pos+2*thres_stand && posD_act > standing_pos) PWM_D_val = -thres_PWM;
  
  
  // these if statements make the motor stop when it is within 0.025 of the the target value 
  if (posA_act >= standing_pos-thres_stand && posA_act <= standing_pos+thres_stand) {
    PWM_A_val = 0;
  }
  if (posB_act >= standing_pos-thres_stand && posB_act <= standing_pos+thres_stand) {
    PWM_B_val = 0;
  }
  if (posC_act >= standing_pos-thres_stand && posC_act <= standing_pos+thres_stand) {
    PWM_C_val = 0;
  }
  if (posD_act >= standing_pos-thres_stand && posD_act <= standing_pos+thres_stand) {
    PWM_D_val = 0;
  }
  

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


void prep_for_gait(){
  unsigned long t5 = millis();
  float a = 1;//1.5
  while (millis() - t5 <= 10000){

  cur_time_B = millis() - dt;
  Serial.print(cur_time_B); Serial.print(",");

  posB_act = (encoder2Pos) / (4480.0);
  if (posB_act >= gait_prep_pos-a*thres_prep) {
    posB_set = gait_prep_pos;
  }
  else{
    posB_set = f_prep*(cur_time_B/1000.0);
  }
//  if (posB_set >= gait_prep_pos) posB_set = gait_prep_pos;

  PWM_B_val = updatePid_B(PWM_B_val);
  Serial.print(error_B);Serial.print(",");


  cur_time_C = millis()-dt;
  if (posC_act >= gait_prep_pos-a*thres_prep) {
    posC_set = gait_prep_pos;
  }
  else{
    posC_set = f_prep*(cur_time_C/1000.0);
  }
//  if (posC_set >= gait_prep_pos) posC_set = gait_prep_pos;

  posC_act = (encoder3Pos) / (4480.0);
  PWM_C_val = updatePid_C(PWM_C_val);
  Serial.print(error_C);Serial.print(",");

  
  cur_time_D = millis()-dt;

  if (posD_act >= gait_prep_pos-a*thres_prep) {
    posD_set = gait_prep_pos;
  }
  else{
    posD_set = f_prep*(cur_time_D/1000.0);
  }
//  if (posD_set >= gait_prep_pos) posD_set = gait_prep_pos;
  
  posD_act = (encoder4Pos) / (4480.0);
  PWM_D_val = updatePid_D(PWM_D_val);
  Serial.print(error_D);Serial.print(",");
  
  cur_time_A = millis() - dt;

  posA_act = ((encoder1Pos) / (4480.0)); 
  if (posA_act >= gait_prep_pos-a*thres_prep) {
    posA_set = gait_prep_pos;
  }
  else{
    posA_set = f_prep*(cur_time_A/1000.0);
  }
//  if (posA_set >= gait_prep_pos) posA_set =gait_prep_pos;

  PWM_A_val = updatePid_A(PWM_A_val);
      Serial.println(error_A);
    
  
  // these if statements make the motor stop when it is within 0.025 of the the target value 
  if (posA_act >= gait_prep_pos-thres_prep && posA_act <= gait_prep_pos+thres_prep) {
    PWM_A_val = 0;
  }
  if (posB_act >= gait_prep_pos-thres_prep && posB_act <= gait_prep_pos+thres_prep) {
    PWM_B_val = 0;
  }
  if (posC_act >= gait_prep_pos-thres_prep && posC_act <= gait_prep_pos+thres_prep) {
    PWM_C_val = 0;
  }
  if (posD_act >= gait_prep_pos-thres_prep && posD_act <= gait_prep_pos+thres_prep) {
    PWM_D_val = 0;
  }
  

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


void set_offset_lateral_gait(){
  t4 = millis();
  bool A_past_min = false;
  bool t5_flag = false;
  bool t6_flag = false;
  bool t7_flag = false;
  PWM_A_val = 0;
  PWM_B_val = 0;
  PWM_C_val = 0;
  PWM_D_val = 0;
  // include a portion where the starting leg rotates and enters the very beginning of the stance phase because rn it starts off in the swing phase
  while(millis() - t4 <= gait_period*1000.0 || A_past_min){ 
  // A portion
    cur_time_A = millis() - t4;
    Serial.print(cur_time_A); Serial.print(",");
    posA_act = (encoder1Pos) / (4480.0);
    if (posA_act >= min_stance && !A_past_min){
      A_past_min = true;
      t4 = millis();
    old_posA_set = (encoder1Pos) / (4480.0);
    }
  
    if (A_past_min){
      set_speed_A = calculate_set_speed(posA_act);
      if (set_speed_A != old_set_speed_A || old_posA_set == posA_act){
        intercept_A = old_posA_set - (set_speed_A)*(cur_time_A/1000.0);
      }
  
      posA_set = (set_speed_A)*(cur_time_A/1000.0) + intercept_A;

      old_posA_set = posA_set;
      old_set_speed_A = set_speed_A;
   
      PWM_A_val = updatePid_A(PWM_A_val);
    }
    else{
      PWM_A_val = 50; //35 when not on ground
    }
  if (PWM_A_val == 0) PWM_A_val = pwm;
  
    if (millis() - t4 >= gait_period*phase_lag*1000.0 && A_past_min){
      Serial.print(error_A);Serial.print(",");
    }
    else{
      Serial.print(error_A);Serial.println(",0,0,0");
    }


    if (millis() - t4 >= gait_period*phase_lag*1000.0 && !t5_flag && A_past_min){
    t5 = millis();
      t5_flag = true;
    old_posB_set = (encoder2Pos) / (4480.0);
    }
    
    
    // B portion
    if (millis() - t4 >= gait_period*phase_lag*1000.0 && A_past_min){
      cur_time_B = millis() - t5;
      posB_act = ((encoder2Pos) / (4480.0));
      speed_B = calculate_set_speed(posB_act);
      if (speed_B != old_speed_B || old_posB_set == posB_act){
        intercept_B = old_posB_set - (speed_B/1000.0)*cur_time_B;
      }
      posB_set = (cur_time_B)*(speed_B/1000.0) + intercept_B;
      old_posB_set = posB_set;
      old_speed_B = speed_B;
      PWM_B_val = updatePid_B(PWM_B_val);
    if (PWM_B_val == 0) PWM_B_val = pwm;
    
      if (millis() - t4 >= gait_period*2*phase_lag*1000.0){
        Serial.print(error_B);Serial.print(",");
      }
      else{
        Serial.print(error_B);Serial.println(",0,0");
      }
      
      if (millis() - t4 >= gait_period*2*phase_lag*1000.0 && !t6_flag){
        t6 = millis();
        t6_flag  = true;
      old_posC_set = (encoder3Pos) / (4480.0);
      }
    }


    // C portion
    if (millis() - t4 >= gait_period*2*phase_lag*1000.0 && A_past_min){
      cur_time_C = millis() - t6;
      posC_act = ((encoder3Pos) / (4480.0));
      speed_C = calculate_set_speed(posC_act);
      if (speed_C != old_speed_C || old_posC_set == posC_act){
        intercept_C = old_posC_set - (speed_C/1000.0)*cur_time_C;
      }
      posC_set = (cur_time_C)*(speed_C/1000.0) + intercept_C;
      old_posC_set = posC_set;
      old_speed_C = speed_C;
      PWM_C_val = updatePid_C(PWM_C_val);
    if (PWM_C_val == 0) PWM_C_val = pwm;
  
      if (millis() - t4 >= gait_period*3*phase_lag*1000.0){
        Serial.print(error_C);Serial.print(",");
      }
      else{
        Serial.print(error_C);Serial.println(",0");
      }
      if (millis() - t4 >= gait_period*3*phase_lag*1000.0 && !t7_flag){
        t7 = millis();
        t7_flag  = true;  
      old_posD_set = (encoder4Pos) / (4480.0);      
      }
    }



    // D portion
    if (millis() - t4 >= gait_period*3*phase_lag*1000.0 && A_past_min){
      cur_time_D = millis() - t7;
      posD_act = ((encoder4Pos) / (4480.0));
      speed_D = calculate_set_speed(posD_act);
      if (speed_D != old_speed_D || old_posD_set == posD_act){
        intercept_D = old_posD_set - (speed_D/1000.0)*cur_time_D;
      }
      posD_set = (cur_time_D)*(speed_D/1000.0) + intercept_D;
      old_posD_set = posD_set;
      old_speed_D = speed_D;
      PWM_D_val = updatePid_D(PWM_D_val);
    if (PWM_D_val == 0) PWM_D_val = pwm;
      Serial.println(error_D);
    }
  
   
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

void check_Positions(){
  while(1){
    m1.rotCW(0);
    m2.rotCW(0);
    m3.rotCCW(0);
    m4.rotCCW(0);
    posB_act = (encoder2Pos) / (4480.0);
    posC_act = (encoder3Pos) / (4480.0);
    posD_act = (encoder4Pos) / (4480.0);
    posA_act = (encoder1Pos) / (4480.0);
    Serial.print(posA_act);Serial.print(",");
    Serial.print(posB_act);Serial.print(",");
    Serial.print(posC_act);Serial.print(",");
    Serial.println(posD_act);
  }
}

