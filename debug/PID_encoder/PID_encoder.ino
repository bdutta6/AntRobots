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

bool A1_set = false;
bool B1_set = false;

bool A2_set = false;
bool B2_set = false;

bool A3_set = false;
bool B3_set = false;

bool A4_set = false;
bool B4_set = false;

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

double Kp =   620.9375; //1390.9; //620.9375; //397.4; //655.71; //1168.8;                      // PID proportional control Gain
double Kd =   44353; //93886; //44353; //59610; //98357; //65746.3;                              // PID Derivitave control gain
double Ki =   0.6272;; //7.7272; //0.6272; //1.7662; //2.9143; //5.1948;
double Ka =   0;//0.26;// * 5;   // integral feedback gain to prevent reset windup
double b = 1.0;//0.35;//0.3;
double c = 1.0; // this inadvertently fixes the derivative kick too


unsigned long lastTime_A;
int pidTerm_A = 0;                                                            // PID correction
double error_A = 0;
double errP_A = 0;
double errD_A = 0;
double last_errD_A = 0;
double iTerm_A = 0;
double PID_feedback_A = 0.0;
double lastInput_A = 0;
double dInput_A = 0;
double dTerm_A = 0;
bool magAtHall_A = false;
double timeChange_A = 0.0;

unsigned long lastTime_B;
int pidTerm_B = 0;                                                            // PID correction
double error_B = 0;
double last_error_B = 0;
double iTerm_B = 0;
double PID_feedback_B = 0.0;
double lastInput_B = 0;
double dInput_B = 0;
bool magAtHall_B = false;

unsigned long lastTime_C;
int pidTerm_C = 0;                                                            // PID correction
double error_C = 0;
double last_error_C = 0;
double iTerm_C = 0;
double PID_feedback_C = 0.0;
double lastInput_C = 0;
double dInput_C = 0;
bool magAtHall_C = false;

unsigned long lastTime_D;
int  pidTerm_D = 0;                                                            // PID correction
double error_D = 0;
double last_error_D = 0;
double iTerm_D = 0;
double PID_feedback_D = 0.0;
double lastInput_D = 0;
double dInput_D = 0;
bool magAtHall_D = false;

unsigned long cur_time_A;
unsigned long cur_time_B;
unsigned long cur_time_C;
unsigned long cur_time_D;

double output = 0.0;
float fast_val = 1.5;
float slow_val = 0.5;

double set_speed_A = 1.0;
double old_set_speed_A = 0.0;
double old_posA_set = 0.0;
double act_speed_A = 0.0;

double set_speed_B = 1.0;
double old_set_speed_B = 0.0;
double old_posB_set = 0.0;
double act_speed_B = 0.0;

double set_speed_C = 1.0;
double old_set_speed_C = 0.0;
double old_posC_set = 0.0;
double act_speed_C = 0.0;

double set_speed_D = 1.0;
double old_set_speed_D = 0.0;
double old_posD_set = 0.0; //get rid of
double act_speed_D = 0.0;

float out = 0;
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
float gait_period = 1.97;
float stance_duration = 0.75; // 75% stance duration (taken from the hildebrand graph)
float phase_lag = 0.25; // 25% phase lag (taken from hildebrand graph)

// modify the above variables to model different types of gaits

float stance_phase = max_stance - min_stance; // 0.2
float swing_phase = 1 - stance_phase; // 0.8

float stance_cycle = gait_period * stance_duration;
float swing_cycle = gait_period - stance_cycle;
float hold_cycle = 0.1*stance_cycle;
bool justEntered = true;
unsigned long t_hold;
int n = 0;

float st_f = stance_phase / stance_cycle; // the speed of the cleg during the stance period
float sw_f = swing_phase / swing_cycle; // the speed of the cleg during the swing period 

float x = 0;

  double const pi = 3.14159;

  double C = (st_f + sw_f)/2;
  double At = st_f - C;
  double Aw = sw_f - C;
  
  double Tt_mark_A = 0; double Tstag_tA = 0; double Tmove_tA = 0;
  double Tw_mark_A = 0; double Tstag_wA = 0; double Tmove_wA = 0;
  double ft_A; double fw_A; bool t_tflagA; bool t_wflagA; 
  double time_A; double dt_tA; double dt_wA; double t_tA; double t_wA;
  double output_A; bool stance_wave_last_A; bool swing_wave_last_A;
  
  double Tt_mark_B = 0; double Tstag_tB = 0; double Tmove_tB = 0;
  double Tw_mark_B = 0; double Tstag_wB = 0; double Tmove_wB = 0;
  double ft_B; double fw_B; bool t_tflagB; bool t_wflagB; 
  double time_B; double dt_tB; double dt_wB; double t_tB; double t_wB;
  double output_B; bool stance_wave_last_B; bool swing_wave_last_B;
  
  double Tt_mark_C = 0; double Tstag_tC = 0; double Tmove_tC = 0;
  double Tw_mark_C = 0; double Tstag_wC = 0; double Tmove_wC = 0;
  double ft_C; double fw_C; bool t_tflagC; bool t_wflagC; 
  double time_C; double dt_tC; double dt_wC; double t_tC; double t_wC;
  double output_C; bool stance_wave_last_C; bool swing_wave_last_C;
  
  double Tt_mark_D = 0; double Tstag_tD = 0; double Tmove_tD = 0;
  double Tw_mark_D = 0; double Tstag_wD = 0; double Tmove_wD = 0;
  double ft_D; double fw_D; bool t_tflagD; bool t_wflagD; 
  double time_D; double dt_tD; double dt_wD; double t_tD; double t_wD;
  double output_D; bool stance_wave_last_D; bool swing_wave_last_D;


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
      // Serial.println("hi");
  unsigned long t1 = millis();
// zeroPosition_all();
 check_Positions();
      // Serial.println("hi");
  dt = millis() - t1;
  // stand();
  // prep_for_gait();
  // set_offset_lateral_gait();
  // set_180_offset_gait();
  dt = millis() - t1;

//  while(1){
//    m1.rotCW(255);
//    m2.rotCW(255);
//    m3.rotCCW(255);
//    m4.rotCCW(255);
//  }
}




double old_old_posA_act = 0;
double old_posA_act = 0;
double old_time_A = 0;

double old_old_posB_act = 0;
double old_posB_act = 0;
double old_time_B = 0;

double old_old_posC_act = 0;
double old_posC_act = 0;
double old_time_C = 0;

double old_old_posD_act = 0;
double old_posD_act = 0;
double old_time_D = 0;

void loop() {
  
    if (t4 == 0) cur_time_A = millis() - dt;
  else cur_time_A = millis() - t4;
  Serial.print(cur_time_A); Serial.print(",");

//  posA_set = 0.0;

//  posA_act = ((encoder1Pos) / (4480.0)); // outputs current position in terms of revolution


   set_speed_A = calculate_set_speed(posA_act); // changes the rotational speed modifier based on the position of the cleg

   // set_speed_A = find_speed_A();
 
   // if (set_speed_A != old_set_speed_A){
     // intercept_A = old_posA_set - (set_speed_A/1000.0)*cur_time_A; // works according to the equation of a line and it is necessary for a smooth change in speed
   // }
      // Serial.print(set_speed_A);Serial.print(",");
      // Serial.print(posA_set_line);Serial.print(",");
      // Serial.print(old_line_speed_A);Serial.print(",");
      // Serial.print(intercept_A);Serial.print(",");
  
//    posA_set = (cur_time_A)*(1.0/1000.0) + intercept_A;

   posA_set += set_speed_A * ((cur_time_A - old_time_A)/1000.0);
   old_time_A = cur_time_A;
  
   old_old_posA_act = old_posA_act;
   old_posA_act = posA_act;
   
   // old_old_posA_set = old_posA_set;
   // old_posA_set = posA_set;
   
   // old_set_speed_A = set_speed_A;

  PWM_A_val = updatePid_A();
  act_speed_A = dInput_A / (timeChange_A/1000.0);  // needs to be revolutions per second
  // if (PWM_A_val == 0) PWM_A_val = pwm;
  
      Serial.print(error_A);Serial.print(",");
      Serial.print(posA_set);Serial.print(",");
      Serial.print(posA_act);Serial.print(",");
      Serial.print(set_speed_A);Serial.print(",");
      Serial.println(act_speed_A);

//      Serial.println(PWM_A_val);//Serial.print(",   ");

  
  // if (t5 == 0) cur_time_B = millis() - dt;
  // else cur_time_B = millis() - t5;
  // // Serial.print(cur_time_B); Serial.print(",");
// //  posB_set = 0.0;

  // posB_act = (encoder2Pos) / (4480.0);
  
   // set_speed_B = find_speed_B();

   // // speed_B = calculate_set_speed(posB_act);
   // // if (speed_B != old_speed_B){
     // // intercept_B = old_posB_set - (speed_B)*(cur_time_B/1000.0);
   // // }
  
   // // posB_set = (speed_B)*(cur_time_B/1000.0) + intercept_B;
  
  // // posB_set = f*(cur_time_B/1000.0);
   // // old_posB_set = posB_set;
   // // old_speed_B = speed_B;
   
  
   // posB_set += set_speed_B * ((cur_time_B - old_time_B)/1000.0);
   // old_time_B = cur_time_B;
   // old_old_posB_act = old_posB_act;
   // old_posB_act = posB_act;
   

   // PWM_B_val = updatePid_B();
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
  
   // set_speed_C = find_speed_C();
  // // speed_C = calculate_set_speed(posC_act);
   // // if (speed_C != old_speed_C){
     // // intercept_C = old_posC_set - (speed_C/1000.0)*cur_time_C;
   // // }
  
   // // posC_set = (cur_time_C)* (speed_C/1000.0) + intercept_C;
   // // old_posC_set = posC_set;
   // // old_speed_C = speed_C;
   
   // posC_set += set_speed_C * ((cur_time_C - old_time_C)/1000.0);
   // old_time_C = cur_time_C;
   // old_old_posC_act = old_posC_act;
   // old_posC_act = posC_act;
   
// //  Serial.print(posC_act); Serial.print(",");
  // PWM_C_val = updatePid_C();
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
  // set_speed_D = find_speed_D();
  
// //  Serial.print(posD_act);Serial.print(",");
  
  // // speed_D = calculate_set_speed(posD_act);
   // // if (speed_D != old_speed_D){
     // // intercept_D = old_posD_set - (speed_D/1000.0)*cur_time_D;
   // // }  
  // // posD_set = (cur_time_D)* (speed_D/1000.0) + intercept_D;
  // // old_posD_set = posD_set;
  // // old_speed_D = speed_D;
  
  
   // posD_set += set_speed_D * ((cur_time_D - old_time_D)/1000.0);
   // old_time_D = cur_time_D;
   // old_old_posD_act = old_posD_act;
   // old_posD_act = posD_act;
  
  // PWM_D_val = updatePid_D();
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
  if (justEntered) {
    t_hold = millis();
    n++;
    justEntered = false;
  }
  
  if (millis() - t_hold <= hold_cycle * 1000) {
    // output = (n*min_stance - posA_set) / ((cur_time_A - old_time_A)/1000.0);
    output = 0;
  } 
  else {
    output = st_f;
  }
  }
  else {
    // if it is not in the stance phase, it must be in the swing phase
  justEntered = true;
  output = sw_f;
  }
  return (output);
}


double find_speed_A() { // this will only output positive numbers
  double pos_A = posA_act - floor(posA_act); // this brings it down to the basic unit circle (1 rev = 0)
  unsigned long now_A = millis();
  time_A = now_A/ 1000.0; // converts the time to double and to seconds
  
  if (posA_act < min_stance){
    output_A = st_f;
  swing_wave_last_A = true;
  }
  else{
    if (swing_wave_last_A){
      // signifies that it just left the swing_phase (ie, in the stance phase)
    Tstag_wA = 0;
    t_wflagA = true; // resets the swing stagnant period value and the time flag variable
    if (t_tflagA){
      dt_tA = time_A; 
      t_tflagA = false; 
      //grabs and saves the time when the leg first enters the stance phase
    }
    t_tA = time_A - dt_tA;
    //uses the saved time to effectively make t_tA go from 0 to the end of the stance duty cycle
  
  
    if (posA_act == old_posA_act && posA_act != old_old_posA_act){
      Tt_mark_A = t_tA; //grabs the time when the leg stops and marks the time in stance period terms 
    }
    if (posA_act == old_posA_act){
      Tstag_tA = t_tA - Tt_mark_A; //continually increase the stagnant period until the leg moves again
    }
  
    Tmove_tA = stance_cycle - Tstag_tA; //changes the moving period to take into account that the leg stopped moving
  
    ft_A = pi/(Tmove_tA);
  
    // output_A = At*sin(ft_A*t_tA) + C; // plots the output speed as being dependent on time rather than position
  output_A = st_f;
    if (t_tA >= stance_cycle || t_tA != 0 && output_A >= C){
      stance_wave_last_A = true;
      swing_wave_last_A = false;
    }
  }
  else if (stance_wave_last_A) { //i dont really like relying on an else if
    // if it is not in the stance phase, it must be in the swing phase
    Tstag_tA = 0;
    t_tflagA = true;
    // same comments as for the stance phase portion 
    if (t_wflagA){
      dt_wA = time_A;
      t_wflagA = false;
    }
    t_wA = time_A - dt_wA;
  
  
    if (posA_act == old_posA_act && posA_act != old_old_posA_act){
      Tw_mark_A = t_wA; //reset dt_speed immediately once the stance phase ends 
    }
    if (posA_act == old_posA_act){
      Tstag_wA = t_wA - Tw_mark_A;
    }
    Tmove_wA = swing_cycle - Tstag_wA;
    fw_A = pi/(Tmove_wA);
  
    // output_A = Aw*sin(fw_A*t_wA) + C;
  output_A = sw_f;
    if (t_wA >= swing_cycle || t_wA != 0 && output_A <= C){
      swing_wave_last_A = true;
      stance_wave_last_A = false;
    }
  }
  }
  return (output_A);
}

double find_speed_B() { 
  double pos_B = posB_act - floor(posB_act); 
  unsigned long now_B = millis();
  time_B = now_B/ 1000.0; 
  
  if (posB_act < min_stance){
    output_B = C;
  swing_wave_last_B = true;
  }
  else{
    if (swing_wave_last_B){
    Tstag_wB = 0;
    t_wflagB = true; 
    if (t_tflagB){
      dt_tB = time_B; 
      t_tflagB = false; 
    }
    t_tB = time_B - dt_tB;
  
  
    if (posB_act == old_posB_act && posB_act != old_old_posB_act){
      Tt_mark_B = t_tB;
    }
    if (posB_act == old_posB_act){
      Tstag_tB = t_tB - Tt_mark_B;
    }
  
    Tmove_tB = stance_cycle - Tstag_tB;
  
    ft_B = pi/(Tmove_tB);
  
    output_B = At*sin(ft_B*t_tB) + C; 
    if (t_tB >= stance_cycle || t_tB != 0 && output_B >= C){
      stance_wave_last_B = true;
      swing_wave_last_B = false;
    }
  }
  else if (stance_wave_last_B) { 
    Tstag_tB = 0;
    t_tflagB = true; 
    if (t_wflagB){
      dt_wB = time_B;
      t_wflagB = false;
    }
    t_wB = time_B - dt_wB;
  
  
    if (posB_act == old_posB_act && posB_act != old_old_posB_act){
      Tw_mark_B = t_wB; 
    }
    if (posB_act == old_posB_act){
      Tstag_wB = t_wB - Tw_mark_B;
    }
    Tmove_wB = swing_cycle - Tstag_wB;
    fw_B = pi/(Tmove_wB);
  
    output_B = Aw*sin(fw_B*t_wB) + C; 
    if (t_wB >= swing_cycle || t_wB != 0 && output_B <= C){
      swing_wave_last_B = true;
      stance_wave_last_B = false;
    }
  }
  }
  return (output_B);
}

double find_speed_C() { 
  double pos_C = posC_act - floor(posC_act); 
  unsigned long now_C = millis();
  time_C = now_C/ 1000.0; 
  
  if (posC_act < min_stance){
    output_C = C;
  swing_wave_last_C = true;
  }
  else{
    if (swing_wave_last_C){
    Tstag_wC = 0;
    t_wflagC = true; 
    if (t_tflagC){
      dt_tC = time_C; 
      t_tflagC = false; 
    }
    t_tC = time_C - dt_tC;
  
  
    if (posC_act == old_posC_act && posC_act != old_old_posC_act){
      Tt_mark_C = t_tC;
    }
    if (posC_act == old_posC_act){
      Tstag_tC = t_tC - Tt_mark_C;
    }
  
    Tmove_tC = stance_cycle - Tstag_tC;
  
    ft_C = pi/(Tmove_tC);
  
    output_C = At*sin(ft_C*t_tC) + C; 
    if (t_tC >= stance_cycle || t_tC != 0 && output_C >= C){
      stance_wave_last_C = true;
      swing_wave_last_C = false;
    }
  }
  else if (stance_wave_last_C) { 
    Tstag_tC = 0;
    t_tflagC = true; 
    if (t_wflagC){
      dt_wC = time_C;
      t_wflagC = false;
    }
    t_wC = time_C - dt_wC;
  
  
    if (posC_act == old_posC_act && posC_act != old_old_posC_act){
      Tw_mark_C = t_wC; 
    }
    if (posC_act == old_posC_act){
      Tstag_wC = t_wC - Tw_mark_C;
    }
    Tmove_wC = swing_cycle - Tstag_wC;
    fw_C = pi/(Tmove_wC);
  
    output_C = Aw*sin(fw_C*t_wC) + C; 
    if (t_wC >= swing_cycle || t_wC != 0 && output_C <= C){
      swing_wave_last_C = true;
      stance_wave_last_C = false;
    }
  }
  }
  return (output_C);
}

double find_speed_D() { 
  double pos_D = posD_act - floor(posD_act); 
  unsigned long now_D = millis();
  time_D = now_D/ 1000.0; 
  
  if (posD_act < min_stance){
    output_D = C;
  swing_wave_last_D = true;
  }
  else{
    if (swing_wave_last_D){
    Tstag_wD = 0;
    t_wflagD = true; 
    if (t_tflagD){
      dt_tD = time_D; 
      t_tflagD = false; 
    }
    t_tD = time_D - dt_tD;
  
  
    if (posD_act == old_posD_act && posD_act != old_old_posD_act){
      Tt_mark_D = t_tD;
    }
    if (posD_act == old_posD_act){
      Tstag_tD = t_tD - Tt_mark_D;
    }
  
    Tmove_tD = stance_cycle - Tstag_tD;
  
    ft_D = pi/(Tmove_tD);
  
    output_D = At*sin(ft_D*t_tD) + C; 
    if (t_tD >= stance_cycle || t_tD != 0 && output_B >= C){
      stance_wave_last_D = true;
      swing_wave_last_D = false;
    }
  }
  else if (stance_wave_last_D) { 
    Tstag_tD = 0;
    t_tflagD = true; 
    if (t_wflagD){
      dt_wD = time_D;
      t_wflagD = false;
    }
    t_wD = time_D - dt_wD;
  
  
    if (posD_act == old_posD_act && posD_act != old_old_posD_act){
      Tw_mark_D = t_wD; 
    }
    if (posD_act == old_posD_act){
      Tstag_wD = t_wD - Tw_mark_D;
    }
    Tmove_wD = swing_cycle - Tstag_wD;
    fw_D = pi/(Tmove_wD);
  
    output_D = Aw*sin(fw_D*t_wD) + C; 
    if (t_wD >= swing_cycle || t_wD != 0 && output_D <= C){
      swing_wave_last_D = true;
      stance_wave_last_D = false;
    }
  }
  }
  return (output_D);
}



int updatePid_A()   {            // compute PWM value
  unsigned long now_A = millis();
  timeChange_A = (double)(now_A - lastTime_A);
  error_A = posA_set - posA_act;
  errP_A = b*posA_set - posA_act;
  errD_A = c*posA_set - posA_act;
  
  dTerm_A = (errD_A - last_errD_A) / (timeChange_A);
  iTerm_A += error_A * (timeChange_A);
  
  pidTerm_A = (Kp * errP_A) + (Kd * dTerm_A) + Ki*iTerm_A;
//      Serial.print(Kd*dInput_A); Serial.print(",");
//      Serial.print(iTerm_A); Serial.print(",");
//      Serial.print(Kp*error_A); Serial.print(",");
//      Serial.print(pidTerm_A);Serial.print(",");
//      Serial.print(pidTerm_A + command_A);Serial.print(",");
  dInput_A = (posA_act - lastInput_A); //purposefully wrong
  lastTime_A = now_A;
  last_errD_A = errD_A;
  lastInput_A = posA_act;
  PID_feedback_A = pidTerm_A - constrain(pidTerm_A, -255, 255);
  return constrain(pidTerm_A, -255, 255);
}

int updatePid_B()   {             // compute PWM value
  unsigned long now_B = millis();
  double timeChange_B = (double)(now_B - lastTime_B);
  error_B = posB_set - posB_act;
  
  dInput_B = (posB_act - lastInput_B) / (timeChange_B);
  iTerm_B += error_B * timeChange_B;

  pidTerm_B = (Kp * error_B) - (Kd * dInput_B) + Ki*iTerm_B;
  last_error_B = error_B;
  lastTime_B = now_B;
  lastInput_B = posB_act;
  PID_feedback_B = pidTerm_B - constrain(pidTerm_B, -255, 255);
  return constrain(pidTerm_B, -255, 255);
}

int updatePid_C()   {             // compute PWM value
  unsigned long now_C = millis();
  double timeChange_C = (double)(now_C - lastTime_C);
  error_C = posC_set - posC_act;
  
  dInput_C = (posC_act - lastInput_C) / timeChange_C;
  iTerm_C += error_C * timeChange_C;

  pidTerm_C = (Kp * error_C) - (Kd * dInput_C) + Ki*iTerm_C;
  last_error_C = error_C;
  lastTime_C = now_C;
  lastInput_C = posC_act;
  PID_feedback_C = pidTerm_C - constrain(pidTerm_C, -255, 255);
  return constrain(pidTerm_C, -255, 255);
}

int updatePid_D()   {             // compute PWM value
  unsigned long now_D = millis();
  double timeChange_D = (double)(now_D - lastTime_D);
  error_D = posD_set - posD_act;
  
  dInput_D = (posD_act - lastInput_D) / timeChange_D;
  iTerm_D += error_D * timeChange_D;

  pidTerm_D = (Kp * error_D) - (Kd * dInput_D) + Ki*iTerm_D;
  last_error_D = error_D;
  lastTime_D = now_D;
  lastInput_D = posD_act;
  PID_feedback_D = pidTerm_D - constrain(pidTerm_D, -255, 255);
  return constrain(pidTerm_D, -255, 255);
}



// Interrupt on A changing state
void doEncoder1A(){
  // Test transition
  A1_set = digitalRead(encoder1PinA) == HIGH;
  // and adjust encoder1Poser + if A leads B
  encoder1Pos += (A1_set != B1_set) ? +1 : -1;
//  posA_act = ((encoder1Pos) / (4480.0));
//  if (digitalRead(hallEffect_A) == HIGH) encoder1Pos = 0;
}

// Interrupt on B changing state
void doEncoder1B(){
  // Test transition
  B1_set = digitalRead(encoder1PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder1Pos += (A1_set == B1_set) ? +1 : -1;
//  posA_act = ((encoder1Pos) / (4480.0));
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
  int pwmZeroA = 25;
  int pwmZeroB = 25;
  int pwmZeroC = 25;
  int pwmZeroD = 25;
  m1.rotCW(pwmZeroA);
  m2.rotCW(pwmZeroB);
  m3.rotCCW(pwmZeroC);
  m4.rotCCW(pwmZeroD);
  while (!magAtHall_A || !magAtHall_B || !magAtHall_C || !magAtHall_D) {
  if (digitalRead(hallEffect_A) == LOW){
      magAtHall_A = true;
      encoder1Pos = 0;
    pwmZeroA = 0;
      m1.rotCW(pwmZeroA);
    }
  if (digitalRead(hallEffect_B) == LOW){
      magAtHall_B = true;
      encoder2Pos = 0;
    pwmZeroB = 0;
    m2.rotCW(pwmZeroB);
    }
  if (digitalRead(hallEffect_C) == LOW){
      magAtHall_C = true;
      encoder3Pos = 0;
    pwmZeroC = 0;
      m3.rotCCW(pwmZeroC);
    }
  if (digitalRead(hallEffect_D) == LOW){
      magAtHall_D = true;
      encoder4Pos = 0;
    pwmZeroD = 0;
      m4.rotCCW(pwmZeroD);
    }
  Serial.print(pwmZeroA);Serial.print(",");
  Serial.print(pwmZeroB);Serial.print(",");
  Serial.print(pwmZeroC);Serial.print(",");
  Serial.println(pwmZeroD);
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

  PWM_B_val = updatePid_B();
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
  PWM_C_val = updatePid_C();
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
  PWM_D_val = updatePid_D();
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

  PWM_A_val = updatePid_A();
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


// void prep_for_gait(){
  // unsigned long t5 = millis();
  // float a = 1;//1.5
  // while (millis() - t5 <= 10000){

  // cur_time_B = millis() - dt;
  // Serial.print(cur_time_B); Serial.print(",");

  // posB_act = (encoder2Pos) / (4480.0);
  // if (posB_act >= gait_prep_pos-a*thres_prep) {
    // posB_set = gait_prep_pos;
  // }
  // else{
    // posB_set = f_prep*(cur_time_B/1000.0);
  // }
// //  if (posB_set >= gait_prep_pos) posB_set = gait_prep_pos;

  // PWM_B_val = updatePid_B();
  // Serial.print(error_B);Serial.print(",");


  // cur_time_C = millis()-dt;
  // if (posC_act >= gait_prep_pos-a*thres_prep) {
    // posC_set = gait_prep_pos;
  // }
  // else{
    // posC_set = f_prep*(cur_time_C/1000.0);
  // }
// //  if (posC_set >= gait_prep_pos) posC_set = gait_prep_pos;

  // posC_act = (encoder3Pos) / (4480.0);
  // PWM_C_val = updatePid_C();
  // Serial.print(error_C);Serial.print(",");

  
  // cur_time_D = millis()-dt;

  // if (posD_act >= gait_prep_pos-a*thres_prep) {
    // posD_set = gait_prep_pos;
  // }
  // else{
    // posD_set = f_prep*(cur_time_D/1000.0);
  // }
// //  if (posD_set >= gait_prep_pos) posD_set = gait_prep_pos;
  
  // posD_act = (encoder4Pos) / (4480.0);
  // PWM_D_val = updatePid_D();
  // Serial.print(error_D);Serial.print(",");
  
  // cur_time_A = millis() - dt;

  // posA_act = ((encoder1Pos) / (4480.0)); 
  // if (posA_act >= gait_prep_pos-a*thres_prep) {
    // posA_set = gait_prep_pos;
  // }
  // else{
    // posA_set = f_prep*(cur_time_A/1000.0);
  // }
// //  if (posA_set >= gait_prep_pos) posA_set =gait_prep_pos;

  // PWM_A_val = updatePid_A();
      // Serial.println(error_A);
    
  
  // // these if statements make the motor stop when it is within 0.025 of the the target value 
  // if (posA_act >= gait_prep_pos-thres_prep && posA_act <= gait_prep_pos+thres_prep) {
    // PWM_A_val = 0;
  // }
  // if (posB_act >= gait_prep_pos-thres_prep && posB_act <= gait_prep_pos+thres_prep) {
    // PWM_B_val = 0;
  // }
  // if (posC_act >= gait_prep_pos-thres_prep && posC_act <= gait_prep_pos+thres_prep) {
    // PWM_C_val = 0;
  // }
  // if (posD_act >= gait_prep_pos-thres_prep && posD_act <= gait_prep_pos+thres_prep) {
    // PWM_D_val = 0;
  // }
  

  // if (PWM_A_val >= 0) {
    // m1.rotCW(abs(PWM_A_val));
  // }
  // else {
    // m1.rotCCW(abs(PWM_A_val));
  // }

  // if (PWM_B_val >= 0) {
    // m2.rotCW(abs(PWM_B_val));
  // }
  // else {
    // m2.rotCCW(abs(PWM_B_val));
  // }

  // if (PWM_C_val >= 0) {
    // m3.rotCCW(abs(PWM_C_val));
  // }
  // else {
    // m3.rotCW(abs(PWM_C_val));
  // }

  // if (PWM_D_val >= 0) {
    // m4.rotCCW(abs(PWM_D_val));
  // }
  // else {
    // m4.rotCW(abs(PWM_D_val));
  // }
  // }
  // return;
// }


// void set_offset_lateral_gait(){
  // t4 = millis();
  // bool A_past_min = false;
  // bool t5_flag = false;
  // bool t6_flag = false;
  // bool t7_flag = false;
  // PWM_A_val = 0;
  // PWM_B_val = 0;
  // PWM_C_val = 0;
  // PWM_D_val = 0;
  // // include a portion where the starting leg rotates and enters the very beginning of the stance phase because rn it starts off in the swing phase
  // while(millis() - t4 <= gait_period*1000.0 || A_past_min){ 
  // // A portion
    // cur_time_A = millis() - t4;
    // Serial.print(cur_time_A); Serial.print(",");
    // posA_act = (encoder1Pos) / (4480.0);
    // if (posA_act >= min_stance && !A_past_min){
      // A_past_min = true;
      // t4 = millis();
    // old_posA_set = (encoder1Pos) / (4480.0);
    // }
  
    // if (A_past_min){
      // set_speed_A = calculate_set_speed(posA_act);
      // if (set_speed_A != old_set_speed_A || old_posA_set == posA_act){
        // intercept_A = old_posA_set - (set_speed_A)*(cur_time_A/1000.0);
      // }
  
      // posA_set = (set_speed_A)*(cur_time_A/1000.0) + intercept_A;

      // old_posA_set = posA_set;
      // old_set_speed_A = set_speed_A;
   
      // PWM_A_val = updatePid_A();
    // }
    // else{
      // PWM_A_val = 50; //35 when not on ground
    // }
  // if (PWM_A_val == 0) PWM_A_val = pwm;
  
    // if (millis() - t4 >= gait_period*phase_lag*1000.0 && A_past_min){
      // Serial.print(error_A);Serial.print(",");
    // }
    // else{
      // Serial.print(error_A);Serial.println(",0,0,0");
    // }


    // if (millis() - t4 >= gait_period*phase_lag*1000.0 && !t5_flag && A_past_min){
    // t5 = millis();
      // t5_flag = true;
    // old_posB_set = (encoder2Pos) / (4480.0);
    // }
    
    
    // // B portion
    // if (millis() - t4 >= gait_period*phase_lag*1000.0 && A_past_min){
      // cur_time_B = millis() - t5;
      // posB_act = ((encoder2Pos) / (4480.0));
      // speed_B = calculate_set_speed(posB_act);
      // if (speed_B != old_speed_B || old_posB_set == posB_act){
        // intercept_B = old_posB_set - (speed_B/1000.0)*cur_time_B;
      // }
      // posB_set = (cur_time_B)*(speed_B/1000.0) + intercept_B;
      // old_posB_set = posB_set;
      // old_speed_B = speed_B;
      // PWM_B_val = updatePid_B();
    // if (PWM_B_val == 0) PWM_B_val = pwm;
    
      // if (millis() - t4 >= gait_period*2*phase_lag*1000.0){
        // Serial.print(error_B);Serial.print(",");
      // }
      // else{
        // Serial.print(error_B);Serial.println(",0,0");
      // }
      
      // if (millis() - t4 >= gait_period*2*phase_lag*1000.0 && !t6_flag){
        // t6 = millis();
        // t6_flag  = true;
      // old_posC_set = (encoder3Pos) / (4480.0);
      // }
    // }


    // // C portion
    // if (millis() - t4 >= gait_period*2*phase_lag*1000.0 && A_past_min){
      // cur_time_C = millis() - t6;
      // posC_act = ((encoder3Pos) / (4480.0));
      // speed_C = calculate_set_speed(posC_act);
      // if (speed_C != old_speed_C || old_posC_set == posC_act){
        // intercept_C = old_posC_set - (speed_C/1000.0)*cur_time_C;
      // }
      // posC_set = (cur_time_C)*(speed_C/1000.0) + intercept_C;
      // old_posC_set = posC_set;
      // old_speed_C = speed_C;
      // PWM_C_val = updatePid_C();
    // if (PWM_C_val == 0) PWM_C_val = pwm;
  
      // if (millis() - t4 >= gait_period*3*phase_lag*1000.0){
        // Serial.print(error_C);Serial.print(",");
      // }
      // else{
        // Serial.print(error_C);Serial.println(",0");
      // }
      // if (millis() - t4 >= gait_period*3*phase_lag*1000.0 && !t7_flag){
        // t7 = millis();
        // t7_flag  = true;  
      // old_posD_set = (encoder4Pos) / (4480.0);      
      // }
    // }



    // // D portion
    // if (millis() - t4 >= gait_period*3*phase_lag*1000.0 && A_past_min){
      // cur_time_D = millis() - t7;
      // posD_act = ((encoder4Pos) / (4480.0));
      // speed_D = calculate_set_speed(posD_act);
      // if (speed_D != old_speed_D || old_posD_set == posD_act){
        // intercept_D = old_posD_set - (speed_D/1000.0)*cur_time_D;
      // }
      // posD_set = (cur_time_D)*(speed_D/1000.0) + intercept_D;
      // old_posD_set = posD_set;
      // old_speed_D = speed_D;
      // PWM_D_val = updatePid_D();
    // if (PWM_D_val == 0) PWM_D_val = pwm;
      // Serial.println(error_D);
    // }
  
   
  // if (PWM_A_val >= 0) {
    // m1.rotCW(abs(PWM_A_val));
  // }
  // else {
    // m1.rotCCW(abs(PWM_A_val));
  // }

  // if (PWM_B_val >= 0) {
    // m2.rotCW(abs(PWM_B_val));
  // }
  // else {
    // m2.rotCCW(abs(PWM_B_val));
  // }

  // if (PWM_C_val >= 0) {
    // m3.rotCCW(abs(PWM_C_val));
  // }
  // else {
    // m3.rotCW(abs(PWM_C_val));
  // }

  // if (PWM_D_val >= 0) {
    // m4.rotCCW(abs(PWM_D_val));
  // }
  // else {
    // m4.rotCW(abs(PWM_D_val));
  // }
  // }
  // return;
  
  
// }

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



// void set_180_offset_gait(){
    // t4 = millis();
  // bool A_past_min = true;
  // bool D_is_done = false;
  // bool t5_flag = false;
  // bool t6_flag = false;
  // bool t7_flag = false;
  // PWM_A_val = 0;
  // PWM_B_val = 0;
  // PWM_C_val = 0;
  // PWM_D_val = 0;
  // // include a portion where the starting leg rotates and enters the very beginning of the stance phase because rn it starts off in the swing phase
  // while(millis() - t4 <= gait_period*1000.0 || !D_is_done){ 
  // // A portion
    // cur_time_A = millis() - t4;
    // Serial.print(cur_time_A); Serial.print(",");
    // posA_act = (encoder1Pos) / (4480.0);

  
    // if (A_past_min){
      // set_speed_A = f; //calculate_set_speed(posA_act);
      // if (set_speed_A != old_set_speed_A || old_posA_set == posA_act){
        // intercept_A = old_posA_set - (set_speed_A)*(cur_time_A/1000.0);
      // }
  
      // posA_set = (set_speed_A)*(cur_time_A/1000.0) + intercept_A;

      // old_posA_set = posA_set;
      // old_set_speed_A = set_speed_A;
   
      // PWM_A_val = updatePid_A();
    // }
    // else{
      // PWM_A_val = 35; //35 when not on ground
    // }
  // if (PWM_A_val == 0) PWM_A_val = pwm;
  
    // if (millis() - t4 >= gait_period*phase_lag*1000.0 && A_past_min){
      // Serial.print(error_A);Serial.print(",");
    // }
    // else{
      // Serial.print(error_A);Serial.println(",0,0,0");
    // }


    // if (millis() - t4 >= gait_period*phase_lag*1000.0 && !t5_flag && A_past_min){
    // t5 = millis();
      // t5_flag = true;
    // old_posB_set = (encoder2Pos) / (4480.0);
    // }
    
    
    // // B portion
    // if (millis() - t4 >= gait_period*phase_lag*1000.0 && A_past_min){
      // cur_time_B = millis() - t5;
      // posB_act = ((encoder2Pos) / (4480.0));
      // speed_B = 0;
      // if (speed_B != old_speed_B || old_posB_set == posB_act){
        // intercept_B = old_posB_set - (speed_B/1000.0)*cur_time_B;
      // }
      // posB_set = (cur_time_B)*(speed_B/1000.0) + intercept_B;
      // old_posB_set = posB_set;
      // old_speed_B = speed_B;
      // PWM_B_val = updatePid_B();
    // if (PWM_B_val == 0) PWM_B_val = pwm;
    
      // if (millis() - t4 >= gait_period*2*phase_lag*1000.0){
        // Serial.print(error_B);Serial.print(",");
      // }
      // else{
        // Serial.print(error_B);Serial.println(",0,0");
      // }
      
      // if (millis() - t4 >= gait_period*2*phase_lag*1000.0 && !t6_flag){
        // t6 = millis();
        // t6_flag  = true;
      // old_posC_set = (encoder3Pos) / (4480.0);
      // }
    // }


    // // C portion
    // if (millis() - t4 >= gait_period*2*phase_lag*1000.0 && A_past_min){
      // cur_time_C = millis() - t6;
      // posC_act = ((encoder3Pos) / (4480.0));
      // speed_C = f;
      // if (speed_C != old_speed_C || old_posC_set == posC_act){
        // intercept_C = old_posC_set - (speed_C/1000.0)*cur_time_C;
      // }
      // posC_set = (cur_time_C)*(speed_C/1000.0) + intercept_C;
      // old_posC_set = posC_set;
      // old_speed_C = speed_C;
      // PWM_C_val = updatePid_C();
    // if (PWM_C_val == 0) PWM_C_val = pwm;
  
      // if (millis() - t4 >= gait_period*3*phase_lag*1000.0){
        // Serial.print(error_C);Serial.print(",");
      // }
      // else{
        // Serial.print(error_C);Serial.println(",0");
      // }
      // if (millis() - t4 >= gait_period*3*phase_lag*1000.0 && !t7_flag){
        // t7 = millis();
        // t7_flag  = true;  
      // old_posD_set = (encoder4Pos) / (4480.0);      
      // }
    // }



    // // D portion
    // if (millis() - t4 >= gait_period*3*phase_lag*1000.0 && A_past_min){
    // D_is_done = true;
      // cur_time_D = millis() - t7;
      // posD_act = ((encoder4Pos) / (4480.0));
      // speed_D = f;
      // if (speed_D != old_speed_D || old_posD_set == posD_act){
        // intercept_D = old_posD_set - (speed_D/1000.0)*cur_time_D;
      // }
      // posD_set = (cur_time_D)*(speed_D/1000.0) + intercept_D;
      // old_posD_set = posD_set;
      // old_speed_D = speed_D;
      // PWM_D_val = updatePid_D();
    // if (PWM_D_val == 0) PWM_D_val = pwm;
      // Serial.println(error_D);
    // }
  
   
  // if (PWM_A_val >= 0) {
    // m1.rotCW(abs(PWM_A_val));
  // }
  // else {
    // m1.rotCCW(abs(PWM_A_val));
  // }

  // if (PWM_B_val >= 0) {
    // m2.rotCW(abs(PWM_B_val));
  // }
  // else {
    // m2.rotCCW(abs(PWM_B_val));
  // }

  // if (PWM_C_val >= 0) {
    // m3.rotCCW(abs(PWM_C_val));
  // }
  // else {
    // m3.rotCW(abs(PWM_C_val));
  // }

  // if (PWM_D_val >= 0) {
    // m4.rotCCW(abs(PWM_D_val));
  // }
  // else {
    // m4.rotCW(abs(PWM_D_val));
  // }
  // }
  // return;
  
  
// }
