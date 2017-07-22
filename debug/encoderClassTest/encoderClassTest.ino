#include "cleg.h"   //loads a custom library to set up drive motors

#define motorA1 14 //53 //A
#define motorA2 15 //51 //B
#define motorA_PWM 2
#define encoder1PinA 52
#define encoder1PinB 50
#define hallEffect_A 25
#define LEDPin 53

double pi = 3.14159;

volatile long encoder1Pos = 0; 

boolean A1_set = false;
boolean B1_set = false;

double posA_set = 10;                               // position (Set Point) (in revolution)
double posA_act = 0.0;                                // position (actual value) (in revolution)
double posA_actOld = 0.0; 
int PWM_A_val = 0;  

double Kp =   2.125; //10 //13 //60;       //critical Kp = 1.0625                         // PID proportional control Gain
double Kd =   1.2495; //925 //435 //1200;                                // PID Derivitave control gain
double Ki =   0;//0.00000000000000001;//1.5;
double Ka =   0;     // integral feedback gain to prevent reset windup

unsigned long lastTime_A;
int pidTerm_A = 0;                                                            // PID correction
double error_A=0;                                   
double last_error_A=0;   
double dErr_A = 0;      
double iTerm_A = 0;
//double outMaxI_A = 0;
//double outMinI_A = 0;
double PID_feedback_A = 0.0;
int output_A = 0;

unsigned long cur_time_A;
bool flag = false;
bool flag2 = true;

bool magAtHall_A = false;

cleg m1(motorA1, motorA2, motorA_PWM, encoder1PinA, encoder1PinB);

void setup() {
  pinMode(hallEffect_A, INPUT);
  pinMode(LEDPin,OUTPUT);
	Serial.begin(9600); //Establishes Serial communication at a specified baud rate. This can be moved inside of the Ant Comm class
	attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallEffect_A), Hall2LED, FALLING);
//  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder1m2, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoder2m2, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoder3PinA), doEncoder1m3, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoder3PinB), doEncoder2m3, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoder4PinA), doEncoder1m4, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoder4PinB), doEncoder2m4, CHANGE);

  zeroPosition();
}

void loop() {
  
  cur_time_A = millis()-10000;
  Serial.print(cur_time_A); Serial.print(",");
  
  posA_set = 1.0*(cur_time_A/1000.0);
  posA_act = ((encoder1Pos)/(4480.0));
  Serial.print(posA_set); Serial.print(",");
  Serial.print(posA_act); Serial.print(",");
  PWM_A_val= updatePid_A(PWM_A_val);
  Serial.print(error_A);Serial.print(",");
  Serial.println(PWM_A_val);

  if (PWM_A_val >= 0) m1.rotCW(abs(PWM_A_val));
  else m1.rotCCW(abs(PWM_A_val));
  
}

int updatePid_A(int command_A)   {             // compute PWM value
  unsigned long now_A = millis();
  double timeChange_A = (double)(now_A - lastTime_A);     
    error_A = posA_set - posA_act; 

    
    dErr_A = (error_A-last_error_A)*timeChange_A;

    iTerm_A += Ki*(error_A);

    pidTerm_A = (Kp * error_A) + (Kd * dErr_A) + iTerm_A - Ka*PID_feedback_A;    
    Serial.print(Kd*dErr_A); Serial.print(",");
    Serial.print(iTerm_A); Serial.print(","); 
    Serial.print(Kp*error_A); Serial.print(","); 
    Serial.print(pidTerm_A);Serial.print(",");
    Serial.print(pidTerm_A + command_A);Serial.print(",");
    last_error_A = error_A;
    lastTime_A = now_A;
    output_A = command_A + pidTerm_A;
    PID_feedback_A = output_A - constrain(output_A,-255,255);
    
//    if (command_A + pidTerm_A > 255){
//      I_feedback_A = pidTerm_A - (255 - command_A); 
//      pidTerm_A = 255 - command_A;
//    }
//    else if (command_A + pidTerm_A < -255){
//      I_feedback_A = pidTerm_A - (-255 - command_A);
//      pidTerm_A = -255 - command_A;
//    }
    Serial.print(-Ka*PID_feedback_A);Serial.print(",");
//    Serial.print(-Ki*I_feedback_A);Serial.print(",");
//    Serial.print(iTerm_A_control);Serial.print(",");
    return constrain(output_A, -255, 255);          
  }


void doEncoder1A(){
  A1_set = digitalRead(encoder1PinA) == HIGH;
  encoder1Pos += (A1_set != B1_set) ? +1 : -1;
}
void doEncoder1B(){
  B1_set = digitalRead(encoder1PinB) == HIGH;
  encoder1Pos += (A1_set == B1_set) ? +1 : -1;
}

void checkHall_A(){
  encoder1Pos = 0;
  magAtHall_A = true;
  detachInterrupt(digitalPinToInterrupt(hallEffect_A));

}

void zeroPosition(){
 attachInterrupt(digitalPinToInterrupt(hallEffect_A), checkHall_A, FALLING);
 while(!magAtHall_A){
  m1.rotCW(100);
  Serial.println("in start loop");
 }

 unsigned long now_A = millis();
  while (millis() - now_A < 10000){
    posA_set = 0;
    posA_act = ((encoder1Pos)/(4480.0));
    PWM_A_val= updatePid_A(PWM_A_val);
    
    if (PWM_A_val >= 0) m1.rotCW(abs(PWM_A_val));
  else m1.rotCCW(abs(PWM_A_val));
  }
}

//void zeroPosition(){
//  attachInterrupt(digitalPinToInterrupt(hallEffect_A), checkHall_A, FALLING);
//  while(flag2){
//  posA_set = 1;
//  posA_act = ((encoder1Pos)/(4480.0));
//  PWM_A_val= updatePid_A(PWM_A_val);
//  m1.rotCW(PWM_A_val);
//  if (posA_act < posA_actOld && !flag){
//    detachInterrupt(digitalPinToInterrupt(hallEffect_A));
//    flag = true;
//  }
//  unsigned long now_A = millis();
//  while (millis() - now_A < 10000 && flag){
//    posA_set = 0;
//    posA_act = ((encoder1Pos)/(4480.0));
//    PWM_A_val= updatePid_A(PWM_A_val);
//    m1.rotCW(PWM_A_val);
//    flag2 = false;
//  }
//   posA_actOld = posA_act;
//  }
//}

void Hall2LED(){
  if (digitalRead(hallEffect_A) == LOW){
    digitalWrite(LEDPin,HIGH);
  }
  else {
    digitalWrite(LEDPin,LOW);
  }
}

