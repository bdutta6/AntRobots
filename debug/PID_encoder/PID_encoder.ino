#define motorA1 14 //A
#define motorA2 15 //B
#define motorA_PWM 2
#define encoder1PinA 52
#define encoder1PinB 50
#define hallEffect_A 25

#define motorB1 16 //A
#define motorB2 17 //B
#define motorB_PWM 3
#define encoder2PinA 48 
#define encoder2PinB 46
#define hallEffect_B 23

#define motorC1 18 //A
#define motorC2 19 //B
#define motorC_PWM 4
#define encoder3PinA 44 
#define encoder3PinB 42
#define hallEffect_C 24

#define motorD1 20 //A
#define motorD2 21 //B
#define motorD_PWM 5
#define encoder4PinA 40 
#define encoder4PinB 38
#define hallEffect_D 22

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

double posA_set = 10;                               // position (Set Point) (in revolution)
double posA_act = 0.0;                                // position (actual value) (in revolution)
int PWM_A_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posB_set = 10;                               // position (Set Point) (in revolution)
double posB_act = 0.0;                                // position (actual value) (in revolution)
int PWM_B_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posC_set = 20;                               // position (Set Point) (in revolution)
double posC_act = 0;                                // position (actual value) (in revolution)
int PWM_C_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

double posD_set = 25;                               // position (Set Point) (in revolution)
double posD_act = 0;                                // position (actual value) (in revolution)
int PWM_D_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

int Kp =    21; //10 //13 //60;                                // PID proportional control Gain
int Kd =    925; //925 //435 //1200;                                // PID Derivitave control gain
float Ki =   1.5;//0.00000000000000001;//1.5;
// these are incredibly close to perfect but there can still be some work done using kd and ki

unsigned long lastTime_A;
int pidTerm_A = 0;                                                            // PID correction
double error_A=0;                                   
double last_error_A=0;   
double dErr_A = 0;      
double iTerm_A = 0;
double outMaxI_A = 0;
double outMinI_A = 0;

unsigned long lastTime_B;
int pidTerm_B = 0;                                                            // PID correction
double error_B=0;                                   
double last_error_B=0;   
double dErr_B = 0;      
double iTerm_B = 0;
double outMaxI_B = 0;
double outMinI_B = 0;

unsigned long lastTime_C;
int pidTerm_C = 0;                                                            // PID correction
double error_C=0;                                   
double last_error_C=0;   
double dErr_C = 0;      
double iTerm_C = 0;
double outMaxI_C = 0;
double outMinI_C = 0;

unsigned long lastTime_D;
int  pidTerm_D = 0;                                                            // PID correction
double error_D=0;                                   
double last_error_D=0;   
double dErr_D = 0.0;      
double iTerm_D = 0;
double outMaxI_D = 0;
double outMinI_D = 0;

unsigned long cur_time_A;
unsigned long cur_time_B;
unsigned long cur_time_C;
unsigned long cur_time_D;

float output = 0.0;
float fast_val = 1.5;
float slow_val = 0.5;

float slope_A = 1.0;
float old_slope_A = 0.0;
float old_posA_set = 0.0;
float intercept_A = 0.0;

float slope_B = 1.0;
float old_slope_B = 0.0;
float old_posB_set = 0.0;
float intercept_B = 0.0;

float out = 0;
double pi = 3.14159;


void setup() {
  // put your setup code here, to run once:
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT);
  pinMode(hallEffect_A, INPUT);
  digitalWrite(encoder1PinA, HIGH);                      // turn on pullup resistor
  digitalWrite(encoder1PinB, HIGH);
                        
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);
  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT);
  pinMode(hallEffect_B, INPUT);
  digitalWrite(encoder2PinA, HIGH);                      
  digitalWrite(encoder2PinB, HIGH);     

  pinMode(motorC1, OUTPUT);
  pinMode(motorC2, OUTPUT);
  pinMode(motorC_PWM, OUTPUT);
  pinMode(encoder3PinA, INPUT); 
  pinMode(encoder3PinB, INPUT);
  pinMode(hallEffect_C, INPUT);
  digitalWrite(encoder3PinA, HIGH);                      
  digitalWrite(encoder3PinB, HIGH);     

  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(motorD_PWM, OUTPUT);
  pinMode(encoder4PinA, INPUT); 
  pinMode(encoder4PinB, INPUT);
  pinMode(hallEffect_D, INPUT);
  digitalWrite(encoder4PinA, HIGH);                      
  digitalWrite(encoder4PinB, HIGH);     

  attachInterrupt(digitalPinToInterrupt(52), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(50), doEncoder1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(25), checkHall_A, FALLING);

  attachInterrupt(digitalPinToInterrupt(48), doEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(46), doEncoder2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), checkHall_B, FALLING);

  attachInterrupt(digitalPinToInterrupt(44), doEncoder3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(42), doEncoder3B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(24), checkHall_C, FALLING);

  attachInterrupt(digitalPinToInterrupt(40), doEncoder4A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(38), doEncoder4B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(22), checkHall_D, FALLING);

  
  Serial.begin (9600);
  homeMethod();

  
//while(1){
//    digitalWrite(motorA1,LOW);
//    digitalWrite(motorA2,HIGH);
//    analogWrite(motorA_PWM,255);
//    
//    digitalWrite(motorB1,LOW);
//    digitalWrite(motorB2,HIGH);
//    analogWrite(motorB_PWM,255);
//    
//    digitalWrite(motorC1,HIGH);
//    digitalWrite(motorC2,LOW);
//    analogWrite(motorC_PWM,255);
//
//    digitalWrite(motorD1,HIGH);
//    digitalWrite(motorD2,LOW);
//    analogWrite(motorD_PWM,255);
//}
}

String magRead = "OFF";
void loop() {
  
  cur_time_A = millis();
  Serial.print(cur_time_A); Serial.print(",   ");

  posA_act = 2*pi*((encoder1Pos)/(4480.0)); // outputs current position in terms of revolution
  if (digitalRead(25) == LOW) magRead = "ON";
  else magRead = "OFF";

////  slope_A = find_slope(posA_act);
//////  slope_A = slope_mod_time(cur_time_A);
////  if (slope_A != old_slope_A){
////    intercept_A = old_posA_set - (slope_A/1000)*cur_time_A;
////  }
////
//  posA_set = (cur_time_A)*(slope_A/1000.0) + intercept_A;

  posA_set = 1.0*(pi*cur_time_A/1000.0);
  posA_set = posA_set - 2*pi*floor(posA_set/(2*pi)); // this is equivalent to mod(posA_set,2*pi) but mod() doesnt work on double values

  
  Serial.print(posA_set); Serial.print(",   ");
//
////  oldTime_A = cur_time_A;
////  old_posA_set = posA_set;
////  old_slope_A = slope_A;
//
//  Serial.println(posA_act);
  Serial.print(posA_act); Serial.print(",   ");
//  Serial.println(magRead);
//  Serial.println(posA_set-posA_act); //prints the error

    PWM_A_val= updatePid_A(PWM_A_val, posA_set, posA_act);
    Serial.print(error_A);Serial.print(",   ");
  Serial.println(PWM_A_val);


//  cur_time_B = millis();
//  Serial.print(cur_time_B); Serial.print(","); 
//
//  posB_act = (encoder2Pos)/(4480.0);
//  
////  slope_B = find_slope(posB_act);
////  if (slope_B != old_slope_B){
////    intercept_B = old_posB_set - (slope_B/1000)*cur_time_B;
////  }
////  
////  posB_set = (cur_time_B)* (slope_B/1000.0) + intercept_B;
////  posB_set = cur_time_B/1000.0;
//  Serial.print(posB_set); Serial.print(",");
//
////  old_posB_set = posB_set;
////  old_slope_B = slope_B;
////  
////  Serial.print(posB_act); Serial.print(",");
//  Serial.println(posB_act);
//  PWM_B_val= updatePid_B(PWM_B_val, posB_set, posB_act);

//  cur_time_C = millis();
//  Serial.print(cur_time_C); Serial.print(","); 
//  posC_set = 1.0*(cur_time_C/1000.0);
//  Serial.print(posC_set); Serial.print(",");
//  posC_act = (encoder3Pos)/(4480.0);
//  Serial.print(posC_act); Serial.print(",");
//  PWM_C_val= updatePid_C(PWM_C_val, posC_set, posC_act);
//
//  cur_time_D = millis();
//  Serial.print(cur_time_D); Serial.print(","); 
//  posD_set = 1.0*(cur_time_D/1000.0);
//  Serial.print(posD_set); Serial.print(",");
//  posD_act = (encoder4Pos)/(4480.0);
//  Serial.println(posD_act);
//  PWM_D_val= updatePid_D(PWM_D_val, posD_set, posD_act);
//  
  

  if (PWM_A_val >= 0){
    digitalWrite(motorA1,LOW);
    digitalWrite(motorA2,HIGH);
    analogWrite(motorA_PWM,abs(PWM_A_val));
  }
  else {
    digitalWrite(motorA1,HIGH);
    digitalWrite(motorA2,LOW);
    analogWrite(motorA_PWM,abs(PWM_A_val));
  }      

  if (PWM_B_val >= 0){
    digitalWrite(motorB1,LOW);
    digitalWrite(motorB2,HIGH);
    analogWrite(motorB_PWM,abs(PWM_B_val));
  }
  else {
    digitalWrite(motorB1,HIGH);
    digitalWrite(motorB2,LOW);
    analogWrite(motorB_PWM,abs(PWM_B_val));
  }

  if (PWM_C_val >= 0){
    digitalWrite(motorC1,HIGH); //LOW
    digitalWrite(motorC2,LOW); //HIGH
    analogWrite(motorC_PWM,abs(PWM_C_val));
  }
  else {
    digitalWrite(motorC1,LOW); //HIGH
    digitalWrite(motorC2,HIGH); //LOW
    analogWrite(motorC_PWM,abs(PWM_C_val));
  }
 
  if (PWM_D_val >= 0){
    digitalWrite(motorD1,HIGH); // LOW
    digitalWrite(motorD2,LOW); //HIGH
    analogWrite(motorD_PWM,abs(PWM_D_val));
  }
  else {
    digitalWrite(motorD1,LOW); //HIGH
    digitalWrite(motorD2,HIGH); //LOW
    analogWrite(motorD_PWM,abs(PWM_D_val));
  }
}

float slope_mod_time(unsigned long cur_time){
  if (cur_time < 7500){
    out = 1.5;
  }
  else {
    out = 0.5;
  }
  return(out);
}

float find_slope(float current_pos){ // this will only output positive numbers
//current_pos is in revolutions but it needs to be converted to radians (this needs to be changed now)
  float current_pos_rad = abs(current_pos*6.28);
//  Serial.print(current_pos_rad); Serial.print(" ");

  float pos_rad = current_pos_rad - 6.28*floor(current_pos_rad/6.28); // this brings it down to the basic unit circle

//  Serial.print(pos_rad); Serial.print(" ");

  if (pos_rad >= 0 && pos_rad < 3.14){
    output = fast_val;
//    Serial.print("first ");Serial.print(" ");
    }
  else if (pos_rad >= 3.14 && pos_rad < 6.28){
    output = slow_val;
//    Serial.print("second ");Serial.print(" ");
    }
//  else if(pos_rad>=4.71 && pos_rad < 6.28){
//    float m = 2*(fast_val-slow_val)/3.14;
//    float n = 4*slow_val - 3*fast_val;
//    output = (m*pos_rad + n)/6.28;
////    Serial.print("third ");Serial.print(" ");
//  }
  else {
    output = 1;
//    Serial.print("fourth ");
  }
//  Serial.println(output);

  return (output);
}

int updatePid_A(int command_A,float targetValue_A, float currentValue_A)   {             // compute PWM value
  unsigned long now_A = millis();
  double timeChange_A = (double)(now_A - lastTime_A);     
    error_A = posA_set - posA_act; 
    double error_A2 = 2*pi - abs(error_A);
    if (error_A >= 0) error_A2 = -error_A2;
    if (abs(error_A2) < abs(error_A)) error_A = error_A2;
    
    dErr_A = (error_A-last_error_A);
    outMaxI_A = abs(pidTerm_A) - (Kp * error_A) - (Kd * dErr_A) - iTerm_A;
    outMinI_A = -abs(pidTerm_A) - (Kp * error_A) - (Kd * dErr_A) - iTerm_A;
    iTerm_A += Ki*error_A;
    if (iTerm_A > outMaxI_A) iTerm_A = outMaxI_A;
    else if (iTerm_A < outMinI_A) iTerm_A = outMinI_A;
    pidTerm_A = (Kp * error_A) + (Kd * dErr_A) + iTerm_A;    
//    Serial.print(error); Serial.print(" "); Serial.print(dErr); Serial.print(" ");
//    Serial.print(Kp*error_A); Serial.print(" "); Serial.print(Kd*dErr_A); Serial.print(" "); Serial.println(Ki*errSum_A);//Serial.print(" ");
//    Serial.print(error_A); Serial.print(" "); Serial.print(last_error_A);Serial.print(" "); Serial.print(dErr_A); Serial.print(" ");
    last_error_A = error_A;
    lastTime_A = now_A;
    if (command_A + pidTerm_A > 255){
      iTerm_A -= pidTerm_A - 255;
      pidTerm_A = 255 - command_A;
    }
    else if (command_A + pidTerm_A < -255){
      iTerm_A += -255 - pidTerm_A;
      pidTerm_A = -255 - command_A;
    }
    return constrain(command_A + int(pidTerm_A), -255, 255);          
  }

int updatePid_B(int command_B, float targetValue_B, float currentValue_B)   {             // compute PWM value
  unsigned long now_B = millis();
  double timeChange_B = (double)(now_B - lastTime_B);     
    error_B = targetValue_B - currentValue_B; 
    dErr_B = (error_B-last_error_B);// / timeChange;
    outMaxI_B = abs(pidTerm_B) - (Kp * error_B) - (Kd * dErr_B) - iTerm_B; //Serial.print(outMaxI_B); Serial.print(",");
    outMinI_B = -abs(pidTerm_B) - (Kp * error_B) - (Kd * dErr_B) - iTerm_B; //Serial.print(outMinI_B); Serial.print(",");
    iTerm_B += Ki*error_B; //Serial.print(iTerm_B); Serial.print(",");
    if (iTerm_B > outMaxI_B) iTerm_B = outMaxI_B;
    else if (iTerm_B < outMinI_B) iTerm_B = outMinI_B;
    pidTerm_B = (Kp * error_B) + (Kd * dErr_B) + iTerm_B;   
    // Serial.print(iTerm_B); Serial.print(",");
//    Serial.print(error); Serial.print(" "); Serial.print(dErr); Serial.print(" ");
//    Serial.print(Kp*error); Serial.print(" "); Serial.print(Kd*dErr); Serial.print(" "); Serial.print(Ki*errSum);Serial.print(" ");
//    Serial.print(error); Serial.print(" "); Serial.print(last_error);Serial.print(" "); Serial.print(dErr); Serial.print(" ");
    last_error_B = error_B;
    lastTime_B = now_B;
    if (command_B + pidTerm_B > 255){
      iTerm_B -= pidTerm_B - 255;
//      pidTerm_B = 255;
      pidTerm_B = 255 - command_B;
    }
    else if (command_B + pidTerm_B < -255){
      iTerm_B += -255 - pidTerm_B;
//      pidTerm_B = -255;
      pidTerm_B = -255 - command_B;
    }
//    Serial.println(iTerm_B);
    return constrain(command_B + int(pidTerm_B),-255,255);          
  }

int updatePid_C(int command_C, float targetValue_C, float currentValue_C)   {             // compute PWM value
  unsigned long now_C = millis();
  double timeChange_C = (double)(now_C - lastTime_C);     
    error_C = targetValue_C - currentValue_C; 
    dErr_C = (error_C-last_error_C);
    outMaxI_C = abs(pidTerm_C) - (Kp * error_C) - (Kd * dErr_C) - iTerm_C;
    outMinI_C = -abs(pidTerm_C) - (Kp * error_C) - (Kd * dErr_C) - iTerm_C;
    iTerm_C += Ki*(error_C * timeChange_C);
    if (iTerm_C > outMaxI_C) iTerm_C = outMaxI_C;
    else if (iTerm_C < outMinI_C) iTerm_C = outMinI_C;
    pidTerm_C = (Kp * error_C) + (Kd * dErr_C) + iTerm_C;    
    last_error_C = error_C;
    lastTime_C = now_C;
    if (command_C + pidTerm_C > 255){
      iTerm_C -= pidTerm_C - 255;
      pidTerm_C = 255 - command_C;
    }
    else if (command_C + pidTerm_C < -255){
      iTerm_C += -255 - pidTerm_C;
      pidTerm_C = -255 - command_C;
    }
    return constrain(command_C + int(pidTerm_C), -255, 255);          
  }

int updatePid_D(int command_D, float targetValue_D, float currentValue_D)   {             // compute PWM value
  unsigned long now_D = millis();
  double timeChange_D = (double)(now_D - lastTime_D);     
    error_D = targetValue_D - currentValue_D; 
    dErr_D = (error_D-last_error_D);
    outMaxI_D = abs(pidTerm_D) - (Kp * error_D) - (Kd * dErr_D) - iTerm_D;
    outMinI_D = -abs(pidTerm_D) - (Kp * error_D) - (Kd * dErr_D) - iTerm_D;
    iTerm_D += Ki*(error_D * timeChange_D);
    if (iTerm_D > outMaxI_D) iTerm_D = outMaxI_D;
    else if (iTerm_D < outMinI_D) iTerm_D = outMinI_D;
    pidTerm_D = (Kp * error_D) + (Kd * dErr_D) + iTerm_D;    
    last_error_D = error_D;
    lastTime_D = now_D;
    if (command_D + pidTerm_D > 255){
      iTerm_D -= pidTerm_D - 255;
      pidTerm_D = 255 - command_D;
    }
    else if (command_D + pidTerm_D < -255){
      iTerm_D += -255 - pidTerm_D;
      pidTerm_D = -255 - command_D;
    }
    return constrain(command_D + int(pidTerm_D), -255, 255);          
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
  // Test transition
  A2_set = digitalRead(encoder2PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder2Pos += (A2_set != B2_set) ? +1 : -1;
//  if (digitalRead(hallEffect_B) == HIGH) encoder2Pos = 0;
}

// Interrupt on B changing state
void doEncoder2B(){
  // Test transition
  B2_set = digitalRead(encoder2PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder2Pos += (A2_set == B2_set) ? +1 : -1;
//  if (digitalRead(hallEffect_B) == HIGH) encoder2Pos = 0;
}

void doEncoder3A(){
  // Test transition
  A3_set = digitalRead(encoder3PinA) == HIGH; //if both encoders read the same thing, its moving CCW and if they read different values, the motor is moving CW
  // and adjust counter + if A leads B
  encoder3Pos += (A3_set != B3_set) ? +1 : -1;
//  if (digitalRead(hallEffect_C) == HIGH) encoder3Pos = 0;
}

// Interrupt on B changing state
void doEncoder3B(){
  // Test transition
  B3_set = digitalRead(encoder3PinB) == LOW; //HIGH
  // and adjust counter + if B follows A
  encoder3Pos += (A3_set == B3_set) ? +1 : -1;
//  if (digitalRead(hallEffect_C) == HIGH) encoder3Pos = 0; 
}

void doEncoder4A(){
  // Test transition
  A4_set = digitalRead(encoder4PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder4Pos += (A4_set != B4_set) ? +1 : -1;
//  if (digitalRead(hallEffect_D) == HIGH) encoder4Pos = 0;
}

// Interrupt on B changing state
void doEncoder4B(){
  // Test transition
  B4_set = digitalRead(encoder4PinB) == LOW; //HIGH
  // and adjust counter + if B follows A
  encoder4Pos += (A4_set == B4_set) ? +1 : -1;
//  if (digitalRead(hallEffect_D) == HIGH) encoder4Pos = 0;
}

void checkHall_A(){
  encoder1Pos = 0;
}

void checkHall_B(){
  encoder2Pos = 0;
}

void checkHall_C(){
  encoder3Pos = 0;
}

void checkHall_D(){
  encoder4Pos = 0;
}

void homeMethod(){
  Serial.println("hey im in the homing method");
  posA_act = 2*pi*((encoder1Pos)/(4480.0));
  float posA_actOld = posA_act;
  bool flag = false;
  while(posA_act != 2*pi){
    posA_act = 2*pi*((encoder1Pos)/(4480.0));
    Serial.println(posA_act);
    PWM_A_val= updatePid_A(PWM_A_val,2*pi,posA_act) - 100;
    
    digitalWrite(motorA1,LOW);
    digitalWrite(motorA2,HIGH);
    analogWrite(motorA_PWM,PWM_A_val);
    if ( abs(posA_act) > pi / 2 ){
      flag = true;
    }
    if (posA_act < posA_actOld && flag){
      break;
    }
    posA_actOld = posA_act;
  }
  digitalWrite(motorA1,LOW);
    digitalWrite(motorA2,HIGH);
    analogWrite(motorA_PWM,0);
    delay(10000);
  unsigned long now_A = millis();
  while (millis() - now_A < 10000){
    posA_act = 2*pi*((encoder1Pos)/(4480.0));
     PWM_A_val= updatePid_A(PWM_A_val, 0,posA_act) -100;
     digitalWrite(motorA1,LOW);
    digitalWrite(motorA2,HIGH);
    analogWrite(motorA_PWM,PWM_A_val);
  }
}

