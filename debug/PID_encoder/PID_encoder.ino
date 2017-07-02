#define motorA1 53 //A
#define motorA2 51 //B
#define motorA_PWM 6
#define encoder1PinA 2
#define encoder1PinB 3

#define LOOPTIME        100                     // PID loop time in milliseconds (unit check worked out)

#define motorB1 49 //A
#define motorB2 47 //B
#define motorB_PWM 7 
#define encoder2PinA 4 
#define encoder2PinB 5

#define motorC1 29 //A
#define motorC2 27 //B
#define motorC_PWM 12
#define encoder3PinA 8 
#define encoder3PinB 9

#define motorD1 23 //A
#define motorD2 25 //B
#define motorD_PWM 13
#define encoder4PinA 10 
#define encoder4PinB 11

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

float posA_set = 10;                               // position (Set Point) (in revolution)
float posA_act = 0;                                // position (actual value) (in revolution)
int PWM_A_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

float posB_set = 15;                               // position (Set Point) (in revolution)
float posB_act = 0;                                // position (actual value) (in revolution)
int PWM_B_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

float posC_set = 20;                               // position (Set Point) (in revolution)
float posC_act = 0;                                // position (actual value) (in revolution)
int PWM_C_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

float posD_set = 25;                               // position (Set Point) (in revolution)
float posD_act = 0;                                // position (actual value) (in revolution)
int PWM_D_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

float Kp =    16; //60;                                // PID proportional control Gain
float Kd =    400; //1200;                                // PID Derivitave control gain
float Ki =    0; //0.0001;
// values can be tuned farther
unsigned long lastTime_A;
float pidTerm_A = 0;                                                            // PID correction
float error_A=0;                                   
float last_error_A=0;   
float dErr_A = 0;      
double errSum_A = 0;

unsigned long lastTime_B;
float pidTerm_B = 0;                                                            // PID correction
float error_B=0;                                   
float last_error_B=0;   
float dErr_B = 0;      
double errSum_B = 0;

unsigned long lastTime_C;
float pidTerm_C = 0;                                                            // PID correction
float error_C=0;                                   
float last_error_C=0;   
float dErr_C = 0;      
double errSum_C = 0;

unsigned long lastTime_D;
float pidTerm_D = 0;                                                            // PID correction
float error_D=0;                                   
float last_error_D=0;   
float dErr_D = 0;      
double errSum_D = 0;

unsigned long cur_time_A;
unsigned long cur_time_B;
unsigned long cur_time_C;
unsigned long cur_time_D;


void setup() {
  // put your setup code here, to run once:
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinA, HIGH);                      // turn on pullup resistor
  digitalWrite(encoder1PinB, HIGH);                      

  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);
  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT);
  digitalWrite(encoder2PinA, HIGH);                      
  digitalWrite(encoder2PinB, HIGH);     

  pinMode(motorC1, OUTPUT);
  pinMode(motorC2, OUTPUT);
  pinMode(motorC_PWM, OUTPUT);
  pinMode(encoder3PinA, INPUT); 
  pinMode(encoder3PinB, INPUT);
  digitalWrite(encoder3PinA, HIGH);                      
  digitalWrite(encoder3PinB, HIGH);     

  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(motorD_PWM, OUTPUT);
  pinMode(encoder4PinA, INPUT); 
  pinMode(encoder4PinB, INPUT);
  digitalWrite(encoder4PinA, HIGH);                      
  digitalWrite(encoder4PinB, HIGH);     


  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), doEncoder1B, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(4), doEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), doEncoder2B, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(8), doEncoder3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(9), doEncoder3B, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(10), doEncoder4A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(11), doEncoder4B, CHANGE);

  
  Serial.begin (9600);
//while(1){
//    digitalWrite(motorA1,LOW);
//    digitalWrite(motorA2,HIGH);
//    analogWrite(motorA_PWM,255);
//    
//    digitalWrite(motorB1,LOW);
//    digitalWrite(motorB2,HIGH);
//    analogWrite(motorB_PWM,255);
//    
//    digitalWrite(motorC1,LOW);
//    digitalWrite(motorC2,HIGH);
//    analogWrite(motorC_PWM,255);
//
//    digitalWrite(motorD1,LOW);
//    digitalWrite(motorD2,HIGH);
//    analogWrite(motorD_PWM,255);
//}
}

void loop() {
  // put your main code here, to run repeatedly:
//  getParam();                                                                 // check keyboard
//  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
//    lastMilli = millis();
//    getMotorData();                                                           // calculate speed, volts and Amps
//    PWM_val= updatePid(PWM_val, speed_req, speed_act);                        // compute PWM value
//    analogWrite(motorA_PWM, PWM_val);                                              // send PWM to motor
//  }

  cur_time_A = millis();
  Serial.print(cur_time_A); Serial.print(",");
  posA_set = 0.5*(cur_time_A/1000.0);
  Serial.print(posA_set); Serial.print(",");
  posA_act = (encoder1Pos)/(4480.0); // outputs current position in terms of revolution
  
//  Serial.println(posA_act);
  Serial.print(posA_act); Serial.print(",");
  PWM_A_val= updatePid_A(PWM_A_val, posA_set, posA_act);
//  Serial.println(PWM_A_val);

  cur_time_B = millis();
  Serial.print(cur_time_B); Serial.print(","); 
  posB_set = 1.0*(cur_time_B/1000.0);
  Serial.print(posB_set); Serial.print(",");
  posB_act = (encoder2Pos)/(4480.0);
  Serial.print(posB_act); Serial.print(",");
  PWM_B_val= updatePid_B(PWM_B_val, posB_set, posB_act);

  cur_time_C = millis();
  Serial.print(cur_time_C); Serial.print(","); 
  posC_set = 1.5*(cur_time_C/1000.0);
  Serial.print(posC_set); Serial.print(",");
  posC_act = (encoder3Pos)/(4480.0);
  Serial.print(posC_act); Serial.print(",");
  PWM_C_val= updatePid_C(PWM_C_val, posC_set, posC_act);

  cur_time_D = millis();
  Serial.print(cur_time_D); Serial.print(","); 
  posD_set = 2.0*(cur_time_D/1000.0);
  Serial.print(posD_set); Serial.print(",");
  posD_act = (encoder4Pos)/(4480.0);
  Serial.println(posD_act);
  PWM_D_val= updatePid_D(PWM_D_val, posD_set, posD_act);
  

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
    digitalWrite(motorC1,LOW);
    digitalWrite(motorC2,HIGH);
    analogWrite(motorC_PWM,abs(PWM_C_val));
  }
  else {
    digitalWrite(motorC1,HIGH);
    digitalWrite(motorC2,LOW);
    analogWrite(motorC_PWM,abs(PWM_C_val));
  }
 
  if (PWM_D_val >= 0){
    digitalWrite(motorD1,LOW);
    digitalWrite(motorD2,HIGH);
    analogWrite(motorD_PWM,abs(PWM_D_val));
  }
  else {
    digitalWrite(motorD1,HIGH);
    digitalWrite(motorD2,LOW);
    analogWrite(motorD_PWM,abs(PWM_D_val));
  }
}


int updatePid_A(int command_A, float targetValue_A, float currentValue_A)   {             // compute PWM value
  unsigned long now_A = millis();
  double timeChange_A = (double)(now_A - lastTime_A);     
    error_A = targetValue_A - currentValue_A; 
    errSum_A += (error_A * timeChange_A);
    dErr_A = (error_A-last_error_A);// / timeChange;
    pidTerm_A = (Kp * error_A) + (Kd * dErr_A) + Ki * errSum_A;    
//    Serial.print(error); Serial.print(" "); Serial.print(dErr); Serial.print(" ");
//    Serial.print(Kp*error_A); Serial.print(" "); Serial.print(Kd*dErr_A); Serial.print(" "); Serial.println(Ki*errSum_A);//Serial.print(" ");
//    Serial.print(error_A); Serial.print(" "); Serial.print(last_error_A);Serial.print(" "); Serial.print(dErr_A); Serial.print(" ");
    last_error_A = error_A;
    lastTime_A = now_A;
    return constrain(command_A + int(pidTerm_A), -255, 255);          
  }

int updatePid_B(int command_B, float targetValue_B, float currentValue_B)   {             // compute PWM value
  unsigned long now_B = millis();
  double timeChange_B = (double)(now_B - lastTime_B);     
    error_B = targetValue_B - currentValue_B; 
    errSum_B += (error_B * timeChange_B);
    dErr_B = (error_B-last_error_B);// / timeChange;
    pidTerm_B = (Kp * error_B) + (Kd * dErr_B) + Ki * errSum_B;    
//    Serial.print(error); Serial.print(" "); Serial.print(dErr); Serial.print(" ");
//    Serial.print(Kp*error); Serial.print(" "); Serial.print(Kd*dErr); Serial.print(" "); Serial.print(Ki*errSum);Serial.print(" ");
//    Serial.print(error); Serial.print(" "); Serial.print(last_error);Serial.print(" "); Serial.print(dErr); Serial.print(" ");
    last_error_B = error_B;
    lastTime_B = now_B;
    return constrain(command_B + int(pidTerm_B), -255, 255);          
  }

int updatePid_C(int command_C, float targetValue_C, float currentValue_C)   {             // compute PWM value
  unsigned long now_C = millis();
  double timeChange_C = (double)(now_C - lastTime_C);     
    error_C = targetValue_C - currentValue_C; 
    errSum_C += (error_C * timeChange_C);
    dErr_C = (error_C-last_error_C);// / timeChange;
    pidTerm_C = (Kp * error_C) + (Kd * dErr_C) + Ki * errSum_C;    
//    Serial.print(error); Serial.print(" "); Serial.print(dErr); Serial.print(" ");
//    Serial.print(Kp*error); Serial.print(" "); Serial.print(Kd*dErr); Serial.print(" "); Serial.print(Ki*errSum);Serial.print(" ");
//    Serial.print(error); Serial.print(" "); Serial.print(last_error);Serial.print(" "); Serial.print(dErr); Serial.print(" ");
    last_error_C = error_C;
    lastTime_C = now_C;
    return constrain(command_C + int(pidTerm_C), -255, 255);          
  }

int updatePid_D(int command_D, float targetValue_D, float currentValue_D)   {             // compute PWM value
  unsigned long now_D = millis();
  double timeChange_D = (double)(now_D - lastTime_D);     
    error_D = targetValue_D - currentValue_D; 
    errSum_D += (error_D * timeChange_D);
    dErr_D = (error_D-last_error_D);// / timeChange;
    pidTerm_D = (Kp * error_D) + (Kd * dErr_D) + Ki * errSum_D;    
//    Serial.print(error); Serial.print(" "); Serial.print(dErr); Serial.print(" ");
//    Serial.print(Kp*error); Serial.print(" "); Serial.print(Kd*dErr); Serial.print(" "); Serial.print(Ki*errSum);Serial.print(" ");
//    Serial.print(error); Serial.print(" "); Serial.print(last_error);Serial.print(" "); Serial.print(dErr); Serial.print(" ");
    last_error_D = error_D;
    lastTime_D = now_D;
    return constrain(command_D + int(pidTerm_D), -255, 255);          
  }

// Interrupt on A changing state
void doEncoder1A(){
  // Test transition
  A1_set = digitalRead(encoder1PinA) == HIGH;
  // and adjust encoder1Poser + if A leads B
  encoder1Pos += (A1_set != B1_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoder1B(){
  // Test transition
  B1_set = digitalRead(encoder1PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder1Pos += (A1_set == B1_set) ? +1 : -1;
}

void doEncoder2A(){
  // Test transition
  A2_set = digitalRead(encoder2PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder2Pos += (A2_set != B2_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoder2B(){
  // Test transition
  B2_set = digitalRead(encoder2PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder2Pos += (A2_set == B2_set) ? +1 : -1;
}

void doEncoder3A(){
  // Test transition
  A3_set = digitalRead(encoder3PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder3Pos += (A3_set != B3_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoder3B(){
  // Test transition
  B3_set = digitalRead(encoder3PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder3Pos += (A3_set == B3_set) ? +1 : -1;
}

void doEncoder4A(){
  // Test transition
  A4_set = digitalRead(encoder4PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder4Pos += (A4_set != B4_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoder4B(){
  // Test transition
  B4_set = digitalRead(encoder4PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder4Pos += (A4_set == B4_set) ? +1 : -1;
}
