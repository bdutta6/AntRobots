#define motorA1 14 //A
#define motorA2 15 //B
#define motorA_PWM 2
#define encoder1PinA 52
#define encoder1PinB 50
#define hallEffect_A 25

double pi = 3.14159;

volatile long encoder1Pos = 0; 

boolean A1_set = false;
boolean B1_set = false;

double posA_set = 5*pi;                               // position (Set Point) (in revolution)
double posA_act = 0.0;                                // position (actual value) (in revolution)
int PWM_A_val = 0;  

float Kp =   .1; //10 //13 //60;                                // PID proportional control Gain
int Kd =    0; //925 //435 //1200;                                // PID Derivitave control gain
float Ki =   0;//0.00000000000000001;//1.5;

unsigned long lastTime_A;
int pidTerm_A = 0;                                                            // PID correction
double error_A=0;                                   
double last_error_A=0;   
double dErr_A = 0;      
double iTerm_A = 0;
double outMaxI_A = 0;
double outMinI_A = 0;

unsigned long cur_time_A;

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT);
  pinMode(hallEffect_A, INPUT);
  digitalWrite(encoder1PinA, HIGH);                      // turn on pullup resistor
  digitalWrite(encoder1PinB, HIGH);

  attachInterrupt(digitalPinToInterrupt(52), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(50), doEncoder1B, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(25), checkHall_A, FALLING);

  Serial.begin (9600);
//  homeMethod();
}

//String magRead = "OFF";
void loop() {
  // put your main code here, to run repeatedly:

  cur_time_A = millis();
  Serial.print(cur_time_A); Serial.print(",   ");
  posA_set = 6*pi;

  posA_act = 2*pi*((encoder1Pos)/(4480.0)); // outputs current position in terms of revolution
//  if (digitalRead(25) == LOW) magRead = "ON";
//  else magRead = "OFF";

//  posA_set = 1.0*(pi*cur_time_A/1000.0);
//  posA_set = posA_set - 2*pi*floor(posA_set/(2*pi));

  Serial.print(posA_set); Serial.print(",   ");
  Serial.print(posA_act); Serial.print(",   ");
  
  PWM_A_val= updatePid_A(PWM_A_val, posA_set, posA_act);
  Serial.println(error_A);//Serial.print(",   ");
//  Serial.println(PWM_A_val);

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
  if (posA_act > 0){
    posA_set = 0;
  }
}

int updatePid_A(int command_A,float targetValue_A, float currentValue_A)   {             // compute PWM value
  unsigned long now_A = millis();
  double timeChange_A = (double)(now_A - lastTime_A);     
    error_A = posA_set - posA_act; 
//    double error_A2 = 2*pi - abs(error_A);
//    if (error_A >= 0) error_A2 = -error_A2;
//    if (abs(error_A2) < abs(error_A)){
//      error_A = error_A2;
//    }
//    else{
//      error_A = error_A;
//    }
    
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

void checkHall_A(){
  encoder1Pos = 0;
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

