#define motorA1 51 //black wire
#define motorA2 53 //brown
#define motorA_PWM 4
#define encoder1PinA 2
#define encoder1PinB 3 
//#define Vpin            0                       // battery monitoring analog pin
//#define Apin            1                       // motor current monitoring analog pin

//#define CURRENT_LIMIT   1000                     // high current warning
//#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time in milliseconds (unit check worked out)
//#define NUMREADINGS     10                      // samples for Amp average

//#define motorB1 55 //black wire
//#define motorB2 57 //brown
//#define motorB_PWM 8
//#define encoder2PinA 6 
//#define encoder2PinB 7

volatile long encoder1Pos = 0;                  //rev counter
//volatile int encoder2Pos = 0;

boolean A1_set = false;
boolean B1_set = false;
//boolean A2_set = false;
//boolean B2_set = false;

float pos_set = 0;                               // position (Set Point) (in revolution)
float pos_act = 0;                                // position (actual value) (in revolution)
int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
float Kp =    16; //60;                                // PID proportional control Gain
float Kd =    400; //1200;                                // PID Derivitave control gain
float Ki =    0; //0.0001;
// values can be tuned farther
unsigned long lastTime;
float pidTerm = 0;                                                            // PID correction
float error=0;                                   
float last_error=0;   
float dErr = 0;      
double errSum = 0;
unsigned long cur_time;



void setup() {
  // put your setup code here, to run once:
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinA, HIGH);                      // turn on pullup resistor
  digitalWrite(encoder1PinB, HIGH);                      


  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, CHANGE);
  
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(3), doEncoder1B, CHANGE);
  

  

//  pinMode(motorB1, OUTPUT);
//  pinMode(motorB2, OUTPUT);
//  pinMode(motorB_PWM, OUTPUT);
//  pinMode(encoder2PinA, INPUT); 
//  pinMode(encoder2PinB, INPUT);
//
//  attachInterrupt(digitalPinToInterrupt(6), doEncoder2A, CHANGE);
//
//  attachInterrupt(digitalPinToInterrupt(7), doEncoder2B, CHANGE);

  
  Serial.begin (9600);
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


//  pos_act = (encoder1Pos)/(48*74.83); // outputs current position in terms of revolution
//  Serial.print(pos_act); Serial.print(" ");
  cur_time = millis(); 
  Serial.print(cur_time); Serial.print(" ");
  pos_set = cur_time/1000.0;
  Serial.print(pos_set); Serial.print(" ");
  pos_act = (encoder1Pos)/(48*74.83); // outputs current position in terms of revolution
  Serial.println(pos_act);
  
//  Serial.print(pos_set); Serial.print(" ");
  PWM_val= updatePid(PWM_val, pos_set, pos_act);
//  Serial.println(PWM_val);


  if (PWM_val >= 0){
    digitalWrite(motorA1,LOW);
    digitalWrite(motorA2,HIGH);
    analogWrite(motorA_PWM,abs(PWM_val));
  }
  else {
    digitalWrite(motorA1,HIGH);
    digitalWrite(motorA2,LOW);
    analogWrite(motorA_PWM,abs(PWM_val));
  }                                                
 
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

int updatePid(int command, float targetValue, float currentValue)   {             // compute PWM value
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);     
    error = targetValue - currentValue; 
    errSum += (error * timeChange);
    dErr = (error-last_error);// / timeChange;
    pidTerm = (Kp * error) + (Kd * dErr) + Ki * errSum;    
//    Serial.print(error); Serial.print(" "); Serial.print(dErr); Serial.print(" ");
//    Serial.print(Kp*error); Serial.print(" "); Serial.print(Kd*dErr); Serial.print(" "); Serial.print(Ki*errSum);Serial.print(" ");
//    Serial.print(error); Serial.print(" "); Serial.print(last_error);Serial.print(" "); Serial.print(dErr); Serial.print(" ");
    last_error = error;
    lastTime = now;
    return constrain(command + int(pidTerm), -255, 255);          
  }



//void doEncoder2A(){
//  // Test transition
//  A2_set = digitalRead(encoder2PinA) == HIGH;
//  // and adjust counter + if A leads B
//  encoder2Pos += (A2_set != B2_set) ? +1 : -1;
//}
//
//// Interrupt on B changing state
//void doEncoder2B(){
//  // Test transition
//  B2_set = digitalRead(encoder2PinB) == HIGH;
//  // and adjust counter + if B follows A
//  encoder2Pos += (A2_set == B2_set) ? +1 : -1;
//}
