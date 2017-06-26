
#define motorA1 51 //black wire
#define motorA2 53 //brown
#define motorA_PWM 4
#define encoder1PinA 2
#define encoder1PinB 3 
//#define Vpin            0                       // battery monitoring analog pin
//#define Apin            1                       // motor current monitoring analog pin

//#define CURRENT_LIMIT   1000                     // high current warning
//#define LOW_BAT         10000                   // low bat warning
#define LOOPTIME        100                     // PID loop time
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

//int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing 
unsigned long lastMilliPrint = 0;               // loop timing
int speed_req = 300;                            // speed (Set Point)
int speed_act = 0;                              // speed (actual value)
int PWM_val = 0;                                // (25% = 64; 50% = 127; 75% = 191; 100% = 255)
int voltage = 0;                                // in mV
int current = 0;                                // in mA
float Kp =   .4;                                // PID proportional control Gain
float Kd =    1;                                // PID Derivitave control gain



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
  
//  for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0

  digitalWrite(motorA1,LOW);
  digitalWrite(motorA2,HIGH);
  analogWrite(motorA_PWM,PWM_val);
  

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
  getParam();                                                                 // check keyboard
  if((millis()-lastMilli) >= LOOPTIME)   {                                    // enter tmed loop
    lastMilli = millis();
    getMotorData();                                                           // calculate speed, volts and Amps
    PWM_val= updatePid(PWM_val, speed_req, speed_act);                        // compute PWM value
    analogWrite(motorA_PWM, PWM_val);                                              // send PWM to motor
  }
 printMotorInfo();                                                           

//  if(encoder1Pos%3592 == 0){
//    rot++;
//    Serial.print ("Rotation ");   Serial.print(rot0);  Serial.print (" complete --- "); Serial.println(encoder1Pos);
//    }
 
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

void getMotorData()  {                                                        // calculate speed, volts and Amps
  static long encoder1Pos_Old = 0;                                                   // last count
  speed_act = ((encoder1Pos - encoder1Pos_Old)*(60*(1000/LOOPTIME)))/(48*74.83);          // 16 pulses X 29 gear ratio = 464 counts per output shaft rev
  encoder1Pos_Old = encoder1Pos;                  
//  voltage = int(analogRead(Vpin) * 3.22 * 12.2/2.2);                          // battery voltage: mV=ADC*3300/1024, voltage divider 10K+2K
//  current = int(analogRead(Apin) * 3.22 * .77 *(1000.0/132.0));               // motor current - output: 130mV per Amp
//  current = digital_smooth(current, readings);                                // remove signal noise
}

int updatePid(int command, int targetValue, int currentValue)   {             // compute PWM value
  float pidTerm = 0;                                                            // PID correction
  int error=0;                                   
  static int last_error=0;                             
  error = abs(targetValue) - abs(currentValue); 
  pidTerm = (Kp * error) + (Kd * (error - last_error));                            
  last_error = error;
  return constrain(command + int(pidTerm), 0, 255);
}

void printMotorInfo()  {                                                      // display data
  if((millis()-lastMilliPrint) >= 500)   {                     
    lastMilliPrint = millis();
//    Serial.print("SP:");             Serial.print(speed_req);  
//    Serial.print("  RPM:");          Serial.print(speed_act);
//    Serial.print("  PWM:");          Serial.print(PWM_val);  
//    Serial.print("  V:");            Serial.print(float(voltage)/1000,1);
//    Serial.print("  mA:");           Serial.println(current);
//
//   if (current > CURRENT_LIMIT)               Serial.println("*** CURRENT_LIMIT ***");                
//   if (voltage > 1000 && voltage < LOW_BAT)   Serial.println("*** LOW_BAT ***");                
 }
}

int getParam()  {
  char param, cmd;
  if(!Serial.available())    return 0;
  delay(10);                  
  param = Serial.read();                              // get parameter byte
  if(!Serial.available())    return 0;
  cmd = Serial.read();                                // get command byte
  Serial.flush();
  switch (param) {
    case 'v':                                         // adjust speed
      if(cmd=='+')  {
        speed_req += 20;
        if(speed_req>400)   speed_req=400;
     }
     if(cmd=='-')    {
       speed_req -= 20;
       if(speed_req<0)   speed_req=0;
     }
     break;
   case 's':                                        // adjust direction
     if(cmd=='+'){
       digitalWrite(motorA1, LOW);
       digitalWrite(motorA2, HIGH);
     }
     if(cmd=='-')   {
       digitalWrite(motorA1, HIGH);
       digitalWrite(motorA2, LOW);
     }
     break;
   case 'o':                                        // user should type "oo"
     digitalWrite(motorA1, LOW);
     digitalWrite(motorA2, LOW);
     speed_req = 0;
     break;
   default: 
     Serial.println("???");
   }
}

//int digital_smooth(int value, int *data_array)  {    // remove signal noise
//  static int ndx=0;                                                         
//  static int count=0;               // count means encoder1Pos           
//  static int total=0;                          
//  total -= data_array[ndx];               
//  data_array[ndx] = value;                
//  total += data_array[ndx];               
//  ndx = (ndx+1) % NUMREADINGS;                                
//  if(count < NUMREADINGS)      count++;
//  return total/count;
//}


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
