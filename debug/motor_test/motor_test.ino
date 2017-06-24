
#define motor1 51 //black wire
#define motor2 53 //brown
#define motorPWM 4
#define encoder0PinA 2 //direction
#define encoder0PinB 3 //step

volatile int encoder0Pos = 0;
int rot = 0;

boolean A_set = false;
boolean B_set = false;


void setup() {
  // put your setup code here, to run once:
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, CHANGE);
  
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, CHANGE);  
  
  Serial.begin (9600);


}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(motor1,LOW);
  digitalWrite(motor2,HIGH);
  analogWrite(motorPWM,150);
  Serial.println (encoder0Pos, DEC);

//  if(encoder0Pos%3592 == 0){
//    rot++;
//    Serial.print ("Rotation ");   Serial.print(rot);  Serial.print (" complete --- "); Serial.println(encoder0Pos);
//    }
  

}

// Interrupt on A changing state
void doEncoderA(){
  // Test transition
  A_set = digitalRead(encoder0PinA) == HIGH;
  // and adjust counter + if A leads B
  encoder0Pos += (A_set != B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB(){
  // Test transition
  B_set = digitalRead(encoder0PinB) == HIGH;
  // and adjust counter + if B follows A
  encoder0Pos += (A_set == B_set) ? +1 : -1;
}

