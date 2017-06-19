


//
//LineSensor::LineSensor(int inputPin){
//  QRE1113_Pin = inputPin;
//}
//
//void LineSensor::setup(){
//  Serial.begin(9600);
//}
//
//
//int LineSensor::readQD(){
//  //Returns value from the QRE1113 
//  //Lower numbers mean more refleacive
//  //More than 3000 means nothing was reflected.
//  pinMode( QRE1113_Pin, OUTPUT );
//  digitalWrite( QRE1113_Pin, HIGH );  
//  delayMicroseconds(10);
//  pinMode( QRE1113_Pin, INPUT );
//
//  long time = micros();
//
//  //time how long the input is HIGH, but quit after 3ms as nothing happens after that
//  while (digitalRead(QRE1113_Pin) == HIGH && micros() - time < 3000); 
//  int diff = micros() - time;
//
//  return diff;
//}
