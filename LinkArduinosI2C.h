//#include "RobotSelector.h"
#include <Wire.h>
#define SLAVE_ADDRESS 9 //address of this board

void initiateMaster(){
Wire.begin(); // join i2c bus (address optional for master)
return;
}

void masterWrite(int protocolCode){
  /* protocolCode is a two byte hex number, defined in MasterSlaveProtocol.h */
Wire.beginTransmission(SLAVE_ADDRESS); // transmit to device #4
//int high=((protocolCode >> 4) & 0XF);
//int low=(protocolCode & 0xF);
//Serial.println(high,HEX);
//Serial.println(low,HEX);
Wire.write( ((protocolCode >> 4) & 0XF)  );  // send high byte, obtained by bit shifting two spots and using AND mask  
Wire.write( (protocolCode & 0xF) );         // send low byte, obtained by  AND mask   
Wire.endTransmission();    // stop transmitting 
}

byte masterRead(){
Wire.requestFrom(SLAVE_ADDRESS, 1);    // request 1 byte from slave device SLAVE_ADDRESS
byte onebyte = Wire.read(); // receive a byte as character
return onebyte;
}
//====================================================



