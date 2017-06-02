#include "Arduino.h"
#include "AntComm.h"
//IMPORTANT. CHECK THE SIZES OF ARRAYS. C++ is 0 based language. Array[5] has elements 0,1,2,3,4  . Start char takes on  0

// AntComm :: AntComm (int CallSign) //constructor
AntComm :: AntComm () //constructor
{
  // _CallSign=CallSign; //storing a string which whill tell other devices where the message originates from
  _Payload[PACKET_SIZE]; //establishes a size of a packet
  
}

void AntComm :: Send(char DataType, float Data,unsigned long now )
{
   String dummy; //initiate dummy string
  _Payload=dummy + START_CHAR + DataType + Data + SPLIT_CHAR + now ;        //assemble a packet consisting of a character marking a start of message, the origin of the message, and the actual message itself
  uint8_t PayloadLength= _Payload.length();
  for (int i=(1+PayloadLength); i <=(PACKET_SIZE-1); i++)  //note the ith character is stored in the i-1 spot in the array since c++ implements zero based indexing.
  {
    _Payload=_Payload+BLANK_CHAR; //fill up the rest of the packet with blanks
  }
  _Payload=_Payload+STOP_CHAR;    //addds a terminator character. I attempted to use '\0' character that comes with each string but failed. I think its because its two bytes
  Serial.println(_Payload);       //sends the payload away  
  //Serial.println(_Payload.length()); debugging line to verify packet size
}

/*
String AntComm :: Read()
{
  String _message; //create an array to store a message

  if ( Serial.available()>0  ){         //if there is something in a buffer
    //    String _message[PACKET_SIZE]; //create an array to store a message
    char _ch=Serial.read();             //read it
    if ( _ch == START_CHAR)             //check if it belongs to a new message, and if so
    {
      Serial.println("Got something");
      //return("Got Something");
      for (int i=0; i<= (PACKET_SIZE-1); i++) {  //keep reading and storing characters into an array untill the null 
        _ch=Serial.read(); //get it
        switch (_ch){
        case STOP_CHAR :
          //do checksum
          if (_message.length() == PACKET_SIZE-2){  //perform a checksum. Note -2, the final message does not contains the start and terminating character
             return _message; //returns a message verbatum if sucessfuk
          }
          else{
            _message="error";    //overwrites a message 
            return _message;     //returns an error report
          }
          break;
//        case BLANK_CHAR :
//          break;
        default:
          _message=_message+_ch; //stores character in a message
          break;
        }   
      }     
    }
    if (_ch != START_CHAR)
    {
    _message="not start";
    return _message;
    }   
  }
  else{
    _message="no message";
    return _message;
  }
}
*/



