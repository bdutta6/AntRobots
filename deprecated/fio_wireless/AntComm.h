/*
  This class sets up a wireless communication between ant robots
  in a v2 version of this library, packets are now containing  a split character to simplfy string parsing process
  
  PACKET FORMAT:
  START_CHAR
  sender's    callsign
  SPLIT_CHAR
  destination code
  SPLIT_CHAR
  nature of data
  SPLIT_CHAR
  data of value (or a state report)
  SPLIT_CHAR
  filler BLANK_CHAR (repeat untill the packet is of size PACKET_SIZE-1 )
  STOP_CHAR
  
  
  destination codes:
  
*/
# ifndef AntComm_h
# define AntComm_h
# include "Arduino.h"

#define PACKET_SIZE  25  //each message sent over the radio will be this many bytes  //1 + 1 + 6 + 2 + 10 + 1 + 1
#define START_CHAR   '*' //set convention to identify the start of each message
#define BLANK_CHAR   '@' //set convention to identify the stop  of each message
#define STOP_CHAR    '$' //a character is used to fill up unused space in each packet
#define SPLIT_CHAR   ',' //a character used to improve string parsing process

class AntComm
{
    public:
      // AntComm(int CallSign);
      AntComm();
        void Send(char DataType, float Data, unsigned long now = millis());
      //String Read();
  
   private:
   // int  _CallSign;
   uint8_t _CallSignLength;  //these two uint8_t's can be removed and hard coded in the .cpp file
   uint8_t _MessageLength;
   String  _Payload;
};

# endif
