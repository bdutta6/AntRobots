#include "Arduino.h"
#include "CurrentSensor.h"
#include "RobotSelector.h"
// CurrentSensor :: CurrentSensor(int analogPin)
CurrentSensor :: CurrentSensor(){}
// {
//  _analogPin=analogPin;
// }

int CurrentSensor :: ReadRaw()
{
  // _rawValue=analogRead(_analogPin);
  _rawValue=analogRead(currentSensorPin);
  return _rawValue;
}



int CurrentSensor :: ReadRawAvg(int CURRENT_SAMPLE_SIZE)
{
 int currentVals[CURRENT_SAMPLE_SIZE]; //create an array to store current value readings
 int i; //create two dumb variables to run loops
 _rawAvg=0; //prepare for summation operation
 for(i=0; i<CURRENT_SAMPLE_SIZE; i++){
    // _rawValue=analogRead(_analogPin);
    _rawValue=analogRead(currentSensorPin);
    _rawAvg=_rawAvg +_rawValue; //sum up all readings
  }
  _rawAvg=_rawAvg/CURRENT_SAMPLE_SIZE; //divide the sum by a sample size to compute average
  return _rawAvg;
}

float CurrentSensor :: Read()
{
 _rawValue=ReadRaw();
  /*
 sensor is center at 2.5V. Analog pin has 10bit resolution and goes from 0 to 5v.
 This means that analog pin can read out a number from 0 to 1023 (note 2^10=2024)
 Hence 2.5V corresponds to about 512 reading
 Sensor has a 185mV/A resolutuin with a typical error +-1.5%. Hence the conversion formula is:
 */
 //---FOR a 5V   Arduino system
 //_sensorValue= (_rawValue - 512 )*2.5*1000 / (512*185); //Note, this can and will overflow if type int is used. 512*185=94720. However, 2^16=65536. Overfloat will occur and so it will think 512*185=94720-65536=29184
 //_sensorValue= (_rawValue - 512 )*0.026393581;  //0.026393581=2.5*1000 / 512 / 185; //im removing redundand calculation to guard against an overflow
 //---FOR a 3.3V Arduino system
 //_sensorValue= (_rawValue - 775 )*2.5*1000 / (512*185); //Note, this can and will overflow if type int is used. 512*185=94720. However, 2^16=65536. Overfloat will occur and so it will think 512*185=94720-65536=29184
 _sensorValue=( _rawValue - 775)*0.026393581;
 return _sensorValue;  
}

float CurrentSensor :: ReadAvg(int CURRENT_SAMPLE_SIZE) //this function needs to be checked.
{
 _rawAvg=ReadRawAvg(CURRENT_SAMPLE_SIZE); 
 _sensorAvg=(_rawAvg - 775 ) *0.026393581; //note that this is for a 3.3v system
 return _sensorAvg;  
}

bool CurrentSensor :: IsStalled( int CURRENT_SAMPLE_SIZE )
{
  _sensorAvg=ReadAvg( CURRENT_SAMPLE_SIZE );
 if ( _sensorAvg >= STALL_THRESH ){
	return 1;
	} else {
	return 0;
 }

}

bool CurrentSensor :: IsStalledRaw(int CURRENT_SAMPLE_SIZE)
{
 _rawValue=ReadRaw();
 if ( _rawValue >= STALL_THRESH_RAW){
 return 1;
 } else {
 return 0;
 }
}