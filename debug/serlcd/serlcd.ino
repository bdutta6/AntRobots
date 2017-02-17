// SparkFun Serial LCD example 1
// Clear the display and say "Hello World!"

// This sketch is for Arduino versions 1.0 and later
// If you're using an Arduino version older than 1.0, use
// the other example code available on the tutorial page.

// Use the softwareserial library to create a new "soft" serial port
// for the display. This prevents display corruption when uploading code.

#include "hardSerLCD.h"

// Attach the serial display's RX line to digital pin 2

hardSerLCD Z;

void setup()
{
	 Z.begin(&Serial2, 9600);  
		

  // Serial2.begin(9600); // set up serial port for 9600 baud
  delay(500); // wait for display to boot up
}

void loop()
{
	Z.clear();
	Z.setBrightness(30);
	Z.print("test1 test2 test3");

	delay(1000);
}
