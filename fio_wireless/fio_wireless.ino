//aduino code that goes on fio with xbee 
#include "AntComm.h"
#include "Voltmeter.h"     //loads a custom library to set up an analog pin to read voltage on a source battery
#include "CurrentSensor.h" //loads a custom library to read data from current sensors
#include <SoftwareSerial.h> //loads the software serial library to make GPIOs as UART pins
#include <SPI.h> // SPI comm with SD card
#include <SD.h> // SD card library
#include "serLCD.h"
#include "MasterSlaveProtocol.h"
#define DEBUG 0 // to debug SD stuff

#define current_sensor_pin 0
#define voltage_pin 1

// Software Serial
#define softwhareSerialRx_pin     7
#define softwhareSerialTx_pin     8
#define lcdRx_pin     4 //bani
#define lcdTx_pin     5 //bani
#define lcdPower_pin	3
#define resetDue_pin		  9 // Pin 9 for Robot D, E, and Pin 2 for some robots//bani jsp modified it to pin 9 for all

// SD variables and macros
/* SPI pins
 * MOSI   - pin 11
 * MISO   - pin 12
 * CLK    - pin 13
 * CS     - pin 10
 */
#define CS 10            // Chip select 
#define SD_CLR_SWITCH 6  // Clear files from SD card
File myFile;

//#define SLAVE_ADDRESS 9 //address of this board
#define DUE_ALIVE 0xFF //slave is not sending, connection is broken or something is wrong
#define NO_DATA        0xF0 //slave is not sending, connection is broken or something is wrong
#define RESET_REQUEST  0x99 //reset DUE if this comes up
#define CHECK_START    0x77 //DUE Requested a reset if CHECK_END doesnt arrive
#define CHECK_END      0x88 
#define CHECK_TIMEOUT  5000 //Reset DUE in this many milliseconds if CHECK_END doesnt come in
#define COMMUNICATION_TIMEOUT 30000 //30 seconds 
//package next two in the separate file
#define ROLLED_DIG     0x66 //decision was made to dig
#define ROLLED_REST    0x55 //decision was made to rest
// #define MASTER_GOING_IN         0xA0
// #define MASTER_DIGGING          0XB0
// #define DISP_GOING_IN      0x44 //BANI LCD//68
// #define DISP_DIGGING      0x33 //BANI LCD//51
// AntComm Comm(1); //quick fix, should really get rid of the constructor input
AntComm Comm; //quick fix, should really get rid of the constructor input

#define CURRENT_SAMPLE_SIZE 100 //sets a number of samples to be used for reading current averages
#define VOLTAGE_SAMPLE_SIZE 100
Voltmeter Voltage(voltage_pin);
CurrentSensor Current(current_sensor_pin); //sets up a current sensor. 

SoftwareSerial fioSerial(softwhareSerialRx_pin,softwhareSerialTx_pin);
//SoftwareSerial mySerial(lcdRx_pin,lcdTx_pin);//bani
String inData=""; //string to hold incoming data
int hexData=0x00;
unsigned long whenLastComm; //timer to keep track of when DUE send a response 

bool isCheckTriggered=false; //keep track if there is a pending request
unsigned long whenCheckStart=millis(); //timer to keep track when check was triggered
serLCD lcd(lcdTx_pin);
void setup() {
  readyResetPin(); //configure reset pin
  Serial.begin(9600); //set up serial communication to send out data via Xbees
	fioSerial.begin(19200);
	pinMode(lcdPower_pin, OUTPUT); //power the lcd
  digitalWrite(lcdPower_pin, HIGH); //power the lcd
  // mySerial.begin(9600);//bani
	// delay(500);//bani
  

	// mySerial.print(124); //4800bps//bani
  // mySerial.print(12);//bani
	//establish softwhare serial to communicate with DUE
 // while(1){
 // pinMode(resetDue_pin,OUTPUT); //put the pin in low impeadence mode 
 // digitalWrite(resetDue_pin,HIGH); //turn on pullup resistor and activate transistor
 // }
  ////test reset button
	// pinMode(CS, OUTPUT);
	// while(1){
	  // if (digitalRead(SD_CLR_SWITCH) == HIGH) {
    // Serial.println("bang");
  // }
	// }
	
	whenLastComm = millis(); //initiate timer 
 
  #if DEBUG
    Serial.print(F("Initializing SD card..."));
  #endif
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    #if DEBUG
      Serial.println(F("initialization failed!"));
    #endif
    // reset fio
  }
  #if DEBUG
    Serial.println(F("initialization done."));
  #endif
  
  pinMode(SD_CLR_SWITCH,INPUT);
  
  // Notify start of data logging in the file
		//bani opens contact logging file
	myFile = SD.open("conlog.txt",FILE_WRITE);
  if (myFile) {
    myFile.println(F("-999\t-999"));
    myFile.close();
    #if DEBUG
      Serial.println(F("contact logging started"));
    #endif
  }
  else {
    #if DEBUG
      Serial.println(F("error opening file"));
    #endif    
  }
	//bani
	
  myFile = SD.open("statelog.txt",FILE_WRITE);
  if (myFile) {
    myFile.println(F("-999\t-999"));
    myFile.close();
    #if DEBUG
      Serial.println(F("state logging started"));
    #endif
  }
  else {
    #if DEBUG
      Serial.println(F("error opening file"));
    #endif    
  }
  myFile = SD.open("currlog.txt",FILE_WRITE);
  if (myFile) {
    myFile.println(F("-999\t-999"));
    myFile.close();
    #if DEBUG
      Serial.println(F("current logging started"));
    #endif
  }
  else {
    #if DEBUG
      Serial.println(F("error opening file"));
    #endif    
  }
  myFile = SD.open("voltlog.txt",FILE_WRITE);
  if (myFile) {
    myFile.println(F("-999\t-999"));
    myFile.close();
    #if DEBUG
      Serial.println(F("voltage logging started"));
    #endif
  }
  else {
    #if DEBUG
      Serial.println(F("error opening file"));
    #endif    
  }
  myFile = SD.open("powerlog.txt",FILE_WRITE);
  if (myFile) {
    myFile.println(F("-999\t-999"));
    myFile.close();
    #if DEBUG
      Serial.println(F("power logging started"));
    #endif
  }
  else {
    #if DEBUG
      Serial.println(F("error opening file"));
    #endif    
  }

}

unsigned long lastPowerReport=millis(); //timer used to make sendPowerUsage() function call at least every second

void loop(){

  // check SD clear switch
	
	// mySerial.write(254); // move cursor to beginning of first line//BANI
  // mySerial.write(128);//BANI

  // mySerial.write("                "); // clear display//BANI
  // mySerial.write("                ");//BANI

  // mySerial.write(254); // move cursor to beginning of first line//BANI 
  // mySerial.write(128);//BANI
 
  // mySerial.write("initialisation test!"); //reset flag	//BANI

  if (digitalRead(SD_CLR_SWITCH) == HIGH) {
		lcd.clear();
		lcd.setBrightness(30);
		lcd.print("RESET SWITCH?!");
    software_Reboot();
  }
  
  if((millis()-lastPowerReport) > 2000) { //make power measurements and send them out every once in a while
    sendPowerUsage();
    lastPowerReport=millis();
  }
  readDataFromDue(); //grabs characters until \n shows up, runs results trough switch case 
  // if ((millis() - whenLastComm) > COMMUNICATION_TIMEOUT) {// DUE has not sent ANY reports back. Code must be stuck
	// resetDue(); //force restart DUE
  // }
  if(isCheckTriggered){
   if( millis() - whenCheckStart > CHECK_TIMEOUT ){
   isCheckTriggered=false; //maybe this is a fix for an infinite restart? 
   whenCheckStart=millis();
   resetDue();
   }  
  }
}


void sendPowerUsage(){
  unsigned long  now; //declare storage var
  float C=Current.ReadAvg(CURRENT_SAMPLE_SIZE);
  float V=Voltage.GrabAvg(VOLTAGE_SAMPLE_SIZE);
  now=millis(); //grab time 
  float Power=C*V; //compute power, P= current times voltage

  writeSDcard('C',C,now);
  writeSDcard('V',V,now);
  writeSDcard('W',Power,now);
  // Serial.print("C \t"); Serial.print(C); Serial.print("\t");
  // Serial.print("V \t"); Serial.println(V);
  //Comm.Send('C',C,now); //send out current
  //Comm.Send('V',V,now); //send out voltage
  //Comm.Send('W',Power,now);
  //delay(1000); //wait to avoid buffer overflow
}
//void sendRobotState(){
// Comm.Send('M',state,whenState)
// whenSendState=millis();
//}

void readDataFromDue(){
// have to make sure that the order at which the bytes arrive doesnt get mixed up
// need to add error handling 
  fioSerial.listen();
  while (fioSerial.available()){ //is timeout working properly?
		char c = fioSerial.read();
		inData += c;
		// Serial.println(c,HEX); //debug
		if ( c == '\n'){ //new line char, also 0xA in hex  //print ln sends '\r' and \'n'
			byte inHigh = char2hex(inData[0]);
			byte inLow = char2hex(inData[1]);
			hexData = ((inHigh << 4) | inLow);
	 
			bool testing_cap = false; 	 // if you want to test the capacitive sensors, test testing_cap to true, and if-statements will be used to control the flow of the program to help debug
			if(testing_cap){ // if we are testing the capacitor, we will enter this clause
				Serial.println(inData);
				lcd.clear();
				lcd.setBrightness(30);
				lcd.print(inData);
				writeSDcard('D',hexData,millis());
		 
				// lcd.clear();
				// lcd.setBrightness(30);
				// lcd.print(inData);
			}
	 //end enable this part when calibrating/testing the capacitive sensor
	 
	 
			switch(hexData){
				case DUE_ALIVE:	
					break;
		
				if(!testing_cap){ 		//begin disable this part when calibrating/testing the capacitive sensor
					case RESET_REQUEST:
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("RESET_REQUEST!!");
						resetDue();
						isCheckTriggered=false; //reset flag	
						break;
		
					case CHECK_START:
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("CHECK_START!!");
						isCheckTriggered=true; //set flag 
						whenCheckStart=millis(); //grab time 
						break;
		
					case CHECK_END:
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("CHECK_END!!");
						isCheckTriggered=false; //reset flag	
						break;
		
				// case DISP_GOING_IN:
				// writeLCD('GO'); //reset flag	
				// break;
				
				// case DISP_DIGGING:
				// writeLCD('DI'); //reset flag	
				// break;
		
					case FRONT_SIDE_ANT:
						writeSDcard('N',hexData,millis());
						writeStringSDcard('N',"Front Ant Contact", millis());
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("FRONT, ANT");
						break;//JSP
		
					case RIGHT_SIDE_ANT:
						writeSDcard('N',hexData,millis()); 
						writeStringSDcard('N',"Right Ant Contact", millis());
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("RIGHT, ANT");
						break;//BANI
		
					case LEFT_SIDE_ANT:
						writeSDcard('N',hexData,millis()); 
						writeStringSDcard('N',"Left Ant Contact", millis());
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("LEFT, ANT");
						break;//BANI
					
					case BACK_SIDE_ANT:
						writeSDcard('N',hexData,millis());
						writeStringSDcard('N',"Back Ant Contact", millis());
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("BACK, ANT");
						break;//BANI
					
					case FRONT_SIDE_WALL:
						writeSDcard('N',hexData,millis()); 
						writeStringSDcard('N',"Front Wall Contact", millis());
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("FRONT, WALL");
						break;//JSP
					
					case RIGHT_SIDE_WALL:
						writeSDcard('N',hexData,millis()); 
						writeStringSDcard('N',"Right Wall Contact", millis());
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("RIGHT, WALL");
						break;//BANI
					
					case LEFT_SIDE_WALL:
						writeSDcard('N',hexData,millis()); 
						writeStringSDcard('N',"Left Wall Contact", millis());
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("LEFT, WALL");
						break;//BANI
					
					case BACK_SIDE_WALL: //BANI
						writeSDcard('N',hexData,millis());
						writeStringSDcard('N',"Back Wall Contact", millis());
						lcd.clear();
						lcd.setBrightness(30);
						lcd.print("BACK, WALL");
						break;//BANI
					
					default:
						//Comm.Send('M',hexData,millis()); //send out via xbee
						writeSDcard('M',hexData,millis());
						// delay(500);
						writeLCD(hexData);//bani
						break;
				}
	// //end disable this part when calibrating/testing capacitive sensor
			}
			whenLastComm = millis();
			inData = "";//reset dummy character storage 
			hexData = 0x00; //reset hex data and ready it for bit shifting
		}
  }
} 

void readyResetPin(){
 pinMode(resetDue_pin,INPUT); //put pin in high impeadence mode
 digitalWrite(resetDue_pin,LOW); //turn pullup resistor off
}

void resetDue(){
 // Serial.println("bang");
  pinMode(resetDue_pin,OUTPUT);//bani JSP RESETTING PROBLEM
 //pinMode(resetDue_pin,INPUT); //put the pin in high impeadence mode 
 digitalWrite(resetDue_pin,HIGH); //turn on pullup resistor and activate transistor
 // delay(2000); //wait, let everything power cycle. Reduced from 5s to 2s
	unsigned long dummyTimer = millis(); //with the due killed, empty the serial buffer
	while( fioSerial.available() ){
	char garbage = fioSerial.read();
	 if( millis() - dummyTimer > 5000 ){
	 break; 
	 }	 
	
	};
  delay(2000);	
 
 
 digitalWrite(resetDue_pin,LOW); //turn the pullup resistor off and deactive transistor
 pinMode(resetDue_pin,INPUT); //put the pin back to high impeadence mode 
 isCheckTriggered=false; //reset flag
 whenCheckStart=millis(); //reset timer
  

 
 asm volatile ("  jmp 0"); //also reset FIO. ugly but whatever

}

void writeLCD(int lcddata) { //this function written by bani
  //mySerial.begin(9600);//bani
	// delay(500);//bani
	//mySerial.listen();
	//while (mySerial.available() > 0) {
	
	switch (lcddata) {
    case MASTER_GOING_IN:
		
			lcd.clear();
			lcd.setBrightness(30);
			lcd.print("Going In");
     	// mySerial.write(254); // move cursor to beginning of first line//BANI
      // mySerial.write(128);//BANI

      // mySerial.write("                "); // clear display//BANI
      // mySerial.write("                ");//BANI

      // mySerial.write(254); // move cursor to beginning of first line//BANI 
      // mySerial.write(128);//BANI
 
      // mySerial.print("Going_In_Mode"); 
      break;
			
    case MASTER_DIGGING:
		
			lcd.clear();
			lcd.setBrightness(30);
			lcd.print("Digging");
				// mySerial.write(254); // move cursor to beginning of first line//BANI
				// mySerial.write(128);//BANI

				// mySerial.write("                "); // clear display//BANI
				// mySerial.write("                ");//BANI

				// mySerial.write(254); // move cursor to beginning of first line//BANI 
				// mySerial.write(128);//BANI
	 
				// mySerial.print("Digging_Mode");
			break;
		case MASTER_GOING_OUT:
		
			lcd.clear();
			lcd.setBrightness(30);
			lcd.print("Going Out");
			break;
		
		case MASTER_DUMPING:
		
			lcd.clear();
			lcd.setBrightness(30);
			lcd.print("Dumping");
			break;
		
		case MASTER_GOING_CHARGING:
		
			lcd.clear();
			lcd.setBrightness(30);
			lcd.print("GoingToCharge");
			break;
		
		case MASTER_CHARGING:
		
			lcd.clear();
			lcd.setBrightness(30);
			lcd.print("Charging");
			break;
		
		case MASTER_TURN_REVERSAL:
			lcd.clear();
			lcd.setBrightness(30);
			lcd.print("Turn Reversal");
			break;
	 
	 case MASTER_EXIT_TUNNEL:
		 lcd.clear();
		 lcd.setBrightness(30);
		 lcd.print("Exit Tunnel");
		 break;
	 
	 default:
		// lcd.clear();
		// lcd.setBrightness(30);
		// lcd.print("DEFAULT"); delay(1000);
    lcd.clear();
		lcd.setBrightness(30);
    lcd.print(lcddata, HEX);
		break;
	}

	
//}
  // mySerial.write(254); // move cursor to beginning of first line//BANI
  // mySerial.write(128);//BANI

  // mySerial.write("                "); // clear display//BANI
  // mySerial.write("                ");//BANI

  // mySerial.write(254); // move cursor to beginning of first line//BANI 
  // mySerial.write(128);//BANI
 
  // mySerial.write("initialisation test!"); //reset flag	//BANI
	
	  // lcd.clear();
		// lcd.setBrightness(30);
    // lcd.print(lcddata,HEX);
}
void writeSDcard(char tag, float data, unsigned long time) {
  // choose file to write
  switch (tag) {
	  case 'N':
      myFile = SD.open("conlog.txt",FILE_WRITE);//BANI
      break;
    case 'M':
      myFile = SD.open("statelog.txt",FILE_WRITE);
      break;
    case 'C':
      myFile = SD.open("currlog.txt",FILE_WRITE);
      break;
    case 'V':
      myFile = SD.open("voltlog.txt",FILE_WRITE);
      break;
    case 'W':
      myFile = SD.open("powerlog.txt",FILE_WRITE);
      break;
		case 'D':
			myFile = SD.open("debuglog.txt",FILE_WRITE);
		
  }
  
  // form payload
  String payload;
  payload = payload + data + '\t' + time;
  
  // write to SD card
  if (myFile) {
    myFile.println(payload);
    myFile.close();
  }
  else {
    #if DEBUG
      Serial.println(F("error opening file"));
    #endif    
  }
}

void writeStringSDcard(char tag, String data, unsigned long time) {
  // choose file to write
  switch (tag) {
	  case 'N':
      myFile = SD.open("conlog.txt",FILE_WRITE);//BANI
      break;
    case 'M':
      myFile = SD.open("statelog.txt",FILE_WRITE);
      break;
    case 'C':
      myFile = SD.open("currlog.txt",FILE_WRITE);
      break;
    case 'V':
      myFile = SD.open("voltlog.txt",FILE_WRITE);
      break;
    case 'W':
      myFile = SD.open("powerlog.txt",FILE_WRITE);
      break;
		case 'D':
			myFile = SD.open("debuglog.txt",FILE_WRITE);
		
  }
  
  // form payload
  String payload;
  payload = payload + data + '\t' + time;
  
  // write to SD card
  if (myFile) {
    myFile.println(payload);
    myFile.close();
  }
  else {
    #if DEBUG
      Serial.println(F("error opening file"));
    #endif    
  }
}

void software_Reboot() {

  if (myFile)
    myFile.close();
  
  // Remove all files
  SD.remove("statelog.txt");
  SD.remove("currlog.txt");
  SD.remove("voltlog.txt");
  SD.remove("powerlog.txt");
	SD.remove("conlog.txt");//BANI
	SD.remove("debuglog.txt");
  
  #if DEBUG
    Serial.println(F("Files Removed"));
  #endif
  resetDue(); //reset DUE  
  delay(1000);  // Close any open files  
  // Restart
  asm volatile ("  jmp 0");
}



byte char2hex(char c){
//this is a simple look up table function
 switch(c){
 case '0':
 return 0x0;
 break;
 case '1':
 return 0x1;
 break;
 case '2':
 return 0x2; 
 break;
 case '3':
 return 0x3;
 break;
 case '4':
 return 0x4;
 break;
 case '5':
 return 0x5;
 break;
 case '6':
 return 0x6;
 break;
 case '7':
 return 0x7;
 break;
 case '8':
 return 0x8;
 break;
 case '9':
 return 0x9;
 break;
 case 'A':
 case 'a':
 return 0xA;
 break;
 case 'B':
 case 'b':
 return 0xB;
 break;
 case 'C':
 case 'c':
 return 0xC;
 break;
 case 'D':
 case 'd':
 return 0xD;
 break;
 case 'E':
 case 'e':
 return 0xE;
 break;
 case 'F':
 case 'f':
 return 0xF;
 break;
 }
} 
