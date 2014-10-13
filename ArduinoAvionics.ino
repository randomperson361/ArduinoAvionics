#include <Arduino.h>
#include <EEPROM.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "ds3234.h"
#include "Adafruit_GPS.h"
#include "DHT.h"

// Define pins used for Arduino operation
#define RADIO_RX_PIN 		0
#define RADIO_TX_PIN 		1

#define DECTALK_RX_PIN		2
#define DECTALK_TX_PIN		3

#define GPS_RX_PIN			7
#define GPS_TX_PIN			8

#define HUMITIDY_PIN		5
#define PITOT_PIN			14			// A0
#define BATT_VOLT_PIN		15			// A1

#define DS3234_SS_PIN 		10
#define SD_SS_PIN 			4
#define MOSI_PIN 			11
#define MISO_PIN 			12
#define SPI_CLK_PIN 		13

#define IMU_SCL_PIN			19			// A5
#define IMU_SDA_PIN			18			// A4

// Define enumerations
enum SPIType {RTC, SDCard};

// Define global variables
File logFile;
static char MessageBuffer[256];
SPIType SPIFunc;

// Define Program Functions
static uint8_t openLogFile()
{
	if(!SD.begin(4))
	{
		return 1;		// SD card error
	}
	logFile = SD.open("log.txt",FILE_WRITE);
	if(logFile)
	{
		return 0;
	}
	else
	{
		return 2;		// file open error
	}
}

static void closeLogFile()
{
	logFile.close();
}

static uint8_t useSDCard()
{
	if(SPIFunc == SDCard)
	{
		return 0;
	}
	else
	{
		DS3234_end();
		SPIFunc = SDCard;
		return openLogFile();
	}
}

static uint8_t useRTC()
{
	if(SPIFunc == RTC)
	{
		return 0;
	}
	else
	{
		closeLogFile();
		SPIFunc = RTC;
		DS3234_init(DS3234_SS_PIN);;
		return 0;
	}
}

static void printTime()
{
	useRTC();
	ts time;
	DS3234_get(DS3234_SS_PIN,&time);
	sprintf(MessageBuffer,"%02u/%02u/%4d %02d:%02d:%02d\t",time.mon,time.mday,time.year,time.hour,time.min,time.sec);
	Serial.print(MessageBuffer);
}

static void processRadio(uint8_t Signal)
{
	// TODO: add radio command processing
}

static void checkRadioCommands()
{
	delay(10);										// wait for data to be received
	while(Serial.available()>0)
	{
		processRadio(Serial.read());
	}
}


//The setup function is called once at startup of the sketch
void setup()
{
	// Initialize pins									// TODO: initialize other pins
	pinMode(SD_SS_PIN,OUTPUT);
	pinMode(DS3234_SS_PIN,OUTPUT);

	digitalWrite(SD_SS_PIN,1);
	digitalWrite(DS3234_SS_PIN,1);

	// Initialize SPI Communication
	DS3234_init(DS3234_SS_PIN);
	SPIFunc = RTC;

	// Initialize radio communication
	Serial.begin(9600,SERIAL_8N1);
	while(!Serial){;}
}

// The loop function is called in an endless loop
void loop()
{






	checkRadioCommands();
	Serial.flush();
}
