// Default Arduino libraries
#include <Arduino.h>
#include <EEPROM.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Adafruit Unified Sensor Library
#include "Adafruit_Sensor.h"
#include "Adafruit_LSM303_U.h"
#include "Adafruit_BMP085_U.h"
#include "Adafruit_L3GD20_U.h"
#include "Adafruit_10DOF.h"
#include "DHT.h"
#include "DHT_U.h"

// Other external libraries
#include "ds3234.h"
#include "Adafruit_GPS.h"


// Define pins used for Arduino operation
#define RADIO_RX_PIN 		0
#define RADIO_TX_PIN 		1

#define DECTALK_RX_PIN		2
#define DECTALK_TX_PIN		3

#define GPS_RX_PIN			7
#define GPS_TX_PIN			8

#define HUMIDITY_PIN		5
#define PITOT_PIN			14			// A0
#define BATT_VOLT_PIN		15			// A1

#define DS3234_SS_PIN 		10
#define SD_SS_PIN 			4
#define MOSI_PIN 			11
#define MISO_PIN 			12
#define SPI_CLK_PIN 		13

#define IMU_SCL_PIN			19			// A5
#define IMU_SDA_PIN			18			// A4

// Define other values
#define DHT_TYPE DHT22

// Define enumerations
enum SPIType {RTC, SDCard};

// Define global variables
File logFile;
static char MessageBuffer[256];
SPIType SPIFunc;
DHT_Unified dht = DHT_Unified(HUMIDITY_PIN,DHT_TYPE);				// TODO: assign unique ID to this sensor

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

	// Initialize sensors
	dht.begin();

}

// The loop function is called in an endless loop
void loop()
{
	sensors_event_t event;

	// TODO: get data from IMU
	// TODO: get data from GPS
	// TODO: get data from pitot tube
	// TODO: get data from RTC
	// TODO: log data to SD card
	// TODO: create data struct

	// GPS


	// DHT 22
	// Get temperature event and print its value.
	dht.temperature().getEvent(&event);
	if (isnan(event.temperature)) {
	  Serial.println("Error reading temperature!");
	}
	else {
	  Serial.print("Temperature: ");
	  Serial.print(event.temperature);
	  Serial.println(" *C");
	}
	// Get humidity event and print its value.
	dht.humidity().getEvent(&event);
	if (isnan(event.relative_humidity)) {
	  Serial.println("Error reading humidity!");
	}
	else {
	  Serial.print("Humidity: ");
	  Serial.print(event.relative_humidity);
	  Serial.println("%");
	}
	// TODO: add heat index calculation back in

	checkRadioCommands();
	Serial.flush();
}
