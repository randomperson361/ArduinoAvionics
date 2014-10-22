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

#define SD_SS_PIN 			4
#define MOSI_PIN 			11
#define MISO_PIN 			12
#define SPI_CLK_PIN 		13

#define IMU_SCL_PIN			19			// A5
#define IMU_SDA_PIN			18			// A4

// Define other values
#define DHT_TYPE DHT22

// Define Software Serial Ports
SoftwareSerial GPSSerial(8, 7);

// Define global variables
File logFile;
//static char MessageBuffer[256];
DHT_Unified dht = DHT_Unified(HUMIDITY_PIN,DHT_TYPE);				// TODO: assign unique ID to this sensor
Adafruit_GPS GPS(&GPSSerial);

// Define Program Functions
static uint8_t openLogFile()
{
	logFile = SD.open("log.txt",FILE_WRITE);
	if(logFile)
	{
		return 0;
	}
	else
	{
		return 1;		// file open error
	}
}

static void closeLogFile()
{
	logFile.close();
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

void printGPS()
{
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix)
    {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
}

void logGPS()
{
    logFile.print("\nTime: ");
    logFile.print(GPS.hour, DEC); logFile.print(':');
    logFile.print(GPS.minute, DEC); logFile.print(':');
    logFile.print(GPS.seconds, DEC); logFile.print('.');
    logFile.println(GPS.milliseconds);
    logFile.print("Date: ");
    logFile.print(GPS.day, DEC); logFile.print('/');
    logFile.print(GPS.month, DEC); logFile.print("/20");
    logFile.println(GPS.year, DEC);
    logFile.print("Fix: "); logFile.print((int)GPS.fix);
    logFile.print(" quality: "); logFile.println((int)GPS.fixquality);
    if (GPS.fix)
    {
      logFile.print("Location: ");
      logFile.print(GPS.latitude, 4); logFile.print(GPS.lat);
      logFile.print(", ");
      logFile.print(GPS.longitude, 4); logFile.println(GPS.lon);

      logFile.print("Speed (knots): "); logFile.println(GPS.speed);
      logFile.print("Angle: "); logFile.println(GPS.angle);
      logFile.print("Altitude: "); logFile.println(GPS.altitude);
      logFile.print("Satellites: "); logFile.println((int)GPS.satellites);
    }
}

//The setup function is called once at startup of the sketch
void setup()
{
	// Initialize pins									// TODO: initialize other pins
	pinMode(SD_SS_PIN,OUTPUT);
	digitalWrite(SD_SS_PIN,1);

	// Initialize radio communication
	Serial.begin(9600);
	while(!Serial){;}

	// Initialize SD Card
	SD.begin(SD_SS_PIN);
	//openLogFile();

	// Initialize sensors
	dht.begin();

	// Initialize GPS
	GPS.begin(9600);
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);				// output RMC (recommended minimum) and GGA (fix data) including altitude
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);   				// 1 Hz update rate
}

// The loop function is called in an endless loop
void loop()
{
	// Read GPS
	//GPSSerial.listen();
	for (int i = 0; i<2; i++)
	{
		while(!GPS.newNMEAreceived())
		{
		GPS.read();
		}
		Serial.println(GPS.lastNMEA());
	}
	//if (GPS.parse(GPS.lastNMEA()))
	{
		//printGPS();
		//logGPS();
	}


	sensors_event_t event;

	/*
	  // if a sentence is received, we can check the checksum, parse it...
	  if (GPS.newNMEAreceived()) {
	    // a tricky thing here is if we print the NMEA sentence, or data
	    // we end up not listening and catching other sentences!
	    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
	    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

	    if ()   // this also sets the newNMEAreceived() flag to false
	      return;  // we can fail to parse a sentence in which case we should just wait for another
	  }

	/*


	// TODO: get data from IMU
	// TODO: get data from GPS
	// TODO: get data from pitot tube
	// TODO: log data to SD card
	// TODO: create data struct

	*/

	// DHT 22
	// Get temperature event and print its value.
	dht.temperature().getEvent(&event);
	if (isnan(event.temperature)) {
	  Serial.println(F("Error reading temperature!"));
	}
	else {
	  Serial.print(F("Temperature: "));
	  Serial.print(event.temperature);
	  Serial.println(F(" *C"));
	}
	// Get humidity event and print its value.
	dht.humidity().getEvent(&event);
	if (isnan(event.relative_humidity)) {
	  Serial.println(F("Error reading humidity!"));
	}
	else {
	  Serial.print(F("Humidity: "));
	  Serial.print(event.relative_humidity);
	  Serial.println(F("%"));
	}
	// TODO: add heat index calculation back in

	//checkRadioCommands();
	//delay(100);

	// Flush serials
	Serial.flush();
	//logFile.flush();
}

