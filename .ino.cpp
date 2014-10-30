//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2014-10-30 10:39:58

#include "Arduino.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SD.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_LSM303_U.h"
#include "Adafruit_BMP085_U.h"
#include "Adafruit_L3GD20_U.h"
#include "Adafruit_10DOF.h"
#include "DHT.h"
#include "DHT_U.h"
#include "Adafruit_GPS.h"
#include "Emic2TtsModule.h"
#include <Math.h>
static uint8_t openLogFile() ;
static void closeLogFile() ;
static void processRadio(uint8_t Signal) ;
static void checkRadioCommands() ;
SIGNAL(TIMER0_COMPA_vect) ;
void useInterrupt(boolean v) ;
void printGPS() ;
void logGPS() ;
void updateIMU() ;
void printIMU() ;
void updateBaro() ;
void printBaro() ;
void updateDHT() ;
void printDHT() ;
void updatePitot() ;
void printPitot() ;
void setup() ;
void loop() ;


#include "ArduinoAvionics.ino"
