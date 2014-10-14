//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2014-10-13 21:57:49

#include "Arduino.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "ds3234.h"
#include "Adafruit_GPS.h"
#include "DHT.h"
static uint8_t openLogFile() ;
static void closeLogFile() ;
static uint8_t useSDCard() ;
static uint8_t useRTC() ;
static void printTime() ;
static void processRadio(uint8_t Signal) ;
static void checkRadioCommands() ;
void setup() ;
void loop() ;


#include "ArduinoAvionics.ino"
