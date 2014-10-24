#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN			  5
#define DHTTYPE           DHT22     // DHT 22 (AM2302)

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified 	  gyro = Adafruit_L3GD20_Unified(20);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
DHT_Unified 				  dht(DHTPIN, DHTTYPE);
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_event_t bmp_event;
sensors_vec_t   orientation;
sensors_event_t dht_event;
sensors_event_t gyro_event;


/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }

  /* Enable auto-ranging */
  gyro.enableAutoRange(true);

  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }

  dht.begin();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");

  /* Initialise the sensors */
  initSensors();
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  /* Read the accelerometer and magnetometer */
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  /* Use the new fusionGetOrientation function to merge accel/mag data */
  if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude */
    Serial.print(F("Alt: "));
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure,
                                        temperature));
    Serial.println(F(""));
    /* Display the temperature */
    Serial.print(F("Temp: "));
    Serial.print(temperature);
    Serial.println(F(""));
    Serial.print(F("Pressure "));
    Serial.print(bmp_event.pressure);
    Serial.println(F(""));
  }

  /* Get a new sensor event */
  gyro.getEvent(&gyro_event);

  /* Display the results (speed is measured in rad/s) */
  Serial.print("X: "); Serial.print(gyro_event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(gyro_event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(gyro_event.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");

	dht.temperature().getEvent(&dht_event);
	if (isnan(dht_event.temperature)) {
	  Serial.println(F("Error reading temperature!"));
	}
	else {
	  Serial.print(F("Temperature: "));
	  Serial.print(dht_event.temperature);
	  Serial.println(F(" *C"));
	}
	// Get humidity event and print its value.
	dht.humidity().getEvent(&dht_event);
	if (isnan(dht_event.relative_humidity)) {
	  Serial.println(F("Error reading humidity!"));
	}
	else {
	  Serial.print(F("Humidity: "));
	  Serial.print(dht_event.relative_humidity);
	  Serial.println(F("%\n\n"));
	}

  delay(2000);
}
