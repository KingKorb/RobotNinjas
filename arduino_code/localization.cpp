/*
   This code is highly inspired by the "full example" from TinyGPSPlus library.
   It is customized for Arduino Nano Every (ANE), using Serial1 to read GPS data. Thus,
   SoftwareSerial is no longer needed. 

   Connect GPS' TX to ANE's RX0, GPS' RX to ANE's TX1.

   Code is prepared in PlatformIO, rename this file to `main.cpp` and put it in src folder.
*/
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
/* Assign a unique ID to magnetometer at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(11111);

static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(GPSBaud);

  /* Initialise magnetometer */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  /* Display some basic information on magnetometer */
  //  displaySensorDetails();

}

void loop()
{
  //  print magnetometer data
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  //  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: +0* 2' E, which is ~0.03 Degrees, or (which we need) 0.0006 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.0006;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  //  Serial.print("Heading (degrees): "); 

  //  print GPS & Magnetometer data
  Serial.print(gps.location.lat(), 5);
  Serial.print(",");
  Serial.print(gps.location.lng(), 5);
  Serial.print(",");
  Serial.println(headingDegrees);
  
//  delay(500);
  smartDelay(500);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
