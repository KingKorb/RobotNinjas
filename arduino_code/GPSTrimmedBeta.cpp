/*
   This code is highly inspired by the "full example" from TinyGPSPlus library.
   It is customized for Arduino Nano Every (ANE), using Serial1 to read GPS data. Thus,
   SoftwareSerial is no longer needed. 

   Connect GPS' TX to ANE's RX0, GPS' RX to ANE's TX1.

   Code is prepared in PlatformIO, rename this file to `main.cpp` and put it in src folder.
*/
#include <Arduino.h>
#include <TinyGPSPlus.h>

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

void setup()
{
  Serial.begin(9600);
  Serial1.begin(GPSBaud);
}

void loop()
{
  print(gps.location.lat());
  print(",");
  println(gps.location.lng());
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}
