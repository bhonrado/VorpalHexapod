#include "UltraSonicSensor.h"


unsigned int readUltrasonic() {  // returns number of centimeters from ultrasonic rangefinder

  pinMode(ULTRAOUTPUTPIN, OUTPUT);
  digitalWrite(ULTRAOUTPUTPIN, LOW);
  delayMicroseconds(5);
  digitalWrite(ULTRAOUTPUTPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRAOUTPUTPIN, LOW);
  
  unsigned int duration = pulseIn(ULTRAINPUTPIN, HIGH, 18000);  // maximum 18 milliseconds which would be about 10 feet distance from object

  //Serial.print("ultra cm:"); Serial.println(duration/58);
  
  if (duration <100) { // Either 0 means timed out, or less than 2cm is out of range as well
    return 1000;   // we will use this large value to mean out of range, since 400 cm is the manufacturer's max range published
  }
  return (duration) / 58;  // this converts microseconds of sound travel time to centimeters. Remember the sound has to go back and forth
                           // so it's traveling twice as far as the object's distance
}


