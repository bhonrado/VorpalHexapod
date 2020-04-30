#ifndef ULTRA_SONIC_SENSOR_H
#define ULTRA_SONIC_SENSOR_H
#include <Arduino.h>


#define ULTRAOUTPUTPIN 7      // TRIG
#define ULTRAINPUTPIN  8      // ECHO

unsigned int readUltrasonic();

#endif