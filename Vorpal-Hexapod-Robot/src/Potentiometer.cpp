#include "Potentiometer.h"
#include <Arduino.h>

int Dialmode;   // What's the robot potentiometer set to?
short priorDialMode = -1;

void PotentiometerInit()
{
    pinMode(POTMETER_PWR, OUTPUT); 
    pinMode(POTMETER_GND, OUTPUT); 
    
    digitalWrite(POTMETER_PWR, HIGH);
    digitalWrite(POTMETER_GND, LOW);
}