#include "Potentiometer.h"
#include <Arduino.h>

int Dialmode;   // What's the robot potentiometer set to?
short priorDialMode = -1;

void PotentiometerInit()
{
    pinMode(POTMETER_POWER, OUTPUT); 
    pinMode(POTMETER_GROUND, OUTPUT); 
    
    digitalWrite(POTMETER_POWER, HIGH);
    digitalWrite(POTMETER_GROUND, LOW);
}