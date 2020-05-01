#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

// A1 and A2 provide power to the potentiometer
#define POTMETER_PWR  A1
#define POTMETER_GND  A2
#define POTMETER_SIG  A0

#define DIALMODE_STAND 0
#define DIALMODE_ADJUST 1
#define DIALMODE_TEST 2
#define DIALMODE_DEMO 3
#define DIALMODE_RC_GRIPARM 4
#define DIALMODE_RC 5

extern int Dialmode;   // What's the robot potentiometer set to?
extern short priorDialMode;

void PotentiometerInit();

#endif