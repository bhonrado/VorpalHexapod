#ifndef SERVO_H
#define SERVO_H

// NOTE: For digital servos such as Genuine Tower Pro MG90S or Turnigy MG90S we recommend putting
// a smal rubber washer on the hip servo shaft before putting the servo horn on. This will reduce or eliminate
// "hunting" behavior which can cause the servo to rapidly oscillate around the target position. Adjusting
// the servo horn screw tightness to be just tight enough to stop any hunting is recommended.
// This is not needed for analog servos and it is not needed for the Vorpal MG90 branded servos.

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include "Potentiometer.h"

#define NUM_LEGS 6
#define NUM_GRIPSERVOS ((Dialmode == DIALMODE_RC_GRIPARM)?2:0)  
// if we're in griparm mode there are 2 griparm servos, else there are none

#define SERVO_IIC_ADDR  (0x40)    // default servo driver IIC address
//Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(SERVO_IIC_ADDR); 
extern Adafruit_PWMServoDriver servoDriver;
#define ServoTypePin 5        // 5 is used to signal digital vs. analog servo mode
#define ServoTypeGroundPin 6  // 6 provides a ground to pull 5 low if digital servos are in use
#define GripElbowCurrentPin A6  // current sensor for grip arm elbow servo, only used if GRIPARM mode
#define GripClawCurrentPin  A7  // current sensor for grip claw servo, only used if GRIPARM mode

extern int FreqMult;   // PWM frequency multiplier, use 1 for analog servos and up to about 3 for digital.
                    // The recommended setting for digital is 2 (probably safe for all digital servos)
                    // A shunt between Nano D5 and D6 will set this to "1" in setup, this allows you
                    // to select digital servo mode (2) or analog servo mode (1) using a shunt 
                    // or short jumper wire.

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!  If you hear buzzing or jittering, you went too far.
// These values are good for MG90S clone small metal gear servos and Genuine Tower Pro MG90S

#define PWMFREQUENCY (60*FreqMult)

#define SERVOMIN  (190*FreqMult) // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  (540*FreqMult) // this is the 'maximum' pulse length count (out of 4096)
//int pulselen = SERVOMIN;
// Definitions for the servos

#define MAX_GRIPSERVOS 2

extern short ServoPos[2*NUM_LEGS+MAX_GRIPSERVOS]; // the last commanded position of each servo
extern long ServoTime[2*NUM_LEGS+MAX_GRIPSERVOS]; // the time that each servo was last commanded to a new position
extern byte ServoTrim[2*NUM_LEGS+MAX_GRIPSERVOS];  // trim values for fine adjustments to servo horn positions

/* TRIM Functions */
extern byte TrimInEffect;
extern byte TrimCurLeg;
extern byte TrimPose;
#define TRIM_ZERO 127   // this value is the midpoint of the trim range (a byte)

void TrimInit();
void resetServoDriver();

extern int ServosDetached;
void attach_all_servos();
void detach_all_servos();

extern byte deferServoSet;
void setServo(int servonum, int position);

void transactServos();
void commitServos();

extern unsigned long freqWatchDog;
extern unsigned long SuppressScamperUntil;  // if we had to wake up the servos, suppress the power hunger scamper mode for a while
void checkForServoSleep();

void checkForCrashingHips();

void save_trims();
void erase_trims();

#endif