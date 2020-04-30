#ifndef HEXAPOD_H
#define HEXAPOD_H
#include <Arduino.h>
#include "Servo.h"


extern byte SomeLegsUp;  // this is a flag to detect situations where a user rapidly switches moves that would
                      // cause the robot to try to come up off the ground using fewer than all the legs 
                      //(and thus over-stressing the servos).

//Pixy CmuCam5; // cmu cam 5 support as an SPI device, this is currently experimental

// Basic functions that move legs take a bit pattern
// indicating which legs to move. The legs are numbered
// clockwise starting with the right front leg being
// number zero, going around
// to the left legs, and finishing with the left front leg
// being number 5



// Bit patterns for different combinations of legs
// bottom six bits. LSB is leg number 0

#define ALL_LEGS      0b111111
#define LEFT_LEGS     0b111000
#define RIGHT_LEGS    0b000111
#define TRIPOD1_LEGS  0b010101
#define TRIPOD2_LEGS  0b101010
#define FRONT_LEGS    0b100001
#define MIDDLE_LEGS   0b010010
#define BACK_LEGS     0b001100
#define NO_LEGS       0b0

// individual leg bitmasks
#define LEG0 0b1
#define LEG1 0b10
#define LEG2 0b100
#define LEG3 0b1000
#define LEG4 0b10000
#define LEG5 0b100000

#define LEG0BIT  0b1
#define LEG1BIT  0b10
#define LEG2BIT  0b100
#define LEG3BIT  0b1000
#define LEG4BIT  0b10000
#define LEG5BIT  0b100000

#define ISFRONTLEG(LEG) (LEG==0||LEG==5)
#define ISMIDLEG(LEG)   (LEG==1||LEG==4)
#define ISBACKLEG(LEG)  (LEG==2||LEG==3)
#define ISLEFTLEG(LEG)  (LEG==0||LEG==1||LEG==2)
#define ISRIGHTLEG(LEG) (LEG==3||LEG==4||LEG==5)

// default positions for knee and hip. Note that hip position is
// automatically reversed for the left side by the setHip function
// These are in degrees

#define KNEE_UP_MAX 180
#define KNEE_UP    150
#define KNEE_RELAX  120  
#define KNEE_NEUTRAL 90 
#define KNEE_CROUCH 110
#define KNEE_HALF_CROUCH 80
#define KNEE_STAND 30
#define KNEE_DOWN  30   
#define KNEE_TIPTOES 5
#define KNEE_FOLD 170

#define KNEE_SCAMPER (KNEE_NEUTRAL-20)

#define KNEE_TRIPOD_UP (KNEE_NEUTRAL-40)
#define KNEE_TRIPOD_ADJ 30

#define HIPSWING 25      // how far to swing hips on gaits like tripod or quadruped
#define HIPSMALLSWING 10  // when in fine adjust mode how far to move hips
#define HIPSWING_RIPPLE 20
#define HIP_FORWARD_MAX 175
#define HIP_FORWARD (HIP_NEUTRAL+HIPSWING)
#define HIP_FORWARD_SMALL (HIP_NEUTRAL+HIPSMALLSWING)
#define HIP_NEUTRAL 90
#define HIP_BACKWARD (HIP_NEUTRAL-HIPSWING)
#define HIP_BACKWARD_SMALL (HIP_NEUTRAL-HIPSMALLSWING)
#define HIP_BACKWARD_MAX 0
#define HIP_FORWARD_RIPPLE (HIP_NEUTRAL+HIPSWING_RIPPLE)
#define HIP_BACKWARD_RIPPLE (HIP_NEUTRAL-HIPSWING_RIPPLE)
#define HIP_FOLD 150

#define NOMOVE (-1)   // fake value meaning this aspect of the leg (knee or hip) shouldn't move

#define LEFT_START 3  // first leg that is on the left side
#define RIGHT_START 0 // first leg that is on the right side
#define KNEE_OFFSET 6 // add this to a leg number to get the knee servo number

void setLeg(int legmask, int hip_pos, int knee_pos, int adj);
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw);
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw, int leanangle);
void setHipRaw(int leg, int pos);
void setHip(int leg, int pos, int adj);
void setHip(int leg, int pos);
void setKnee(int leg, int pos);

// Definitions for the Grip Arm optional attachment

#define GRIPARM_ELBOW_SERVO 12
#define GRIPARM_CLAW_SERVO  13
#define GRIPARM_ELBOW_DEFAULT 90
#define GRIPARM_CLAW_DEFAULT 90
#define GRIPARM_ELBOW_MIN 30
#define GRIPARM_ELBOW_MAX 180
#define GRIPARM_CLAW_MIN 30
#define GRIPARM_CLAW_MAX 120
#define GRIPARM_CURRENT_DANGER (980)

#define GRIPARM_ELBOW_NEUTRAL 90
#define GRIPARM_CLAW_NEUTRAL 90

extern int GripArmElbowTarget;
extern int GripArmClawTarget;
extern int GripArmElbowDestination;
extern short GripArmElbowIncrement;

void setGrip(int elbow, int claw);
void griparm_mode(char dpad);



/*------------ GAIT -----------------*/

#define G_STAND 0
#define G_TURN  1
#define G_TRIPOD 2
#define G_SCAMPER 3
#define G_DANCE 4
#define G_BOOGIE 5
#define G_FIGHT 6
#define G_TEETER 7
#define G_BALLET 8

#define G_NUMGATES 9

extern int curGait;
extern int curReverse;
extern unsigned long nextGaitTime;

extern unsigned int LastGgaittype;
extern unsigned int LastGreverse;
extern unsigned int LastGhipforward;
extern unsigned int LastGhipbackward;
extern unsigned int LastGkneeup;
extern unsigned int LastGkneedown;
extern unsigned int LastGtimeperiod;
extern int LastGleanangle;   // this can be negative so don't make it unsigned

void gait_command(int gaittype, int reverse, int hipforward, int hipbackward, int kneeup, int kneedown, int leanangle, int timeperiod);

extern long startedStanding;   // the last time we started standing, or reset to -1 if we didn't stand recently
extern long LastReceiveTime;   // last time we got a bluetooth packet
extern unsigned long LastValidReceiveTime;  // last time we got a completely valid packet including correct checksum

void stand();
void stand_90_degrees();
void laydown();
void tiptoes();
void wave(int dpad);
void foldup();
void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod);
void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod, int leanangle);

void gait_tripod(int reverse, int hipforward, int hipbackward, 
          int kneeup, int kneedown, long timeperiod);
void gait_tripod(int reverse, int hipforward, int hipbackward, 
          int kneeup, int kneedown, long timeperiod, int leanangle);
void gait_ripple(int reverse, int hipforward, int hipbackward, int kneeup, 
          int kneedown, long timeperiod, int leanangle);
void gait_sidestep(int left, long timeperiod);

extern int ScamperPhase;
extern unsigned long NextScamperPhaseTime;
extern long ScamperTracker;
void gait_tripod_scamper(int reverse, int turn);

void boogie_woogie(int legs_flat, int submode, int timingfactor);
void flutter();
void random_gait(int timingfactor);
void dance_dab(int timingfactor);
void dance_ballet(int dpad);
void dance_hands(int dpad);
void dance(int legs_up, int submode, int timingfactor);

void checkForSmoothMoves();

/* MODES */
// these modes are used to interpret incoming bluetooth commands

#define TRIPOD_CYCLE_TIME 750
#define RIPPLE_CYCLE_TIME 1800
#define FIGHT_CYCLE_TIME 660

#define MODE_WALK   'W'
#define MODE_DANCE  'D'
#define MODE_FIGHT  'F'
#define MODE_RECORD 'R'
#define MODE_LEG    'L'       // comes from scratch
#define MODE_GAIT   'G'       // comes from scratch
#define MODE_TRIM   'T'       // gamepad in trim mode

#define SUBMODE_1 '1'
#define SUBMODE_2 '2'
#define SUBMODE_3 '3'
#define SUBMODE_4 '4'

extern byte mode; // default
extern byte submode;     // standard submode.

/* -------- FIGHT MODE -----*/

extern unsigned short KneeTarget[NUM_LEGS];
extern unsigned short HipTarget[NUM_LEGS];

void fight_mode(char dpad, int mode, long timeperiod);

#endif