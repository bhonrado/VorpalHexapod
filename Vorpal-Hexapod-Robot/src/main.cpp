#include <Arduino.h>


////////////////////////////////////////////////////////////////////////////////
//           Vorpal Hexapod Control Program  
//
// Copyright (C) 2017, 2018 Vorpal Robotics, LLC.
//
// See below all the license comments for new features in this version. (Search for NEW FEATURES)

const char *Version = "#RV2r1a";

//////////// FOR MORE INFORMATION ///////////////////////////////////
// Main website:                  http://www.vorpalrobotics.com
// Store (for parts and kits):    http://store.vorpalrobotics.com
// VORPAL WIKI RELATED TECHNICAL LINKS
// Main entry for hexapod:   http://vorpalrobotics.com/wiki/index.php?title=Vorpal_The_Hexapod
// Radio protocol:           http://vorpalrobotics.com/wiki/index.php/Vorpal_The_Hexapod_Radio_Protocol_Technical_Information
// Grip arm add-on:          http://vorpalrobotics.com/wiki/index.php?title=Vorpal_The_Hexapod_Grip_Arm
// Downloading 3D files:     http://vorpalrobotics.com/wiki/index.php/Vorpal_Hexapod_Source_Files
// Other Vorpal projects:    http://www.vorpalrobotics.com/wiki
/////////////////////////////////////////////////////////////////////


///////////  LICENSE:
//
// This work is licensed under the Creative Commons 
// Attribution-NonCommercial-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
// Attribution for derivations of this work should be made to: Vorpal Robotics, LLC
// 
// You may use this work for noncommercial purposes without cost as long as you give us
// credit for our work (attribution) and any improvements you make are licensed in a way
// no more restrictive than our license.
//
// For example, you may build a Hexapod yourself and use this code for your own experiments,
// or you can build one and give the hexapod running this code to a friend, as long as you
// don't charge for it.
//
// If you have a question about whether a contemplated use is in compliance with the license,
// just ask us. We're friendly. Email us at support@vorpalrobotics.com
//
// For information on licensing this work for commercial purposes, 
// please send email to support@vorpalrobotics.com and we'll work with you to come up with
// something fair.
//
// This is the program that goes on the Robot, not the gamepad!
//
// For more technical informatio, see http://www.vorpalrobotics.com
//

// NOTICE:
// This software uses the Adafruit Servo Driver library. For more information
// see www.adafruit.com
//
// The following information is required to be posted because we are using
// the Adafruit library:

/***************************************************************************

ADAFRUIT PWM SERVO DRIVER LIBRARY
(See https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)

Software License Agreement (BSD License)

Copyright (c) 2012, Adafruit Industries
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/////////////////////////////////////////////////////////////////////////////////

////////////// NEW IN THIS RELEASE /////////////////////
// This release has two major new items:
// 1) Grip Arm support. When the robot dial is set to a position between DEMO and RC this code replaces
//    F2 fight mode with grip arm controls. Forward and Backward DPAD buttons raise and lower
//    the grip arm. Left opens and right closes the claw. The grip arm kit is open source, see
//    the list of links near the top of this file for more information.
//
// 2) Safe Legs. [Currently disabled] The old code allowed gamepad users to rapidly switch between any two motions
//    and some combinations of switches would super stress the servos. We believe this was a
//    major cause of servo wear. For example, if you switched from D1 LEFT (three legs up dancing)
//    to D1 RIGHT (the other three legs up) then depending on timing, you could slam the robot to
//    the ground and then it would attempt to rise up using only 3 legs. It really takes at least 4
//    legs to get the robot off the ground but preferably when rising up from the ground you should
//    try to use all six to keep the servos happy. So this really stresses the three servos trying to lift
//    the robot. Often the robot would only partially rise in this situation, causing the three servos
//    on the ground to stall and they would rapidly overheat if the user keeps the robot like this
//    for any length of time. This kind of thing can also happen with certain combinations of D4 (Mr. Hands
//    Mode) and F1 or F2 (fight mode in which two legs are off the ground.
//    We noticed that in public demos, little kids would do things like this all the time, and usually by
//    the end of the day we'd have a dead servo on half the demo robots, or more if it was a really rough crowd.
//    Anway, to make a long story longer, this release attempts to detect this situation and inserts a short (0.2 second)
//    extra move where all six legs go to the ground, followed by the requested action.
//    The method employed is a bit crude, we just keep a flag SomeLegsUp that is set to 1 whenever a requested move
//    would have less than all the legs on the ground. If a new requested move also would not have all legs on the ground
//    then a short intermediate move is inserted to get the robot up using all the legs. 
//    There's a better way that we will pursue in a future
//    release which involves keeping a model of how many legs are on the ground and detecting situations where the
//    intermediate move is needed. This other method is harder to implement but has the advantage that it would
//    detect and correct even Scratch programs that would overstress the servos. The method implemented in this release
//    will not be able to correct for an errant scratch program that triggers this situation.

#include <Wire.h>
#include <SPI.h>
//#include <Pixy.h>

#include "pinout.h"
#include "Led.h"
#include "Buzzer.h"
#include "Potentiometer.h"
#include "Hexapod.h"
#include "Communication.h"
#include "UltraSonicSensor.h"

#define BATTERYSAVER 5000   // milliseconds in stand mode before servos all detach to save power and heat buildup

Led arduinoLed(BUILT_IN_LED_PIN);

void setup() {
  Serial.begin(9600);
  Serial.println("");
  Serial.println(Version);

  buzzer.Init();
  buzzer.Beep(200);
  
  TrimInit();
  
  arduinoLed.Init();
  /* Make a characteristic flashing pattern to indicate the robot code is loaded */
  arduinoLed.Flash(FlashPattern::Robot);

  PotentiometerInit();

  pinMode(ServoTypeGroundPin, OUTPUT);    // will provide a ground for shunt on D6 to indicate digital servo mode
  digitalWrite(ServoTypeGroundPin, LOW);
  pinMode(ServoTypePin, INPUT_PULLUP);    // if high we default to analog servo mode, if pulled to ground
                                          // (via a shunt to D6) then we'll double the PWM frequency for digital servos
  digitalWrite(13, LOW);
  
  delay(300); // give hardware a chance to come up and stabalize

  BlueTooth.begin(38400);

  BlueTooth.println("");
  delay(250);
  BlueTooth.println(Version);

  delay(250);

  if (digitalRead(ServoTypePin) == LOW) { // Analog servo mode
    FreqMult = 3-FreqMult;  // If FreqMult was 1, this makes it 2. If it was 2, this makes it 1.
                            // In this way the global default of 1 or 2 will reverse if the shunt
                            // is on ServoTypePin
  }
                   
  // Chirp a number of times equal to FreqMult so we confirm what servo mode is in use
  for (int i = 0; i < FreqMult; i++) {
    buzzer.Beep(800, 50);
    delay(100);
  }

  resetServoDriver();
  delay(250);
  
  stand();
  setGrip(90, 90);  // neutral grip arm (if installed)
  
  delay(300);
  
  //CmuCam5.init();   // we're still working out some issues with CmuCam5
  
  buzzer.Beep(400); // Signals end of startup sequence

  yield();
}


int flash(unsigned long t) {
  // the following code will return HIGH for t milliseconds
  // followed by LOW for t milliseconds. Useful for flashing
  // the LED on pin 13 without blocking
  return (millis()%(2*t)) > t;
}


void checkLegStressSituation() {
      return; // This is experimental and for now we're disabling it by immediately returning

#if 0
      
      // ok we got new data. Awesome! If it's not the same mode as the old data and would result in the robot
      // attempting to lift off the ground with less than all six legs, then insert a 200 millisecond
      // attempt to get the robot lifted back off the ground. For now we're just hard coding the
      // major cases that would cause this in practice. A better solution would be to have a watchdog
      // that models the robot's ground-to-standing state at a low level in the servo subroutines.
      // We will do that in a future release, but for now this will save a lot of stress on the servos and
      // is easy.

      // first, let's see if all six legs are plausibly on the ground which we'll define as the hips all
      // having been commanded to a standing angle at least 200 milliseconds ago
      long now = millis();
      byte alldown = 1;
      for (int i = 0; i < NUM_LEGS; i++) {
        if ( (ServoTime[i+KNEE_OFFSET] <= now - 200) && ServoPos[i+KNEE_OFFSET] <= KNEE_STAND) {
          continue;  // this leg meets the criteria
        }
        // if we get here we found a leg that's possibly not all the way down
        alldown = 0;
        break;
      }
      if (alldown) {
        return;   // no need to continue, all the legs are down
      }

      // ok, we're in a dangerous situation. Not every leg is down and we're switching to a new mode.
      // for safety, if the new mode doesn't have all the legs down, we're going to insert a short
      // extra move to bring the legs all down to the ground.
      // the modes that don't have all legs down are: W2*, F1*, F2* if not with GRIPARM, D1l, D1r, D3*, D4*
      // while technically some walking modes may not have all the legs down at certain times, only W2 (high step)
      // would have the legs so far off the ground that it would be a major issue.

      if (
            (mode == 'W' && submode == '2' && lastCmd != 's') ||
            (mode == 'F' && (submode == '1' || (submode == '2' && Dialmode != DIALMODE_RC_GRIPARM) )) ||
            (mode == 'D' && (submode == '3' || submode == '4')) ||
            (mode == 'D' && submode == '1' && (lastCmd == 'r' || lastCmd == 'l'))
          ) {
            // if we get here, we do in fact have a danger situation so command all the hips down
            // to a standing position momentarily
            stand();
            delay(200);
          }

#endif
}



byte timingfactor = 1;   // default is full speed. If this is greater than 1 it multiplies the cycle time making the robot slower

unsigned long ReportTime = 0;
unsigned long SuppressModesUntil = 0;

void loop() {

  checkForServoSleep();
  checkForCrashingHips();
  checkForSmoothMoves();
  
  ////////////////////
  int p = analogRead(A0);
  int factor = 1;
  
  if (p < 50) {
    Dialmode = DIALMODE_STAND;
  } else if (p < 150) {
    Dialmode = DIALMODE_ADJUST;
  } else if (p < 300) {
    Dialmode = DIALMODE_TEST;
  } else if (p < 750) {
    Dialmode = DIALMODE_DEMO;
  } else if (p < 950) {
    Dialmode = DIALMODE_RC_GRIPARM;
  } else {
    Dialmode = DIALMODE_RC;
  }

  if (Dialmode != priorDialMode && priorDialMode != -1) {
    buzzer.Beep(100+100*Dialmode,60);   // audio feedback that a new mode has been entered
    SuppressModesUntil = millis() + 1000;
  }
  priorDialMode = Dialmode;

  if (millis() < SuppressModesUntil) {
    return;
  }
  
  //Serial.print("Analog0="); Serial.println(p);
  
  if (Dialmode == DIALMODE_STAND) { // STAND STILL MODE
    
    digitalWrite(13, LOW);  // turn off LED13 in stand mode
    //resetServoDriver();
    delay(250);
    stand();
    setGrip(90,90);   // in stand mode set the grip arms to neutral positions
    // in Stand mode we will also dump out all sensor values once per second to aid in debugging hardware issues
    if (millis() > ReportTime) {
          ReportTime = millis() + 1000;
          Serial.println("Stand Mode, Sensors:");
          Serial.print(" A3="); Serial.print(analogRead(A3));
          Serial.print(" A6="); Serial.print(analogRead(A6));
          Serial.print(" A7="); Serial.print(analogRead(A7));
          Serial.print(" Dist="); Serial.print(readUltrasonic());
          Serial.println("");
    }

  } else if (Dialmode == DIALMODE_ADJUST) {  // Servo adjust mode, put all servos at 90 degrees
    
    digitalWrite(13, flash(100));  // Flash LED13 rapidly in adjust mode
    stand_90_degrees();

    if (millis() > ReportTime) {
          ReportTime = millis() + 1000;
          Serial.println("AdjustMode");
    }
    
  } else if (Dialmode == DIALMODE_TEST) {   // Test each servo one by one
    pinMode(13, flash(500));      // flash LED13 moderately fast in servo test mode
    
    for (int i = 0; i < 2*NUM_LEGS+NUM_GRIPSERVOS; i++) {
      p = analogRead(A0);
      if (p > 300 || p < 150) {
        break;
      }
      setServo(i, 140);
      delay(500);
      if (p > 300 || p < 150) {
        break;
      }
      setServo(i, 40);
      delay(500);
      setServo(i, 90);
      delay(100);
      Serial.print("SERVO: "); Serial.println(i);
    }
    
  } else if (Dialmode == DIALMODE_DEMO) {  // demo mode

    digitalWrite(13, flash(2000));  // flash LED13 very slowly in demo mode
    random_gait(timingfactor);
    if (millis() > ReportTime) {
          ReportTime = millis() + 1000;
          Serial.println("Demo Mode");
    }
    return;

  } else { // bluetooth mode (regardless of whether it's with or without the grip arm)

    digitalWrite(13, HIGH);   // LED13 is set to steady on in bluetooth mode
    if (millis() > ReportTime) {
          ReportTime = millis() + 2000;
          Serial.print("RC Mode:"); Serial.print(ServosDetached); Serial.write(lastCmd); Serial.write(mode); Serial.write(submode); Serial.println("");
    }
    int gotnewdata = receiveDataHandler();  // handle any new incoming data first
    //Serial.print(gotnewdata); Serial.print(" ");

      // if its been more than 1 second since we got a valid bluetooth command
      // then for safety just stand still.

      if (millis() > LastValidReceiveTime + 1000) {
        if (millis() > LastValidReceiveTime + 15000) {
          // after 15 full seconds of not receiving a valid command, reset the bluetooth connection
          Serial.println("Loss of Signal: resetting bluetooth");
          // Make a three tone chirp to indicate reset
          buzzer.Beep(200,40); // loss of connection test
          delay(100);
          buzzer.Beep(400, 40);
          delay(100);
          buzzer.Beep(600, 40);
          BlueTooth.begin(38400);
          LastReceiveTime = LastValidReceiveTime = millis();
          lastCmd = -1;  // for safety put it in stop mode
        }
        long losstime = millis() - LastValidReceiveTime;
        Serial.print("LOS "); Serial.println(losstime);  // LOS stands for "Loss of Signal"
        return;  // don't repeat commands if we haven't seen valid data in a while
      }

    if (gotnewdata == 0) {
      // we didn't receive any new instructions so repeat the last command unless it was binary
      // or unless we're in fight adjust mode
      if (lastCmd == -1) {
        //Serial.println("REP");
        return;
      }


      // fight submodes 3 and 4 should not be repeated without receiving
      // a packet because otherwise they'll zoom right to the end state instead
      // of giving the user a chance to make fine adjustments to position
      if (mode == MODE_FIGHT && (submode == SUBMODE_3 || submode == SUBMODE_4)) {
        //Serial.print("f");
        return;
      }

      // If the griparm in enabled then fight mode 2 really is grip control mode and
      // this mode is incremental in nature so the user can adjust the grip up/down/open/closed
      // bit by bit
      if (Dialmode == DIALMODE_RC_GRIPARM && mode == MODE_FIGHT && (submode == SUBMODE_2)) {
        return;
      }

    } else {
      LastReceiveTime = millis();

      checkLegStressSituation();
      
    }
    // Leg set mode should also not be repeated
    if (mode == MODE_LEG) {
      //Serial.print("l");
      return;
    } else if (mode == MODE_GAIT) {
        // repeat the last Gait command (from scratch typically)
        gait_command(LastGgaittype, LastGreverse, LastGhipforward, LastGhipbackward, 
                LastGkneeup, LastGkneedown, LastGleanangle, LastGtimeperiod);
        return;
    }
    //
    // Now we're either repeating the last command, or reading the new bluetooth command
    //

    ScamperTracker -= 1;
    if (ScamperTracker < 0) {
      ScamperTracker = 0;
    } else {
      //Serial.println(ScamperTracker);
    }
    
    switch(lastCmd) {
      case '?': BlueTooth.println("#Vorpal Hexapod"); 
        break;
      case 'W': 
        mode = MODE_WALK; 
        break;
      case 'F': 
        mode = MODE_FIGHT; startedStanding = -1;
        break;
      case 'D': 
        mode = MODE_DANCE; startedStanding = -1;
        break;
      case '1': 
      case '2': 
      case '3': 
      case '4': 
        submode = lastCmd;
        break;
      case 'w':  // weapon mode, special depending on mode
        startedStanding = -1;
        switch (mode) {
          case MODE_FIGHT:
            fight_mode(lastCmd, submode, 660*timingfactor);
            break;
          case MODE_DANCE:
            if (submode == SUBMODE_1) {
              dance_dab(timingfactor);
            } else if (submode == SUBMODE_2) {
              dance_ballet(lastCmd);
            } else if (submode == SUBMODE_3) {
              wave(lastCmd);
            } else if (submode == SUBMODE_4) {
              dance_hands(lastCmd);
            }
            break;
          case MODE_WALK: {
              buzzer.Beep(400);
              // stomp in place while beeping horn
              if (submode == SUBMODE_2) { // high step
                factor = 2;
              }
              int cyc = TRIPOD_CYCLE_TIME*factor;
              if (submode == SUBMODE_4) {
                cyc = TRIPOD_CYCLE_TIME/2;  // faster stomp in scamper mode
              }
              gait_tripod(1, 90, 90, 
                      KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, KNEE_DOWN, 
                      cyc);
            }  
            break;
          default:     // for any other mode implement a "horn"
            buzzer.Beep(400);
            break;
        }
        break;
        
      case 'f':  // forward
        startedStanding = -1;
        switch (mode) {
          case MODE_WALK:
              if (submode == SUBMODE_4 && SuppressScamperUntil < millis()) {
                gait_tripod_scamper(0,0);
              } else {
                if (submode == SUBMODE_2) { // high step
                  factor = 2;
                }
                gait_tripod(1, (submode==SUBMODE_3)?HIP_BACKWARD_SMALL:HIP_BACKWARD, 
                  (submode==SUBMODE_3)?HIP_FORWARD_SMALL:HIP_FORWARD, 
                  KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, KNEE_DOWN, 
                  TRIPOD_CYCLE_TIME*factor);
              }
              break;
          case MODE_DANCE:
              if (submode == SUBMODE_1) {
                dance(NO_LEGS, submode, timingfactor);
              } else if (submode == SUBMODE_2) {
                dance_ballet(lastCmd);
              } else if (submode == SUBMODE_3) {
                wave(lastCmd);
              } else if (submode == SUBMODE_4) {
                dance_hands(lastCmd);
              }
              break;
          case MODE_FIGHT:
            fight_mode(lastCmd, submode, FIGHT_CYCLE_TIME*timingfactor);
            break;
        }
        break;

      case 'b':  // backward
        startedStanding = -1;
        switch (mode) {
          case MODE_WALK:
          if (submode == SUBMODE_4 && SuppressScamperUntil < millis()) {
              gait_tripod_scamper(1,0);
          } else {
            if (submode == SUBMODE_2) {
              factor = 2;
            }
            gait_tripod(0, (submode==SUBMODE_3)?HIP_BACKWARD_SMALL:HIP_BACKWARD, 
                (submode==SUBMODE_3)?HIP_FORWARD_SMALL:HIP_FORWARD, 
                KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, KNEE_DOWN, TRIPOD_CYCLE_TIME*factor);
          }
          break;
          case MODE_DANCE:
              if (submode == SUBMODE_1) {
                boogie_woogie(NO_LEGS, submode, timingfactor);
              } else if (submode == SUBMODE_2) {
                dance_ballet(lastCmd);
              } else if (submode == SUBMODE_3) {
                wave(lastCmd);   
              } else if (submode == SUBMODE_4) {
                dance_hands(lastCmd);
              }         
              break;
          case MODE_FIGHT:
              fight_mode(lastCmd, submode, FIGHT_CYCLE_TIME*timingfactor);
            break;
        }
        break;

      case 'l': // left
        startedStanding = -1;
        switch (mode) {
          case MODE_WALK:
            if (submode == SUBMODE_2) {
              factor = 2;
            }
            if (submode == SUBMODE_4 && SuppressScamperUntil < millis()) {
              gait_tripod_scamper(1,1);
            } else {
              turn(0, (submode==SUBMODE_3)?HIP_BACKWARD_SMALL:HIP_BACKWARD, 
                  (submode==SUBMODE_3)?HIP_FORWARD_SMALL:HIP_FORWARD, 
                  KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, KNEE_DOWN, TRIPOD_CYCLE_TIME*factor);
            }
            break;
          case MODE_DANCE:      
            if (submode == SUBMODE_1) {
              dance(TRIPOD1_LEGS, submode, timingfactor);
            } else if (submode == SUBMODE_2) {
              dance_ballet(lastCmd);
            } else if (submode == SUBMODE_3) {
              wave(lastCmd);
            } else if (submode == SUBMODE_4) {
              dance_hands(lastCmd);
            }
            break;
          case MODE_FIGHT:
            fight_mode(lastCmd, submode, FIGHT_CYCLE_TIME*timingfactor);
            break;
        }
        break;

      case 'r':  // right
        startedStanding = -1;
        switch (mode) {
          case MODE_WALK:
            if (submode == SUBMODE_2) {
              factor = 2;
            }
            if (submode == SUBMODE_4 && SuppressScamperUntil < millis()) {
              gait_tripod_scamper(0,1);
            } else {
              turn(1, (submode==SUBMODE_3)?HIP_BACKWARD_SMALL:HIP_BACKWARD, 
                  (submode==SUBMODE_3)?HIP_FORWARD_SMALL:HIP_FORWARD, 
                  KNEE_TRIPOD_UP+factor*KNEE_TRIPOD_ADJ, 
                  KNEE_DOWN, TRIPOD_CYCLE_TIME*factor);
            }
            break;
          case MODE_DANCE:
            if (submode == SUBMODE_1) {
              dance(TRIPOD2_LEGS, submode, timingfactor);
            } else if (submode == SUBMODE_2) {
              dance_ballet(lastCmd);
            } else if (submode == SUBMODE_3) {
              wave(lastCmd);
            } else if (submode == SUBMODE_4) {
              dance_hands(lastCmd);
            }
            break;
          case MODE_FIGHT:
              fight_mode(lastCmd, submode, FIGHT_CYCLE_TIME*timingfactor);
            break;
        }
        break;

      case 's':  // stop and just stand there
        if (startedStanding == -1) {
          startedStanding = millis();
        }
        if (mode == MODE_FIGHT) {
          startedStanding = millis();  // reset in fight mode, never sleep the legs
          fight_mode(lastCmd, submode, 660*timingfactor);
        } else if (mode == MODE_DANCE && submode == SUBMODE_2) { // ballet
          tiptoes();
        } else if (mode == MODE_DANCE && submode == SUBMODE_4) {
          dance_hands(lastCmd);
        } else {
            if (millis() - startedStanding > BATTERYSAVER) {
              //Serial.print("DET LC=");Serial.write(lastCmd); Serial.println("");
              detach_all_servos();
              return;
            }
          stand();
        }


        break;

      case 'a': // adjust mode
        stand_90_degrees();
        break;
       
       default:
        Serial.print("BAD CHAR:"); Serial.write(lastCmd); Serial.println("");
        buzzer.Beep(100,20);
    }  // end of switch
    

  }  // end of main if statement
  


}