#include "Hexapod.h"

int GripArmElbowTarget=90, GripArmClawTarget=90;
int GripArmElbowDestination = 90;
short GripArmElbowIncrement = 0;

int curGait = G_STAND;
int curReverse = 0;
unsigned long nextGaitTime = 0;

unsigned int LastGgaittype;
unsigned int LastGreverse;
unsigned int LastGhipforward;
unsigned int LastGhipbackward;
unsigned int LastGkneeup;
unsigned int LastGkneedown;
unsigned int LastGtimeperiod;
int LastGleanangle;   // this can be negative so don't make it unsigned

long startedStanding = 0;   // the last time we started standing, or reset to -1 if we didn't stand recently
long LastReceiveTime = 0;   // last time we got a bluetooth packet
unsigned long LastValidReceiveTime = 0;  // last time we got a completely valid packet including correct checksum

int ScamperPhase = 0;
unsigned long NextScamperPhaseTime = 0;
long ScamperTracker = 0;

byte mode = MODE_WALK; // default
byte submode = SUBMODE_1;     // standard submode.

unsigned short KneeTarget[NUM_LEGS];
unsigned short HipTarget[NUM_LEGS];

byte SomeLegsUp = 0;

// This function sets the positions of both the knee and hip in 
// a single command.  For hip, the left side is reversed so
// forward direction is consistent.

// This function takes a bitmask to specify legs to move, note that
// the basic setHip and setKnee functions take leg numbers, not masks

// if a position is -1 then that means don't change that item


void setLeg(int legmask, int hip_pos, int knee_pos, int adj) {
  setLeg(legmask, hip_pos, knee_pos, adj, 0, 0);  // use the non-raw version with leanangle=0
}

// version with leanangle = 0
void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw) {
  setLeg(legmask, hip_pos, knee_pos, adj, raw, 0);
}

void setLeg(int legmask, int hip_pos, int knee_pos, int adj, int raw, int leanangle) {
  for (int i = 0; i < NUM_LEGS; i++) {
    if (legmask & 0b1) {  // if the lowest bit is ON
      if (hip_pos != NOMOVE) {
        if (!raw) {
          setHip(i, hip_pos, adj);
        } else {
          setHipRaw(i, hip_pos);
        }
      }
      if (knee_pos != NOMOVE) {
        int pos = knee_pos;
        if (leanangle != 0) {
          switch (i) {
            case 0: case 6: case 5: case 11:
              if (leanangle < 0) pos -= leanangle;
              break;
            case 1: case 7: case 4: case 10:
              pos += abs(leanangle/2);
              break;
            case 2: case 8: case 3: case 9:
              if (leanangle > 0) pos += leanangle;
              break;
          }
          //Serial.print("Lean:"); Serial.print(leanangle); Serial.print("pos="); Serial.println(pos);
        }
        
        setKnee(i, pos);
      }
    }
    legmask = (legmask>>1);  // shift down one bit position
  }
}

// this version of setHip does no processing at all (for example
// to distinguish left from right sides)
void setHipRaw(int leg, int pos) {
  setServo(leg, pos);
}

// this version of setHip adjusts for left and right legs so
// that 0 degrees moves "forward" i.e. toward legs 5-0 which is
// nominally the front of the robot

void setHip(int leg, int pos) {
  // reverse the left side for consistent forward motion
  if (leg >= LEFT_START) {
    pos = 180 - pos;
  }
  setHipRaw(leg, pos);
}

// this version of setHip adjusts not only for left and right,
// but also shifts the front legs a little back and the back legs
// forward to make a better balance for certain gaits like tripod or quadruped

void setHip(int leg, int pos, int adj) {
  if (ISFRONTLEG(leg)) {
    pos -= adj;
  } else if (ISBACKLEG(leg)) {
    pos += adj;
  }
  // reverse the left side for consistent forward motion
  if (leg >= LEFT_START) {
    pos = 180 - pos;
  }

  setHipRaw(leg, pos);
}

void setKnee(int leg, int pos) {
  // find the knee associated with leg if this is not already a knee
  if (leg < KNEE_OFFSET) {
    leg += KNEE_OFFSET;
  }
  setServo(leg, pos);
}

void setGrip(int elbow, int claw) {
    setServo(GRIPARM_ELBOW_SERVO, elbow);
    setServo(GRIPARM_CLAW_SERVO, claw);
}

void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod) {
  turn(ccw, hipforward, hipbackward, kneeup, kneedown, timeperiod, 0);
}

void turn(int ccw, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod, int leanangle) {
  // use tripod groups to turn in place
  if (ccw) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_TURN_PHASES 6
#define FBSHIFT_TURN    40   // shift front legs back, back legs forward, this much
  
  long t = millis()%timeperiod;
  long phase = (NUM_TURN_PHASES*t)/timeperiod;

  //Serial.print("PHASE: ");
  //Serial.println(phase);

  switch (phase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at
      // the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, kneeup, 0);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move clockwise
      // at the hips, while the rest of the legs move CCW at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT_TURN, 1);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT_TURN, 1);
      break;

    case 2: 
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, kneeup, 0);
      break;
      
    case 4:
      // similar to phase 1, move raised legs CW and lowered legs CCW
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT_TURN, 1);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT_TURN, 1);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0);
      break;  
  }
  
}


void stand() {
  transactServos();
    setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_STAND, 0);
  commitServos();
}

void stand_90_degrees() {  // used to install servos, sets all servos to 90 degrees
  transactServos();
  setLeg(ALL_LEGS, 90, 90, 0);
  setGrip(90,90);
  commitServos();
}

void laydown() {
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_UP, 0);
}

void tiptoes() {
  setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
}

void wave(int dpad) {
  
#define NUM_WAVE_PHASES 12
#define WAVE_CYCLE_TIME 900
#define KNEE_WAVE  60
  long t = millis()%WAVE_CYCLE_TIME;
  long phase = (NUM_WAVE_PHASES*t)/WAVE_CYCLE_TIME;

  if (dpad == 'b') {
    phase = 11-phase;  // go backwards
  }

  switch (dpad) {
    case 'f':
    case 'b':
      // swirl around
      setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0); // keep hips stable at 90 degrees
      if (phase < NUM_LEGS) {
        setKnee(phase, KNEE_WAVE);
      } else {
        setKnee(phase-NUM_LEGS, KNEE_STAND);
      }
      break;
    case 'l':
      // teeter totter around font/back legs
   
      if (phase < NUM_WAVE_PHASES/2) {
        setKnee(0, KNEE_TIPTOES);
        setKnee(5, KNEE_STAND);
        setHipRaw(0, HIP_FORWARD);
        setHipRaw(5, HIP_BACKWARD-40);
        setKnee(2, KNEE_TIPTOES);
        setKnee(3, KNEE_STAND);
        setHipRaw(2, HIP_BACKWARD);
        setHipRaw(3, HIP_FORWARD+40);
                
        setLeg(LEG1, HIP_NEUTRAL, KNEE_TIPTOES, 0);
        setLeg(LEG4, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
      } else {
        setKnee(0, KNEE_STAND);
        setKnee(5, KNEE_TIPTOES);
        setHipRaw(0, HIP_FORWARD+40);
        setHipRaw(5, HIP_BACKWARD);
        setKnee(2, KNEE_STAND);
        setKnee(3, KNEE_TIPTOES);
        setHipRaw(2, HIP_BACKWARD-40);
        setHipRaw(3, HIP_FORWARD);
           
        setLeg(LEG1, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
        setLeg(LEG4, HIP_NEUTRAL, KNEE_TIPTOES, 0);
      }
      break;
    case 'r':
      // teeter totter around middle legs
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, KNEE_STAND, 0);
      if (phase < NUM_LEGS) {
        setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, 0);
        setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
      } else {
        setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_TIPTOES, 0);
        setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, 0);       
      }
      break;
    case 'w':
      // lay on ground and make legs go around in a wave
      setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0);
      int p = phase/2;
      for (int i = 0; i < NUM_LEGS; i++) {
        if (i == p) {
          setKnee(i, KNEE_UP_MAX);
        } else {
          setKnee(i, KNEE_NEUTRAL);
        }
      }
      return;
      if (phase < NUM_LEGS) {
        setKnee(phase/2, KNEE_UP);
      } else {
        int p = phase-NUM_LEGS;
        if (p < 0) p+=NUM_LEGS;
        setKnee(p/2, KNEE_NEUTRAL+10);
      }
      break;
  }
}

void gait_sidestep(int left, long timeperiod) {

  // the gait consists of 6 phases and uses tripod definitions

#define NUM_SIDESTEP_PHASES 6
#define FBSHIFTSS    50   // shift front legs back, back legs forward, this much
  
  long t = millis()%timeperiod;
  long phase = (NUM_SIDESTEP_PHASES*t)/timeperiod;
  int side1 = LEFT_LEGS;
  int side2 = RIGHT_LEGS;

  if (left == 0) {
    side1 = RIGHT_LEGS;
    side2 = LEFT_LEGS;
  }

  //Serial.print("PHASE: ");
  //Serial.println(phase);

  transactServos();
  
  switch (phase) {
    case 0:
      // Lift up tripod group 1 while group 2 goes to neutral setting
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, KNEE_UP, FBSHIFTSS);
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFTSS);
      break;

    case 1:
      // slide over by curling one side under the body while extending the other side
      setLeg(TRIPOD2_LEGS&side1, HIP_NEUTRAL, KNEE_DOWN, FBSHIFTSS);
      setLeg(TRIPOD2_LEGS&side2, HIP_NEUTRAL, KNEE_RELAX, FBSHIFTSS);
      break;

    case 2: 
      // now put the first set of legs back down on the ground
      // and at the sametime put the curled legs into neutral position
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFTSS);
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFTSS);
      break;

    case 3:
      // Lift up tripod group 2 while group 2 goes to neutral setting
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, KNEE_UP, FBSHIFTSS);
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFTSS);  
      break;
      
    case 4:
      // slide over by curling one side under the body while extending the other side
      setLeg(TRIPOD1_LEGS&side1, HIP_NEUTRAL, KNEE_DOWN, FBSHIFTSS);
      setLeg(TRIPOD1_LEGS&side2, HIP_NEUTRAL, KNEE_RELAX, FBSHIFTSS);
      break;

    case 5:
      // now put all the legs back down on the ground, then the cycle repeats
      setLeg(TRIPOD1_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFTSS);
      setLeg(TRIPOD2_LEGS, HIP_NEUTRAL, KNEE_NEUTRAL, FBSHIFTSS);
      break;
  }
  commitServos();
}



void griparm_mode(char dpad) {
    // this mode retains state and moves slowly

    //Serial.print("Grip:"); Serial.print(dpad); 

    Serial.println();
      switch (dpad) {
      case 's': 
        // do nothing in stop mode, just hold current position
        GripArmElbowTarget = ServoPos[GRIPARM_ELBOW_SERVO];
        GripArmClawTarget = ServoPos[GRIPARM_CLAW_SERVO];
        break;
      case 'w':  // reset to standard grip arm position, arm raised to mid-level and grip open a medium amount
        GripArmElbowTarget = GRIPARM_ELBOW_DEFAULT;
        GripArmClawTarget = GRIPARM_CLAW_DEFAULT;
        break;
      case 'f': // Elbow up
        GripArmElbowTarget = GRIPARM_ELBOW_MIN;
        break;
      case 'b': // Elbow down
        GripArmElbowTarget = GRIPARM_ELBOW_MAX;
        break;
      case 'l':  // Claw closed
        GripArmClawTarget = GRIPARM_CLAW_MAX;
        break;
      case 'r': // Claw open
        GripArmClawTarget = GRIPARM_CLAW_MIN;
        break;
    }

    // Now that all the targets are adjusted, move the servos toward their targets, slowly

#define GRIPMOVEINCREMENT 10  // degrees per transmission time delay

      int h, k;
      h = ServoPos[GRIPARM_ELBOW_SERVO];
      k = ServoPos[GRIPARM_CLAW_SERVO];
      int diff = GripArmClawTarget - k;
      
      if (diff <= -GRIPMOVEINCREMENT) {
        // the claw has a greater value than the target
        k -= GRIPMOVEINCREMENT;
      } else if (diff >= GRIPMOVEINCREMENT) {
        // the claw has a smaller value than the target
        k += GRIPMOVEINCREMENT;
      } else {
        // the claw is within MOVEINCREMENT of the target so just go to target
        k = GripArmClawTarget;
      }

      setServo(GRIPARM_CLAW_SERVO, k);

      diff = GripArmElbowTarget - h;

      // smooth move mode on elbow
      if (diff < -GRIPMOVEINCREMENT) {
        // the elbow has a greater value than the target
        h -= GRIPMOVEINCREMENT;
      } else if (diff >= GRIPMOVEINCREMENT) {
        // the elbow has a smaller value than the target
        h += GRIPMOVEINCREMENT;
      } else {
        // the elbow is within MOVEINCREMENT of the target so just go to target
        h = GripArmElbowTarget;
      }
      GripArmElbowDestination = h;
      GripArmElbowIncrement = (h < ServoPos[GRIPARM_ELBOW_SERVO])?-2:2;
      //Serial.print("GADest="); Serial.print(h); Serial.print(" GAinc="); Serial.println(GripArmElbowIncrement);
    
}

void gait_command(int gaittype, int reverse, int hipforward, int hipbackward, int kneeup, int kneedown, int leanangle, int timeperiod) {

       if (ServosDetached) { // wake up any sleeping servos
        attach_all_servos();
       }

       switch (gaittype) {
        case 0:
        default:
                   gait_tripod(reverse, hipforward, hipbackward, kneeup, kneedown, timeperiod, leanangle);
                   break;
        case 1:
                   turn(reverse, hipforward, hipbackward, kneeup, kneedown, timeperiod, leanangle);
                   break;
        case 2:
                   gait_ripple(reverse, hipforward, hipbackward, kneeup, kneedown, timeperiod, leanangle);
                   break;
        case 3:
                   gait_sidestep(reverse, timeperiod);
                   break;
       }

#if 0
       Serial.print("GAIT: style="); Serial.print(gaittype); Serial.print(" dir="); Serial.print(reverse,DEC); Serial.print(" angles=");Serial.print(hipforward);
       Serial.print("/"); Serial.print(hipbackward); Serial.print("/"); Serial.print(kneeup,DEC); Serial.print("/"); Serial.print(kneedown); 
       Serial.print("/"); Serial.println(leanangle);
#endif

       mode = MODE_GAIT;   // this stops auto-repeat of gamepad mode commands
}

void fight_mode(char dpad, int mode, long timeperiod) {

#define HIP_FISTS_FORWARD 130

  if (Dialmode == DIALMODE_RC_GRIPARM && mode == SUBMODE_2) {
    // we're really not fighting, we're controlling the grip arm if GRIPARM is nonzero
    griparm_mode(dpad);
    return;
  }
 
  if (mode == SUBMODE_3) {
    // in this mode the robot leans forward, left, or right by adjusting hips only

    // this mode retains state and moves slowly, it's for getting somethign like the joust or 
    // capture the flag accessories in position

      switch (dpad) {
      case 's': 
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          KneeTarget[i] = ServoPos[i+NUM_LEGS];
          HipTarget[i] = ServoPos[i];
        }
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          KneeTarget[i] = KNEE_STAND;
          HipTarget[i] = 90;
        }
        break;
      case 'f': // swing hips forward, mirrored
        HipTarget[5] = HipTarget[4] = HipTarget[3] = 125;
        HipTarget[0] = HipTarget[1] = HipTarget[2] = 55;
        break;
      case 'b': // move the knees back up to standing position, leave hips alone
        HipTarget[5] = HipTarget[4] = HipTarget[3] = 55;
        HipTarget[0] = HipTarget[1] = HipTarget[2] = 125;
        break;
      case 'l':
        for (int i = 0; i < NUM_LEGS; i++) {
          HipTarget[i] = 170;
        }
        break;
      case 'r':
        for (int i = 0; i < NUM_LEGS; i++) {
          HipTarget[i] = 10;
        }
        break;
    }
    
  } else if (mode == SUBMODE_4) {
    // in this mode the entire robot leans in the direction of the pushbuttons
    // and the weapon button makes the robot return to standing position.

    // Only knees are altered by this, not hips (other than the reset action for
    // the special D-PAD button)

    // this mode does not immediately set servos to final positions, instead it
    // moves them toward targets slowly.
    
    switch (dpad) {
      case 's': 
        // do nothing in stop mode, just hold current position
        for (int i = 0; i < NUM_LEGS; i++) {
          KneeTarget[i] = ServoPos[i+NUM_LEGS];
          HipTarget[i] = ServoPos[i];
        }
        break;
      case 'w':  // reset to standard standing position, resets both hips and knees
        for (int i = 0; i < NUM_LEGS; i++) {
          KneeTarget[i] = KNEE_STAND;
          HipTarget[i] = 90;
        }
        break;
      case 'f': // move knees into forward crouch, leave hips alone

        if (ServoPos[8] == KNEE_STAND) { // the back legs are standing, so crouch the front legs
          KneeTarget[0]=KneeTarget[5]=KNEE_CROUCH;
          KneeTarget[1]=KneeTarget[4]=KNEE_HALF_CROUCH;
          KneeTarget[2]=KneeTarget[3]=KNEE_STAND;
        } else { // bring the back legs up first
          for (int i = 0; i < NUM_LEGS; i++) {
            KneeTarget[i] = KNEE_STAND;
          }
        }
        break;
      case 'b': // move back legs down so robot tips backwards
        if (ServoPos[6] == KNEE_STAND) { // move the back legs down
          KneeTarget[0]=KneeTarget[5]=KNEE_STAND;
          KneeTarget[1]=KneeTarget[4]=KNEE_HALF_CROUCH;
          KneeTarget[2]=KneeTarget[3]=KNEE_CROUCH;
        } else { // front legs are down, return to stand first
            for (int i = 0; i < NUM_LEGS; i++) {
              KneeTarget[i] = KNEE_STAND;
            }
        }
        break;
     case 'l':
        if (ServoPos[9] == KNEE_STAND) {
          KneeTarget[0]=KneeTarget[2] = KNEE_HALF_CROUCH;
          KneeTarget[1]=KNEE_CROUCH;
          KneeTarget[3]=KneeTarget[4]=KneeTarget[5]=KNEE_STAND;
        } else {
          for (int i = 0; i < NUM_LEGS; i++) {
            KneeTarget[i] = KNEE_STAND;
          }
        }
        break;
      case 'r':
        if (ServoPos[6] == KNEE_STAND) {
          KneeTarget[0]=KneeTarget[1]=KneeTarget[2] = KNEE_STAND;
          KneeTarget[3]=KneeTarget[5]=KNEE_HALF_CROUCH;
          KneeTarget[4]=KNEE_CROUCH;
        } else {
          for (int i = 0; i < NUM_LEGS; i++) {
            KneeTarget[i] = KNEE_STAND;
          }
        }
        break;

    }
  }

  if (mode == SUBMODE_4 || mode == SUBMODE_3) { // incremental moves

    // move servos toward their targets
#define MOVEINCREMENT 10  // degrees per transmission time delay

    for (int i = 0; i < NUM_LEGS; i++) {
      int h, k;
      h = ServoPos[i];
      k = ServoPos[i+KNEE_OFFSET];
      int diff = KneeTarget[i] - k;
      
      if (diff <= -MOVEINCREMENT) {
        // the knee has a greater value than the target
        k -= MOVEINCREMENT;
      } else if (diff >= MOVEINCREMENT) {
        // the knee has a smaller value than the target
        k += MOVEINCREMENT;
      } else {
        // the knee is within MOVEINCREMENT of the target so just go to target
        k = KneeTarget[i];
      }

      setKnee(i, k);

      diff = HipTarget[i] - h;

      if (diff <= -MOVEINCREMENT) {
        // the hip has a greater value than the target
        h -= MOVEINCREMENT;
      } else if (diff >= MOVEINCREMENT) {
        // the hip has a smaller value than the target
        h += MOVEINCREMENT;
      } else {
        // the knee is within MOVEINCREMENT of the target so just go to target
        h = HipTarget[i];
      }
        
      setHipRaw(i, h);
      //Serial.print("RAW "); Serial.print(i); Serial.print(" "); Serial.println(h);

    }
    return;  // /this mode does not execute the rest of the actions
  }

  // If we get here, we are in either submode A or B
  //
  // submode A: fight with two front legs, individual movement
  // submode B: fight with two front legs, in unison
  
  setLeg(MIDDLE_LEGS, HIP_FORWARD+10, KNEE_STAND, 0);
  setLeg(BACK_LEGS, HIP_BACKWARD, KNEE_STAND, 0);
  
  switch (dpad) {
    case 's':  // stop mode: both legs straight out forward
      setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_NEUTRAL, 0);

      break;
      
    case 'f':  // both front legs move up in unison
      setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_UP, 0);
      break;
    
    case 'b':  // both front legs move down in unison
      setLeg(FRONT_LEGS, HIP_FORWARD, KNEE_STAND, 0);
      break;
    
    case 'l':  // left front leg moves left, right stays forward
      if (mode == SUBMODE_1) {
        setLeg(LEG0, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_RELAX, 0);
      } else {
        // both legs move in unison in submode B
        setLeg(LEG0, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG5, HIP_FISTS_FORWARD+30, KNEE_RELAX, 0);
      }
      break;
    
    case 'r':  // right front leg moves right, left stays forward
      if (mode == SUBMODE_1) {
        setLeg(LEG5, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_RELAX, 0);
      } else { // submode B
        setLeg(LEG5, HIP_NEUTRAL, KNEE_UP, 0);
        setLeg(LEG0, HIP_FISTS_FORWARD+30, KNEE_RELAX, 0);
      }
      break;
    
    case 'w':  // automatic ninja motion mode with both legs swinging left/right/up/down furiously!

#define NUM_PUGIL_PHASES 8
        {  // we need a new scope for this because there are local variables
        
        long t = millis()%timeperiod;
        long phase = (NUM_PUGIL_PHASES*t)/timeperiod;
      
        //Serial.print("PHASE: ");
        //Serial.println(phase);
    
        switch (phase) {
          case 0:
            // Knees down, hips forward
            setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_RELAX, 0);
            break;
      
          case 1:
            // Knees up, hips forward
            setLeg(FRONT_LEGS, HIP_FISTS_FORWARD, KNEE_UP, 0);
            break;
      
          case 2:
            // Knees neutral, hips neutral
            setLeg(FRONT_LEGS, HIP_BACKWARD, KNEE_NEUTRAL, 0);
            break;
      
          case 3:
            // Knees up, hips neutral
            setLeg(FRONT_LEGS, HIP_BACKWARD, KNEE_UP, 0);
            break;
      
          case 4:
             // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_UP, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_STAND, 0);
             break;
      
          case 5:
              // kick phase 2
              // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, (mode==SUBMODE_2)?KNEE_DOWN:KNEE_STAND, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_UP, 0);
             break;
      
          case 6:
             // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_UP, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_DOWN, 0);
             break;
      
          case 7:
              // kick phase 2
              // hips forward, kick
             setLeg(LEG0, HIP_FISTS_FORWARD, KNEE_DOWN, 0);
             setLeg(LEG5, HIP_FISTS_FORWARD, KNEE_UP, 0);
             break;
        }
      }
  }

}

void gait_tripod(int reverse, int hipforward, int hipbackward, 
          int kneeup, int kneedown, long timeperiod) {

    // this version makes leanangle zero
    gait_tripod(reverse, hipforward, hipbackward, 
          kneeup, kneedown, timeperiod, 0);      
}

void gait_tripod(int reverse, int hipforward, int hipbackward, 
          int kneeup, int kneedown, long timeperiod, int leanangle) {

  // the gait consists of 6 phases. This code determines what phase
  // we are currently in by using the millis clock modulo the 
  // desired time period that all six  phases should consume.
  // Right now each phase is an equal amount of time but this may not be optimal

  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_TRIPOD_PHASES 6
#define FBSHIFT    15   // shift front legs back, back legs forward, this much
  
  long t = millis()%timeperiod;
  long phase = (NUM_TRIPOD_PHASES*t)/timeperiod;

  //Serial.print("PHASE: ");
  //Serial.println(phase);

  transactServos(); // defer leg motions until after checking for crashes
  switch (phase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at
      // the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, kneeup, 0, 0, leanangle);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move forward
      // at the hips, while the rest of the legs move backward at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT);
      break;

    case 2: 
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, kneedown, 0, 0, leanangle);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, kneeup, 0, 0, leanangle);
      break;
      
    case 4:
      // similar to phase 1, move raised legs forward and lowered legs backward
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, kneedown, 0, 0, leanangle);
      break;  
  }
  commitServos(); // implement all leg motions
}



void gait_tripod_scamper(int reverse, int turn) {

  ScamperTracker += 2;  // for tracking if the user is over-doing it with scamper

  // this is a tripod gait that tries to go as fast as possible by not waiting
  // for knee motions to complete before beginning the next hip motion

  // this was experimentally determined and assumes the battery is maintaining
  // +5v to the servos and they are MG90S or equivalent speed. There is very
  // little room left for slower servo motion. If the battery voltage drops below
  // 6.5V then the BEC may not be able to maintain 5.0V to the servos and they may
  // not complete motions fast enough for this to work.

  int hipforward, hipbackward;
  
  if (reverse) {
    hipforward = HIP_BACKWARD;
    hipbackward = HIP_FORWARD;
  } else {
    hipforward = HIP_FORWARD;
    hipbackward = HIP_BACKWARD;
  }

#define FBSHIFT    15   // shift front legs back, back legs forward, this much
#define SCAMPERPHASES 6

#define KNEEDELAY 35   //30
#define HIPDELAY 100   //90

  if (millis() >= NextScamperPhaseTime) {
    ScamperPhase++;
    if (ScamperPhase >= SCAMPERPHASES) {
      ScamperPhase = 0;
    }
    switch (ScamperPhase) {
      case 0: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 1: NextScamperPhaseTime = millis()+HIPDELAY; break;
      case 2: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 3: NextScamperPhaseTime = millis()+KNEEDELAY; break;
      case 4: NextScamperPhaseTime = millis()+HIPDELAY; break;
      case 5: NextScamperPhaseTime = millis()+KNEEDELAY; break;
    }

  }

  //Serial.print("ScamperPhase: "); Serial.println(ScamperPhase);

  transactServos();
  switch (ScamperPhase) {
    case 0:
      // in this phase, center-left and noncenter-right legs raise up at
      // the knee
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_SCAMPER, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;

    case 1:
      // in this phase, the center-left and noncenter-right legs move forward
      // at the hips, while the rest of the legs move backward at the hip
      setLeg(TRIPOD1_LEGS, hipforward, NOMOVE, FBSHIFT, turn);
      setLeg(TRIPOD2_LEGS, hipbackward, NOMOVE, FBSHIFT, turn);
      break;

    case 2: 
      // now put the first set of legs back down on the ground
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;

    case 3:
      // lift up the other set of legs at the knee
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_SCAMPER, 0, turn);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0, turn);
      break;
      
    case 4:
      // similar to phase 1, move raised legs forward and lowered legs backward
      setLeg(TRIPOD1_LEGS, hipbackward, NOMOVE, FBSHIFT, turn);
      setLeg(TRIPOD2_LEGS, hipforward, NOMOVE, FBSHIFT, turn);
      break;

    case 5:
      // put the second set of legs down, and the cycle repeats
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_DOWN, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_DOWN, 0);
      break;  
  }
  commitServos();
}

// call gait_ripple with leanangle = 0
void gait_ripple(int reverse, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod) {
  gait_ripple(reverse, hipforward, hipbackward, kneeup, kneedown, timeperiod, 0);
}

void gait_ripple(int reverse, int hipforward, int hipbackward, int kneeup, int kneedown, long timeperiod, int leanangle) {
  // the gait consists of 10 phases. This code determines what phase
  // we are currently in by using the millis clock modulo the 
  // desired time period that all phases should consume.
  // Right now each phase is an equal amount of time but this may not be optimal

  if (reverse) {
    int tmp = hipforward;
    hipforward = hipbackward;
    hipbackward = tmp;
  }

#define NUM_RIPPLE_PHASES 19
  
  long t = millis()%timeperiod;
  long phase = (NUM_RIPPLE_PHASES*t)/timeperiod;

  //Serial.print("PHASE: ");
  //Serial.println(phase);

  transactServos();
  
  if (phase == 18) {
    setLeg(ALL_LEGS, hipbackward, NOMOVE, FBSHIFT);
  } else {
    int leg = phase/3;  // this will be a number between 0 and 2
    leg = 1<<leg;
    int subphase = phase%3;

    switch (subphase) {
      case 0:
        setLeg(leg, NOMOVE, kneeup, 0);
        break;
      case 1:
        setLeg(leg, hipforward, NOMOVE, FBSHIFT);
        break;
      case 2:
        setLeg(leg, NOMOVE, kneedown, 0);
        break;     
    }
  }
  commitServos();
}


void random_gait(int timingfactor) {

#define GATETIME 3500  // number of milliseconds for each demo

  if (millis() > nextGaitTime) {
    curGait++;
    if (curGait >= G_NUMGATES) {
      curGait = 0;
    }
    nextGaitTime = millis() + GATETIME;

    // when switching demo modes, briefly go into a standing position so 
    // we're starting at the same position every time.
    setLeg(ALL_LEGS, HIP_NEUTRAL, KNEE_STAND, 0);
    delay(600);
  }

  switch (curGait) {
    case G_STAND:
      stand(); 
      break;
    case G_TURN:
      turn(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, TRIPOD_CYCLE_TIME); // 700
      break;
    case G_TRIPOD:
      gait_tripod(1, HIP_FORWARD, HIP_BACKWARD, KNEE_NEUTRAL, KNEE_DOWN, TRIPOD_CYCLE_TIME); // 900
      break;
    case G_SCAMPER:
      gait_tripod_scamper((nextGaitTime-(millis())<GATETIME/2),0);  // reverse direction halfway through
      break;
    case G_DANCE:
      stand();
      for (int i = 0; i < NUM_LEGS; i++) 
        setHipRaw(i, 145);
      delay(350);
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 35);
      delay(350);
      break;
    case G_BOOGIE:
       boogie_woogie(NO_LEGS, SUBMODE_1, 2);
       break;
    case G_FIGHT:
      fight_mode('w', SUBMODE_1, FIGHT_CYCLE_TIME);
      break;

    case G_TEETER:
      wave('r');
      break;

    case G_BALLET:
      flutter();
      break;
      
  }
  
}

void foldup() {
  setLeg(ALL_LEGS, NOMOVE, KNEE_FOLD, 0);
  for (int i = 0; i < NUM_LEGS; i++) 
        setHipRaw(i, HIP_FOLD);
}

void dance_dab(int timingfactor) {
#define NUM_DAB_PHASES 3
  
  long t = millis()%(1100*timingfactor);
  long phase = (NUM_DAB_PHASES*t)/(1100*timingfactor);

  switch (phase) {
    case 0: 
      stand(); break;

    case 1: 
      setKnee(6, KNEE_UP); break;

    case 2: 
      for (int i = 0; i < NUM_LEGS; i++)
         if (i != 0) setHipRaw(i, 40);
      setHipRaw(0, 140);
      break;
  }
}

void flutter() {   // ballet flutter legs on pointe
#define NUM_FLUTTER_PHASES 4
#define FLUTTER_TIME 200
#define KNEE_FLUTTER (KNEE_TIPTOES+20)

  long t = millis()%(FLUTTER_TIME);
  long phase = (NUM_FLUTTER_PHASES*t)/(FLUTTER_TIME);

  setLeg(ALL_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
  
  switch (phase) {
    case 0:
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_FLUTTER, 0, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 1:
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 2:
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_FLUTTER, 0, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
    case 3:
      setLeg(TRIPOD2_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      setLeg(TRIPOD1_LEGS, NOMOVE, KNEE_TIPTOES, 0, 0);
      break;
  }

}


void dance_ballet(int dpad) {   // ballet flutter legs on pointe

#define BALLET_TIME 250

  switch (dpad) {

    default:
    case 's': tiptoes(); return;

    case 'w': flutter(); return;

    case 'l':
      turn(1, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;

    case 'r':
      turn(0, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME); 
      break;

    case 'f':
      gait_tripod(0, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;

    case 'b':
      gait_tripod(1, HIP_FORWARD_SMALL, HIP_BACKWARD_SMALL, KNEE_FLUTTER, KNEE_TIPTOES, BALLET_TIME);
      break;
  }
}


void dance_hands(int dpad) {

  setLeg(FRONT_LEGS, HIP_NEUTRAL, KNEE_STAND, 0, 0); 
  setLeg(BACK_LEGS, HIP_NEUTRAL, KNEE_STAND, 0, 0); 

  switch (dpad) {
    case 's':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, KNEE_UP, 0, 0);
      break;
    case 'f':
      setLeg(MIDDLE_LEGS, HIP_FORWARD_MAX, KNEE_UP_MAX, 0, 0);
      break;
    case 'b':
      setLeg(MIDDLE_LEGS, HIP_BACKWARD_MAX, KNEE_UP_MAX, 0, 0);
      break;
    case 'l':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
      setLeg(LEG1, NOMOVE, KNEE_NEUTRAL, 0, 0);
      setLeg(LEG4, NOMOVE, KNEE_UP_MAX, 0, 0);
      break;
    case 'r':
      setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
      setLeg(LEG1, NOMOVE, KNEE_UP_MAX, 0, 0);
      setLeg(LEG4, NOMOVE, KNEE_NEUTRAL, 0, 0);
      break;
    case 'w':
      // AUTOMATIC MODE
#define NUM_HANDS_PHASES 2
#define HANDS_TIME_PERIOD 400
        {  // we need a new scope for this because there are local variables
        
        long t = millis()%HANDS_TIME_PERIOD;
        long phase = (NUM_HANDS_PHASES*t)/HANDS_TIME_PERIOD;
     
    
        switch (phase) {
          case 0:
            setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
            setLeg(LEG1, NOMOVE, KNEE_NEUTRAL, 0, 0);
            setLeg(LEG4, NOMOVE, KNEE_UP_MAX, 0, 0);
            break;
      
          case 1:
            setLeg(MIDDLE_LEGS, HIP_NEUTRAL, NOMOVE, 0, 0);
            setLeg(LEG1, NOMOVE, KNEE_UP_MAX, 0, 0);
            setLeg(LEG4, NOMOVE, KNEE_NEUTRAL, 0, 0);
            break;

        }
      }
      break;
  }
}

void dance(int legs_up, int submode, int timingfactor) {
   setLeg(legs_up, NOMOVE, KNEE_UP, 0, 0);
   setLeg((legs_up^0b111111), NOMOVE, ((submode==SUBMODE_1)?KNEE_STAND:KNEE_TIPTOES), 0, 0);

#define NUM_DANCE_PHASES 2
  
  long t = millis()%(600*timingfactor);
  long phase = (NUM_DANCE_PHASES*t)/(600*timingfactor);

  switch (phase) {
    case 0:
      for (int i = 0; i < NUM_LEGS; i++) 
        setHipRaw(i, 140);
      break;
    case 1:
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 40);
      break;
  }
}

void boogie_woogie(int legs_flat, int submode, int timingfactor) {
  
      setLeg(ALL_LEGS, NOMOVE, KNEE_UP, 0);
      //setLeg(legs_flat, NOMOVE, KNEE_RELAX, 0, 0);

#define NUM_BOOGIE_PHASES 2
  
  long t = millis()%(400*timingfactor);
  long phase = (NUM_BOOGIE_PHASES*t)/(400*timingfactor);

  switch (phase) {
    case 0:
      for (int i = 0; i < NUM_LEGS; i++) 
        setHipRaw(i, 140);
      break;
      
    case 1: 
      for (int i = 0; i < NUM_LEGS; i++)
        setHipRaw(i, 40);
      break;
  }
}



void checkForSmoothMoves() {
  // This is kind of a hack right now for making the grip arm move smoothly. Really there should be a
  // general mechanism that would apply to all servos for things like lean and twist mode as well as
  // servos added by the user for use by Scratch. We'll kind of use this hack as a prototype for a
  // more general mechanism to be implemented in a major revision later

  if (abs(ServoPos[GRIPARM_ELBOW_SERVO] - GripArmElbowDestination) <= 1) { 
    // uncomment the following line to debug grip elbow movement
    //Serial.print("GA close pos="); Serial.print(ServoPos[GRIPARM_ELBOW_SERVO]); Serial.print(" dest="); Serial.println(GripArmElbowDestination);
    return; // we're close enough to the intended destination already
  }
  
  #define SMOOTHINCREMENTTIME 20    // number of milliseconds between interpolated moves
  static long LastSmoothMoveTime = 0;

  long now = millis();
  if (now >= LastSmoothMoveTime + SMOOTHINCREMENTTIME) {
    LastSmoothMoveTime = now;
    //Serial.print("Set GAE="); Serial.println(ServoPos[GRIPARM_ELBOW_SERVO]+GripArmElbowIncrement);
    deferServoSet = 0;
    setServo(GRIPARM_ELBOW_SERVO, ServoPos[GRIPARM_ELBOW_SERVO]+GripArmElbowIncrement);
    
  } else {
    //Serial.println("not time");
  }
}