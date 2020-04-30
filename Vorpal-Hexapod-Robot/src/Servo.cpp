#include "Servo.h"

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(SERVO_IIC_ADDR); 
int FreqMult = 1; 
byte deferServoSet = 0;
short ServoPos[2*NUM_LEGS+MAX_GRIPSERVOS]; // the last commanded position of each servo
long ServoTime[2*NUM_LEGS+MAX_GRIPSERVOS]; // the time that each servo was last commanded to a new position
byte ServoTrim[2*NUM_LEGS+MAX_GRIPSERVOS];  // trim values for fine adjustments to servo horn positions

byte TrimInEffect = 1;
byte TrimCurLeg = 0;
byte TrimPose = 0;

int ServosDetached = 0;

unsigned long freqWatchDog = 0;
unsigned long SuppressScamperUntil = 0; 

void TrimInit()
{
    // read in trim values from eeprom if available
  if (EEPROM.read(0) == 'V') {
    // if the first byte in the eeprom is a capital letter V that means there are trim values
    // available. Note that eeprom from the factory is set to all 255 values.
    Serial.print("TRIMS: ");
    for (int i = 0; i < NUM_LEGS*2; i++) {
      ServoTrim[i] = EEPROM.read(i+1);
      Serial.print(ServoTrim[i]-TRIM_ZERO); Serial.print(" ");
    }
    Serial.println("");
  } else {
    Serial.println("TRIMS:unset");
    // init trim values to zero, no trim
    for (int i = 0; i < NUM_LEGS*2; i++) {
      ServoTrim[i] = TRIM_ZERO;   // this is the middle of the trim range and will result in no trim
    }
  }
}

void resetServoDriver() {
  servoDriver.begin(); 
  servoDriver.setPWMFreq(PWMFREQUENCY);  // Analog servos run at ~60 Hz updates
}


void attach_all_servos() {
  Serial.print("ATTACH");
  for (int i = 0; i < 2*NUM_LEGS+NUM_GRIPSERVOS; i++) {
    setServo(i, ServoPos[i]);
    Serial.print(ServoPos[i]); Serial.print(":");
  }
  Serial.println("");
  ServosDetached = 0;
  return;
}
void detach_all_servos() {
  //Serial.println("DETACH");
  for (int i = 0; i < 16; i++) {
    servoDriver.setPin(i,0,false); // stop pulses which will quickly detach the servo
  }
  ServosDetached = 1;
}

// setServo is the lowest level function for setting servo positions.
// It handles trims too.
void setServo(int servonum, int position) {
  if (position != ServoPos[servonum]) {
    ServoTime[servonum] = millis();
  }
  ServoPos[servonum] = position;  // keep data on where the servo was last commanded to go
  
  int p = map(position,0,180,SERVOMIN,SERVOMAX);
  
  if (TrimInEffect && servonum < 12) {
    //Serial.print("Trim leg "); Serial.print(servonum); Serial.print(" "); Serial.println(ServoTrim[servonum] - TRIM_ZERO);
    p += ServoTrim[servonum] - TRIM_ZERO;   // adjust microseconds by trim value which is renormalized to the range -127 to 128    
  }

  if (!deferServoSet) {
    servoDriver.setPWM(servonum, 0, p);    
  }

                          
  // DEBUG: Uncomment the next line to debug setservo problems. It causes some lagginess due to all the printing
  //Serial.print("SS:");Serial.print(servonum);Serial.print(":");Serial.println(position);
}

void transactServos() {
  deferServoSet = 1;
}

void commitServos() {
  checkForCrashingHips();
  deferServoSet = 0;
  for (int servo = 0; servo < 2*NUM_LEGS+NUM_GRIPSERVOS; servo++) {
    setServo(servo, ServoPos[servo]);
  }
}

//
// Short power dips can cause the servo driver to put itself to sleep
// the checkForServoSleep() function uses IIC protocol to ask the servo
// driver if it's asleep. If it is, this function wakes it back up.
// You'll see the robot stutter step for about half a second and a chirp
// is output to indicate what happened.
// This happens more often on low battery conditions. When the battery gets low
// enough, however, this code will not be able to wake it up again.
// If your robot constantly resets even though the battery is fully charged, you
// may have too much friction on the leg hinges, or you may have a bad servo that's
// drawing more power than usual. A bad BEC can also cause the issue.
//
#include "Buzzer.h"
void checkForServoSleep() {

  if (millis() > freqWatchDog) {

    // See if the servo driver module went to sleep, probably due to a short power dip
    Wire.beginTransmission(SERVO_IIC_ADDR);
    Wire.write(0);  // address 0 is the MODE1 location of the servo driver, see documentation on the PCA9685 chip for more info
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)SERVO_IIC_ADDR, (uint8_t)1);
    int mode1 = Wire.read();
    if (mode1 & 16) { // the fifth bit up from the bottom is 1 if controller was asleep
      // wake it up!
      resetServoDriver();
      buzzer.Beep(1200,100);  // chirp to warn user of brown out on servo controller
      SuppressScamperUntil = millis() + 10000;  // no scamper for you! (for 10 seconds because we ran out of power, give the battery
                                                // a bit of time for charge migration and let the servos cool down)
    }
    freqWatchDog = millis() + 100;
  }
}




// checkForCrashingHips takes a look at the leg angles and tries to figure out if the commanded
// positions might cause servo stall.  Now the correct way to do this would be to do fairly extensive
// trig computations to see if the edges of the hips touch. However, Arduino isn't really set up to
// do complicated trig stuff. It would take a lot of code space and a lot of time. So we're just using
// a simple approximation. In practice it stops very hard stall situations. Very minor stalls (where the
// motor is commanded a few degress farther than it can physically go) may still occur, but those won't
// draw much power (the current draw is proportional to how far off the mark the servo is).
void checkForCrashingHips() {
  
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    if (ServoPos[leg] > 85) {
      continue; // it's not possible to crash into the next leg in line unless the angle is 85 or less
    }
    int nextleg = ((leg+1)%NUM_LEGS);
    if (ServoPos[nextleg] < 100) {
      continue;   // it's not possible for there to be a crash if the next leg is less than 100 degrees
                  // there is a slight assymmetry due to the way the servo shafts are positioned, that's why
                  // this number does not match the 85 number above
    }
    int diff = ServoPos[nextleg] - ServoPos[leg];
    // There's a fairly linear relationship
    if (diff <= 85) {
      // if the difference between the two leg positions is less than about 85 then there
      // is not going to be a crash (or maybe just a slight touch that won't really cause issues)
      continue;
    }
    // if we get here then the legs are touching, we will adjust them so that the difference is less than 85
    int adjust = (diff-85)/2 + 1;  // each leg will get adjusted half the amount needed to avoid the crash
    
    // to debug crash detection, make the following line #if 1, else make it #if 0
#if 1
    Serial.print("#CRASH:");
    Serial.print(leg);Serial.print("="); Serial.print(ServoPos[leg]);
    Serial.print("/");Serial.print(nextleg);Serial.print("="); Serial.print(ServoPos[nextleg]);
    Serial.print(" Diff="); Serial.print(diff); Serial.print(" ADJ="); Serial.println(adjust);
#endif

    setServo(leg, ServoPos[leg] + adjust);   
    setServo(nextleg, ServoPos[nextleg] - adjust);

  }
}

///////////////////////////////////////////////////////////////
// Trim functions
///////////////////////////////////////////////////////////////


void save_trims() {
  Serial.print("SAVE TRIMS:");
  for (int i = 0; i < NUM_LEGS*2; i++) {
    EEPROM.update(i+1, ServoTrim[i]);
    Serial.print(ServoTrim[i]); Serial.print(" ");
  }
  Serial.println("");
  EEPROM.update(0, 'V');

}
void erase_trims() {
  Serial.println("ERASE TRIMS");
  for (int i = 0; i < NUM_LEGS*2; i++) {
    ServoTrim[i] = TRIM_ZERO;
  }
}