#include "Buzzer.h"
#include <Arduino.h>

void beep(int f, int t) {
  if (f > 0 && t > 0) {
    tone(BeeperPin, f, t);
  } else {
    noTone(BeeperPin);
  }
}

void beep(int f) {  // if no second param is given we'll default to 250 milliseconds for the beep
  beep(f, 250);
}