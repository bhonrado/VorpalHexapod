#include "Potentiometer.h"
#include <Arduino.h>

Potentiometer::Potentiometer(uint8_t sig_pin, uint8_t pwr_pin, uint8_t gnd_pin)
    : sig_pin_(sig_pin), pwr_pin_(pwr_pin), gnd_pin_(gnd_pin) {}

void Potentiometer::Init() {
  pinMode(pwr_pin_, OUTPUT);
  pinMode(gnd_pin_, OUTPUT);
  pinMode(sig_pin_, INPUT);

  digitalWrite(pwr_pin_, HIGH);
  digitalWrite(gnd_pin_, LOW);
}

uint16_t
Potentiometer::Read() {  // arduino nano has 10 bit adc resolution, therefore it
                         // can only read values between 0-1023
  uint16_t p = analogRead(sig_pin_);
  //Serial.print("Analog0="); Serial.println(p);
  return p;
}
