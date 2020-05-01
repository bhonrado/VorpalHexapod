#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H
#include <Arduino.h>

class Potentiometer {
 private:
  uint8_t sig_pin_;
  uint8_t pwr_pin_;
  uint8_t gnd_pin_;

 public:
  Potentiometer(uint8_t sig_pin, uint8_t pwr_pin, uint8_t gnd_pin);
  void Init();
  uint16_t Read();
};


#endif