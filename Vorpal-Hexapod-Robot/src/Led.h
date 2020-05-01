#ifndef LED_H
#define LED_H
#include <Arduino.h>

#define SHORT_FLASH_INTERVAL    150
#define LONG_FLASH_INTERVAL     300

enum class FlashPattern
{
    Robot,
    Gamepad
};

class Led
{
private:
    uint8_t pin;
public:
    Led(uint8_t pin);
    void Init();
    void Flash(FlashPattern pattern);
    void Flash(unsigned long t);
    void On();
    void Off();
    void Toggle();
};

#endif