#include "Led.h"


Led::Led(uint8_t pin)
    : pin(pin)
{
}

void Led::Init()
{
    pinMode(pin, OUTPUT);
}

void Led::On()
{
    digitalWrite(pin, HIGH);
}

void Led::Off()
{
    digitalWrite(pin, LOW);
}

void Led::Toggle()
{
    digitalWrite(pin, !digitalRead(pin));
}

// make a characteristic flashing pattern to indicate the robot code is loaded (as opposed to the gamepad)
// There will be a brief flash after hitting the RESET button, then a long flash followed by a short flash.
// The gamepaid is brief flash on reset, short flash, long flash.
void Led::Flash(FlashPattern pattern)
{
    switch (pattern)
    {
    case FlashPattern::Robot:
        {
            On();
            delay(LONG_FLASH_INTERVAL);
            Off();
            delay(SHORT_FLASH_INTERVAL);
            On();
            delay(SHORT_FLASH_INTERVAL);
            Off();
            break;
        }
     
    case FlashPattern::Gamepad:
        {
            On();
            delay(SHORT_FLASH_INTERVAL);
            Off();
            delay(SHORT_FLASH_INTERVAL);
            On();
            delay(LONG_FLASH_INTERVAL);
            Off();
            break;
        }
    default:
        break;
    }

}