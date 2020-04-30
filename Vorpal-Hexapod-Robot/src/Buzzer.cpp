#include "Buzzer.h"
#include "pinout.h"

Buzzer buzzer(BUZZER_PIN);

Buzzer::Buzzer(uint8_t pin)
    : pin(pin)
{ }

void Buzzer::Init()
{
  pinMode(pin, OUTPUT);
}

void Buzzer::Beep(int freq, int duration)
{
    if (freq > 0 && duration > 0) {
    tone(pin, freq, duration);
  } else {
    noTone(pin);
  }
}

void Buzzer::BeepError()
{
  Beep(BF_ERROR, BD_MED);
}