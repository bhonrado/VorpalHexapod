#include "Dial.h"
#include "pinout.h"

Potentiometer potmeter(POTMETER_SIG, POTMETER_PWR, POTMETER_GND);
Dial dial(potmeter);

Dial::Dial(Potentiometer& potmeter)
    : potmeter_(potmeter),
      current_mode_(DialMode::INIT),
      prior_mode_(DialMode::INIT) {}

void Dial::Init() { potmeter_.Init(); }

DialMode Dial::ReadMode() {
  DialMode mode;
  uint16_t pot_value = potmeter_.Read();

  if (pot_value < DIALMODE_STAND_EDGE) {
    mode = DialMode::STAND;
  } else if (pot_value < DIALMODE_ADJUST_EDGE) {
    mode = DialMode::ADJUST;
  } else if (pot_value < DIALMODE_TEST_EDGE) {
    mode = DialMode::TEST;
  } else if (pot_value < DIALMODE_DEMO_EDGE) {
    mode = DialMode::DEMO;
  } else if (pot_value < DIALMODE_RC_GRIPARM_EDGE) {
    mode = DialMode::RC_GRIPARM;
  } else {
    mode = DialMode::RC;
  }
  return mode;
}

bool Dial::UpdateMode() {
  bool updated = false;
  current_mode_ = ReadMode();

  if (current_mode_ != prior_mode_) {
    updated = true;
    prior_mode_ = current_mode_;
  }

  return updated;
}

DialMode Dial::Mode()
{
    return current_mode_;
}

DialMode Dial::PriorMode()
{
    return prior_mode_;
}

/*
void Dial::PrintCurrentMode()
{
    Serial.print("Current dialmode: ");
    Serial.println(mode_string_[(int)current_mode_+1]);
}
*/