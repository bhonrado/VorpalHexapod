#ifndef DIAL_H_
#define DIAL_H_
#include "Potentiometer.h"

#define DIALMODE_NUM 7

enum class DialMode {
  INIT = -1,
  STAND,
  ADJUST,
  TEST,
  DEMO,
  RC_GRIPARM,
  RC /* able to receive instructions via bluetooth*/
};

#define DIALMODE_STAND_EDGE 50
#define DIALMODE_ADJUST_EDGE 150
#define DIALMODE_TEST_EDGE 300
#define DIALMODE_DEMO_EDGE 750
#define DIALMODE_RC_GRIPARM_EDGE 950
//#define DIALMODE_RC_EDGE

class Dial {
 private:
  Potentiometer& potmeter_;
  DialMode current_mode_;
  DialMode prior_mode_;
 /* String mode_string_[DIALMODE_NUM] = {
      "Init", "Stand", "Adjust", "Test", "Demo", "RC_GripArm", "RC"
  };*/

  /**
   * @brief Reads the potmeter value and translates to a specific robot mode
   * @note It does not update the current and prior modes.
   * @return DialMode
   */
  DialMode ReadMode();

 public:
  Dial(Potentiometer& potmeter);
  void Init();

  /**
   * @brief returns the current mode.
   * @note it does not check the potentiometer
   * @return returns the value of current_mode_
   */
  DialMode Mode();

  DialMode PriorMode();

  /**
   * @brief Reads the dial current mode and updates its current value.
   * @return return true if the mode has been updated or false if it has not
   * changed.
   */
  bool UpdateMode();

  //void PrintCurrentMode();
};

extern Dial dial;

#endif  // DIAL_H_
