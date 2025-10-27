#ifndef VALVE_H
#define VALVE_H

#include <Arduino.h>
#include "LowFreqPWM.h"

class Valve {
private:
  LowFreqPWM* v_e;
  LowFreqPWM* v_r;
  uint8_t lastDcE = 0;
  uint8_t lastDcR = 0;

  void updatePWMs(float move);

public:
  Valve(uint8_t _pin_e, uint8_t _pin_r);

  // Provides a wrapper for PWM updates and functions to read the latest duty cycles
  void UpdatePWM(float pidOutput);
  void update();
  uint8_t getLastExtendDuty() const;
  uint8_t getLastRetractDuty() const;
};

#endif // VALVE_H
