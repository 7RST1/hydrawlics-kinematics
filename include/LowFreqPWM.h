#ifndef LOW_FREQ_PWM_H
#define LOW_FREQ_PWM_H

#include <Arduino.h>

// =====================================================================
//  CLASS: LowFreqPWM
//  Generates low frequency PWM signals (e.g. 1 Hz) for relay control.
//  This simulates proportional valve activation using on/off pulses.
// =====================================================================
class LowFreqPWM {
private:
  uint8_t pin;
  unsigned long period;
  unsigned long minStateTime;  // Minimum time in each state
  unsigned long lastToggle;
  uint8_t dutyCycle;
  bool state;

public:
  LowFreqPWM(uint8_t _pin, float frequency, float maxActionRate);
  void setDutyCycle(uint8_t dc);
  void update();
};

#endif // LOW_FREQ_PWM_H
