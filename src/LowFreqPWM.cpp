#include "LowFreqPWM.h"

LowFreqPWM::LowFreqPWM(uint8_t _pin, float frequency, float maxActionRate) {
  pin = _pin;
  period = 1000.0 / frequency;  // PWM period in ms
  minStateTime = 1000.0 / maxActionRate;  // Min state duration in ms
  dutyCycle = 0;
  state = false;
  lastToggle = 0;
  pinMode(pin, OUTPUT);
}

void LowFreqPWM::setDutyCycle(uint8_t dc) {
  // Clamp to valid ranges
  // Note! These could be adjusted to include some small values
  // in the sims it was 0.002. This is likely needed because the PID will
  // will never not move. Converge but never match.
  if (dc == 0 || dc == 100) {
    dutyCycle = dc;  // Full on/off is allowed
  } else {
    // Calculate min/max valid duty cycles
    uint8_t minDC = (minStateTime * 100) / period;
    uint8_t maxDC = 100 - minDC;

    // Clamp to valid range
    dutyCycle = constrain(dc, minDC, maxDC);

    #ifdef VERBOSE
    // notify if clamped, and verbose
    if (false && dutyCycle != dc) {
      Serial.print(" c ");
      Serial.print(dc);
      Serial.print(" => ");
      Serial.print(dutyCycle);
    }
    #endif
  }
}

void LowFreqPWM::update() {
  unsigned long now = millis();
  unsigned long elapsed = (now - lastToggle) % period;
  unsigned long onTime = (period * dutyCycle) / 100;

  #ifdef VERBOSE
  Serial.print(elapsed);
  Serial.print(" ");
  Serial.print(onTime);
  Serial.print(" ");
  Serial.println(dutyCycle);
  #endif

  bool shouldBeOn = (elapsed < onTime) && (dutyCycle > 0);

  if (shouldBeOn != state && ((now - lastToggle) >= minStateTime)) {
    state = shouldBeOn;
    lastToggle = millis();
    digitalWrite(pin, state);
  }
}
