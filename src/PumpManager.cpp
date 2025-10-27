#include "PumpManager.h"

// Pump configuration
constexpr uint8_t PUMP_PIN = 30;
constexpr bool RELAY_ACTIVE_LOW = true;

// Helper function for pump control
inline void pumpWrite(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(PUMP_PIN, on ? LOW : HIGH);
  else                  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

void PumpManager::begin() {
  pinMode(PUMP_PIN, OUTPUT);
  pumpWrite(false);
}

void PumpManager::update(bool demand) {
  unsigned long now = millis();
  if (demand) {
    lastDemandMs_ = now;
    if (!isOn_) { isOn_ = true; pumpWrite(true); }
  } else if (isOn_ && (now - lastDemandMs_ > holdMs_)) {
    isOn_ = false; pumpWrite(false);
  }
}

bool PumpManager::isOn() const {
  return isOn_;
}
