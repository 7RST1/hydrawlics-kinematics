#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include "Valve.h"

struct JointConfig {
  uint8_t pin_valve_e;
  uint8_t pin_valve_r;
  uint8_t pin_potmeter;

  /** The base attachment in parent segment space */
  float pistonBaseDistance;
  float pistonBaseAngleInParentSpace;

  /** The end attachment in child segment space (rotates with the joint) */
  float pistonEndDistance;
  float pistonEndAngle;
};

// Joint describes the whole joint, including;
// - Valve - controlling its angle
// - Piston - the piston the valve controles
// - Potmeter - reads the angle of the joint as it is
class Joint {
private:
  Valve* v;
  uint8_t pin_potmeter;

  // Calibration (from real measurements)
  const float m_deg_per_adc = -0.04320625f;
  const float b_deg_offset  = -81.79999694f;
  const float angle_min_deg = -126.1f;
  const float angle_max_deg = -81.8f;

  /** The base attachment in parent segment space */
  float pistonBaseDistance;
  float pistonBaseAngleInParentSpace;

  /** The end attachment in child segment space (rotates with the joint) */
  float pistonEndDistance;
  float pistonEndAngle;

  float minPistonLength = 0.128f;
  float maxPistonLength = 0.1657f;

  float kP = 2.0f;
  float kI = 0.1f;
  float kD = 0.5f;

  // PID state
  float integralError = 0;
  float previousError = 0;

  float targetAngleDeg = 0.0f;
  float currentAngleDeg = NAN;

  long lastUpdate = 0;

  float calculatePistonLength(float jointAngle);
  float mapAdcToDeg(int adc) const;

public:
  Joint(JointConfig config);

  // Equivalent to Update() in Unity
  // Reads the potentiometer angle, compares it to the target angle,
  // and adjusts valve outputs using proportional control with a 0.5° deadband
  void update();

  // Resets the joint's target angle to its mid position
  void resetToInit();

  // Accessors for current state and control values
  void setTargetAngle(float deg);
  float getTargetAngleDeg() const;
  float getCurrentAngleDeg() const;
  uint8_t getExtendDuty() const;
  uint8_t getRetractDuty() const;
};

#endif // JOINT_H
