#include "Joint.h"
#include <math.h>

constexpr float DegToRad = M_PI / 180;
constexpr float RadToDeg = 180 / M_PI;

Joint::Joint(const JointConfig &config) {
  v = new Valve(config.pin_valve_e, config.pin_valve_r);
  pin_potmeter = config.pin_potmeter;

  /** The base attachment in parent segment space */
  pistonBaseDistance = config.pistonBaseDistance;
  pistonBaseAngleInParentSpace = config.pistonBaseAngleInParentSpace;

  /** The end attachment in child segment space (rotates with the joint) */
  pistonEndDistance = config.pistonEndDistance;
  pistonEndAngle = config.pistonEndAngle;

  pinMode(pin_potmeter, INPUT);
  targetAngleDeg = (angle_min_deg + angle_max_deg) * 0.5f; // Start mid position
}

float Joint::calculatePistonLength(const float jointAngle) const {
  // Calculate the angle in the triangle at the joint pivot
  // pistonBaseAngleInParentSpace: angle to base attachment (in parent's space, doesn't change with joint rotation)
  // pistonEndAngle: angle to end attachment (in joint's local space)
  // jointAngle: current rotation of the joint (in parent's space)

  // The end attachment's angle in parent space is: pistonEndAngle + jointAngle
  // The triangle angle at the joint is the difference between the two attachment angles
  const float triangleAngle = (pistonEndAngle + jointAngle) - pistonBaseAngleInParentSpace;
  const float triangleAngleRad = triangleAngle * DegToRad;

  // Use law of cosines: c² = a² + b² - 2ab*cos(C)
  // where c is the piston length, a and b are the attachment distances
  //Debug.Log(_pistonBaseDistance +" "+ _pistonEndDistance);

  const float length = sqrt(
      pistonBaseDistance * pistonBaseDistance +
      pistonEndDistance * pistonEndDistance -
      2 * pistonBaseDistance * pistonEndDistance * cos(triangleAngleRad)
  );

  return constrain(length, minPistonLength, maxPistonLength);
}

float Joint::mapAdcToDeg(const int adc) const {
  const float deg = m_deg_per_adc * adc + b_deg_offset;
  return constrain(deg, angle_min_deg, angle_max_deg);
}

// Equivalent to Update() in Unity
// Reads the potentiometer angle, compares it to the target angle,
// and adjusts valve outputs using proportional control with a 0.5° deadband
void Joint::update() {
  const long deltaTime = millis() - lastUpdate;

  // --- Step 1: Read current angle from potentiometer ---
  const int adc = analogRead(pin_potmeter);
  currentAngleDeg = mapAdcToDeg(adc);
  //currentAngleDeg = -110;
#ifdef VERBOSE
  Serial.print("currentAngle:");
  Serial.print(currentAngleDeg);
  Serial.print(" targetAngle:");
  Serial.println(targetAngleDeg);
#endif

  // --- Step 2: Calculate target piston length for desired angle ---
  const float targetLength = calculatePistonLength(targetAngleDeg);
  const float currentPistonLength = calculatePistonLength(currentAngleDeg);

  // PID control to get desired piston velocity
  float error = targetLength - currentPistonLength;

  // Deadband: reset integral when very close to target (prevents lingering)
  constexpr float deadband = 0.001f; // 1mm tolerance
  if (abs(error) < deadband) {
      integralError = 0;
      error = 0; // Stop control signal completely within deadband
  }

  const float derivative = (error - previousError) / deltaTime;

  float pidOutput = kP * error + kI * integralError + kD * derivative;

  // Anti-windup: integrate if not saturated, OR if integrating would reduce saturation
  const bool saturatedHigh = pidOutput > 1.0;
  const bool saturatedLow = pidOutput < -1.0;
  const bool shouldIntegrate = (!saturatedHigh && !saturatedLow) ||
                         (saturatedHigh && error < 0) ||
                         (saturatedLow && error > 0);

  if (shouldIntegrate && abs(error) >= deadband) {
      integralError += error * deltaTime;
  }

  // Clamp output to valve range
  pidOutput = constrain(pidOutput, -1.0, 1.0);

  lastPID = pidOutput;

#ifdef VERBOSE
  Serial.print("pid:");
  Serial.println(pidOutput);
  Serial.print("integral:");
  Serial.println(integralError);
  Serial.print("currentLength:");
  Serial.println(currentPistonLength, 6);
  Serial.print("targetLength: ");
  Serial.println(targetLength, 6);
#endif
  previousError = error;

  lastUpdate = millis();

  // --- Step 3: Send control signal to valve ---
  v->UpdatePWM(pidOutput);
  v->update();
}

// Resets the joint's target angle to its mid position
void Joint::resetToInit() {
  setTargetAngle((angle_min_deg + angle_max_deg) * 0.5f);
}

// Accessors for current state and control values
void Joint::setTargetAngle(const float deg) {
  targetAngleDeg = constrain(deg, angle_min_deg, angle_max_deg);
}

float Joint::getTargetAngleDeg() const {
  return targetAngleDeg;
}

float Joint::getCurrentAngleDeg() const {
  return currentAngleDeg;
}

uint8_t Joint::getExtendDuty() const {
  return v->getLastExtendDuty();
}

uint8_t Joint::getRetractDuty() const {
  return v->getLastRetractDuty();
}

float Joint::getLastPID() const {
  return lastPID;
}

bool Joint::isAtTarget(const float degreeTolerance) const {
  const float angleDiff = abs(targetAngleDeg - currentAngleDeg);
  return angleDiff <= degreeTolerance;
}