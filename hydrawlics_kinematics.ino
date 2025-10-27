//String TEST_LINE = String("");
//#define VERBOSE

// =====================================================================
//  HYDRAWLICS - JOINT & VALVE CONTROL SYSTEM
//  Hardware implementation matching the Unity simulation structure.
//  Includes LCD feedback, pump manager, self-test, and safety deadband.
// =====================================================================

#define SELFTEST_ON_START 1          // Run relay polarity test at startup (disable after confirmed)
#define RELAY_ACTIVE_LOW   true      // Set false if relay board is active-HIGH (depends on module type)
#define VERBOSE
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define LCD_ADDR 0x27 // LCD setup
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// --- Pump relay (state) ---
constexpr uint8_t PUMP_PIN = 30; // dedicated pump relay pin
inline void pumpWrite(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(PUMP_PIN, on ? LOW : HIGH);
  else                  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

// --- Custom degree symbol for LCD ---
const uint8_t DEG_CHAR = 0;
byte degreeGlyph[8] = {
  B00110,B01001,B01001,B00110,
  B00000,B00000,B00000,B00000
};

const float DegToRad = M_PI / 180;
const float RadToDeg = 180 / M_PI;

// Forward declarations
void updateValves();
void serialRead();
uint8_t calculateChecksum(String &line);

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
  LowFreqPWM(uint8_t _pin, float frequency, float maxActionRate) {
    pin = _pin;
    period = 1000.0 / frequency;  // PWM period in ms
    minStateTime = 1000.0 / maxActionRate;  // Min state duration in ms
    dutyCycle = 0;
    state = false;
    lastToggle = 0;
    pinMode(pin, OUTPUT);
  }

  void setDutyCycle(uint8_t dc) {
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

  void update() {
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
};

class Valve {
private:
  LowFreqPWM* v_e;
  LowFreqPWM* v_r;
  uint8_t lastDcE = 0;
  uint8_t lastDcR = 0;

public:
  Valve(uint8_t _pin_e, uint8_t _pin_r) {
    // Create PWM controllers for both directions
    v_e = new LowFreqPWM(_pin_e, 1.0, 5.0); // 1Hz PWM, 5Hz max action rate
    v_r = new LowFreqPWM(_pin_r, 1.0, 5.0);
  }

  // move ∈ [-1, 1]  (− = retract, + = extend, 0 = hold)
  void updatePWMs(float move) {
    Serial.println(move);
    float normalized = constrain(move, -1.0, 1.0);
    uint8_t dc = (uint8_t)(fabsf(normalized) * 100.0f);

    if (normalized < 0) {             // RETRACT
      v_e->setDutyCycle(0);
      v_r->setDutyCycle(dc);
      lastDcE = 0; lastDcR = dc;
    } else if (normalized > 0) {      // EXTEND
      v_e->setDutyCycle(dc);
      v_r->setDutyCycle(0);
      lastDcE = dc; lastDcR = 0;
    } else {                          // IDLE
      v_e->setDutyCycle(0);
      v_r->setDutyCycle(0);
      lastDcE = 0; lastDcR = 0;
    }
  }

  // Provides a wrapper for PWM updates and functions to read the latest duty cycles
  void UpdatePWM(float pidOutput) { updatePWMs(pidOutput); }

  void update() { v_e->update(); v_r->update(); }
  uint8_t getLastExtendDuty() const { return lastDcE; }
  uint8_t getLastRetractDuty() const { return lastDcR; }
};

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
  const float b_deg_offset  = 98.19999694f;
  const float angle_min_deg = 54.0f;
  const float angle_max_deg = 98.2f;

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

  float calculatePistonLength(float jointAngle) {
    // Calculate the angle in the triangle at the joint pivot
    // pistonBaseAngleInParentSpace: angle to base attachment (in parent's space, doesn't change with joint rotation)
    // pistonEndAngle: angle to end attachment (in joint's local space)
    // jointAngle: current rotation of the joint (in parent's space)

    // The end attachment's angle in parent space is: pistonEndAngle + jointAngle
    // The triangle angle at the joint is the difference between the two attachment angles
    float triangleAngle = (pistonEndAngle + jointAngle) - pistonBaseAngleInParentSpace;
    float triangleAngleRad = triangleAngle * DegToRad;

    // Use law of cosines: c² = a² + b² - 2ab*cos(C)
    // where c is the piston length, a and b are the attachment distances
    //Debug.Log(_pistonBaseDistance +" "+ _pistonEndDistance);
    
    float length = sqrt(
        pistonBaseDistance * pistonBaseDistance +
        pistonEndDistance * pistonEndDistance -
        2 * pistonBaseDistance * pistonEndDistance * cos(triangleAngleRad)
    );

    return constrain(length, minPistonLength, maxPistonLength);
  }

  float mapAdcToDeg(int adc) const {
    float deg = m_deg_per_adc * adc + b_deg_offset;
    return constrain(deg, angle_min_deg, angle_max_deg);
  }

public:
  Joint(JointConfig config) {
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

// Equivalent to Update() in Unity
// Reads the potentiometer angle, compares it to the target angle,
// and adjusts valve outputs using proportional control with a 0.5° deadband
void update() {
    long deltaTime = millis() - lastUpdate;

    // --- Step 1: Read current angle from potentiometer ---
    int adc = analogRead(pin_potmeter);
    currentAngleDeg = mapAdcToDeg(adc);
    Serial.print("currentAngle:");
    Serial.println(currentAngleDeg);

    // --- Step 2: Calculate target piston length for desired angle ---
    float targetLength = calculatePistonLength(targetAngleDeg);
    float currentPistonLength = calculatePistonLength(currentAngleDeg);
    
    // PID control to get desired piston velocity
    float error = targetLength - currentPistonLength;
    integralError += error * deltaTime;
    float derivative = (error - previousError) / deltaTime;
    
    float pidOutput = kP * error + kI * integralError + kD * derivative;
    Serial.print("pid:");
    Serial.println(pidOutput);
    Serial.print("target:");
    Serial.println(targetLength);
    Serial.print("Length:");
    Serial.println(currentPistonLength);
    previousError = error;

    lastUpdate = millis();

    // --- Step 3: Send control signal to valve ---
    v->updatePWMs(pidOutput);
    v->update();
}

// Resets the joint's target angle to its mid position
void resetToInit() {
    setTargetAngle((angle_min_deg + angle_max_deg) * 0.5f);
}

// Accessors for current state and control values
void setTargetAngle(float deg) { targetAngleDeg = constrain(deg, angle_min_deg, angle_max_deg); }
float getTargetAngleDeg()  const { return targetAngleDeg; }
float getCurrentAngleDeg() const { return currentAngleDeg; }
uint8_t getExtendDuty()    const { return v->getLastExtendDuty(); }
uint8_t getRetractDuty()   const { return v->getLastRetractDuty(); }
};

class PumpManager {
  //  CLASS: PumpManager
  //  Controls the hydraulic pump. The pump is ON only while a valve is active.
  //  Holds for a short delay after movement to stabilize pressure.
  bool isOn_ = false;
  unsigned long lastDemandMs_ = 0;
  const unsigned long holdMs_ = 400; // Hold time after motion stops (ms)

public:
  void begin() {
    pinMode(PUMP_PIN, OUTPUT);
    pumpWrite(false);
  }
  void update(bool demand) {
    unsigned long now = millis();
    if (demand) {
      lastDemandMs_ = now;
      if (!isOn_) { isOn_ = true; pumpWrite(true); }
    } else if (isOn_ && (now - lastDemandMs_ > holdMs_)) {
      isOn_ = false; pumpWrite(false);
    }
  }
  bool isOn() const { return isOn_; }
};
PumpManager pumpMgr;

//  Instances
//  Only one joint is active in this prototype.
Joint j1({
  22, 23, A3,
  0.050, -90,   // base distance and angle
  0.151, 90   // end distance and angle
});
Joint joints[] = { j1 };


//  LCD for visuals
void lcdClearLine(uint8_t row) { lcd.setCursor(0,row); for (int i=0;i<16;i++) lcd.print(' '); lcd.setCursor(0,row); }
void printFloatOrDash(float v, uint8_t d){ if (isnan(v)||isinf(v)) lcd.print("--"); else lcd.print(v,d); }

//  Self-Test
//  Verifies relay polarity and pin wiring at startup.
void selfTestOnce() {
#if SELFTEST_ON_START
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Hydrawlics");
  lcd.setCursor(0,1); lcd.print("Pump+Valves check");

  pinMode(22, OUTPUT); pinMode(23, OUTPUT);

  // EXTEND
  if (RELAY_ACTIVE_LOW){ digitalWrite(22, LOW);  digitalWrite(23, HIGH); }
  else                 { digitalWrite(22, HIGH); digitalWrite(23, LOW);  }
  delay(1000);

  // OFF
  if (RELAY_ACTIVE_LOW){ digitalWrite(22, HIGH); digitalWrite(23, HIGH); }
  else                 { digitalWrite(22, LOW);  digitalWrite(23, LOW);  }
  delay(600);

  // RETRACT
  if (RELAY_ACTIVE_LOW){ digitalWrite(22, HIGH); digitalWrite(23, LOW); }
  else                 { digitalWrite(22, LOW);  digitalWrite(23, HIGH); }
  delay(1000);

  // OFF
  if (RELAY_ACTIVE_LOW){ digitalWrite(22, HIGH); digitalWrite(23, HIGH); }
  else                 { digitalWrite(22, LOW);  digitalWrite(23, LOW);  }
  delay(600);

  lcd.clear();
#endif
}

//  Setup() - Equivalent to Start() in Unity
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(500);
  Serial.println("Connected");

  lcd.init(); lcd.backlight(); lcd.createChar(DEG_CHAR, degreeGlyph);
  lcd.clear(); lcd.setCursor(0,0); lcd.print("Hydrawlics valve test");
  lcd.setCursor(0,1); lcd.print("Angle ctrl ready");
  delay(700); lcd.clear();

  pumpMgr.begin();
  selfTestOnce();  // One-time hardware verification
}

//  Loop() - Equivalent to Update() in Unity
void loop() {
  serialRead();
  updateValves();

  // LCD feedback
  static unsigned long tLCD = 0;
  if (millis() - tLCD > 200) {
    tLCD = millis();
    float cur = joints[0].getCurrentAngleDeg();
    float tar = joints[0].getTargetAngleDeg();
    Serial.print("cur:");
    Serial.println(cur);
    lcdClearLine(0); lcd.print("Cur: "); printFloatOrDash(cur,1); lcd.write(byte(DEG_CHAR));
    lcdClearLine(1); lcd.print("Tar: "); printFloatOrDash(tar,1); lcd.write(byte(DEG_CHAR));
    lcd.setCursor(12,1); lcd.print("P:"); lcd.print(pumpMgr.isOn() ? '1' : '0');
  }
}

//  updateValves()
//  Called once per loop to update the joint logic and pump state.
void updateValves() {
  static int count = 0;
  static float lastupdate = 0;
  /*if (millis() - lastupdate > 2000) {
    count = (count + 10) % 110;
    lastupdate = millis();
    Serial.println(count);
  }
*/
  for (Joint& j : joints) {

    j.update();
  }
}

//  Serial Communication
//  Command format:
//     A,<deg>  → set target angle for J2
void serialRead() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();

  if (line.startsWith("A,")) {
    float deg = line.substring(2).toFloat();
    joints[0].setTargetAngle(deg);
    Serial.print("OK TargetAngle="); Serial.println(deg,1);
    return;
  }

  Serial.print("OK "); Serial.println(calculateChecksum(line));
}

//  calculateChecksum()
//  Returns simple XOR checksum for serial confirmation.
uint8_t calculateChecksum(String &line) {
  uint8_t checksum = 0; for (int i=0;i<line.length();i++) checksum ^= (uint8_t)line[i];
  return checksum;
}
