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
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "Joint.h"
#include "PumpManager.h"

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

// Forward declarations
void updateValves();
void serialRead();
uint8_t calculateChecksum(String &line);

PumpManager pumpMgr;

//  Instances
//  Only one joint is active in this prototype.
Joint j1({
  22, 23, A3,
  0.050, -90,   // base distance and angle
  0.151, 90   // end distance and angle
});
Joint* joints[] = { &j1 };


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
    float cur = joints[0]->getCurrentAngleDeg();
    float tar = joints[0]->getTargetAngleDeg();
    Serial.print("cur:");
    Serial.println(cur);
    lcdClearLine(0); lcd.print("C:"); printFloatOrDash(cur,1); lcd.write(byte(DEG_CHAR));
    lcd.setCursor(9,0); lcd.print("p:"); lcd.print(joints[0]->getLastPID());
    lcdClearLine(1); lcd.print("T:"); printFloatOrDash(tar,1); lcd.write(byte(DEG_CHAR));
    lcd.setCursor(9,1); lcd.print("P:"); lcd.print(pumpMgr.isOn() ? '1' : '0');
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
  bool demand = false;
  for (Joint* j : joints) {
    j->update();
    // check if this joint has any demand
    demand |= (j->getExtendDuty() > 0 || j->getRetractDuty() > 0);
  }
  // if any of the joints have any demand, close the pump cicuit
  pumpMgr.update(demand);
}

//  Serial Communication
//  Command format:
//     A,<deg>  → set target angle for J2
void serialRead() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();

  if (line.startsWith("A,")) {
    float deg = line.substring(2).toFloat();
    joints[0]->setTargetAngle(deg);
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
