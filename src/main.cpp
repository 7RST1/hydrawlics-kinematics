//String TEST_LINE = String("");
//#define VERBOSE

// =====================================================================
//  HYDRAWLICS - JOINT & VALVE CONTROL SYSTEM
//  Hardware implementation matching the Unity simulation structure.
//  Includes LCD feedback, pump manager, self-test, and safety deadband.
// =====================================================================

#define SELFTEST_ON_START 1          // Run relay polarity test at startup (disable after confirmed)
#define RELAY_ACTIVE_LOW   true      // Set false if relay board is active-HIGH (depends on module type)
//#define VERBOSE

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "Joint.h"
#include "PumpManager.h"
#include "ArmController.h"
#include "GCodeCommandQueue.h"

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
void processCommandQueue();
uint8_t calculateChecksum(String &line);

PumpManager pumpMgr;
GCodeCommandQueue gcodeQueue;

//  Instances
//  Only one joint is active in this prototype.
Joint j0({
  22, 23, A3,
  0.050, -90,   // base distance and angle
  0.151, 90   // end distance and angle
});
Joint j1({
  10, 11, A4,
  0.050, -90,   // base distance and angle
  0.151, 90   // end distance and angle
});
Joint j2({
  12, 13, A5,
  0.050, -90,   // base distance and angle
  0.151, 90   // end distance and angle
});
Joint j3({
  14, 15, A6,
  0.050, -90,   // base distance and angle
  0.151, 90   // end distance and angle
});
Joint* joints[] = { &j0, &j1, &j2, &j3 };
ArmController armController(&j0, &j1, &j2, &j3);


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
  // processes incoming serial communication from the python script running on Raspberry Pi and enqueues into gcodequeue
  serialRead();
  // runs next gcode command in queue if the one executing currently is within tolerances
  processCommandQueue();
  updateValves();

  // LCD feedback
  static unsigned long tLCD = 0;
  if (millis() - tLCD > 200) {
    tLCD = millis();
    float cur = joints[0]->getCurrentAngleDeg();
    float tar = joints[0]->getTargetAngleDeg();
#ifdef VERBOSE
    Serial.print("cur:");
    Serial.println(cur);
#endif
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
// define to emulate slowely filling beyond limit
//#define TIMEOUT_READY

//  Serial Communication with GCode Queue
//  Protocol: P sends 1 line → A responds "OK <checksum>"
//  When queue < 5, A sends "Ready" to continue
void serialRead() {
  static bool lastSentReady = false;
#ifdef TIMEOUT_READY
  static unsigned long lastSentReadyTime = 0;
#endif

  // Check if we should send Ready signal
  if (
    (gcodeQueue.shouldSendReady() &&
    !lastSentReady)
#ifdef TIMEOUT_READY
    || millis() - lastSentReadyTime > 5000
#endif
    ) {
    Serial.println("Ready");
    lastSentReady = true;
#ifdef TIMEOUT_READY
    lastSentReadyTime = millis();
#endif
  } else if (!gcodeQueue.shouldSendReady()) {
    lastSentReady = false;
  }

  // Read one line at a time
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) return;

    // Parse and enqueue GCode command
    GCodeCommand cmd;
    switch (armController.parseGCodeLine(line, cmd)) {
      case GCodeParseResult::Success:
        if (gcodeQueue.enqueue(cmd)) {
          Serial.print("OK ");
          Serial.println(calculateChecksum(line));
        } else {
          Serial.println("ERR Queue Full");
        }
        break;
      case GCodeParseResult::InvalidCommand:
        Serial.println("ERR Invalid GCode");
      case GCodeParseResult::EmptyLine:
      case GCodeParseResult::ModeChange:
      default:
        // do nothing
        break;
    }
  }
}

//  Process commands from the queue
void processCommandQueue() {
  if (!gcodeQueue.isEmpty() && armController.isAtTarget()) {
    GCodeCommand cmd;
    if (gcodeQueue.dequeue(cmd)) {
      armController.processGCodeCommand(cmd);

      #ifdef VERBOSE
      Serial.print("Processed: ");
      Serial.print(cmd.commandType);
      if (cmd.hasX) { Serial.print(" X"); Serial.print(cmd.x); }
      if (cmd.hasY) { Serial.print(" Y"); Serial.print(cmd.y); }
      if (cmd.hasZ) { Serial.print(" Z"); Serial.print(cmd.z); }
      Serial.print(" | Queue: ");
      Serial.println(gcodeQueue.count());
      #endif
    }
  }
}

//  calculateChecksum()
//  Returns simple XOR checksum for serial confirmation.
uint8_t calculateChecksum(String &line) {
  uint8_t checksum = 0; for (int i=0;i<line.length();i++) checksum ^= (uint8_t)line[i];
  return checksum;
}
