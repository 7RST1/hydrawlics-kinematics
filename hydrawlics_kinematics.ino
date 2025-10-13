//String TEST_LINE = String("");
//#define VERBOSE

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
  LowFreqPWM* v_e;  // Pin 9, 1 Hz PWM, 5 Hz max action rate
  LowFreqPWM* v_r;
public:
  Valve(uint8_t _pin_e, uint8_t _pin_r) {
    v_e = new LowFreqPWM(_pin_e, 1.0, 5.0);  // Pin 9, 1 Hz PWM, 5 Hz max action rate
    v_r = new LowFreqPWM(_pin_r, 1.0, 5.0);
  }

  void updatePWMs(float move) {
    // move will be normalized to -1 (for retract) and 1 (for extend). 0 for stand still

    float normalized_move = constrain(move, -1, 1);

    float dc = abs(normalized_move);

    if (normalized_move < 0) {
      v_e->setDutyCycle(0);
      v_r->setDutyCycle(dc);
    } else if (normalized_move > 0) {
      v_e->setDutyCycle(dc);
      v_r->setDutyCycle(0);
    } else {
      v_e->setDutyCycle(0);
      v_r->setDutyCycle(0);
    }
  }
};

struct JointConfig {
  uint8_t pin_valve_e;
  uint8_t pin_valve_r;
  uint8_t pin_potmeter;
};

// Joint describes the whole joint, including;
// - Valve - controlling its angle
// - Piston - the piston the valve controles
// - Potmeter - reads the angle of the joint as it is
class Joint {
private:
  Valve* v;  // Pin 9, 1 Hz PWM, 5 Hz max action rate
  uint8_t pin_potmeter;
public:
  Joint(JointConfig config) {
    v = new Valve(config.pin_valve_e, config.pin_valve_r);
    pin_potmeter = config.pin_potmeter;
    pinMode(pin_potmeter, INPUT);
    //analogReadResolution(10);
  }

  // This will be synonumous with Update in sim
  void update() {
    float currentAngle = 0; // Insert math from Lisa

    
  }
};

Joint j1({2, 3, A0});
Joint j2({2, 3, A1});
Joint j3({2, 3, A2});
Joint j4({2, 3, A3});

Joint joints[4] = {j1, j2, j3, j4};

// NOTE: The board resets automatically when a Serial connection is estabished..
// Therefore the whole 

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(500);
  Serial.println("Connected");
}


void loop() {

  //serialRead();

  updateValves();

}

void updateValves() {
  static int count = 0;
  static float lastupdate = 0;
  if (millis() - lastupdate > 2000) {
    count = (count + 10) % 110;
    lastupdate = millis();
    Serial.println(count);
  }

  for (Joint j : joints) {

    j.update();
  }

  //valve.setDutyCycle(count);  // Will be clamped to 20-80% range
  //valve.update();
}

void serialRead() {
  // some start up time
  //delay(2000);

  if(Serial.available()) {
    //String readed = Seial.readStringUntil('\n');
    String readed = Serial.readStringUntil('\0');;
    readed.trim();
    //processLine(readed);

    // Send OK with checksum
    Serial.print("OK ");
    Serial.println(calculateChecksum(readed));
  }
};

uint8_t calculateChecksum(String &line) {
  uint8_t checksum = 0;
  for(int i = 0; i < line.length(); i++) {
      checksum ^= line[i];  // XOR all bytes
  }
  Serial.println(String("OK ") + checksum);
}
