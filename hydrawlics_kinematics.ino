//String TEST_LINE = String("");

String TEST_LINE
{R"(G0 Z1
G0 X30 Y50
G0 Z0
G1 X35 Y45
G1 X40 Y42
G1 X45 Y40
G1 X50 Y40
G1 X55 Y40
G1 X60 Y42
G1 X65 Y45
G1 X70 Y50
G0 Z1
G0 X45 Y60
G0 Z0
G1 X45 Y65
G0 Z1
G0 X55 Y60
G0 Z0
G1 X55 Y65
G0 Z1
)"};

// NOTE: The board resets automatically when a Serial connection is estabished..
// Therefore the whole 

void setup(){
  Serial.begin(115200);
  Serial.setTimeout(500);
  Serial.println("Connected");
}


void loop() {

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
}

uint8_t calculateChecksum(String &line) {
  uint8_t checksum = 0;
  for(int i = 0; i < line.length(); i++) {
      checksum ^= line[i];  // XOR all bytes
  }
  return checksum;
}