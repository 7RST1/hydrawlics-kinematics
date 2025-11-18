#include "RotaryEncoder.h"
#include <EEPROM.h>

#define AS5600_ANGLE_REG 0x0C 

RotaryEncoder::RotaryEncoder(uint8_t tcaChannel,
                             uint8_t tcaAddress,
                             uint8_t as5600Address)
    : _tcaAddr(tcaAddress),
      _as5600Addr(as5600Address),
      _channel(tcaChannel),
      _offsetDeg(0.0f)
{
}

void RotaryEncoder::begin() {
    // Forutsetter at Wire.begin() er kalt i main-setup().
    // Her kan vi ev. teste en første lesing hvis ønskelig.
}

// Velger riktig kanal på TCA9548A
void RotaryEncoder::selectTCAChannel() const {
    if (_channel > 7) return; // enkel guard

    Wire.beginTransmission(_tcaAddr);
    Wire.write(1 << _channel); // enable valgt kanal
    Wire.endTransmission();
}

// Leser rå 12-bit vinkel fra AS5600
uint16_t RotaryEncoder::readRawAngleRegister() {
    selectTCAChannel();  // sørger for at riktig kanal er aktiv

    // Pek på vinkelregisteret
    Wire.beginTransmission(_as5600Addr);
    Wire.write(AS5600_ANGLE_REG);
    Wire.endTransmission(false); // repeated start

    // Les 2 bytes
    Wire.requestFrom((int)_as5600Addr, 2);
    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte  = Wire.read();

        uint16_t angle = ((uint16_t)highByte << 8) | lowByte;
        angle &= 0x0FFF; // 12-bit maske
        return angle;
    }

    // Ved feil – returner 0 (gyldig verdi, men greit og enkelt).
    // Om du vil kan du endre dette til f.eks. 0xFFFF og sjekke det i getRawAngle().
    return 0;
}

uint16_t RotaryEncoder::getRawAngle() {
    return readRawAngleRegister();
}

float RotaryEncoder::getAngleDeg() {
    uint16_t raw = readRawAngleRegister();

    // Konverter til grader 0–360
    float angle = (static_cast<float>(raw) / 4096.0f) * 360.0f;

    // Legg til offset
    angle += _offsetDeg;

    // Normaliser til 0–360
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f)    angle += 360.0f;

    return angle;
}

void RotaryEncoder::setOffsetDeg(float offsetDeg) {
    _offsetDeg = offsetDeg;
}

float RotaryEncoder::getOffsetDeg() const {
    return _offsetDeg;
}

// Lagrer offset (float) til EEPROM
void RotaryEncoder::saveOffsetToEEPROM(int eepromAddress) const {
    EEPROM.put(eepromAddress, _offsetDeg);
}

// Leser offset (float) fra EEPROM
void RotaryEncoder::loadOffsetFromEEPROM(int eepromAddress) {
    EEPROM.get(eepromAddress, _offsetDeg);
}