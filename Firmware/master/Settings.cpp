#include "Settings.h"
#include "Arduino.h"
Settings defaultSettings;

void Settings::loadFromEEPROM() {
  EEPROM.get(0, *this);  // Load settings from EEPROM

  // Perform a CRC check
  int crc = CRC;
  // Compare calculated CRC with the one stored in EEPROM
  if (CRCV != crc) {
    Serial.println("CRC Mismatch");
    // CRC mismatch, use default settings
    *this = defaultSettings;
    Settings::saveToEEPROM();
  }
}

void Settings::saveToEEPROM() const {
  EEPROM.put(0, *this);  // Save settings to EEPROM
  EEPROM.commit();
}
