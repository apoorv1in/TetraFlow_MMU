#ifndef SETTINGS_H
#define SETTINGS_H
#include "mmu_config.h"
#include "mmu_hardware_config.h"
#include <EEPROM.h>
#define CRCV 15509


struct Settings {
  int CRC = CRCV;
  float distanceFromSwitchToHead = DISTANCE_FROM_SWITCH_TO_Head;
  float distanceFromSwitchToSafeZone = DISTANCE_FROM_SWITCH_TO_SAFEZONE;
  float filamentEncoderDetectionLength = FILAMENT_ENCODER_DETECTION_LENGTH;
  float aStallValue = A_STALL_VALUE;
  float eRotationDistanceT0 = E_ROTATION_DISTAMCE_T0;
  float eRotationDistanceT1 = E_ROTATION_DISTAMCE_T1;
  float eRotationDistanceT2 = E_ROTATION_DISTAMCE_T2;
  float eRotationDistanceT3 = E_ROTATION_DISTAMCE_T3;

  // Load settings from EEPROM, perform CRC check, and use default settings if CRC mismatch
  void loadFromEEPROM();
  
  // Save settings to EEPROM
  void saveToEEPROM() const;
};

// Default settings instance
extern Settings defaultSettings;

#endif
