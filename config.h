#pragma once
#include "pins.h"

#include "EEPROM.h"

typedef IOPin<GPIOD, 6>  PinARM; 
typedef IOPin<GPIOB, 0>  PinLED;
typedef IOPin<GPIOD, 7>  PinBuzzer; 

const int adcChanBattery = 3;
const int adcChanTempExt = 6;
const int adcChanTempInt = 7;

const int fskBaudrate = 300;

const char txPreamble[] = "";
const uint8_t txPreambleLength = sizeof(txPreamble) / sizeof(char);

enum DebugMode {
  DbgConsole,
  DbgNMEA,
  DbgUKHAS
};

struct Config {
  uint16_t  blockId;

  FString<6>   payloadName;

  // Last stored GPS fix info
  uint16_t  lastAltitude;
  char      lastLatitude[7];
  char      lastLongitude[7];
  char      lastTime[6]; 

  uint8_t enableTX       : 1;
  uint8_t enableLastFix  : 1;
  
  DebugMode debugMode;    // Debug console mode

  static void defaults() {
    config.blockId = 0;
    config.payloadName.assign("ZGND");
    config.lastAltitude = 0;
    config.lastLatitude[0] = '\0';
    config.lastLongitude[0] = '\0';
    memcpy(config.lastTime, "000000", 6);

    config.enableTX = 1;
    config.enableLastFix = 1;
    
    config.debugMode = DbgConsole;    
  }

  static void save() {
    eeprom_update_block((void *)&config, &nvConfig, sizeof(config));
  }

  static void restore() {
    eeprom_read_block((void *)&config, &nvConfig, sizeof(config));
    Serial.print("EEPROM block id: ");
    Serial.println(config.blockId);
    if (config.blockId == 0xFFFF) defaults();
    config.blockId++;
  }

  static Config EEMEM nvConfig;
  static Config config;
};
