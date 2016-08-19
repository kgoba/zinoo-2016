#pragma once
#include "gps.h"
#include "pins.h"
#include "fstring.h"

#include "EEPROM.h"

/******* HARDWARE CONFIG *******/

typedef IOPin<GPIOD, 6>  PinARM; 
typedef IOPin<GPIOB, 0>  PinLED;
typedef IOPin<GPIOD, 7>  PinBuzzer; 

const int adcChanBattery = 3;
const int adcChanTempExt = 6;
const int adcChanTempInt = 7;


/******* SOFTWARE CONFIG *******/

const uint16_t kTimeSlots = 4;

const uint16_t fskBaudrate  = 300;  // FSK transmitter baudrate

const uint16_t kFixTimeout  = 180;  // Maximum allowed time (sec) to GPS fix before error

const uint16_t kAltitudeThreshold = 1000;     // Beep below this altitude (meters)

const uint16_t kBuzzerPattern1 = 0b0000000101001111;      // Retrieval
const uint16_t kBuzzerPattern2 = 0b0000111100001111;      // Indicates GPS fix problems
const uint16_t kLEDPattern1    = 0b0000000011111111;      // Normal 50% flashing
const uint16_t kLEDPattern2    = 0b1001000000000000;      // Retrieval flashing

const char txTestString[] = "AAAAA";
const char txTestStringLength = sizeof(txTestString) / sizeof(char);

enum {
  kStatusNoGPSLock  = 0,
  kStatusAfterReset = 1,
  kStatusCameraOff  = 2
};


enum DebugMode {
  DbgConsole,
  DbgNMEA,
  DbgUKHAS
};

struct ResetState {
  uint16_t  resetCount;
  uint16_t  sentenceID;
  GPSInfo   gpsInfo;

  ResetState() {
  }

  void clear() {
    sentenceID = 1;
    resetCount = 0;
    gpsInfo.clear();    
  }
};

struct Config {
  uint16_t  blockId;

  FString<6>   payloadName;

  uint8_t enableTX       : 1;
  uint8_t enableLastFix  : 1;
  
  DebugMode debugMode;    // Debug console mode
  

  static void defaults() {
    Serial.println("No EEPROM data found, configuring default settings");
    config.blockId = 0;
    config.payloadName.assign("ZGND");

    config.enableTX = 1;
    config.enableLastFix = 1;
    
    config.debugMode = DbgConsole;    
  }

  static void save() {
    eeprom_update_block((void *)&config, &nvConfig, sizeof(config));
  }

  static void restore() {
    eeprom_read_block((void *)&config, &nvConfig, sizeof(config));
    //Serial.print("EEPROM block id: ");
    //Serial.println(config.blockId);
    if (config.blockId == 0xFFFF) defaults();
    config.blockId++;
  }

  static uint8_t getMyID() {
    char myId = Config::config.payloadName[-1]; // Get the last character of payload name
      if (myId >= '0' && myId <= '9') {
        myId -= '0';
        return myId;
      }
    return 0;
  }

  static uint8_t getTimeSlots() {
    return kTimeSlots;
  }  

  static Config EEMEM nvConfig;
  static Config config;
};

