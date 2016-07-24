#pragma once
#include "Arduino.h"

#include "fstring.h"
#include "gps.h"
#include "crc.h"

struct FlightData {
  FlightData();
  
  GPSInfo gpsInfo;

  byte temperatureInternal;   // range 00..99 corresponding to -60C .. 40C
  byte temperatureExternal;   // range 00..99 corresponding to -60C .. 40C
  byte batteryVoltage;        // range 00..99 corresponding to 0.50V .. 1.50V
  byte status;
};


class UKHASPacketizer {
public:
  UKHASPacketizer(const char *payloadName = "ZINOOGND");
  void makePacket(const FlightData &data);
  const uint8_t *getPacketBuffer();
  uint8_t getPacketLength();

private:
  uint16_t    sentenceID;
  const char  * payloadName;
  
  FString<80> packet;
  CRC16_CCITT crc;
};
