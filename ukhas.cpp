#include "ukhas.h"

FlightData::FlightData() {
  temperatureInternal = 0;
  temperatureExternal = 0;
  batteryVoltage = 0;
  status = 0;
}

UKHASPacketizer::UKHASPacketizer(const char *payloadName) {
  sentenceID = 1;
  this->payloadName = payloadName;
}

void UKHASPacketizer::makePacket(const FlightData &data) {
  char statusChar = (char)(data.status & 0x3F) | 0x40;
  byte satCount = data.gpsInfo.satCount;
  if (satCount > 9) satCount = 9;
  
  packet.clear();
  packet.append("$$");
  packet.append(payloadName);
  packet.append(','); packet.append(sentenceID);
  packet.append(','); packet.append(FString<7>(data.gpsInfo.latitude));
  packet.append(','); packet.append(FString<7>(data.gpsInfo.longitude));
  packet.append(','); packet.append(FString<8>(data.gpsInfo.altitude));
  packet.append(','); packet.append(FString<6>(data.gpsInfo.time));
  packet.append(','); packet.append(satCount);
  packet.append(','); packet.append(data.temperatureInternal);
  packet.append(','); packet.append(data.temperatureExternal);
  packet.append(','); packet.append(data.batteryVoltage);
  packet.append(','); packet.append(statusChar);

  // Now compute CRC checksum and append it
  crc.clear();
  uint16_t prefixLength = 2;    // Length of prefix to be ignored
  uint16_t checksum = crc.update((uint8_t *)(packet.buf + prefixLength), packet.size - prefixLength);
  packet.append('*'); packet.append(checksum, HEX);

  // Must send a newline character as well
  packet.append('\n');

  // Add terminating zero and print (only for debug)
  packet.append('\0');
  Serial.print(packet.buf);

  sentenceID++;
}

const uint8_t * UKHASPacketizer::getPacketBuffer() {
  return (const uint8_t *)packet.buf;
}

uint8_t UKHASPacketizer::getPacketLength() {
  return packet.size;
}

