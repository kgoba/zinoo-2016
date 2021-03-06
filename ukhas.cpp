#include "ukhas.h"

#include "adc.h"
#include "config.h"

FlightData::FlightData() {
  temperatureInternal = 0;
  temperatureExternal = 0;
  batteryVoltage = 0;
  pressure = 0;
  barometricAltitude = 0;
  status = 0;
}

void FlightData::updateGPS(const GPSInfo &gps) {
  if (gps.time[0]) time.assign(gps.time);
  if (gps.latitude[0]) latitude.assign(gps.latitude);
  if (gps.longitude[0]) longitude.assign(gps.longitude);
  if (gps.altitude[0]) {
    int16_t tmp = atoi(gps.altitude);
    if (tmp > 0) {
      altitude = tmp;
    }
    else {
      altitude = 0;
    }
  }
  else {
    altitude = 0;
  }

  satCount = gps.satCount;
}

int8_t FlightData::getSeconds() {
  if (time.size < 6) return -1;
  FString<2> seconds = time.substr<2>(4);
  return seconds.toUInt16();
}

int8_t FlightData::getMinutes() {
  if (time.size < 6) return -1;
  FString<2> minutes = time.substr<2>(2);
  return minutes.toUInt16();
}

void FlightData::updateTime() {  
}

void FlightData::print() {
  Serial.print("FD: ");
  if (fix == '3') Serial.print("3D fix ");

  if (time.size > 0) {
    time.print(); Serial.print(" ");
  }
  if (latitude.size > 0) {
    latitude.print(); Serial.print("N ");
  }
  if (longitude.size > 0) {
    longitude.print(); Serial.print("E ");
  }
  if (altitude > 0) {
    Serial.print(altitude); Serial.print("m ");
  }

  Serial.print(satCount); Serial.print(" ");

  if (pressure > 0) {
    Serial.print(4 * (uint32_t)pressure); Serial.print("Pa ");
    Serial.print(barometricAltitude); Serial.print("m ");
  }

  Serial.print(temperatureInternal); Serial.print("C ");
  Serial.print(temperatureExternal); Serial.print("C ");
  Serial.print(batteryVoltage / 100.0); Serial.print("V ");
  Serial.println();
}

int8_t convertTemperature(uint16_t rawADC) {
  // NTC N_03P00223, Rdiv=100k, R0=22k, B=4220K, alpha_25C=-4.7
  // ADC 10 bit values for temperatures in range -60..+40C in steps of 5C
  const uint16_t tempTable[] = {
    16, 25, 38, 57, 82, 116, 159, 212, 274, 344, 418, 494, 567, 
    636, 698, 753, 800, 839, 872, 899, 921
  };
  const uint8_t stepSize = 5;     // 5C

  // Find the pair of closest lookup table entries
  uint8_t idx = 0;
  int8_t temp = -60;
  while (tempTable[idx] < rawADC) {
    idx++;
    temp += 5;
    if (temp > 40) {
      return 40;
    }
  }
  if (idx == 0) return -60;

  //Serial.print("T raw: ");
  //Serial.print(rawADC);

  //Serial.print(" calc: ");
  //Serial.println(temp);

  // Linear interpolation between adjacent table entries
  uint8_t diff = tempTable[idx] - tempTable[idx - 1];
  temp -= (stepSize * (tempTable[idx] - rawADC) + diff/2) / diff;
  
  //Serial.print(" calc: ");
  //  Serial.println(temp);

  return temp;
}

void FlightData::updateTemperature() {
  temperatureInternal = convertTemperature(adcRead(adcChanTempInt));
  temperatureExternal = convertTemperature(adcRead(adcChanTempExt));
  batteryVoltage = adcRead(adcChanBattery) / 3;    // Approximation of x*330/1024
}

UKHASPacketizer::UKHASPacketizer(const char *payloadName) {
  sentenceID = 1;
  setPayloadName(payloadName);
}

void UKHASPacketizer::setPayloadName(const char *payloadName) {
  this->payloadName.assign(payloadName);
}

void UKHASPacketizer::makePacket(const FlightData &data) {
  char statusChar = (char)(data.status & 0x3F) | 0x40;

  uint8_t nibble = data.status >> 4;  
  char statusChar1 = (nibble > 9) ? nibble - 10 + 'A' : nibble + '0';
  nibble = data.status & 0x0F;
  char statusChar2 = (nibble > 9) ? nibble - 10 + 'A' : nibble + '0';

  uint16_t altitude = (data.altitude > 0) ? data.altitude : data.barometricAltitude;
  
  byte satCount = data.satCount;
  if (satCount > 9) satCount = 9;

  int16_t tempInt = 60 + data.temperatureInternal;
  if (tempInt > 99) tempInt = 99;
  else if (tempInt < 0) tempInt = 0;

  int16_t tempExt = 60 + data.temperatureExternal;
  if (tempExt > 99) tempExt = 99;
  else if (tempExt < 0) tempExt = 0;

  //int16_t battVoltage = -60 + data.batteryVoltage;
  //if (battVoltage > 99) battVoltage = 99;
  //else if (battVoltage < 0) battVoltage = 0;
  int16_t battVoltage = data.batteryVoltage;

  packet.clear();
  packet.append("$$");
  packet.append(payloadName);
  packet.append(','); packet.append(sentenceID);
  //packet.append(','); packet.append(FString<7>(data.gpsInfo.latitude));
  //packet.append(','); packet.append(FString<7>(data.gpsInfo.longitude));
  //packet.append(','); packet.append(FString<8>(data.gpsInfo.altitude));
  //packet.append(','); packet.append(FString<6>(data.gpsInfo.time));

  packet.append(','); packet.append(data.latitude);
  packet.append(','); packet.append(data.longitude);
  packet.append(','); packet.append((5 + altitude) / 10);
  packet.append(','); packet.append(data.time);

  packet.append(','); packet.append(satCount);
  
  //packet.append(','); packet.append(tempInt);
  packet.append(','); packet.append(data.temperatureInternal);
  
  //packet.append(','); packet.append(tempExt);
  packet.append(','); packet.append(data.temperatureExternal);
  
  packet.append(','); packet.append(battVoltage);
  //packet.append(','); packet.append(statusChar);
  packet.append(','); 
  if (statusChar1 != '0') packet.append(statusChar1); 
  packet.append(statusChar2);

  uint16_t pressureMBar = (data.pressure + 12) / 25;
  packet.append(','); packet.append(pressureMBar);
  packet.append(','); packet.append((5 + data.barometricAltitude) / 10);

  /*
  if (data.pressure > 0) {
    uint16_t pressureMBar = (data.pressure + 12) / 25;
    packet.append(pressureMBar);
    packet.append('/');
    packet.append(data.barometricAltitude);
  }
  */

  // Now compute CRC checksum and append it
  crc.clear();
  uint16_t prefixLength = 2;    // Length of prefix to be ignored
  uint16_t checksum = crc.update((uint8_t *)(packet.buf + prefixLength), packet.size - prefixLength);
  packet.append('*'); packet.append(checksum, HEX);

  // Must send a newline character as well
  packet.append('\n');

  // Add terminating zero and print (only for debug)
  packet.append('\0');
  //Serial.print(packet.buf);

  sentenceID++;
}

const uint8_t * UKHASPacketizer::getPacketBuffer() {
  return (const uint8_t *)packet.buf;
}

uint8_t UKHASPacketizer::getPacketLength() {
  return packet.size;
}

