#pragma once
#include "Arduino.h"
#include "pins.h"

class FSKTransmitter {
public:
  FSKTransmitter();

  void enable();
  void disable();
  void transmit(const uint8_t *buffer, uint16_t length);
  void tick();

  void test(byte idx);

  bool isBusy();

private:
  PinD2   pinTXMod;
  PinD3   pinTXEnable;

  const uint8_t * txBuffer;
  uint16_t  txLength;
  uint8_t   bitIndex;
  uint8_t   txShift;
  bool      autoShutdown;
  bool      active;

  void mark();
  void space();
  void shiftNew();
};

