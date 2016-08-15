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

  void mark();
  void space();

  void test(byte idx);

  bool isBusy();

private:
  IOPin<GPIOC, 1>   pinTXMod;
  IOPin<GPIOC, 2>   pinTXEnable;

  const uint8_t * txBuffer;
  uint16_t  txLength;
  uint8_t   bitIndex;
  uint8_t   txShift;
  bool      autoShutdown;
  bool      active;

  void shiftNew();
};

