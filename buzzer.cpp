#include "buzzer.h"

uint16_t buzzerPattern;
uint16_t buzzerPeriod;

PinBuzzer pinBuzzer;

void buzzerSet(uint16_t pattern, uint16_t period) {
  buzzerPattern = pattern;
  buzzerPeriod = period;
}

void buzzerBegin() {
  pinBuzzer.mode(IOMode::OutputLow);  
}

void buzzerTick() {
  static uint16_t counter;
  static uint8_t patIndex;

  static uint16_t pat;

  if (++counter < buzzerPeriod) return;
  counter = 0;

  pinBuzzer.write(pat & 1);
  pat >>= 1;

  if (++patIndex < 16) return;

  patIndex = 0;
  pat = buzzerPattern;
}

void buzzerBeep() {
  // Short beep
  pinBuzzer.write(1);
  delay(250);
  pinBuzzer.write(0);  
}


uint16_t ledPattern;
uint16_t ledPeriod;

PinLED pinLED;

void ledSet(uint16_t pattern, uint16_t period) {
  ledPattern = pattern;
  ledPeriod = period;
}

void ledBegin() {
  pinLED.mode(IOMode::OutputLow);  
}

void ledTick() {
  static uint8_t patIndex;
  static uint16_t pat;
  static uint16_t counter;

  if (++counter < ledPeriod) return;
  counter = 0;

  pinLED.write(! (pat & 1) );
  pat >>= 1;

  if (++patIndex < 16) return;

  patIndex = 0;
  pat = ledPattern;
}


