#pragma once
#include "Arduino.h"

template<byte capacity>
struct FString {
  FString() { size = 0; }
  FString(const char *str) { assign(str); }
  FString(const char *str, byte size) { this->size = (size > capacity) ? capacity : size; memcpy(buf, str, this->size); }

  void assign(const char *str) { 
    size = strlen(str); 
    size = (size > capacity) ? capacity : size; 
    memcpy(buf, str, this->size); 
  }

  template<byte cap2>
  void append(const FString<cap2> &s) {
    word newSize = s.size + size;
    if (newSize > capacity) newSize = capacity;
    memcpy(buf + size, s.buf, newSize - size);
    size = newSize;
  }

  void append(const char *str) {
    word newSize = strlen(str) + size;
    if (newSize > capacity) newSize = capacity;
    memcpy(buf + size, str, newSize - size);
    size = newSize;
  }

  void append(const char *str, uint8_t size) {
    word newSize = this->size + size;
    if (newSize > capacity) newSize = capacity;
    memcpy(buf + this->size, str, newSize - this->size);
    this->size = newSize;
  }

  void append(char c) {
    word newSize = 1 + size;
    if (newSize > capacity) return;
    buf[size] = c;
    size = newSize;
  }

  void append(int16_t number) {
    if (number < 0) {
      append('-');
      number = -number;
    }
    append((uint16_t) number);
  }

  void append(uint16_t number, byte mode = DEC) {
    if (mode == HEX) {
      append16(number);
    }
    else {
      appendDecimal(number);
    }
  }

  void clear() {
    size = 0;
  }

  uint16_t toUInt16() {
    uint16_t result = 0;
    char *ptr = buf;
    while (*ptr) {
      if (*ptr > '9' || *ptr < '0') break;
      result *= 10;
      result += (*ptr - '0');
      ptr++;
    }
    return result;
  }

  byte size;
  char buf[capacity];

private:
  void appendDecimal(uint16_t number) {
    byte d;
    bool skipZeros = true;

    for (d = 0; number >= 10000; d++) number -= 10000;
    if (d > 0 || !skipZeros) {
      append((char)('0' + d));
      skipZeros = false;
    }
    for (d = 0; number >= 1000; d++) number -= 1000;
    if (d > 0 || !skipZeros) {
      append((char)('0' + d));
      skipZeros = false;
    }
    for (d = 0; number >= 100; d++) number -= 100;
    if (d > 0 || !skipZeros) {
      append((char)('0' + d));
      skipZeros = false;
    }
    for (d = 0; number >= 10; d++) number -= 10;
    if (d > 0 || !skipZeros) {
      append((char)('0' + d));
      skipZeros = false;
    }
    d = number;
    append((char)('0' + d));
  }

  void append8(byte number) {
    byte d;
    d = number >> 4;
    append((char)(d + ((d > 9) ? 'A' - 10 : '0')));
    d = number & 0x0F;
    append((char)(d + ((d > 9) ? 'A' - 10 : '0')));
  }

  void append16(uint16_t number) {
    append8(number >> 8);
    append8(number & 0xFF);
  }
};


