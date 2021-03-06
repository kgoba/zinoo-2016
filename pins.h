#pragma once
#include "Arduino.h"

class GPIOB {
public:
  static volatile uint8_t * getPORT() { return &PORTB; }
  static volatile uint8_t * getDDR() { return &DDRB; }
  static volatile uint8_t * getPIN() { return &PINB; }
};

class GPIOC {
public:
  static volatile uint8_t * getPORT() { return &PORTC; }
  static volatile uint8_t * getDDR() { return &DDRC; }
  static volatile uint8_t * getPIN() { return &PINC; }
};

class GPIOD {
public:
  static volatile uint8_t * getPORT() { return &PORTD; }
  static volatile uint8_t * getDDR() { return &DDRD; }
  static volatile uint8_t * getPIN() { return &PIND; }
};

#if defined PORTE
class GPIOE {
public:
  static volatile uint8_t * getPORT() { return &PORTE; }
  static volatile uint8_t * getDDR() { return &DDRE; }
  static volatile uint8_t * getPIN() { return &PINE; }
};
#endif

class IOMode {
public:
  enum Mode {
    OutputLow,
    OutputHigh,
    Input,
    InputPullup
  };  
};

template<class Port, byte index>
class IOPin : public IOMode {
public:
  static bool read() { return *(Port::getPIN()) & (1 << index); }

  static void write(bool on) { if (on) high(); else low(); }
  static void high() { *(Port::getPORT()) |= (1 << index); }
  static void low() { *(Port::getPORT()) &= ~(1 << index); }
  static void toggle() { *(Port::getPORT()) ^= (1 << index); }
  
  static void output() { *(Port::getDDR()) |= (1 << index); }
  static void input() { *(Port::getDDR()) &= ~(1 << index); }

  static void mode(Mode m) {
    switch (m) {
      case OutputLow:   low();  output(); break;
      case OutputHigh:  high(); output(); break;
      case Input:       low();  input();  break;
      case InputPullup: high(); input();  break;
    }
  }
};


// Leonardo board (ATMega32u4)
#if defined (__AVR_ATmega32U4__)
  typedef IOPin<GPIOD, 2> PinD0;
  typedef IOPin<GPIOD, 3> PinD1;
  typedef IOPin<GPIOD, 1> PinD2;
  typedef IOPin<GPIOD, 0> PinD3;
  typedef IOPin<GPIOD, 4> PinD4;
  typedef IOPin<GPIOC, 6> PinD5;
  typedef IOPin<GPIOD, 7> PinD6;
  typedef IOPin<GPIOE, 6> PinD7;
  typedef IOPin<GPIOB, 4> PinD8;
  typedef IOPin<GPIOB, 5> PinD9;
  typedef IOPin<GPIOB, 6> PinD10;
  typedef IOPin<GPIOB, 7> PinD11;
  typedef IOPin<GPIOD, 6> PinD12;
  typedef IOPin<GPIOC, 7> PinD13;
#endif
#if defined (__AVR_ATmega328P__)
  typedef IOPin<GPIOD, 0> PinD0;
  typedef IOPin<GPIOD, 1> PinD1;
  typedef IOPin<GPIOD, 2> PinD2;
  typedef IOPin<GPIOD, 3> PinD3;
  typedef IOPin<GPIOD, 4> PinD4;
  typedef IOPin<GPIOD, 5> PinD5;
  typedef IOPin<GPIOD, 6> PinD6;
  typedef IOPin<GPIOD, 7> PinD7;
  typedef IOPin<GPIOB, 0> PinD8;
  typedef IOPin<GPIOB, 1> PinD9;
  typedef IOPin<GPIOB, 2> PinD10;
  typedef IOPin<GPIOB, 3> PinD11;
  typedef IOPin<GPIOB, 4> PinD12;
  typedef IOPin<GPIOB, 5> PinD13;
#endif

