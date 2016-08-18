#include "gps.h"
#include "pins.h"

const int gpsBaudrate = 9600;

IOPin<GPIOC, 0>   pinGPSRX;       // Hardware RX pin


GPSInfo::GPSInfo() {
  altitude[0] = latitude[0] = longitude[0] = time[0] = 0;
  fix = 0;
  satCount = 0;
}

void GPSInfo::setFix(char fix) {
  if (this->fix == fix) return;
  this->fix = fix;
}

void GPSInfo::setTime(const char *time) {
  strncpy(this->time, time, 12);
}

void GPSInfo::setLatitude(const char *latitude) {
  strncpy(this->latitude, latitude, 12);
}

void GPSInfo::setLongitude(const char *longitude) {
  if (*longitude == '0') longitude++;
  strncpy(this->longitude, longitude, 12);
}  

void GPSInfo::setAltitude(const char *altitude) {
  strncpy(this->altitude, altitude, 8);
}  

void GPSInfo::setSatCount(const char *satCount) {
  this->satCount = atoi(satCount);
}  

void GPSInfo::print() {
  Serial.print("GPS:");
  if (fix) {
    Serial.print(" Fix: "); 
    Serial.print(fix);
  }
  if (time[0]) {
    Serial.print(" Time: "); 
    Serial.print(time);
  }
  if (latitude[0]) {
    Serial.print(" Lat: "); 
    Serial.print(latitude);
  }
  if (longitude[0]) {
    Serial.print(" Lon: "); 
    Serial.print(longitude);
  }
  if (altitude[0]) {
    Serial.print(" Alt: "); 
    Serial.print(altitude);
  }
  if (satCount > 0) {
    Serial.print(" Sat: "); 
    Serial.print(satCount);
  }
  Serial.println();
}


void GPSParser::parse(char c) {
  enum State {
    INIT,
    FIELD
  };

  static State state = INIT;
  static SentenceType sentence;
  static char fieldValue[13];
  static byte fieldLength;
  static byte fieldIndex;

  switch (state) {
    case INIT:
      if (c == '$') {
        state = FIELD;
        fieldIndex = 0;
        fieldLength = 0;
      }
      break;
    case FIELD:
      if (c == ',') {
        // Process field
        if (fieldIndex == 0) {
          if (memcmp(fieldValue, "GPGSA", 5) == 0) {
            sentence = SENTENCE_GSA;
          }
          else if (memcmp(fieldValue, "GPGGA", 5) == 0) {
            sentence = SENTENCE_GGA;
          }
          else sentence = SENTENCE_NONE;
        }
        // Add terminating zero
        fieldValue[fieldLength] = 0;
        processField(sentence, fieldIndex, fieldValue, fieldLength);
        fieldIndex++;
        fieldLength = 0;
      }
      else if (c == '\n') {
        if (sentence == SENTENCE_GGA) {
          //gpsInfo.print();
        }
        state = INIT;
      }
      else {
        if (fieldLength < 12) {
          fieldValue[fieldLength++] = c;
        }
      }
      break;    
  }
}

void GPSParser::processField (byte sentence, byte index, const char *value, byte length) {
  switch (sentence) {
    case SENTENCE_GSA: 
      switch (index) {
        case 2: gpsInfo.setFix(value[0]); break;   
      }
      break;
    case SENTENCE_GGA:
      switch (index) {
        case 1: gpsInfo.setTime(value); break;
        case 2: gpsInfo.setLatitude(value); break;
        case 4: gpsInfo.setLongitude(value); break;
        case 7: gpsInfo.setSatCount(value); break;
        case 9: gpsInfo.setAltitude(value); break;
      }
      break;
    default: 
      break;
  }
}


#ifdef HARDWARE_SERIAL

void gpsBegin() {
  Serial1.begin(gpsBaudrate);
}

byte gpsAvailable() {
  return Serial1.available();
}

byte gpsRead() {
  return Serial1.read();
}

#else 

#define RXBUFFER_SIZE   32
volatile byte rxBuff[RXBUFFER_SIZE];
volatile byte rxBuffHead = 0;
volatile byte rxBuffSize = 0;

volatile byte rxBitIdx;
volatile bool rxError;
volatile byte rxData;

void gpsReceive(byte data) {
  if (rxBuffSize >= RXBUFFER_SIZE) return;

  byte tail = (rxBuffHead + rxBuffSize) % RXBUFFER_SIZE;
  rxBuff[tail] = data;
  rxBuffSize++;
}

byte gpsAvailable() {
  return rxBuffSize;
}

byte gpsRead() {
  uint8_t sreg_save = SREG;
  cli();

  byte data = rxBuff[rxBuffHead];
  if (rxBuffSize > 0) {
    rxBuffHead = (rxBuffHead + 1) % RXBUFFER_SIZE;
    rxBuffSize--;
  }

  SREG = sreg_save;
  return data;
}

#if defined (__AVR_ATmega32U4__)

#define BAUDRATE_PERIOD  (F_CPU / 1 / gpsBaudrate)

void gpsBegin() {
  byte mode = 14;
  byte presc = 1;     // CK/1
  TCCR3A = (mode & 3);
  TCCR3B = ((mode & 12) << 1) | presc;
  ICR3   = BAUDRATE_PERIOD - 1;

  // Enable pullup on RX
  PORTB |= _BV(PB4);

  PCICR |= _BV(PCIE0);
  
  // Enable pin change interrupt
  PCMSK0 |= _BV(PCINT4);
}

ISR(TIMER3_OVF_vect) { 
  byte state = PINB & _BV(PB4);

  if (rxBitIdx == 10) {
    // Disable timer interrupt
    TIMSK3 &= ~_BV(TOIE3);
    // Transmission complete, check stop bit
    if (state == 0) {
      rxError = true;
    }
    if (!rxError) {
      gpsReceive(rxData);
    }
    // Enable pin change interrupt
    PCMSK0 |= _BV(PCINT4);
  }
  else if (rxBitIdx == 0) {
      // Check start bit
      if (state != 0) {
        rxError = true;
      }
  }
  else {
      // Shift in bit (LSB first)
      rxData >>= 1;
      if (state) rxData |= 0x80;
  }
  // Next bit
  rxBitIdx++;
}

ISR(PCINT0_vect) { 
  byte state = PINB & _BV(PB4);

  // Check for falling edge
  if (state == 0) {
    // Disable pin change interrupt
    PCMSK0 &= ~_BV(PCINT4);

    rxBitIdx = 0;
    rxData = 0;
    rxError = false;

    // Set up half the bit period
    TCNT3 = BAUDRATE_PERIOD / 2;
    // Enable timer interrupt
    TIMSK3 |= _BV(TOIE3);
  }
}

#endif

#if defined (__AVR_ATmega328P__)

#define BAUDRATE_PERIOD  (F_CPU / 8 / gpsBaudrate)

#define GPS_PCINT_SETUP   PCICR |= _BV(PCIE1)
#define GPS_PCINT_ENABLE  PCMSK1 |= _BV(PCINT8)
#define GPS_PCINT_DISABLE PCMSK1 &= ~_BV(PCINT8)
#define GPS_TIMER_ENABLE  TIMSK2 |= _BV(TOIE2)
#define GPS_TIMER_DISABLE TIMSK2 &= ~_BV(TOIE2)

void gpsBegin() {
  byte mode = 7;
  byte presc = 2;     // CK/8
  TCCR2A = (mode & 3);
  TCCR2B = ((mode & 12) << 1) | presc;
  OCR2A   = BAUDRATE_PERIOD - 1;

  // Enable pullup on RX
  pinGPSRX.mode(IOMode::InputPullup);

  GPS_PCINT_SETUP;
  
  // Enable pin change interrupt
  GPS_PCINT_ENABLE;
}

ISR(TIMER2_OVF_vect) { 
  byte state = pinGPSRX.read();

  if (rxBitIdx == 10) {
    // Disable timer interrupt
    GPS_TIMER_DISABLE;
    // Transmission complete, check stop bit
    if (state == 0) {
      rxError = true;
    }
    if (!rxError) {
      gpsReceive(rxData);
    }
    // Enable pin change interrupt
    GPS_PCINT_ENABLE;
  }
  else if (rxBitIdx == 0) {
      // Check start bit
      if (state != 0) {
        rxError = true;
      }
  }
  else {
      // Shift in bit (LSB first)
      rxData >>= 1;
      if (state) rxData |= 0x80;
  }
  // Next bit
  rxBitIdx++;
}

ISR(PCINT1_vect) { 
  byte state = pinGPSRX.read();

  // Check for falling edge
  if (state == 0) {
    // Disable pin change interrupt
    GPS_PCINT_DISABLE;

    rxBitIdx = 0;
    rxData = 0;
    rxError = false;

    // Set up half the bit period
    TCNT2 = BAUDRATE_PERIOD / 2;
    // Enable timer interrupt
    GPS_TIMER_ENABLE;
  }
}

#endif

#endif
