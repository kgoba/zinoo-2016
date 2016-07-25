#include "gps.h"
#include "ukhas.h"
#include "fstring.h"
#include "crc.h"
#include "fsk.h"
#include "pins.h"

/*
 * Code currently supports Leonardo board, but should be portable to Uno
 */


PinD13 pinLED;
PinD4 pinBuzzer;


const int timerFrequency = 600;

bool ledBlink = false;
word ledPeriod = 1;

void gpsBegin();
byte gpsAvailable();
byte gpsRead();

volatile byte gFlags;

enum {
  FLAG_SECOND
};

FlightData flightData;

GPSParser gpsParser;
UKHASPacketizer packetizer;
FSKTransmitter transmitter;

class PeriodicTimer {
public:
  typedef void (* TimerCallback) (void);

  void setPeriodMillis(uint16_t milliseconds);
  void setPeriodMicros(uint16_t microseconds);

  void attach(TimerCallback callback);
};

void setup() 
{
  pinLED.mode(IOMode::OutputHigh);

  int startTime = millis();
  while (!Serial) {
    if (millis() - startTime > 3000) {
      break;
    }
  }
  Serial.begin(9600);

  // Configure Timer1
  byte mode = 14;
  byte presc = 3;
  TCCR1A = (mode & 3);
  TCCR1B = ((mode & 12) << 1) | presc;
  ICR1   = (F_CPU / 64 / timerFrequency) - 1;
  TIMSK1 = _BV(TOIE1);

  gpsBegin();

  sei();
  Serial.println("Reset");

  // Check CRC
  CRC16_CCITT crc;
  uint16_t checksum = crc.update("habitat");
  Serial.print("CRC test value (should be 3EFB): ");
  Serial.println(checksum, HEX);
}

void loop() 
{
  // Read GPS serial data
  while (gpsAvailable() > 0) {
    char c = gpsRead();
    //Serial.print(c);
    gpsParser.parse(c);
  }

  // Check flags
  if (gFlags & (1 << FLAG_SECOND)) {
    gFlags &= ~(1 << FLAG_SECOND);

    // Indicate fix by flashing LED
    char fix = gpsParser.gpsInfo.fix;
    if (fix == '3') {
      ledPeriod = 300;
    }
    else ledPeriod = 30;
    ledBlink = true;

    
    if (!transmitter.isBusy()) {
      flightData.gpsInfo = gpsParser.gpsInfo;
      packetizer.makePacket(flightData);
      if (fix == '3') 
        transmitter.transmit(packetizer.getPacketBuffer(), packetizer.getPacketLength() - 1);
    }
  }
}

ISR(TIMER1_OVF_vect) { 
  static word count;
  static bool phase;
  static word count2;

  transmitter.tick();

  count++;
  if (count > ledPeriod) {
    pinLED.write(!ledBlink | phase);
    phase = !phase;
    count = 0;
  }

  count2++;
  if (count2 >= 1 * timerFrequency) {
    gFlags |= (1 << FLAG_SECOND);
    count2 = 0;
  }
}

