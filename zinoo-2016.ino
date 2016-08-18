#include <EEPROM.h>

#include "gps.h"
#include "ukhas.h"
#include "fstring.h"
#include "crc.h"
#include "fsk.h"
#include "adc.h"
#include "pins.h"
#include "MS5607.h"

#include "config.h"

/*
 * Code currently supports Leonardo and Pro Mini (328P @ 3.3V)
 */

const char compile_date[] = __DATE__ " " __TIME__;


enum {
  FLAG_SECOND,
};

enum State {
  STATE_RESET,
  STATE_SAFE,
  STATE_FLIGHT
};

enum TXState {
  TXSTATE_IDLE,
  TXSTATE_PREAMBLE,
  TXSTATE_CARRIER,
  TXSTATE_TRANSMIT,
  TXSTATE_TEST
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variables
// 
// 

bool ledBlink = false;
word ledPeriod = 1;

// This just reserves space in EEPROM for the non-volatile configuration
Config EEMEM Config::nvConfig;    
Config Config::config;  // this is volatile configuration in RAM

FlightData flightData;  // current payload data

I2C bus;
BarometerSimple barometer(bus);

GPSParser gpsParser;
UKHASPacketizer packetizer;
FSKTransmitter transmitter;

volatile byte gFlags;

char        gTime[6];
State       gState;
TXState     gTXState;

PinLED    pinLED;
PinARM    pinARM;
PinBuzzer pinBuzzer;

void testPressureFormula() {
  Serial.print("Pressure");
  Serial.print('\t');
  Serial.println("Altitude");
  for (uint16_t pressure = 250; pressure < 26000; pressure += 250) {
    uint16_t altitude = barometer.getAltitude(pressure);
    Serial.print((uint32_t)pressure * 4);
    Serial.print('\t');
    Serial.println(altitude);
  }  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization and setup after MCU reset
// 
// 

void setup() 
{
  gState = STATE_RESET;
  gTXState = TXSTATE_IDLE;

  // Setup GPIO lines
  pinBuzzer.mode(IOMode::OutputLow);
  pinLED.mode(IOMode::OutputHigh);

  // Short beep
  pinBuzzer.write(1);
  delay(250);
  pinBuzzer.write(0);

  // Try to start USB (necessary for Leonardo board)
  int startTime = millis();
  while (!Serial) {
    // Check timeout
    if (millis() - startTime > 3000) {
      break;
    }
  }
  Serial.begin(9600);

  // Set ADC reference from AVCC, set clock prescaler 1:16
  adcSetup(ADCRefSupply, 4);

  // Configure Timer1 and enable interrupts at FSK baudrate
  byte mode = 14;   // Fast PWM with TOP=ICR1
  byte presc = 3;   // Prescaler /64
  TCCR1A = (mode & 3);
  TCCR1B = ((mode & 12) << 1) | presc;
  ICR1   = (F_CPU / 64 / fskBaudrate) - 1;
  TIMSK1 = _BV(TOIE1);

  // Initialize GPS software serial
  gpsBegin();

  Config::restore();
  packetizer.setPayloadName(Config::config.payloadName);

  // Enable interrupts
  sei();
  
  Serial.println("Reset");

  // Check CRC algorithm validity
  CRC16_CCITT crc;
  uint16_t checksum = crc.update("habitat");
  Serial.print("CRC test value (should be 3EFB): ");
  Serial.println(checksum, HEX);

  barometer.initialize();

  Serial.print("> ");
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
// 
// Reads and parses GPS data, reads and parses console input and executes commands,
// checks time, builds UKHAS packet and initiates radio transmission of it
//

template<byte N> void print(const FString<N> &str) {
  for (uint8_t idx = 0; idx < str.size; idx++) {
    Serial.print(str.buf[idx]);
  }
}

void parseCommand(const char *command) 
{
  const char *ptr1 = command;
  const char *ptr2 = command;

  enum CmdType {
    kCommandUnknown,
    kCommandId,
    kCommandTX,
    kCommandPkt,
    kCommandVer,
    kCommandGPS,
    kCommandBaro,
    kCommandSave
  };

  uint8_t idx = 0;
  CmdType cmd = kCommandUnknown;

  while (*ptr1 != 0) {
    ptr2 = ptr1;
    while (*ptr2 != ' ') {
      ptr2++;
      if (*ptr2 == 0) break;
    }
    uint8_t len = ptr2 - ptr1;

    if (idx == 0) {
      if (strncmp(ptr1, "id", len) == 0) {
        cmd = kCommandId;
      }
      else if (strncmp(ptr1, "pkt", len) == 0) {
        cmd = kCommandPkt;
      }
      else if (strncmp(ptr1, "baro", len) == 0) {
        cmd = kCommandBaro;
      }
      else if (strncmp(ptr1, "tx", len) == 0) {
        cmd = kCommandTX;
      }      
      else if (strncmp(ptr1, "save", len) == 0) {
        cmd = kCommandSave;
      }
      else if (strncmp(ptr1, "ver", len) == 0) {
        cmd = kCommandVer;
      }      
      else if (strncmp(ptr1, "gps", len) == 0) {
        cmd = kCommandGPS;
      }      
    }
    else if (idx == 1) {
      bool success = false;
      switch (cmd) {
        case kCommandId: 
          Config::config.payloadName.assign(ptr1, len);
          packetizer.setPayloadName(Config::config.payloadName); 
          success = true; 
          break; 
        case kCommandTX: 
          if (len == 1) {
            if (*ptr1 == '0') {
              Config::config.enableTX = 0;
              success = true;
            }
            else if (*ptr1 == '1') {
              Config::config.enableTX = 1;
              success = true;
            }
            else if (*ptr1 == 't') {
              gTXState = TXSTATE_TEST;
              transmitter.enable();
              success = true;
            }
          }
          break;
      }
      if (success) {
        Serial.println("OK");
      }
      else {
        Serial.println("ERROR");
      }
    }

    ptr1 = ptr2;
    while (*ptr1 == ' ') {
      ptr1++;
      if (*ptr1 == 0) break;
    }
    idx++;
  }

  if (idx == 1) {
    switch (cmd) {
      case kCommandSave: 
        Config::save();
        Serial.println("OK");
        break;
      case kCommandVer: 
        Serial.println(compile_date);
        break;
      case kCommandGPS: 
        gpsParser.gpsInfo.print();
        break;
      case kCommandId: print(Config::config.payloadName); Serial.println(); break;     
      case kCommandTX: Serial.println(Config::config.enableTX); break;     
      case kCommandPkt: Serial.print((const char *)packetizer.getPacketBuffer()); break;
      case kCommandBaro:       
        Serial.print("P: ");
        Serial.print((uint32_t)barometer.getPressure() * 4);
        Serial.print(" H: ");
        Serial.println(barometer.getAltitude());
        break;
      default:
        Serial.println("?");
    }
  }
}

void loop() 
{
  // Read GPS serial data
  while (gpsAvailable() > 0) {
    char c = gpsRead();
    //Serial.print(c);
    gpsParser.parse(c);
  }

  static FString<32> consoleCmd;

  // Read serial console data
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c != '\x0D' && c != '\x0A') {
      consoleCmd.append(c);
      Serial.print(c);                    // Echo back
    }
    else if (consoleCmd.size > 0) {
      Serial.println();
      consoleCmd.append('\0');
      parseCommand(consoleCmd.buf);
      consoleCmd.clear();
      Serial.print("> ");
    }
  }

  static uint8_t preambleIdx;
  
  switch (gTXState) {
    case TXSTATE_IDLE:
      if (!transmitter.isBusy()) {
        transmitter.disable();
      }
      break;

    case TXSTATE_PREAMBLE:
      if (!transmitter.isBusy()) {
        if (preambleIdx < 80) {
          transmitter.transmit((const uint8_t *)"RY", 2);
        }
        else {
          transmitter.transmit((const uint8_t *)"\r\n", 2);
          if (preambleIdx >= 82) {
            gTXState = TXSTATE_TRANSMIT;
            preambleIdx = 0;
          }
        }
        preambleIdx++;
      }
      break;
    case TXSTATE_TRANSMIT:
      if (!transmitter.isBusy()) {       
        packetizer.makePacket(flightData);
        transmitter.transmit(packetizer.getPacketBuffer(), packetizer.getPacketLength() - 1);
      }
      break;
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

    static bool txPhase;
    if (gTXState == TXSTATE_TEST) {
      txPhase = !txPhase;
      if (txPhase) transmitter.mark();
      else transmitter.space();
    }

    if (barometer.update()) {
      flightData.pressure = barometer.getPressure();
      flightData.barometricAltitude = barometer.getAltitude();
    }

    //gpsParser.gpsInfo.print();
    flightData.updateGPS( gpsParser.gpsInfo );
    flightData.updateTemperature();

    if (Config::config.enableTX) {
      // Check TX time division
      int8_t minutes = flightData.getMinutes();
      int8_t seconds = flightData.getSeconds();
      if (minutes >= 0) {     // Check for valid time
        char myId = Config::config.payloadName[-1]; // Get the last character of payload name
        if (myId >= '0' && myId <= '9') {
          myId -= '0';
          if (minutes % 6 == myId) {
            if (gTXState == TXSTATE_IDLE) {
              gTXState = TXSTATE_CARRIER;
              transmitter.enable();
            }
            else if (gTXState == TXSTATE_CARRIER && seconds >= 10) {
              gTXState = TXSTATE_TRANSMIT;
            }
          }
          else {
            gTXState = TXSTATE_IDLE;
          }
        }
      }    
    }
  }

  delay(1);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer1 overflow vector
//
// Responsible for FSK transmitter tick at every transmitted bit,
// flashing LEDs and keeping track of time
//

ISR(TIMER1_OVF_vect) { 
  static word countLED;
  static word countToSecond;
  static bool phase;

  transmitter.tick();

  countLED++;
  if (countLED > ledPeriod) {
    pinLED.write(!ledBlink | phase);
    phase = !phase;
    countLED = 0;
  }

  countToSecond++;
  if (countToSecond >= fskBaudrate) {
    gFlags |= (1 << FLAG_SECOND);
    countToSecond = 0;
  }
}

