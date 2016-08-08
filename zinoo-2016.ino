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

const char PROGMEM compile_date[] = __DATE__ " " __TIME__;

enum DebugMode {
  DbgConsole,
  DbgNMEA,
  DbgUKHAS
};

struct PersistentConfig {
  uint16_t  id;

  char      payloadName[6];

  // Last stored GPS fix info
  uint16_t  lastAltitude;
  char      lastLatitude[7];
  char      lastLongitude[7];
  char      lastTime[6]; 

  uint8_t disableTX       : 1;
  uint8_t disableLastFix  : 1;
  
  DebugMode debugMode;    // Debug console mode

  PersistentConfig() {
  }

  void defaults() {
    id = 0;
    strncpy(payloadName, "ZGND", 6);
    lastAltitude = 0;
    lastLatitude[0] = '\0';
    lastLongitude[0] = '\0';
    memcpy(lastTime, "000000", 6);

    disableTX = true;
    disableLastFix = false;
    
    debugMode = DbgConsole;    
  }

  void save() {
    eeprom_write_block(&nvPersistentConfig, (void *)this, sizeof(*this));
    id++;
  }

  void restore() {
    eeprom_read_block((void *)this, &nvPersistentConfig, sizeof(*this));
    if (id == 0xFFFF) defaults();
  }

  static PersistentConfig EEMEM nvPersistentConfig;
};

enum {
  FLAG_SECOND
};

enum State {
  STATE_RESET,
  STATE_SAFE,
  STATE_FLIGHT
};


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variables
// 
// 

bool ledBlink = false;
word ledPeriod = 1;

// This just reserves space in EEPROM for the non-volatile configuration
PersistentConfig EEMEM PersistentConfig::nvPersistentConfig;    
PersistentConfig config;  // this is volatile configuration in RAM

FlightData flightData;  // current payload data

I2C bus;
BarometerSimple barometer(bus);

GPSParser gpsParser;
UKHASPacketizer packetizer;
FSKTransmitter transmitter;

volatile byte gFlags;

char        gTime[6];
State       gState;

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

  config.restore();
  packetizer.setPayloadName(config.payloadName);

  // Enable interrupts
  sei();
  
  Serial.println("Reset");

  // Check CRC algorithm validity
  CRC16_CCITT crc;
  uint16_t checksum = crc.update("habitat");
  Serial.print("CRC test value (should be 3EFB): ");
  Serial.println(checksum, HEX);

  barometer.initialize();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
// 
// Reads and parses GPS data, reads and parses console input and executes commands,
// checks time, builds UKHAS packet and initiates radio transmission of it
//

FString<32> consoleCmd;

void parseCommand() {
  
}

void loop() 
{
  // Read GPS serial data
  while (gpsAvailable() > 0) {
    char c = gpsRead();
    //Serial.print(c);
    gpsParser.parse(c);
  }

  // Read serial console data
  while (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print(c);
    if (c != 0x0D && c != 0x0A) {
      consoleCmd.append(c);
    }
    else {
      parseCommand();
    }
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
      barometer.update();
      Serial.print("P: ");
      Serial.print((uint32_t)barometer.getPressure() * 4);
      Serial.print(" H: ");
      Serial.print(barometer.getAltitude());
      Serial.print(' ');

      //gpsParser.gpsInfo.print();
      flightData.updateGPS( gpsParser.gpsInfo );
      flightData.updateTemperature();

      packetizer.makePacket(flightData);
      //if (fix == '3') 
      {
        //transmitter.transmit(packetizer.getPacketBuffer(), packetizer.getPacketLength() - 1);
        //transmitter.transmit((const uint8_t *)"TX 0123456789\n", 11);
      }
    }
  }
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

