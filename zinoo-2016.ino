//#include <EEPROM.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

#include "gps.h"
#include "ukhas.h"
#include "fstring.h"
#include "crc.h"
#include "fsk.h"
#include "adc.h"
#include "pins.h"
#include "MS5607.h"
#include "buzzer.h"

#include "config.h"

/*
 * Code currently supports Leonardo and Pro Mini (328P @ 3.3V)
 */

const char compile_date[] = __DATE__ " " __TIME__;


enum {
  FLAG_SECOND,
  FLAG_MINUTE
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

// This just reserves space in EEPROM for the non-volatile configuration
Config EEMEM Config::nvConfig;    
Config Config::config;  // this is volatile configuration in RAM

ResetState EEMEM nvResetState;      // Last stored GPS fix info
ResetState gResetState;

FlightData flightData;  // current payload data

I2C bus;
BarometerSimple barometer(bus);

GPSParser gpsParser;
UKHASPacketizer packetizer;
FSKTransmitter transmitter;

volatile byte gFlags;

char        gTime[6];

uint16_t    gTimeSinceReset;
State       gState;
TXState     gTXState;
uint16_t    gTimeToFix;
uint8_t     gCameraStatus;

PinARM    pinARM;

void testPressureFormula() {
  Serial.print(F("Pressure"));
  Serial.print('\t');
  Serial.println(F("Altitude"));
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
  buzzerBegin();
  ledBegin();

  buzzerBeep();

  // Try to start USB (necessary for Leonardo board)
  int startTime = millis();
  while (!Serial) {
    // Check timeout
    if (millis() - startTime > 3000) {
      break;
    }
  }
  Serial.begin(9600);
  Serial.println(F("Reset"));

  // Set ADC reference from AVCC, set clock prescaler 1:16
  adcSetup(ADCRefSupply, 4);

  // Configure Timer1 and enable interrupts at FSK baudrate
  byte mode = 14;   // Fast PWM with TOP=ICR1
  byte presc = 3;   // Prescaler /64
  TCCR1A = (mode & 3);
  TCCR1B = ((mode & 12) << 1) | presc;
  ICR1   = (F_CPU / 64 / fskBaudrate) - 1;
  TIMSK1 = _BV(TOIE1);

  // Initialize GPS software serial receiver
  gpsBegin();

  bus.setSlow();  

  // Restore configuration from EEPROM
  Config::restore();
  packetizer.setPayloadName(Config::config.payloadName);

  // Read persistent state
  eeprom_read_block((void *)&gResetState, &nvResetState, sizeof(ResetState));

  // Enable interrupts
  sei();

  // Print some debug info
  Config::config.payloadName.print();
  Serial.print(F(" ver. "));
  Serial.println(compile_date);

  // Try to initialize barometer
  if (barometer.initialize()) {
    Serial.println(F("Barometer OK"));
  }
  else {
    Serial.println(F("No barometer found"));
  }

  // Check CRC algorithm validity
  CRC16_CCITT crc;
  uint16_t checksum = crc.update("habitat");
  Serial.println((checksum == 0x3EFB) ? F("CRC ok") : F("CRC error"));

  // Check safe/arm pin
  if (pinARM.read() == 0) {
    // Pin removed

    // Increase reset counter
    gResetState.resetCount++;
    eeprom_update_block((void *)&gResetState, &nvResetState, sizeof(ResetState));

    Serial.print(F("Reset #")); Serial.print(gResetState.resetCount);
    Serial.println(F("... restoring state from EEPROM"));
    if (gResetState.sentenceID != 0xFFFF) {
      packetizer.sentenceID = gResetState.sentenceID + 50;      // Allow for some number of transmitted packets
      Serial.print(F("Sentence ID: ")); Serial.println(packetizer.sentenceID);
      gResetState.gpsInfo.print();
      flightData.updateGPS(gResetState.gpsInfo);
    }
  }
  else {
    // Pin present
  }  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
// 
// Reads and parses GPS data, reads and parses console input and executes commands,
// checks time, builds UKHAS packet and initiates radio transmission of it
//

void cameraCommand(uint8_t command) {
  /*
  0 - rec/#off (R/W)
  1 - error (RO)
  2 - offline (RO)
  3 - charging(RO)
  4 -
  5 -
  6 -
  7 - executing command (RO)
  */

  bus.write(0x23, &command, 1);
}

void cameraRecord() {
  cameraCommand(0x01);
}

void cameraStop() {
  cameraCommand(0x00);
}

uint8_t cameraStatus() {
  uint8_t status;
  bus.read(0x23, &status, 1);
  return status;
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
    kCommandCam,
    kCommandStat,
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
      else if (strncmp(ptr1, "tx", len) == 0) {
        cmd = kCommandTX;
      }      
      else if (strncmp(ptr1, "stat", len) == 0) {
        cmd = kCommandStat;
      }      
      else if (strncmp(ptr1, "save", len) == 0) {
        cmd = kCommandSave;
      }
      else if (strncmp(ptr1, "ver", len) == 0) {
        cmd = kCommandVer;
      }      
      else if (strncmp(ptr1, "cam", len) == 0) {
        cmd = kCommandCam;
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
        case kCommandCam: 
          if (len == 1) {
            if (*ptr1 == '0') {
              cameraStop();
              success = true;
            }
            else if (*ptr1 == '1') {
              cameraRecord();
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
      case kCommandId: Config::config.payloadName.print(); Serial.println(); break;     
      case kCommandTX: Serial.println(Config::config.enableTX); break;     
      case kCommandPkt: 
        packetizer.makePacket(flightData);
        Serial.print((const char *)packetizer.getPacketBuffer()); 
        break;
      case kCommandStat:       
        Serial.print("Uptime: "); Serial.println(gTimeSinceReset);
        gpsParser.gpsInfo.print();
        flightData.print();
        break;
      case kCommandCam: 
        Serial.println(cameraStatus(), HEX);
        break;
      default:
        Serial.println("?");
    }
  }
}


void oncePerSecond() {

  static uint8_t camCount;
    if (++camCount >= 5) {
      camCount = 0;

      gCameraStatus = cameraStatus();
      
      // Check safe/arm pin
      if (pinARM.read() == 0) {    
        // Try to start camera recording
        if (0 == (gCameraStatus & 1)) {
          cameraRecord();
        }      
      }
      else {
        // Try to stop camera recording
        if (1 == (gCameraStatus & 1)) {
          cameraStop();
        }      
      }

      if (barometer.update()) {
        flightData.pressure = barometer.getPressure();
        flightData.barometricAltitude = barometer.getAltitude();
      }
    }

  // Check safe/arm pin
  if (pinARM.read() == 0) {
    // Pin removed
    if (gState != STATE_FLIGHT) {
      Serial.println(F("STATE -> FLIGHT"));
      gState = STATE_FLIGHT;
    }

    if (flightData.altitude < kAltitudeThreshold) {
      buzzerSet(kBuzzerPattern1, 60);       // Retrieval pattern
      ledSet(kLEDPattern2, 60);             // Slow short flashing
    }
    else {
      ledSet(0, 0);             // No flashing of green LED above threshold
      buzzerSet(0, 0);          // Silence
    }
  }
  else {
    // Pin present
    if (gState != STATE_SAFE) {
      Serial.println(F("STATE -> SAFE"));
      gState = STATE_SAFE;
      gTimeToFix = 0;
      
      gResetState.clear();
      packetizer.sentenceID = gResetState.sentenceID;
    }

    // Indicate fix by flashing LED
    if (gpsParser.gpsInfo.fix == '3') {
      // 3D fix
      gTimeToFix = 0;
      ledSet(kLEDPattern1, 30);  // Slow 50% flashing
    }
    else {
      // No GPS fix or 2D fix
      gTimeToFix++;
      ledSet(kLEDPattern1, 5);   // Fast 50% flashing
    }

    // Check for GPS fix timeout
    if (gTimeToFix > kFixTimeout) {
      // Indicate error
      buzzerSet(kBuzzerPattern2, 30); 
    }
    else {
      buzzerSet(0, 0);           // Silence      
    }
  }

  // Check time since reset and reflect it in flight status
  if (gState == STATE_FLIGHT && gTimeSinceReset < 600) {
    flightData.status |= _BV(kStatusAfterReset);
  }
  else {
    flightData.status &= ~_BV(kStatusAfterReset);
  }

  // Check GPS fix
  if (gpsParser.gpsInfo.fix != '3') {
    flightData.status |= _BV(kStatusNoGPSLock);
  }
  else {
    flightData.status &= ~_BV(kStatusNoGPSLock);
  }
  
  // Check camera
  if (gState == STATE_FLIGHT && 0 == (gCameraStatus & 1)) {
    flightData.status |= _BV(kStatusCameraOff);
  }
  else {
    flightData.status &= ~_BV(kStatusCameraOff);
  }

  flightData.updateGPS( gpsParser.gpsInfo );
  flightData.updateTemperature();


  // Check TX time division
  int8_t minutes = flightData.getMinutes();
  int8_t seconds = flightData.getSeconds();
  if (minutes >= 0) {     // Check for valid time
    uint8_t myId = Config::getMyID(); // Get the last character of payload name
    bool myTurn = myId == 0 || (minutes % Config::getTimeSlots() == (myId - 1));

    if (myTurn || gState == STATE_SAFE) {
      if (gTXState == TXSTATE_IDLE) {
        gTXState = TXSTATE_CARRIER;
        if (Config::config.enableTX) transmitter.enable();
      }
      else if (gTXState == TXSTATE_CARRIER && seconds >= 10) {
        gTXState = TXSTATE_TRANSMIT;
      }
    }
    else {
      gTXState = TXSTATE_IDLE;
    }
  }
  else {
    gTXState = TXSTATE_IDLE;
  }

  
}


void loop() 
{
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_mode();
  sleep_disable();

  // Read GPS serial data
  while (gpsAvailable() > 0) {
    char c = gpsRead();
    //Serial.print(c);
    gpsParser.parse(c);
  }

  static FString<16> consoleCmd;

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

  // Check radio transmitter state and queue new data if necessary
  switch (gTXState) {
    case TXSTATE_IDLE:
      if (!transmitter.isBusy()) {
        transmitter.disable();
      }
      break;

    case TXSTATE_TRANSMIT:
      if (Config::config.enableTX && !transmitter.isBusy()) {       
        packetizer.makePacket(flightData);
        //Serial.print((const char *)packetizer.getPacketBuffer());
        transmitter.transmit(packetizer.getPacketBuffer(), packetizer.getPacketLength() - 1);
      }
      break;
    case TXSTATE_TEST:
      if (!transmitter.isBusy()) {     
        transmitter.transmit(txTestString, txTestStringLength);
      }
      break;
  }

  // Check flags
  if (gFlags & (1 << FLAG_SECOND)) {
    gFlags &= ~(1 << FLAG_SECOND);

    oncePerSecond();
  }
  if (gFlags & (1 << FLAG_MINUTE)) {
    gFlags &= ~(1 << FLAG_MINUTE);

    // Store last GPS lock info
    Serial.println("Saving state info to EEPROM");
    if (gpsParser.gpsInfo.fix == '3' || gpsParser.gpsInfo.fix == '2') {
      gpsParser.gpsInfo.print();
      gResetState.gpsInfo = gpsParser.gpsInfo;
    }
    gResetState.sentenceID = packetizer.sentenceID;
    eeprom_update_block((void *)&gResetState, &nvResetState, sizeof(ResetState));
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer1 overflow vector
//
// Responsible for FSK transmitter tick at every transmitted bit,
// flashing LEDs and keeping track of time
//

ISR(TIMER1_OVF_vect) { 
  static word countToSecond;
  static byte countToMinute;

  transmitter.tick();
  buzzerTick();
  ledTick();

  if (++countToSecond >= fskBaudrate) {
    gFlags |= (1 << FLAG_SECOND);
    countToSecond = 0;

    gTimeSinceReset++;

    if (++countToMinute >= 60) {
      countToMinute = 0;
      gFlags |= (1 << FLAG_MINUTE);
    }
  }
}

