#pragma once
#include "pins.h"

typedef IOPin<GPIOD, 6>  PinARM; 
typedef IOPin<GPIOB, 0>  PinLED;
typedef IOPin<GPIOD, 7>  PinBuzzer; 

const int adcChanBattery = 3;
const int adcChanTempExt = 6;
const int adcChanTempInt = 7;

const int fskBaudrate = 300;


