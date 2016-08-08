#pragma once
#include <avr/io.h>

enum ADCReference {
  ADCRefExternal  = 0,
  ADCRefSupply    = 1,
  ADCRefInternal  = 3
};

enum ADCPrescaler {
  
};


void adcSetup(ADCReference reference, uint8_t prescaler);

uint16_t adcRead(uint8_t channel);

