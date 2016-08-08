#include "adc.h"

void adcSetup(ADCReference reference, uint8_t prescaler) {
  ADCSRA = _BV(ADEN) | (prescaler & 0x07);
  ADMUX = (ADMUX & 0x0F) | (((uint8_t)reference & 0x03) << 6);
}

uint16_t adcRead(uint8_t channel) {
  ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);
  ADCSRA = (ADCSRA & 0x0F) | _BV(ADEN) | _BV(ADSC) | _BV(ADIF);
  ADCSRB = (ADCSRB & 0xDF) | (channel & 0x20);
  while (!(_BV(ADIF) & ADCSRA)) {
    // Do nothing and wait for conversion to finish
  }
  return ADC;
}

