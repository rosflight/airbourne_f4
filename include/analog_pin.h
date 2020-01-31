#ifndef ANALOG_H
#define ANALOG_H

#include "system.h"
#include "gpio.h"
#include "analog_digital_converter.h"

class AnalogPin
{
public:
  void init(AnalogDigitalConverter *adc_, GPIO_TypeDef *BasePort, uint16_t pin, uint8_t adc_channel);
  double read() const; // Returns a reading in volts
  uint16_t read_raw() const; // Returns a raw reading. See AnalogDigitalConverter.read()

private:
  GPIO gpio_;
  AnalogDigitalConverter *adc_;
  uint8_t rank;
};

#endif // ANALOG_H
