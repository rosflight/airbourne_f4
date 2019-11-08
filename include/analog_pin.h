#ifndef ANALOG_H
#define ANALOG_H

#endif // ANALOG_H

#include "system.h"
#include "gpio.h"
#include "analog_digital_converter.h"

class AnalogPin
{
public:
  void init(AnalogDigitalConverter *adc_, GPIO_TypeDef *BasePort, uint16_t pin, uint8_t adc_channel);
  double read();
  uint16_t read_raw();

private:
  GPIO gpio_;
  AnalogDigitalConverter *adc_;
  uint8_t rank;
};

