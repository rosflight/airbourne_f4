#include "analog_pin.h"

void AnalogPin::init(AnalogDigitalConverter *adc, GPIO_TypeDef *basePort, uint16_t pin, uint8_t adc_channel)
{
  this->adc_ = adc;
  this->gpio_.init(basePort, pin, GPIO::ANALOG);
  this->rank = this->adc_->add_channel(adc_channel);
}

double AnalogPin::read() const
{
  return read_raw() * AnalogDigitalConverter::REFERENCE_VOLTAGE / AnalogDigitalConverter::RAW_READING_MAX;
}

uint16_t AnalogPin::read_raw() const
{
  return adc_->read(rank);
}
