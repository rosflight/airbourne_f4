#include "analog_pin.h"

void AnalogPin::init(AnalogDigitalConverter *adc, GPIO_TypeDef *BasePort, uint16_t pin, uint8_t adc_channel)
{
  this->adc_ = adc;
  this->gpio_.init(BasePort, pin, GPIO::ANALOG);
  this->rank = this->adc_->add_channel(adc_channel);
}

double AnalogPin::read()
{
  return this->read_raw() * AnalogDigitalConverter::REFERENCE_VOLTAGE / AnalogDigitalConverter::RAW_READING_MAX;
}

uint16_t AnalogPin::read_raw()
{
  return this->adc_->read(this->rank);
}
