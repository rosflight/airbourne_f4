#include "battery_monitor.h"

void BatteryMonitor::init(const battery_monitor_hardware_struct_t &def, AnalogDigitalConverter *adc, float voltage_multiplier, float current_multiplier)
{
  voltage_pin_.init(adc, def.voltage_gpio, def.voltage_pin, def.voltage_adc_channel);
  current_pin_.init(adc, def.current_gpio, def.current_pin, def.current_adc_channel);
  voltage_multiplier_ = voltage_multiplier;
  current_multiplier_ = current_multiplier;
}
float BatteryMonitor::read_voltage()
{
  return static_cast<float>(this->voltage_pin_.read()) * this->voltage_multiplier_;
}
float BatteryMonitor::read_current()
{
  return static_cast<float>(this->current_pin_.read()) * this->current_multiplier_;
}

void BatteryMonitor::set_voltage_multiplier(double multiplier)
{
  this->voltage_multiplier_  = multiplier;
}

void BatteryMonitor::set_current_multiplier(double multiplier)
{
  this->current_multiplier_ = multiplier;
}

bool BatteryMonitor::has_voltage_sense()
{
  return (this->voltage_multiplier_ != 0);
}

bool BatteryMonitor::has_current_sense()
{
  return (this->current_multiplier_ != 0);
}
