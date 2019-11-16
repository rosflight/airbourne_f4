#include "battery_monitor.h"


void BatteryMonitor::init(AnalogPin *voltage_pin, double voltage_multiplier, AnalogPin *current_pin,
                          double current_multiplier)
{
  this->voltage_pin_ = voltage_pin;
  this->voltage_multiplier_ = voltage_multiplier;
  this->current_pin_ = current_pin;
  this->current_multiplier_ = current_multiplier;
}
void BatteryMonitor::init(AnalogPin *voltage_pin, double voltage_multiplier)
{
  this->voltage_pin_ = voltage_pin;
  this->voltage_multiplier_ = voltage_multiplier;
  this->current_pin_ = nullptr;
  this->current_multiplier_ = 0;
}
double BatteryMonitor::read_voltage()
{
  return this->voltage_pin_->read() * this->voltage_multiplier_;
}
double BatteryMonitor::read_current()
{
  if(this->current_pin == nullptr)
    return 0;
  else
    return this->current_pin_->read() * this->current_multiplier_;
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
