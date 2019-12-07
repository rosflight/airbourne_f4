#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H

#include "analog_pin.h"
#include "system.h"

class BatteryMonitor
{
public:
  void init(const battery_monitor_hardware_struct_t &def, AnalogDigitalConverter *adc, float voltage_multiplier, float current_multiplier);
  float read_voltage();
  float read_current();
  void set_voltage_multiplier(double multiplier);
  void set_current_multiplier(double multiplier);
  bool has_voltage_sense();
  bool has_current_sense();
private:
  AnalogPin voltage_pin_;
  AnalogPin current_pin_;
  float voltage_multiplier_;
  float current_multiplier_;
};

#endif // BATTERYMONITOR_H
