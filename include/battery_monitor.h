#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H

#include "analog_pin.h"

class BatteryMonitor
{
public:
  void init(AnalogPin *voltage_pin, double voltage_multiplier, AnalogPin *current_pin, double current_multiplier);
  void init(AnalogPin *voltage_pin, double voltage_multiplier);
  double read_voltage();
  double read_current();
  void set_voltage_multiplier(double multiplier);
  void set_current_multiplier(double multiplier);
  bool has_voltage_sense();
  bool has_current_sense();
private:
  AnalogPin *voltage_pin_;
  AnalogPin *current_pin_;
  double voltage_multiplier_;
  double current_multiplier_;
};

#endif // BATTERYMONITOR_H
