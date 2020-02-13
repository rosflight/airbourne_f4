#ifndef BATTERYMONITOR_H
#define BATTERYMONITOR_H

#include "analog_pin.h"
#include "system.h"

/**
 * @brief A driver for analog battery monitors
 * @details These battery monitors provide an analog voltage proportional to the battery
 * voltage and/or current. This class handles setting up pins and unit conversions.
 * This class does not initialize the ADC.
 */
class BatteryMonitor
{
public:
  /**
   * @brief Initialize the battery monitor.
   * @details Does not initialize the ADC. The ADC must be initialized before calling.
   * @param def The hardware definition struct for the battery monitor, mostly pin info
   * @param adc The ADC used for the monitor. Must be initialized already
   * @param voltage_multiplier The voltage multiplier. Set to 0 for no voltage sense.
   * See set_voltage_multiplier.
   * @param current_multiplier The current multiplier. Set to 0 for no current sense.
   * See set_current_multiplier
   */
  void init(const BatteryMonitorHardwareStruct &def, AnalogDigitalConverter *adc, float voltage_multiplier=0, float current_multiplier=0);
  /**
   * @brief Read the current battery voltage
   * @return The current battery voltage, or 0 if voltage sense is not available
   */
  float read_voltage() const;
  /**
   * @brief Read the current battery current
   * @return The current battery current, or 0 if current sense is not available
   */
  float read_current() const;
  /**
   * @brief Sets the voltage multiplier for the battery monitor.
   * @details Because the battery voltage is usually much too high for the ADC to read,
   * analog battery monitors use a voltage divider or similar to reduce the voltage to a
   * reasonable level. The voltage read by the ADC is multiplied by the voltage multiplier
   * to get the actual reading. This number can be obtained from the spec sheet for the
   * battery monitor, or through simple math for DIY monitors. This value is in volts per volt
   * (i.e. unitless).
   * @param multiplier The new voltage multiplier
   */
  void set_voltage_multiplier(double multiplier);
  /**
   * @brief Sets the current multiplier for the battery monitor.
   * @details Analog battery monitors which support current measurement generate a voltage
   * proportional to the current through them. This voltage is read by the ADC, and then
   * multiplied by the current multiplier to measure the current. This number can be obtained
   * from the spec sheet for the battery monitor. This value is in amps per volt.
   * @param multiplier
   */
  void set_current_multiplier(double multiplier);
  /**
   * @brief Checks if voltage sense is available. As there isn't hardware to detect this,
   * this simply checks if the voltage multiplier is non-zero.
   * @return If the voltage multiplier is non-zero.
   */
  bool has_voltage_sense() const;
  /**
   * @brief Checks if current sense is available. As there isn't hardware to detect this,
   * this simply checks if the current multiplier is non-zero.
   * @return If the current multiplier is non-zero.
   */
  bool has_current_sense() const;
private:
  AnalogPin voltage_pin_;
  AnalogPin current_pin_;
  float voltage_multiplier_{0};
  float current_multiplier_{0};
};

#endif // BATTERYMONITOR_H
