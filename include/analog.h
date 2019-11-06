#ifndef ANALOG_H
#define ANALOG_H

#endif // ANALOG_H

#include "system.h"
#include "gpio.h"

class Analog
{
public:
  void init(ADC_TypeDef *adc, GPIO_TypeDef *BasePort, uint16_t pin, uint8_t adc_channel);
  double read();
  uint16_t read_raw();

  static constexpr double REFERENCE_VOLTAGE{3.3};
  static constexpr uint16_t UINT12_MAX{0xFFF};
private:
  GPIO gpio_;
  static bool is_adc_initialized(ADC_TypeDef *adc);
  static bool init_adc(ADC_TypeDef *adc);
  static uint8_t get_current_channel_count(ADC_TypeDef *adc);
  static void increment_channel_count(ADC_TypeDef *adc);

  static constexpr uint32_t SQR1_L_MASK{~0xFF0FFFFF};
  static constexpr uint8_t SQR1_L_OFFSET{20};
};

