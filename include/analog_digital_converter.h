#ifndef ADC_H
#define ADC_H

#include "system.h"

class AnalogDigitalConverter
{
public:
  void init(const adc_hardware_struct_t *adc_def);
  uint8_t add_channel(uint8_t channel); // returns the rank of the added channel
  uint16_t read(uint8_t rank);
  bool is_initialized();

  static constexpr double REFERENCE_VOLTAGE{3.3};
  static constexpr uint16_t RAW_READING_MAX{0xFFF};

private:
  const adc_hardware_struct_t *adc_def_;
  bool is_initialized_{false};
  uint8_t current_channels;

  static constexpr uint8_t CHANNEL_COUNT{16};
  static constexpr uint32_t SQR1_L_MASK{~0xFF0FFFFF};
  static constexpr uint8_t SQR1_L_OFFSET{20};
  static constexpr uint16_t NO_READING{0xFFFF};

  volatile uint32_t buffer[CHANNEL_COUNT];

  void init_dma();
  uint8_t get_current_channel_count();
  void start_dma();


};

#endif // ADC_H
