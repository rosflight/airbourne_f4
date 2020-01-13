#ifndef ADC_H
#define ADC_H

#include "system.h"

class AnalogDigitalConverter
{
public:
  void init(const adc_hardware_struct_t *adc_def);
  // Adds a channel to the list that is scanned.
  // Returns the rank assigned to the channel. This is needed for reading
  // This method does not check for errors. Do not add more than CHANNEL_COUNT channels.
  // (i.e don't call add_channel more than CHANNEL_COUNT times
  uint8_t add_channel(uint8_t channel);
  // Returns a value between 0 and RAW_READING_MAX. 0 represents 0V, and RAW_READING_MAX
  // represents REFERENCE_VOLTAGE
  // The parameter, rank, is the return value from add_channel when the channel was added
  uint16_t read(uint8_t rank) const;
  bool is_initialized() const;
  // The current number of pins using this ADC. Must not exceed CHANNEL_COUNT
  uint8_t get_current_channel_count() const;

  static constexpr double REFERENCE_VOLTAGE{3.3};
  static constexpr uint16_t RAW_READING_MAX{0xFFF};
  static constexpr uint16_t NO_READING{0xFFFF};
  static constexpr uint8_t CHANNEL_COUNT{16};

private:
  const adc_hardware_struct_t *adc_def_;
  bool is_initialized_{false};
  uint8_t current_channels;

  static constexpr uint32_t SQR1_L_MASK{~0xFF0FFFFF};
  static constexpr uint8_t SQR1_L_OFFSET{20};

  volatile uint32_t buffer[CHANNEL_COUNT];

  void init_dma();
  void start_dma();
};

#endif // ADC_H
