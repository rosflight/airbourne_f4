#ifndef RC_PPM_H
#define RC_PPM_H

#include "rc.h"

class RC_PPM
{
private:
  GPIO pin_;
  uint8_t chan_ = 0;
  uint16_t current_capture_ = 0;
  uint16_t last_capture_ = 0;
  uint16_t rc_raw_[8];
  uint32_t last_pulse_ms_;

public:
  void init();
  float read(uint8_t channel);
  void pulse_callback();
  bool lost();
};

#endif // RC_PPM_H
