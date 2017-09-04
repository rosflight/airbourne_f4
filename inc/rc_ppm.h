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
  uint16_t rc_raw_[PWM_NUM_RC_INPUTS];
  uint32_t last_pulse_ms_;

public:
  void init();
  float read(uint8_t channel);
  void pulse_callback();
  bool lost();
};

extern RC_PPM* RC_PPM_Ptr;

#endif // RC_PPM_H
