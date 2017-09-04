#ifndef RC_H
#define RC_H

#include "revo_f4.h"

#include "gpio.h"

class RC
{
public:
  typedef enum
  {
    PARALLEL_PWM,
    PPM,
  } RC_type_t;

private:
  uint32_t pulse_[PWM_NUM_RC_INPUTS];

public:
  virtual void init() = 0;
  virtual float read(uint8_t channel) = 0;
};

#endif // RC_H
