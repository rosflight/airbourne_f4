#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "vector3.h"

class IMU
{
public:
  IMU(){}
  virtual bool read_all(vector3* accel, vector3* gyro, float* temp) = 0;
  virtual void request_update() = 0;

  virtual void register_external_interrupt_callback(void (*CB)(void)) = 0;
  virtual void register_new_data_callback(void (*CB)()) = 0;
};

#endif // IMU_H
