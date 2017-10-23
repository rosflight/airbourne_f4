#include "drv_mb1242.h"

void mb1242_init(I2C& i2cIn)
{
  i2c=&i2cIn;
    new_data=0;
  value=0;
  last_update=millis()-UPDATE_WAIT_MILLIS;//this makes it so that async_update can be called right away
    ready_to_read=1;
}
void mb1242_async_update()
{
  uint64_t now=millis();
  if (now>last_update+UPDATE_WAIT_MILLIS)
  {
        last_update=now;
    if (ready_to_read)
      i2c->write(DEFAULT_ADDRESS, DEFAULT_REGISTER, READ_COMMAND, cb_start_read);
    else
            i2c->read(DEFAULT_ADDRESS, DEFAULT_REGISTER, 2, buffer, cb_finished_read, true);
  }

}
float mb1242_async_read()
{
    mb1242_async_update();
  if (new_data)
  {
    uint16_t centimeters=buffer[1]<<8|buffer[0];//Convert to a single number
    //Calibration from BreezySTM32 by Simon D. Levy 
    value=(1.071*(float)centimeters+3.103)/100.0;
  }
  return value;
}
void cb_start_read()
{
    ready_to_read=0;
}
void cb_finished_read()
{
    new_data=1;
    ready_to_read=1;
}
