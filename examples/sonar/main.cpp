#include <string>
#include "revo_f4.h"

#include "spi.h"
#include "i2c.h"
#include "mpu6000.h"
#include "mb1242.h"



int main() {

  systemInit();


  I2C i2c1;
  i2c1.init(&i2c_config[EXTERNAL_I2C]);
  I2CSonar sonar(&i2c1);
  

  volatile float dist;
  while(true)
  {
    sonar.async_update();
    delay(100);
    dist=sonar.async_read();
    //I usual put a breakpoint here when testing to read the dist value
    delay(100);
  }

}
