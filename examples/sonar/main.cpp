#include <string>
#include "revo_f4.h"

#include "drv_spi.h"
#include "drv_i2c.h"
#include "mpu6000.h"
#include "drv_led.h"
#include "drv_mb1242.h"



int main() {

  systemInit();

  LED warn(LED1_GPIO, LED1_PIN);
  LED info(LED2_GPIO, LED2_PIN);

  delay(500);

  info.on();
  I2C i2c1(I2C2);
  I2CSonar sonar(i2c1);
  

  volatile float dist;
  while(true)
  {
    sonar.async_update();
    delay(100);
    dist=sonar.async_read();
    delay(500);
  }

}
