#include <string>
#include "revo_f4.h"

#include "spi.h"
#include "i2c.h"
#include "mpu6000.h"
#include "drv_mb1242.h"
#include "vcp.h"
#include "printf.h"

VCP* uartPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    uartPtr->put_byte(c);
}

int main() {

  systemInit();

//  VCP vcp;
//  vcp.init();
//  uartPtr = &vcp;

//  init_printf(NULL, _putc);


  I2C i2c;
  i2c.init(&i2c_config[EXTERNAL_I2C]);
  MB1242 sonar(i2c);


  volatile float dist;
  while(true)
  {
    sonar.async_update();
    delay(100);

    if (sonar.present())
      dist = sonar.async_read();
    delay(500);
  }

}
