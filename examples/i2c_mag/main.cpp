#include "system.h"
#include "i2c.h"
#include "hmc5883l.h"
#include "led.h"
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

  VCP vcp;
  vcp.init();
  uartPtr = &vcp;
  init_printf(NULL, _putc);

  LED warn;
  warn.init(LED1_GPIO, LED1_PIN);
  LED info;
  info.init(LED2_GPIO, LED2_PIN);

  delay(500);

  info.on();
  I2C i2c1;
  i2c1.init(&i2c_config[MAG_I2C]);
  HMC5883L mag;


  if (!mag.init(&i2c1))
  {
    warn.on();
    delay(100);
    warn.off();
  }

  float mag_data[3] = {0., 0., 0.};
  while(1) {
    info.toggle();
    mag.update();
    if (mag.read(mag_data))
    {
      warn.off();
      printf("%d, %d, %d\n",
             (int32_t)(mag_data[0]),
             (int32_t)(mag_data[1]),
             (int32_t)(mag_data[2]));

    }
    else
    {
      warn.on();
      printf("error\n");
    }
    delay(10);
  }
}
