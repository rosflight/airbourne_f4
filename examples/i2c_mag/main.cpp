#include "system.h"
#include "drv_i2c.h"
#include "hmc5883l.h"
#include "drv_led.h"
#include "vcp.h"
#include "printf.h"

VCP* vcpPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    vcpPtr->put_byte(c);
}

int main() {

  systemInit();

  VCP vcp;
  vcpPtr = &vcp;
  init_printf(NULL, _putc);

  LED warn(LED1_GPIO, LED1_PIN);
  LED info(LED2_GPIO, LED2_PIN);

  delay(500);

  info.on();
  I2C i2c1(I2C2);
  HMC5883L mag(&i2c1);

  if (!mag.init()) {
    warn.on();
    delay(100);
    warn.off();
  }

  float mag_data[3] = {0., 0., 0.};
  while(1) {
    info.toggle();
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
