#include "system.h"
#include "i2c.h"
#include "ms4525.h"
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
  i2c1.init(&i2c_config[EXTERNAL_I2C]);
  MS4525 airspeed;


  if (!airspeed.init(&i2c1))
  {
    warn.on();
    delay(100);
    warn.off();
  }

  float diff_press, temp;
  while(1) {
    info.toggle();
    airspeed.update();
    if (airspeed.present())
    {
      airspeed.read(&diff_press, &temp);
      warn.off();
      printf("%d.%dPa, %d.%dC\n",
             (int32_t)(diff_press), (int32_t)(diff_press*1000)%1000,
             (int32_t)(temp), (int32_t)(temp*1000)%1000);
    }
    else
    {
      warn.on();
      printf("error\n");
    }
    delay(50);
  }
}
