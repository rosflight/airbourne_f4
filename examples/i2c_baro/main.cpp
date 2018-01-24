#include "system.h"
#include "i2c.h"
#include "ms5611.h"
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
  i2c1.init(&i2c_config[MS5611_I2C]);
  MS5611 baro;

  if (!baro.init(&i2c1))
  {
    while(1)
    {
      warn.toggle();
      delay(100);
    }
  }

  float pressure, temperature;
  while(1)
  {
    info.toggle();
    baro.update();
    baro.read(&pressure, &temperature);
    printf("%d Pa, %d.%d K\n",
           (int32_t)(pressure),
           (int32_t)(temperature),
           (int32_t)(temperature*100)%100);

    delay(10);
  }
}
