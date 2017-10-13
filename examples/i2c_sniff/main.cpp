#include "system.h"
#include "drv_i2c.h"
#include "drv_led.h"
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
  uartPtr = &vcp;
  init_printf(NULL, _putc);

  LED warn(LED1_GPIO, LED1_PIN);
  LED info(LED2_GPIO, LED2_PIN);

  info.on();
  I2C i2c1(I2C2);

  while(1)
  {
    for (int i = 0; i < 128; i++)
    {
      uint8_t data = 0;
      if (i2c1.write(i, 0xFF, data) > 0)
      {
        printf("found device at 0x%X\n", i);
      }
      delay(1);
    }
    printf("-------------------------\n");
    delay(100);
  }
}