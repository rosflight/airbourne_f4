#include "system.h"
#include "i2c.h"
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

  warn.on();
  I2C i2c[NUM_I2C];
  for (int i = 0; i < NUM_I2C; i++)
  {
    i2c[i].init(&i2c_config[i]);
  }
  warn.off();

  while(1)
  {
    info.toggle();
    for (int i = 0; i < NUM_I2C; i++)
    {
      for (int j = 0; j < 128; j++)
      {
        uint8_t data = 0;
        if (i2c[i].write(j, 0xFF, data) > 0)
        {
          printf("I2C%d: found device at 0x%X\n", i+1, j);
        }
        delay(1);
      }
    }
    printf("--------------------------\n");
    delay(100);
  }
}
