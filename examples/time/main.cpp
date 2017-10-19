#include "revo_f4.h"

#include "drv_spi.h"
#include "mpu6000.h"
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

  LED warn;
  warn.init(LED1_GPIO, LED1_PIN);
  LED info;
  info.init(LED2_GPIO, LED2_PIN);
  info.on();

  int i = 0;
  int delays[6] = {1000000, 100, 5000, 500000, 70000, 1000};
  while(1)
  {
    warn.toggle();
    info.toggle();
    printf("time = %d ms, %ul us\n", millis(), micros());
    delayMicroseconds(delays[i++ % 6]);
  }
}
