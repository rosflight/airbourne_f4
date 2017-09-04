#include "revo_f4.h"

#include "drv_spi.h"
#include "mpu6000.h"
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
  info.on();

  int i = 0;
  while(1)
  {
    warn.toggle();
    info.toggle();
    printf("time = %d ms, %ul us\n", millis(), micros());
    delay(500);
  }
}
