#include "revo_f4.h"

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
  info.off();
  warn.on();

  int i = 0;
  while(1)
  {
    warn.toggle();
    info.toggle();
    printf("time = %d s, %d ms, %u us\n", i++, millis(), micros());
    delay(1000);
  }
}
