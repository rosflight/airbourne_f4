#include "system.h"
#include "drv_led.h"

int main()
{
  systemInit();

  LED warn(LED1_GPIO, LED1_PIN);
  LED info(LED2_GPIO, LED2_PIN);

  warn.on();
  while(1)
  {
    delay(200);
    warn.toggle();
    info.toggle();
  }
}
