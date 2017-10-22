#include "revo_f4.h"
#include "drv_led.h"

int main()
{
  systemInit();

  LED warn;
  warn.init(LED1_GPIO, LED1_PIN);
  LED info;
  info.init(LED2_GPIO, LED2_PIN);

  warn.off();
  while(1)
  {
    delay(200);
    warn.toggle();
    info.toggle();
  }
}
