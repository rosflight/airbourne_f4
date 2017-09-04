/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes */
#include "system.h"
#include "vcp.h"
#include "printf.h"
#include "rc_ppm.h"
#include "drv_led.h"

VCP* vcpPtr = NULL;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    vcpPtr->put_byte(c);
}

int main() {

  systemInit();

//  VCP vcp;
//  vcpPtr = &vcp;

//  init_printf(NULL, _putc);

  LED warn(LED1_GPIO, LED1_PIN);
  LED info(LED2_GPIO, LED2_PIN);

  RC_PPM rc;

  rc.init();

  while(1)
  {
    if (rc.lost())
    {
      info.off();
      warn.on();
//      printf("rc lost\n");
    }
    else
    {
      warn.off();
      info.on();
//      printf("%d, %d, %d, %d, %d, %d\n",
//             (uint32_t)(1000*rc.read(0)),
//             (uint32_t)(1000*rc.read(1)),
//             (uint32_t)(1000*rc.read(2)),
//             (uint32_t)(1000*rc.read(3)),
//             (uint32_t)(1000*rc.read(4)),
//             (uint32_t)(1000*rc.read(5)));
    }
  }
  delay(20);
}
