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
#include "uart.h"

UART* uartPtr = NULL;

void rx_callback(uint8_t byte)
{
  uartPtr->put_byte(byte);
}

int main()
{
  systemInit();

  UART uart(USART1);
  uartPtr = &uart;

  uart.register_rx_callback(rx_callback);  // Uncomment to test callback version

  while(1)
  {
    uint8_t hello_string[9] = "testing\n";
//    uart.write(hello_string, 8); // Uncomment to test Tx
    delay(200);

    // Polling version (uncomment to test)
//    while (uart.rx_bytes_waiting())
//    {
//      uint8_t byte = uart.read_byte();
//      uartPtr->put_byte(byte);
//    }

  }
}
