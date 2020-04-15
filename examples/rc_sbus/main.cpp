/*
 * Copyright (c) 2017, James Jackson
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "printf.h"
#include "rc_sbus.h"
#include "revo_f4.h"
#include "system.h"
#include "uart.h"
#include "vcp.h"

VCP* uartPtr = NULL;

static void _putc(void* p, char c)
{
  (void)p; // avoid compiler warning about unused variable
  uartPtr->put_byte(c);
}

int main()
{
  static GPIO inv_pin;
  static UART uart;
  static RC_SBUS rc;
  static VCP vcp;

  systemInit();
  vcp.init();
  uartPtr = &vcp;
  init_printf(NULL, _putc);

  uart.init(&uart_config[0], 100000, UART::MODE_8E2);
  inv_pin.init(SBUS_INV_GPIO, SBUS_INV_PIN, GPIO::OUTPUT);
  rc.init(&inv_pin, &uart);

  float rc_raw[16];
  while (1)
  {
    for (int i = 0; i < 8; i++)
    {
      rc_raw[i] = rc.read(i);
      printf("%d, ", (uint32_t)(rc_raw[i] * 1000));
    }
    printf("\n");
    delay(20);
  }
}
