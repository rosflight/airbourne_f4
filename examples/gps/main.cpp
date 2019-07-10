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

#include "serial.h"
#include "system.h"
#include "uart.h"
#include "vcp.h"
#include "ublox.h"
#include "revo_f4.h"
#include "printf.h"
#include "led.h"

Serial *serPtr = NULL;

static void _putc(void *p, char c)
{
  (void)p; // avoid compiler warning about unused variable
  serPtr->put_byte(c);
}

int main()
{
  systemInit();

  VCP vcp;
  vcp.init();
  serPtr = &vcp;

  UART uart;
  uart.init(&uart_config[UART1], 115200);

  init_printf(NULL, _putc);

  UBLOX gps;
  gps.init(&uart);

  LED led1;
  led1.init(LED1_GPIO, LED1_PIN);

  while (1)
  {
    if (gps.new_data())
    {
      UBLOX::NAV_PVT_t raw = gps.read_raw();
      printf("fix: %s\tt: %d\tlla: %d, %d, %d\tvel: %d, %d, %d\n",
             fix_names[raw.fixType].c_str(), raw.iTOW,
             raw.lat, raw.lon, raw.height,
             raw.velN, raw.velE, raw.velD);
      led1.toggle();
    }
    delay(10);
  }
}
