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

#include "system.h"
#include "i2c.h"
#include "ms5611.h"
#include "hmc5883l.h"
#include "ms4525.h"
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
  
  delay(500);
  
  info.on();
  
  // Initialize the I2C peripherals1
  I2C i2c1;
  I2C i2c2;
  i2c1.init(&i2c_config[BARO_I2C]);
  i2c2.init(&i2c_config[EXTERNAL_I2C]);
  
  // Initialize the sensors
  MS5611 baro;
  HMC5883L mag;
  MS4525 airspeed;
  
  
  // Initialize the barometer
  float pressure, temperature;
  if (!baro.init(&i2c1))
  {
    while(1)
    {
      warn.toggle();
      delay(100);
    }
  }
  
  // Initialize the Magnetometer
  float mag_data[3] = {0., 0., 0.};
  if (!mag.init(&i2c1))
  {
    while(1)
    {
      warn.toggle();
      delay(100);
    }
  }
  
  // Initialize the airspeed Sensor
  float diff_press, temp;
  if (!airspeed.init(&i2c2))
  {
    while (1)
    {
      warn.toggle();
      delay(100);
    }
  }
  

  while(1)
  {
    info.toggle();
    mag.update();
    baro.update();
    airspeed.update();
    baro.read(&pressure, &temperature);
    mag.read(mag_data);
    airspeed.read(&diff_press, &temp);
    printf("%d Pa, %d.%d K\n",
           (int32_t)(pressure),
           (int32_t)(temperature),
           (int32_t)(temperature*100)%100);
    
    delay(10);
  }
}
