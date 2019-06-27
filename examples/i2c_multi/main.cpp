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
#include "mb1242.h"
#include "tfmini.h"
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
//  I2C i2c1;
  I2C i2c2;
//  i2c1.init(&i2c_config[EXTERNAL_I2C]);
  i2c2.init(&i2c_config[EXTERNAL_I2C]);
  
  // Initialize the sensors
  MS5611 baro;
  HMC5883L mag;
  MS4525 airspeed;
  I2CSonar sonar;
  TFMini laser;
  
  laser.init(&i2c2);
  
  // Initialize the barometer
  float pressure(0.0), baro_temp(0.0);
  baro.init(&i2c2);
  
  // Initialize the Magnetometer
  float mag_data[3] = {0., 0., 0.};
  mag.init(&i2c2);
  
  // Initialize the airspeed Sensor
  float diff_press(0.0), as_temp(0.0);
  airspeed.init(&i2c2);

  sonar.init(&i2c2);
  
  uint32_t last_print = 0;
  while(1)
  {
    baro.update();
    mag.update();
    airspeed.update();
    sonar.update();
    laser.update();

    if (millis() < last_print + 20)
      continue;

    printf("t: %.2f, ", (double)(millis()/1000.0));


    if (baro.present())
    {
      baro.read(&pressure, &baro_temp);
      printf("b: %.2f K, ", (double)(baro_temp));
    }
    else {
      printf("b: -- K, ");
    }

    if (mag.present())
    {
      mag.read(mag_data);
      printf("m: %d, ", (int32_t)(mag_data[0]));
    }
    else
    {
       printf("m: --, ");
    }

    if (airspeed.present())
    {
      airspeed.read(&diff_press, &as_temp);
      printf("a: %.2f K, ", (double)(as_temp));
    }
    else
    {
      printf("a: -- K, ");
    }

    if (sonar.present())
    {
      float sonar_dist = sonar.read();
      printf("s: %.2f m, ", (double)(sonar_dist));
    }
    else {
      printf("s: -- m, ");
    }

    if (laser.present())
    {
      float laser_dist = laser.distance();
      printf("l: %.2f m, ", (double)(laser_dist));
    }
    else {
      printf("l: -- m, ");
    }
           
    printf("\n");
    delay(1);
  }
}
    
