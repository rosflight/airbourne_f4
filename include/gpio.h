/*
 * gpio.h - Class for abstracting GPIO functionality on the STM32
 * Copyright (c) 2016 James Jackson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef GPIO_H
#define GPIO_H

#include "system.h"

class GPIO
{

public:
  typedef enum
  {
    HIGH,
    LOW
  } gpio_write_t;

  typedef enum
  {
    INPUT,
    OUTPUT,
    PERIPH_OUT,
    PERIPH_IN,
    PERIPH_IN_OUT,
    ANALOG,
    EXTERNAL_INTERRUPT
  }gpio_mode_t;


  GPIO();
  GPIO(GPIO_TypeDef *BasePort, uint16_t pin, gpio_mode_t mode);

  void init(GPIO_TypeDef* BasePort, uint16_t pin, gpio_mode_t mode);
  void write(gpio_write_t state);
  void toggle(void);
  void set_mode(gpio_mode_t mode);
  bool read();

private:
  uint16_t pin_;
  GPIO_TypeDef* port_;
  gpio_mode_t mode_;

};

#endif // GPIO_H
