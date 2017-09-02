
/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SYSTEM_H
#define SYSTEM_H

#define ARM_MATH_CM4

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <stdarg.h>

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

typedef struct {
	GPIO_TypeDef* gpio;
	uint16_t gpio_pin;
	uint8_t gpio_pin_source;
	TIM_TypeDef* tim;
	uint8_t tim_channel;
	uint8_t tim_af_config;
} pwm_hardware_struct_t;

#ifdef __cplusplus
extern "C" {
#endif

void systemInit(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint32_t micros(void);
uint32_t millis(void);

// bootloader/IAP
void systemReset(void);
void systemResetToBootloader(void);

#ifdef __cplusplus
}
#endif

#endif //SYSTEM_H
