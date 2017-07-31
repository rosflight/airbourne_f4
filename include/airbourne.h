#ifndef AIRBOURNE_H
#define AIRBOURNE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "board.h"
#include "barometer.h"
#include "eeprom.h"
#include "gpio.h"
#include "hmc5883l.h"
#include "i2c.h"
#include "imu.h"
#include "led.h"
#include "magnetometer.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "printf.h"
#include "pwm.h"
#include "simonk_esc.h"
#include "servo.h"
#include "rc.h"
#include "rc_dsm.h"
#include "rc_ppm.h"
#include "serialport.h"
#include "uart.h"

#include "vector3.h"
#include "quaternion.h"
#include "turbotrig.h"

void airbourne_init();

// LEDs
extern LED _LED0;
extern LED _LED1;

// UARTs
extern UART _uart1;
extern UART _uart2;

// I2C
extern I2C _i2c;

// Sensors
extern MPU6050 _mpu6050;
extern HMC5883L _hmc5883l;
extern MS5611 _ms5611;

// Motors
extern PWM_Out* _motors[13];
extern Servo _servos[13];
extern SimonK_ESC _simonk[13];

// RC
extern RC_DSM _rc_dsm;
extern RC_PPM _rc_ppm;

extern RC* _rc;

extern EEPROM _eeprom;

#endif // AIRBOURNE_H
