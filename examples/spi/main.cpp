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
#include "VCP.h"
#include "spi.h"
#include "mpu6000_spi.h"

VCP* vcpPtr = NULL;

void rx_callback(uint8_t byte)
{
  vcpPtr->put_byte(byte);
  vcpPtr->flush();
}

int main()
{
  systemInit();

  VCP vcp(serial_config[VCP_INDEX]);
  vcpPtr = &vcp;
  vcp.register_rx_callback(&rx_callback);

  SPI spi(spi_config[0]);

  MPU6000_SPI imu(&spi);

  vector3 acc, gyro;
  float temp;
  while(1)
  {
    acc.zero();
    gyro.zero();
    temp = 0;
    imu.read_all(&acc, &gyro, &temp);
    delay(200);
  }
}
