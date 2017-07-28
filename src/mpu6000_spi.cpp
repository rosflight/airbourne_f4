#include "mpu6000_spi.h"
#include <stdlib.h>

MPU6000_SPI::MPU6000_SPI(SPI* spi_dev_ptr)
{
  spi = spi_dev_ptr;

  spi->set_divisor(SPI::INITIALIZATION);

  // reset
  write_register(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
  delay(10);
  write_register(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
  delay(10);

  // Set clock source to gyro Z axis
  write_register(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
  delayMicroseconds(15);

  // Disable I2C
  write_register(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
  delayMicroseconds(15);

  // Go fast
  write_register(MPU_RA_PWR_MGMT_2, 0x00);
  delayMicroseconds(15);

  // Sample at 1kHz
  /// TODO: Sample at 8kHz
  write_register(MPU_RA_SMPLRT_DIV, 0);

  // set DLPF (if set, throttles Gyro reads to 1kHz
  write_register(MPU_RA_CONFIG, BITS_DLPF_CFG_98HZ);

  // Gyro +/- 2000 DPS Full Scale
  write_register(MPU_RA_GYRO_CONFIG, BITS_FS_2000DPS);
  delayMicroseconds(15);

  // Accel +/- 8 G Full Scale
  write_register(MPU_RA_ACCEL_CONFIG, BITS_FS_8G);
  delayMicroseconds(15);

  // Enable External Interrupt Pin
  write_register(MPU_RA_INT_ENABLE, BIT_RAW_RDY_EN);
  delayMicroseconds(15);
  write_register(MPU_RA_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR);
  delayMicroseconds(15);

  spi->set_divisor(SPI::FAST);
  delayMicroseconds(1);

  accel_scale_ = ((float)8 * 9.80665f) / ((float)0x7FFF);
  gyro_scale_ = ((float)2000 * 3.14159f/180.0f) / ((float)0x7FFF);
}

bool MPU6000_SPI::read_all(vector3 *accel, vector3 *gyro, float *temp)
{
  uint8_t buf[14];
  for (int i = 0; i < 14; i++)
    buf[i] = 0;
  read_register(MPU_RA_ACCEL_XOUT_H, 14, buf);

  accel->x = ((float)(int16_t)((buf[0] << 8) | buf[1])) * accel_scale_;
  accel->y = ((float)(int16_t)((buf[2] << 8) | buf[3])) * accel_scale_;
  accel->z = ((float)(int16_t)((buf[4] << 8) | buf[5])) * accel_scale_;

  *temp = ((float)(int16_t)((buf[6] << 8) | buf[7]))/340.0f + 36.53f;

  gyro->x = ((float)(int16_t)((buf[8] << 8) | buf[9])) * gyro_scale_;
  gyro->y = ((float)(int16_t)((buf[10] << 8) | buf[11])) * gyro_scale_;
  gyro->z = ((float)(int16_t)((buf[12] << 8) | buf[13])) * gyro_scale_;

}

void MPU6000_SPI::request_update()
{

}

void MPU6000_SPI::register_external_interrupt_callback(void (*CB)())
{

}

void MPU6000_SPI::register_new_data_callback(void (*CB)())
{

}

bool MPU6000_SPI::write_register(uint8_t reg, uint8_t data)
{
  spi->set_ss_low();
  uint8_t to_send = reg | 0x80;
  spi->transfer(&to_send, 1, (uint8_t*)NULL);
  spi->transfer(&data, 1, NULL);
  spi->set_ss_high();
  return true;
}

bool MPU6000_SPI::read_register(uint8_t reg, uint8_t len, uint8_t *data)
{
  spi->set_ss_low();
  uint8_t to_send = reg | 0x80;
  spi->transfer(&to_send, 1, NULL);
  spi->transfer(NULL, len, data);
  spi->set_ss_high();
  return true;
}
