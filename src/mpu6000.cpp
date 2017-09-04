#include "mpu6000.h"


MPU6000::MPU6000(SPI* spi_drv) {
  spi = spi_drv;

  spi->enable();
  spi->transfer_byte(MPU_RA_PWR_MGMT_1); // Device Reset
  spi->transfer_byte(MPU_BIT_H_RESET);
  spi->disable();

  delay(150);

  spi->enable();
  spi->transfer_byte(MPU_RA_PWR_MGMT_1); // Clock Source PPL with Z axis gyro reference
  spi->transfer_byte(MPU_CLK_SEL_PLLGYROZ);
  spi->disable();

  delayMicroseconds(1);

  spi->enable();
  spi->transfer_byte(MPU_RA_USER_CTRL); // Disable Primary I2C Interface
  spi->transfer_byte(MPU_BIT_I2C_IF_DIS);
  spi->disable();

  delayMicroseconds(1);

  spi->enable();
  spi->transfer_byte(MPU_RA_PWR_MGMT_2);
  spi->transfer_byte(0x00);
  spi->disable();

  delayMicroseconds(1);

  spi->enable();
  spi->transfer_byte(MPU_RA_SMPLRT_DIV); // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
  spi->transfer_byte(0x00);
  spi->disable();

  delayMicroseconds(1);

  spi->enable();
  spi->transfer_byte(MPU_RA_CONFIG); // Accel and Gyro DLPF Setting
  spi->transfer_byte(MPU_BITS_DLPF_CFG_98HZ);
  spi->disable();

  delayMicroseconds(1);

  spi->enable();
  spi->transfer_byte(MPU_RA_ACCEL_CONFIG); // Accel +/- 4 G Full Scale
  spi->transfer_byte(MPU_BITS_FS_4G);
  spi->disable();

  delayMicroseconds(1);

  spi->enable();
  spi->transfer_byte(MPU_RA_GYRO_CONFIG); // Gyro +/- 2000 DPS Full Scale
  spi->transfer_byte(MPU_BITS_FS_2000DPS);
  spi->disable();

  spi->set_divisor(2); // 21 MHz SPI clock (within 20 +/- 10%)

  // set the accel and gyro scale parameters
  accel_scale_ = (4.0 * 9.80665f) / ((float)0x7FFF);
  gyro_scale_= (2000.0 * 3.14159f/180.0f) / ((float)0x7FFF);
}

void MPU6000::read_sensors(float (&accel_data)[3], float (&gyro_data)[3], float* temp_data)
{
  uint8_t raw[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  spi->enable();
  spi->transfer_byte(MPU_RA_ACCEL_XOUT_H | 0x80);
  spi->transfer(raw, 14);
  spi->disable();

  accel_data[0] = (float)((int16_t)((raw[0] << 8) | raw[1])) * accel_scale_;
  accel_data[1] = (float)((int16_t)((raw[2] << 8) | raw[3])) * accel_scale_;
  accel_data[2] = (float)((int16_t)((raw[4] << 8) | raw[5])) * accel_scale_;

  (*temp_data)  = (float)((int16_t)((raw[6] << 8) | raw[7])) / 340.0f * 36.53f;

  gyro_data[0]  = (float)((int16_t)((raw[8]  << 8) | raw[9])) * gyro_scale_;
  gyro_data[1]  = (float)((int16_t)((raw[10] << 8) | raw[11])) * gyro_scale_;
  gyro_data[2]  = (float)((int16_t)((raw[12] << 8) | raw[13])) * gyro_scale_;
}
