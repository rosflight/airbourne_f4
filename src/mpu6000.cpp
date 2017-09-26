#include "mpu6000.h"

uint8_t raw[15] = {MPU_RA_ACCEL_XOUT_H | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

MPU6000* IMU_Ptr;

void data_transfer_cb(void)
{
  IMU_Ptr->data_transfer_callback();
}



void MPU6000::write(uint8_t reg, uint8_t data)
{
  spi->enable();
  spi->transfer_byte(reg);
  spi->transfer_byte(data);
  spi->disable();
  delayMicroseconds(1);
}

MPU6000::MPU6000(SPI* spi_drv) {
  IMU_Ptr = this;
  spi = spi_drv;

  write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET);
  delay(150);

  write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
  write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS);
  write(MPU_RA_PWR_MGMT_2, 0x00);
  write(MPU_RA_SMPLRT_DIV, 0x00);
  write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_98HZ);
  write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_4G);
  write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS);
  write(MPU_RA_INT_ENABLE, 0x01);

  /* Enable clock for GPIOD */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* Enable clock for SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  // Set up the EXTI pin
  //  exti_.init(GPIOC, GPIO_Pin_4, GPIO::INPUT);
  /* Set pin as input */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line4;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  spi->set_divisor(2); // 21 MHz SPI clock (within 20 +/- 10%)

  // set the accel and gyro scale parameters
  accel_scale_ = (4.0 * 9.80665f) / ((float)0x7FFF);
  gyro_scale_= (2000.0 * 3.14159f/180.0f) / ((float)0x7FFF);

  spi->register_complete_cb(&data_transfer_cb);
}

void MPU6000::update()
{
  static auto last_update_time_us = 0;
  if (new_exti_)
  {
    raw[0] = MPU_RA_ACCEL_XOUT_H | 0x80;
    spi->transfer(raw, 15, raw);
  }
}

void MPU6000::data_transfer_callback()
{
  new_data_ = true;
  acc_[0] = (float)((int16_t)((raw[1] << 8) | raw[2])) * accel_scale_;
  acc_[1] = (float)((int16_t)((raw[3] << 8) | raw[4])) * accel_scale_;
  acc_[2] = (float)((int16_t)((raw[5] << 8) | raw[6])) * accel_scale_;

  temp_  = (float)((int16_t)((raw[7] << 8) | raw[8])) / 340.0f * 36.53f;

  gyro_[0]  = (float)((int16_t)((raw[9]  << 8) | raw[10])) * gyro_scale_;
  gyro_[1]  = (float)((int16_t)((raw[11] << 8) | raw[12])) * gyro_scale_;
  gyro_[2]  = (float)((int16_t)((raw[13] << 8) | raw[14])) * gyro_scale_;
}

void MPU6000::read(float (&accel_data)[3], float (&gyro_data)[3], float* temp_data)
{
  accel_data[0] = acc_[0];
  accel_data[1] = acc_[1];
  accel_data[2] = acc_[2];
  gyro_data[0] = gyro_[0];
  gyro_data[1] = gyro_[1];
  gyro_data[2] = gyro_[2];
  *temp_data = temp_;
}

void MPU6000::exti_cb()
{
  new_exti_ = true;
  imu_timestamp_ = micros();
}

extern "C"
{

void EXTI4_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line4);
  IMU_Ptr->exti_cb();
}

}
