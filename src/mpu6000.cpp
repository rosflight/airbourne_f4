#include "mpu6000.h"

MPU6000* IMU_Ptr = NULL;

MPU6000::MPU6000(SPI* spi_drv) {
  spi_ = spi_drv;

  IMU_Ptr = this;

  write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET); // Device Reset

  delay(150);

  write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ); // Clock Source PPL with Z axis gyro reference
  write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS); // Disable Primary I2C Interface
  write(MPU_RA_PWR_MGMT_2, 0x00);
  write(MPU_RA_SMPLRT_DIV, 0x00); // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
  write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_188HZ);
  write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_4G);
  write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS);
  write(MPU_RA_INT_ENABLE, 0x01);

  spi_->set_divisor(2); // 21 MHz SPI clock (within 20 +/- 10%)

  // Set up the EXTI pin
  exti_.init(GPIOC, GPIO_Pin_4, GPIO::INPUT);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

  EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.EXTI_Line = EXTI_Line4;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStruct);

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x03;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);


  // set the accel and gyro scale parameters
  accel_scale_ = (4.0 * 9.80665f) / ((float)0x7FFF);
  gyro_scale_= (2000.0 * 3.14159f/180.0f) / ((float)0x7FFF);
}

void MPU6000::write(uint8_t reg, uint8_t data)
{
  spi_->enable();
  spi_->transfer_byte(reg);
  spi_->transfer_byte(data);
  spi_->disable();
  delayMicroseconds(1);
}

void MPU6000::read_sensors(float (&accel_data)[3], float (&gyro_data)[3], float* temp_data, uint64_t* stamp_us)
{
  uint8_t raw[15] = { MPU_RA_ACCEL_XOUT_H | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  spi_->enable();
  spi_->transfer(raw, 15); // The first byte is the register, the rest are for data
  new_data_ = false;
  spi_->disable();

  accel_data[0] = (float)((int16_t)((raw[1] << 8) | raw[2])) * accel_scale_;
  accel_data[1] = (float)((int16_t)((raw[3] << 8) | raw[4])) * accel_scale_;
  accel_data[2] = (float)((int16_t)((raw[5] << 8) | raw[6])) * accel_scale_;

  (*temp_data)  = (float)((int16_t)((raw[7] << 8) | raw[8])) / 340.0f + 36.53f;

  gyro_data[0]  = (float)((int16_t)((raw[9]  << 8) | raw[10])) * gyro_scale_;
  gyro_data[1]  = (float)((int16_t)((raw[11] << 8) | raw[12])) * gyro_scale_;
  gyro_data[2]  = (float)((int16_t)((raw[13] << 8) | raw[14])) * gyro_scale_;

  (*stamp_us) = timestamp_us_;
}

void MPU6000::set_new_data()
{
  timestamp_us_ = micros();
  new_data_ = true;
}

extern "C"
{

void EXTI4_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line4);
  IMU_Ptr->set_new_data();
}

}
