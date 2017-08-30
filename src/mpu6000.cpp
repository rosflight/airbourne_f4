#include "mpu6000.h"


MPU6000::MPU6000(SPI* spi_drv) {
	spi = spi_drv;

	spi->enable();
    spi->transfer(MPU_RA_PWR_MGMT_1); // Device Reset
    spi->transfer(MPU_BIT_H_RESET);
    spi->disable();

    delay(150);

    spi->enable();
    spi->transfer(MPU_RA_PWR_MGMT_1); // Clock Source PPL with Z axis gyro reference
    spi->transfer(MPU_CLK_SEL_PLLGYROZ);
    spi->disable();

    delayMicroseconds(1);

    spi->enable();
    spi->transfer(MPU_RA_USER_CTRL); // Disable Primary I2C Interface
    spi->transfer(MPU_BIT_I2C_IF_DIS);
    spi->disable();

    delayMicroseconds(1);

    spi->enable();
    spi->transfer(MPU_RA_PWR_MGMT_2);
    spi->transfer(0x00);
    spi->disable();

    delayMicroseconds(1);

    spi->enable();
    spi->transfer(MPU_RA_SMPLRT_DIV); // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
    spi->transfer(0x00);
    spi->disable();

    delayMicroseconds(1);

    spi->enable();
    spi->transfer(MPU_RA_CONFIG); // Accel and Gyro DLPF Setting
    spi->transfer(MPU_BITS_DLPF_CFG_98HZ);
    spi->disable();

    delayMicroseconds(1);

    spi->enable();
    spi->transfer(MPU_RA_ACCEL_CONFIG); // Accel +/- 4 G Full Scale
    spi->transfer(MPU_BITS_FS_4G);
    spi->disable();

    delayMicroseconds(1);

    spi->enable();
    spi->transfer(MPU_RA_GYRO_CONFIG); // Gyro +/- 1000 DPS Full Scale
    spi->transfer(MPU_BITS_FS_1000DPS);
    spi->disable();

    ///////////////////////////////////

    spi->set_divisor(2); // 21 MHz SPI clock (within 20 +/- 10%)

    ///////////////////////////////////

    delay(100);

}

void MPU6000::read_sensors(int16_t (&accel_data)[3], int16_t (&gyro_data)[3], int16_t* temp_data) {
    uint8_t raw[14];
	for (int i = 0; i < 14; i++) {
    	raw[i] = 0;
	}

	spi->enable();
	spi->transfer(MPU_RA_ACCEL_XOUT_H | 0x80);
	for (int i = 0; i < 14; ++i) {
		raw[i] = spi->transfer(0x00);
	}
	spi->disable();

	accel_data[0] = (int16_t)((raw[0] << 8) | raw[1]);
	accel_data[1] = (int16_t)((raw[2] << 8) | raw[3]);
	accel_data[2] = (int16_t)((raw[4] << 8) | raw[5]);

	(*temp_data)  = (int16_t)((raw[6] << 8) | raw[7]);

	gyro_data[0]  = (int16_t)((raw[8]  << 8) | raw[9]);
	gyro_data[1]  = (int16_t)((raw[10] << 8) | raw[11]);
	gyro_data[2]  = (int16_t)((raw[12] << 8) | raw[13]); 
}
