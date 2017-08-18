#include "system.h"
#include "drv_spi.h"
#include "mpu6000.h"
#include "drv_led.h"

int main() {

	systemInit();

    LED warn(LED1_GPIO, LED1_PIN);
    LED info(LED2_GPIO, LED2_PIN);

	SPI mpu_spi(MPU6000_SPI);

	MPU6000 imu(&mpu_spi);
	int16_t temp;
    int16_t acc[3];
    int16_t gyro[3];
    while(1) {
        info.toggle();
        imu.read_sensors(acc, gyro, &temp);
        if (acc[0] == 0xFFFF || acc[0] == 0x0000) {
            warn.on();
        }
        else {
        	warn.off();
        }
		delay(200);
	}
}
