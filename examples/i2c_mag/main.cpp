#include "system.h"
#include "drv_i2c.h"
#include "hmc5883l.h"
#include "drv_led.h"

int main() {
	systemInit();
	//R/W0 = write

	LED warn(LED1_GPIO, LED1_PIN);
	LED info(LED2_GPIO, LED2_PIN);

    info.on();
	I2C i2c1(I2C1);
	HMC5883L mag(&i2c1);

	if (!mag.init()) {
		warn.on();
		delay(2000);
		warn.off();
	}

	int16_t mag_data[3];
	while(1) {
		info.toggle();
		if (mag.read(mag_data)) {
			warn.off();
		}
		else {
			warn.on();
		}
		delay(100);
	}
}
