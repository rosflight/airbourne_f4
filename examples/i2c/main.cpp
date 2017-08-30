#include "system.h"
#include "drv_i2c.h"
#include "drv_led.h"

int main() {
	systemInit();
	//R/W0 = write

	LED warn(LED1_GPIO, LED1_PIN);
	LED info(LED2_GPIO, LED2_PIN);

    info.on();
	I2C i2c2(I2C2);

	while(1) {
		uint8_t addr;
		for (addr=0; addr<128; ++addr) {
			info.toggle();
            if (i2c2.read(addr, 0x00, 0x00)) {
				warn.off();
			}
			else {
				warn.on();
			}
			delay(200);
		}
	}
}
