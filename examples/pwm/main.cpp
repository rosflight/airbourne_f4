#include "system.h"
#include "drv_pwm_out.h"
#include "drv_led.h"

int main() {
	systemInit();

	LED warn(LED1_GPIO, LED1_PIN);
	LED info(LED2_GPIO, LED2_PIN);

	PWM_OUT esc_out[NUM_PWM_OUTPUTS];
	for (int i = 0; i < NUM_PWM_OUTPUTS; ++i)
	{
		esc_out[i] = PWM_OUT(pwm_hardware[i], 490, 2000, 1000);
	}

	float throttle = 0.0;
	while(1) {
		for (int i = 0; i < NUM_PWM_OUTPUTS; ++i)
		{
			esc_out[i].write(throttle);
		}
		throttle += 0.001;
		if (throttle > 1.0)
		{
			throttle = 0.0;
			warn.toggle();
		}
		delay(1);
		info.toggle();
	}
}