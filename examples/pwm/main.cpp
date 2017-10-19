#include "system.h"
#include "drv_pwm_out.h"
#include "drv_led.h"

int main() {
	systemInit();

	LED info;
	info.init(LED2_GPIO, LED2_PIN);

	PWM_OUT esc_out[PWM_NUM_OUTPUTS];
	for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
	{
        esc_out[i].init(&pwm_hardware[i], 490, PWM_MAX_US, PWM_MIN_US);
	}

    float throttle = 0.0;
	while(1) {
		for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
		{
			esc_out[i].write(throttle);
		}
        throttle += 0.001;
        if (throttle > 1.0)
		{
			throttle = 0.0;
            info.toggle();
		}
        delay(2);
	}
}
