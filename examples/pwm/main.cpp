#include "system.h"
#include "revo_f4.h"
#include "drv_pwm_out.h"
#include "drv_led.h"

int main() {
	systemInit();

	LED info(LED2_GPIO, LED2_PIN);

	PWM_OUT esc_out[NUM_PWM_OUTPUTS];
	for (int i = 0; i < NUM_PWM_OUTPUTS; ++i)
	{
        esc_out[i].init(&pwm_hardware[i], 50, 2000, 1000);
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
            info.toggle();
		}
		delay(2);
	}
}
