#include "system.h"
#include "pwm.h"
#include "led.h"

#include "revo_f4.h"

int main() {
	systemInit();

	LED info;
	info.init(LED2_GPIO, LED2_PIN);

	PWM_OUT esc_out[PWM_NUM_OUTPUTS];
	for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
	{
				esc_out[i].init(&pwm_config[i], 490, 2000, 1000);
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
