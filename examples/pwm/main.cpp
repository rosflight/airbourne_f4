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
		esc_out[i].write(1.0);
	}

	// Calibrate ESC
	while (millis() < 5000);

	for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
	{
		esc_out[i].write(0.0);
	}

	while (millis() < 1000);


	bool use_us_driver = true;;
	uint32_t throttle = 1000;
	while(1)
	{
		for (int i = 0; i < PWM_NUM_OUTPUTS; ++i)
		{
			if (use_us_driver)
			{
				esc_out[i].writeUs(throttle);
			}
			else
			{
				esc_out[i].write((float)(throttle - 1000) / 1000.0);
			}
		}
		throttle += 1;
		if (throttle > 2000)
		{
			throttle = 0;
			info.toggle();
			use_us_driver = !use_us_driver;
		}
		delay(2);
	}
}
