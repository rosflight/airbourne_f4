#ifndef DRV_PWM_OUT_H
#define DRV_PWM_OUT_H

#include "system.h"

class PWM_OUT {
public:
	PWM_OUT();

	void init(const pwm_hardware_struct_t* pwm_init, uint16_t frequency, uint32_t max_us, uint32_t min_us);
	void enable();
	void disable();
	void write(float value);

private:
	volatile uint32_t* CCR_;

	uint16_t max_cyc_;
	uint16_t min_cyc_;

	GPIO_TypeDef*	port_;
	uint16_t		pin_; 
};

#endif //DRV_PWM_H