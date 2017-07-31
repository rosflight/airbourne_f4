#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include "gpio.h"

enum{
  PWM_PORT = 0x1,
  RC_PORT = 0x2
};

class PWM_Out
{
public:
    PWM_Out();

    void init(uint8_t pin){}
    void init(uint8_t pin, uint16_t frequency, uint32_t max_us, uint32_t min_us);
    void enable();
    void disable();
    void write(float value);
    void writeus(uint32_t us);

private:
    uint8_t index_;
    GPIO pin_;
    volatile uint16_t* CCR_;

    TIM_TypeDef* TIM_;

    uint16_t cycles_per_us_;
    uint16_t max_cyc_;
    uint16_t min_cyc_;
};

#endif // PWM_H
