#ifndef SIMONK_ESC_H
#define SIMONK_ESC_H

#include "board.h"
#include "pwm.h"


class SimonK_ESC : public PWM_Out
{
public:
    using PWM_Out::init;
    void init(uint8_t pin);
};

#endif // SIMONK_ESC_H
