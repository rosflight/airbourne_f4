#ifndef SERVO_H
#define SERVO_H

#include "board.h"
#include "pwm.h"

class Servo : public PWM_Out
{
public:
    using PWM_Out::init;
    void init(uint8_t pin);
};

#endif // SERVO_H
