#ifndef RC_PPM_H
#define RC_PPM_H

#include "rc.h"
#include "gpio.h"
#include "board.h"


class RC_PPM : public RC
{
public:
    RC_PPM();
    void init();

    uint32_t readus(uint8_t channel);
    float read(uint8_t channel);

    void pulse_callback(void);

private:
    GPIO pin_;

    GPIO_TypeDef* GPIO_;
    TIM_TypeDef* TIM_;
    uint16_t pin_number_;
    uint8_t channel_;
    uint8_t IRQ_Channel_;

    uint16_t current_capture_;
    uint16_t last_capture_;
    uint8_t chan_;
    uint32_t RC_raw_[8];
};

extern RC_PPM* RC_PPMPtr;


#endif // RC_PPM_H
