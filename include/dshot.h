
#ifndef DRV_DSHOT_H
#define DRV_DSHOT_H

#include "revo_f4.h"

#define DHSOT_RESET_BIT_PULSE_WIDTH_NS 625
#define DHSOT_SET_BIT_PULSE_WIDTH_NS 1250

class DSHOT_OUT {
public:
    DSHOT_OUT();

    void init();

    void enable();
    void disable();
    void setRequestTelemetry(bool request_telemetry);

    uint16_t write(float value);

    float getNSCyc();

private:
    uint16_t prepareDshotPacket(float value);
    bool request_telemetry_;

    uint16_t max_throttle_val_;
    uint16_t min_throttle_val_;

    float cycles_per_ns_;
    uint32_t cycles_per_reset_bit_;
    uint32_t cycles_per_set_bit_;

    GPIO_TypeDef* port_;
    uint16_t pin_;
    uint32_t out_buffer_[16];

};

#endif // DRV_DSHOT_H