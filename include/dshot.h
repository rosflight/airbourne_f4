
#ifndef DRV_DSHOT_H
#define DRV_DSHOT_H

#include "revo_f4.h"

// we have 16 timer values for the actual payload itself
// then the 17th time value is a zero to confirm we hold
// the output at 0 until the bit period is over (i think)
#define DSHOT_OUT_BUFF_SIZE 17

class DSHOT_OUT {
public:
    DSHOT_OUT();

    // TODO: temp, change bitrate to an enum
    void init(int dshot_bitrate);

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

    uint32_t out_buffer_[DSHOT_OUT_BUFF_SIZE];

};

#endif // DRV_DSHOT_H