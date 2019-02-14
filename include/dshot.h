
#ifndef DRV_DSHOT_H
#define DRV_DSHOT_H

#include "revo_f4.h"

class DSHOT_OUT {
public:
    DSHOT_OUT();

    void init();

    void enable();
    void disable();
    void setRequestTelemetry(bool request_telemetry);

    uint16_t write(float value);

private:
    uint16_t prepareDshotPacket(float value);
    bool request_telemetry_;

    uint16_t max_throttle_val_;
    uint16_t min_throttle_val_;

};

#endif // DRV_DSHOT_H