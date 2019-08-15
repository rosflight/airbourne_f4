
#ifndef DRV_DSHOT_H
#define DRV_DSHOT_H

#include <array>

#include "revo_f4.h"
#include "system.h"

// we have 16 timer values for the actual payload itself
// then the 17th time value is a zero so we arent transmitting
// garbage after a packet
#define DSHOT_OUT_BUFF_SIZE 17

class DSHOT_OUT {
public:

    // Speed options for DSHOT. Not all ESCs can handle the faster speeds, so 
    // check what speed you need
    typedef enum : uint8_t {
        DSHOT_1200 = 1, // the enum values are used to determine timer prescaler
        DSHOT_600  = 2,
        DSHOT_300  = 4,
        DSHOT_150  = 8
    } dshot_speed_t;

    DSHOT_OUT();

    void init(const dshot_hardware_struct_t* dshot_config, dshot_speed_t dshot_speed);
    void enable();
    void disable();
    void setRequestTelemetry(bool request_telemetry);
    void write(float value);

private:
    uint16_t prepareDshotPacket(float value);
    bool request_telemetry_;

    const dshot_hardware_struct_t* config_;

    // offset to access dma interrupt flags for this dshot instance
    uint8_t dma_interrupt_flag_offset_;

    uint16_t out_buffer_[DSHOT_OUT_BUFF_SIZE];
};

#endif // DRV_DSHOT_H