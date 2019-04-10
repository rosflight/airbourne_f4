
#ifndef DRV_DSHOT_H
#define DRV_DSHOT_H

#include "revo_f4.h"
#include <array>

// we have 16 timer values for the actual payload itself
// then the 17th time value is a zero to confirm we hold
// the output at 0 until the bit period is over (i think)
#define DSHOT_OUT_BUFF_SIZE 17

class DSHOT_OUT {
public:
    DSHOT_OUT();

    // TODO: temp, change bitrate to an enum
    void init(int dshot_bitrate);
    void init2(int dshot_bitrate);
    void init3();
    void kflyInit();

    void enable();
    void disable();
    void setRequestTelemetry(bool request_telemetry);

    void write(float value);
    void write2(float value);
    void write3(float value);
    void kflyWrite(float value);
    uint32_t dshot_freq_hz;

private:
    uint16_t prepareDshotPacket(float value);
    bool request_telemetry_;

    volatile uint32_t* CCR_;

    TIM_TypeDef* TIMPtr;
    DMA_Stream_TypeDef* DMAPtr;
    DMA_TypeDef* DMABasePtr;
    GPIO_TypeDef* port_;
    uint16_t pin_;

    uint16_t out_buffer_[DSHOT_OUT_BUFF_SIZE];

    std::array<uint32_t, 17> payload_;

};

#endif // DRV_DSHOT_H