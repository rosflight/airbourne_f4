#ifndef RC_DSM_H
#define RC_DSM_H

#include "rc.h"
#include "uart.h"
#include "gpio.h"
#include "board.h"

#define DSM_2048_MAX_CHANNELS 8
#define DSM_1024_MAX_CHANNELS 7
#define DSM_FRAME_SIZE 16

class RC_DSM : public RC
{
public:

    typedef enum {
        DSM_1024,
        DSM_2048
    } dsm_type_t;

    RC_DSM();

    void init();
    void init(dsm_type_t dsm_type);
    uint32_t readus(uint8_t channel);
    float read(uint8_t channel);

    void handle_byte(uint8_t c);

private:
    UART uart_;

    volatile uint8_t frame_[DSM_FRAME_SIZE];

    bool new_frame_;
    bool incoming_;
    uint8_t pos_;
    uint64_t last_data_receive_us_;
    uint8_t shift_;
    uint8_t mask_;
    dsm_type_t dsm_type_;
    uint8_t num_channels_;

    uint32_t RC_raw_[8];
};

extern RC_DSM* RC_DSMPtr;

#endif // RC_DSM_H
