#ifndef RC_H
#define RC_H

#include <board.h>
#include <stdint.h>

class RC
{
public:
    typedef enum
    {
        PPM,
        DSM
    } mode_t;

    RC() {}

    void init(){}

    uint32_t readus(uint8_t channel){}
    float read(uint8_t channel){}
};


#include "rc_dsm.h"
#include "rc_ppm.h"

#endif // RC_H
