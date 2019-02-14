
#include "dshot.h"

DSHOT_OUT::DSHOT_OUT(){}

void DSHOT_OUT::init() {
    request_telemetry_ = false;
    max_throttle_val_ = 2047; // 11 bits of all 1s
    min_throttle_val_ = 48; // there are some lower values (0-47) that are reserved by the protocol
}

uint16_t DSHOT_OUT::write(float value) {
    // currently just returning uint16_t for debugging.. change to void

    return prepareDshotPacket(value);
}

void DSHOT_OUT::setRequestTelemetry(bool request_telemetry) {
    request_telemetry_ = request_telemetry;
}

uint16_t DSHOT_OUT::prepareDshotPacket(float value) {
    // DSHOT packet:
    // Bits:
    //    0-10:  throttle value (48-2047)
    //    11:    provide telemetry bit
    //    12-15: checksum
    // EX: 0000 0110 0000 0110
    //    Hex: 0x0606 -> throttle 48, no telemetry request

    // decide on throttle value, this makes up bits 0-10
    uint16_t packet = min_throttle_val_ + static_cast<uint16_t>((max_throttle_val_ - min_throttle_val_) * value);
    packet = (packet << 1) | request_telemetry_; // or in request telemetry into current lsb - eventually bit 11

    // calculate checksum - bits 12-15
    uint16_t csum = 0;
    uint16_t csum_data = packet;

    for (int i = 0; i < 3; i++)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    // put checksum in our final packet
    packet = (packet << 4) | (csum & 0xf);
    return packet;
}