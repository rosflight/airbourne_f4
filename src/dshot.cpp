
#include "dshot.h"

DSHOT_OUT::DSHOT_OUT(){}

void DSHOT_OUT::init() {
    request_telemetry_ = false;
    max_throttle_val_ = 2047; // 11 bits of all 1s
    min_throttle_val_ = 48; // there are some lower values (0-47) that are reserved by the protocol

    GPIO_InitTypeDef gpio_init_struct;
    TIM_TimeBaseInitTypeDef tim_init_struct;
    TIM_OCInitTypeDef tim_oc_init_struct;

    // SETUP GPIO PIN FOR OUTPUT
    // TODO: parameterize. rn just going to pwm output 0
    port_ = GPIOB;
    pin_  = GPIO_Pin_0;

    // Configure GPIO as alternative function, fast speed (not high speed)
    GPIO_PinAFConfig(port_, GPIO_PinSource0, GPIO_AF_TIM3);

    gpio_init_struct.GPIO_Pin 	= pin_;
    gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF; // AF is "alternative function"
    gpio_init_struct.GPIO_Speed	= GPIO_Speed_50MHz; // aka "Fast mode"
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;
    gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_DOWN;
    GPIO_Init(port_, &gpio_init_struct);

    // SETUP DMA, which will hold packets as they go out to esc

    // disable while we configure
    DMA_Cmd(DMA2_Stream0, DISABLE);
    DMA_DeInit(DMA2_Stream0);

    DMA_InitTypeDef  DMA_InitStructure_;
    DMA_InitStructure_.DMA_Channel = DMA_Channel_6; // TODO: not sure how much this matters yet
    // DMA_InitStructure_.DMA_PeripheralBaseAddr = ADDRESS TO TIMER DMAR
    // DMA_InitStructure_.DMA_Memory0BaseAddr = // create a memory buffer (an array) for this to point to
    DMA_InitStructure_.DMA_DIR = DMA_DIR_MemoryToPeripheral; // going from memory to esc
    
    /*Specifies the buffer size, in data unit, of the specified Stream. 
    The data unit is equal to the configuration set in
    DMA_MemoryDataSize members depending in the transfer direction.*/
    // DMA_InitStructure_.DMA_BufferSize // how many elements are in your array?

    DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // dont increment the address to the peripheral
    DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable; // should the memory address be incremented (after we send a block?)
    DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // peripheral datasize (what size of thing is peripheral expecting?) half-word == 16 bits
    DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // size of memory chunks in buffer we're sending
    DMA_InitStructure_.DMA_Priority = DMA_Priority_VeryHigh; // timely motor control is mildly important ;)
    // fifo mode
    // fifo threshold
    DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single; // burst transfer a single block
    DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    TIM_TypeDef* TIMPtr = TIM3;


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