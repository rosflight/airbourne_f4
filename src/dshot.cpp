#include "dshot.h"

DSHOT_OUT::DSHOT_OUT(){}

void DSHOT_OUT::init() {

    request_telemetry_ = false;
    max_throttle_val_ = 2047; // 11 bits of all 1s
    min_throttle_val_ = 48; // there are some lower values (0-47) that are reserved by the protocol

    GPIO_InitTypeDef gpio_init_struct;
    TIM_TimeBaseInitTypeDef tim_init_struct;
    TIM_OCInitTypeDef tim_oc_init_struct;

    TIM_TypeDef* TIMPtr = TIM3;

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
    DMA_InitStructure_.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&TIMPtr->DMAR); /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    // DMA_InitStructure_.DMA_Memory0BaseAddr = // create a memory buffer (an array) for this to point to
    DMA_InitStructure_.DMA_DIR = DMA_DIR_MemoryToPeripheral; // going from memory to esc
    
    /*Specifies the buffer size, in data unit, of the specified Stream. 
    The data unit is equal to the configuration set in
    DMA_MemoryDataSize members depending in the transfer direction.*/
    // DMA_InitStructure_.DMA_BufferSize // how many elements are in your array?

    DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // dont increment the address to the peripheral
    DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable; // should the memory address be incremented (after we send a block?)
    DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // peripheral datasize (what size of thing is peripheral expecting?) half-word == 16 bits
    DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; // size of memory chunks in buffer we're sending
    DMA_InitStructure_.DMA_Priority = DMA_Priority_VeryHigh; // timely motor control is mildly important ;)
    // fifo mode
    // fifo threshold
    DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single; // burst transfer a single block
    DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;


    //calculate timer values as we do in the pwm driver
    //This is dependent on how fast the SystemCoreClock is. (ie will change between stm32fX models)
    // "The prescaler can divide the counter clock frequency by any factor between 1 and 65536"
    const uint16_t prescaler_default = 1; //GOTTA GO FASTTTTT
    uint32_t freq_prescale = prescaler_default * 2;
    uint32_t tim_prescaler = prescaler_default;

    if (TIMPtr == TIM9 || TIMPtr == TIM10 || TIMPtr == TIM11)
    {
        //For F4's (possibly others) TIM9-11 have a max timer clk double that of all the other TIMs
        //compensate for this by doubling its prescaler
        tim_prescaler = tim_prescaler * 2;
    }
    uint32_t timer_freq_hz = SystemCoreClock / freq_prescale;

    cycles_per_ns_ = timer_freq_hz / 1000000000.0;//E^9
    
    // im not sure what the heck this is doing:
    // constexpr const auto bit_period_ = (TIM_RATE + 1200000/2) / 1200000;

    // calculate how long the timer needs to run for a 0 bit and a 1 bit in DSHOT
    // make sure we round properly before truncate with a cast. it's probably good to 
    // make the pulse slightly longer than necessary
    cycles_per_reset_bit_ = static_cast<uint32_t>((cycles_per_ns_ * DHSOT_RESET_BIT_PULSE_WIDTH_NS) + 0.5);
    cycles_per_set_bit_   = static_cast<uint32_t>((cycles_per_ns_ * DHSOT_SET_BIT_PULSE_WIDTH_NS) + 0.5);

    // setup the timer:
    // DELETE ME: i dont think you need todo output compare here

}

float DSHOT_OUT::getNSCyc() {
    return cycles_per_ns_;
}

uint16_t DSHOT_OUT::write(float value) {
    // currently just returning uint16_t for debugging.. change to void

    // you fill the dma with 32 bit values of pulse widths the timer should run.
    // remember that the different bits take different amounts of time in DSHOT:
    //  For a bit to be 0, the pulse width is 625 nanosecond
    //  For a bit to be 1, the pulse width is 1250 nanoseconds

    uint16_t packet = prepareDshotPacket(value);

    for (uint8_t i = 0; i < 16; i++)
    {
      // Dshot is MSB first
      // put timer values into our DMA out buffer. DMA will automatically 
      // feed these to the timer
      out_buffer_[i] = (packet & 0x8000) ? cycles_per_set_bit_ : cycles_per_reset_bit_;
      packet <<= 1; // get ready to process the next bit
    }

    return packet;
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