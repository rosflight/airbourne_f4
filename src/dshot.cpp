#include "dshot.h"

// The timer frequency is scaled so that these are always true
#define DSHOT_PERIOD_CYCLES_COUNT          70
#define DSHOT_BIT_0_CYCLES_COUNT           26
#define DSHOT_BIT_1_CYCLES_COUNT           53
#define DSHOT_MIN_THROTTLE                 48 // there are some lower values (0-47) that are reserved by the protocol
#define DSHOT_MAX_THROTTLE                 2047 // 11 bits of all 1s

DSHOT_OUT::DSHOT_OUT(){}

void DSHOT_OUT::init(dshot_speed_t dshot_speed) {

    DMAPtr      = DMA1_Stream2;
    TIMPtr      = TIM3;
    DMABasePtr  = DMA1;
    port_       = GPIOB;
    pin_        = GPIO_Pin_0;


    // Configure the hardware interrupt for our DMA stream.
    // We'll use this to turn DMA off once our transfer is complete
    // we put this under a pretty high priority since properly driving motors is important
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_Init(&NVIC_InitStruct);

    /// GPIO init. The GPIO we use depends on the output pin and TIM. Check the 
    ///     schematic of your board to make sure you're connecting the two right 
    GPIO_InitTypeDef gpio_init_struct;
    GPIO_PinAFConfig(port_, GPIO_PinSource0, GPIO_AF_TIM3);

    gpio_init_struct.GPIO_Pin 	= pin_;
    gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF;      // Alternative-Function mode
    gpio_init_struct.GPIO_Speed	= GPIO_Speed_100MHz; // DSHOT is stupid fast
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;     // Push-Pull with a default of reset
    gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_DOWN;
    GPIO_Init(port_, &gpio_init_struct);
    // --END GPIO init--

    //// DMA init: Again, the DMA used depends on the TIM. Check STM32F4 reference manual.
    ////        The DMA section has a table showing that TIMx_UP match with what DMA streams/channels
    // the RCC DMA clock should have already been enabled by system.c -- if not, enable it here

    // de-init disables the stream and resets a lot of DMA's important register 
    // that we'll be changing below
    DMA_DeInit(DMAPtr);
        
    // now configure DMA:
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct); // initialize the struct the default values (basically all zeros)

    DMA_InitStruct.DMA_Channel            = DMA_Channel_5; // channel is depended on TIMx_UP connection to your DMA stream
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &TIMPtr->DMAR; // the TIM DMAR is a register where DMA values are pre-loaded before being placed in their proper register
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t) out_buffer_; // point the base address for DMA transfers to our output buffer address
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral; // we're moving stuff from our out_buffer_ -> the timer's CCR register(s)
    DMA_InitStruct.DMA_BufferSize         = DSHOT_OUT_BUFF_SIZE; // we perform as many transfers as the size of our output buffer
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable; // we want to increment the memory address DMA is transfering as the transfer takes place
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // peripheral should be expecting 16 bit half-words
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord; // we're transfering 16 bit half-words from memory (out_buffer_ is uint16)
    DMA_InitStruct.DMA_Priority           = DMA_Priority_VeryHigh; // getting commands to motors is important
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull; // this is just a default setting.. dont ask me why
    DMA_Init(DMAPtr, &DMA_InitStruct); // apply all them settings!
    DMA_ITConfig(DMAPtr, DMA_IT_TC, ENABLE); // enable the transfer_complete interrupt

    out_buffer_[DSHOT_OUT_BUFF_SIZE - 1] = 0; // the 17th value in our output buffer is always a 0 so the timer doesn't transfer continuous 0bits or 1bits, but holds a 0 after a DMA transfer is complete
    // --END DMA init--

    // TIM INIT:
    TIM_DeInit(TIMPtr); // resets the clock used by the Timer. The timer clock should have already been enabled in system.c

    TIMPtr->CR1  = 0;
    TIMPtr->CR2  = 0;
    TIMPtr->CNT  = 0;
    TIMPtr->CCR1 = 0;
    TIMPtr->CCR2 = 0;
    TIMPtr->CCR3 = 0;
    TIMPtr->CCR4 = 0;
    TIMPtr->CCER = 0;

    TIMPtr->ARR = DSHOT_PERIOD_CYCLES_COUNT - 1; // 0 based. How high the timer should count for one period
    TIMPtr->PSC = 1792; //  (84,000,000 / (84,000,000 / 8)) - 1  you could also scale up PSC for slower output and easier debugging - 1792

    TIMPtr->CCMR1 = TIM_OCMode_PWM1 | (TIM_OCMode_PWM1 << 8) | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIMPtr->CCMR2 = TIM_OCMode_PWM1 | (TIM_OCMode_PWM1 << 8) | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
    TIMPtr->CCER |= TIM_CCER_CC3E; // JUST ENABLING CCR3 FOR NOW

    TIMPtr->DCR  = TIM_DMABase_CCR3 | TIM_DMABurstLength_1Transfer; // single transfers to CCR3. Since the revo's pwm out 1 is tied to TIM3-CHN3
    TIMPtr->DIER = TIM_DIER_UDE;

    TIMPtr->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;

}

void DSHOT_OUT::write(float value) {

    if ((DMAPtr->CR & DMA_SxCR_EN) == 0) {

        uint16_t packet = prepareDshotPacket(value);

        for (uint8_t i = 0; i < 16; i++) {
            // here we're converting each bit of the outgoing data packet into timer values
            // put timer values into our DMA out buffer. DMA will automatically 
            // feed these to the timer
            // Dshot is MSB first
            out_buffer_[i] = (packet & 0x8000) ? DSHOT_BIT_1_CYCLES_COUNT : DSHOT_BIT_0_CYCLES_COUNT;
            packet <<= 1; // get ready to process the next bit
        }

        DMAPtr->NDTR = 17;

        uint32_t currentFlags = DMABasePtr->LISR;
        //0x3D 
        DMABasePtr->LIFCR = currentFlags & (0x3D << 16U); //TODO: this shift depends on the stream, and is a sucky way todo this

        TIMPtr->EGR |= TIM_EGR_UG; // 0x1 generate an update event for the counter. this reinitializes and updates the registers
        DMAPtr->CR |= DMA_SxCR_EN; // enable DMA
    }
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
    uint16_t packet = DSHOT_MIN_THROTTLE + static_cast<uint16_t>((DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) * value);
    packet = (packet << 1) | request_telemetry_; // OR in request telemetry into current lsb - eventually bit 11

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

extern "C"
{
void DMA1_Stream2_IRQHandler() {
    DMA_Stream_TypeDef* stream = DMA1_Stream2;
    DMA_TypeDef* DMABasePtr = DMA1;
    if ((DMA1->LISR & 0x200000) != 0) { //DMA_GetITStatus(stream, DMA_FLAG_TCIF2)){
        DMA_ClearITPendingBit(stream, DMA_FLAG_TCIF2);
        DMA_Cmd(stream, DISABLE);
    } 
}
}