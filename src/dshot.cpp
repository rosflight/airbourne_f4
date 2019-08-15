#include "dshot.h"
#include "system_helpers.h"

// The timer frequency is scaled so that these are always true
#define DSHOT_PERIOD_CYCLES_COUNT   70
#define DSHOT_BIT_0_CYCLES_COUNT    26
#define DSHOT_BIT_1_CYCLES_COUNT    53
#define DSHOT_MIN_THROTTLE          48 // there are some lower values (0-47) that are reserved by the protocol
#define DSHOT_MAX_THROTTLE          2047 // 11 bits of all 1s

#define DSHOT_DMA_IRQ_PRIORITY      0x02
#define DSHOT_DMA_IRQ_SUBPRIORITY   0x02

DSHOT_OUT::DSHOT_OUT(){}

void DSHOT_OUT::init(const dshot_hardware_struct_t* dshot_config, dshot_speed_t dshot_speed) {
    if (dshot_config == NULL) {
        return;
    }
    config_ = dshot_config;
    dma_interrupt_flag_offset_ = system_helpers_dma_stream_interrupt_flag_offset(config_->tx_dma_stream);
    if (dma_interrupt_flag_offset_ == UINT8_MAX) {
        return;
    }

    IRQn_Type dma_irq_channel = system_helpers_dma_stream_to_dma_irqn(config_->tx_dma_stream);
    if (dma_irq_channel == 0) return; // TODO: add some error codes

    // Configure the hardware interrupt for our DMA stream.
    //  Use this to turn DMA off once a dshot transfer is complete
    //  put under a high priority since properly driving motors is important
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel                   = dma_irq_channel;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = DSHOT_DMA_IRQ_PRIORITY;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = DSHOT_DMA_IRQ_SUBPRIORITY;
    NVIC_Init(&NVIC_InitStruct);

    /// GPIO init. The GPIO used depends on the output pin and TIM. Check the
    ///     schematic of your board to make sure you're connecting the two right 
    uint16_t pin_source = system_helpers_gpio_pin_to_gpio_pinsource(config_->gpio_pin);
    if (pin_source == GPIO_PIN_SOURCE_INVALID) return;
    uint8_t tim_af = system_helpers_gpio_af_tim_from_tim(config_->tx_tim);
    if (tim_af == GPIO_TIM_AF_INVALID) return;
    GPIO_InitTypeDef gpio_init_struct;
    GPIO_PinAFConfig(config_->gpio_port, pin_source, tim_af);

    gpio_init_struct.GPIO_Pin 	= config_->gpio_pin;
    gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF;      // Alternative-Function mode
    gpio_init_struct.GPIO_Speed	= GPIO_Speed_100MHz; // DSHOT is stupid fast
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;     // Push-Pull with a default of reset
    gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_DOWN;
    GPIO_Init(config_->gpio_port, &gpio_init_struct);
    // --END GPIO init--

    //// DMA init: Again, the DMA used depends on the TIM. Check STM32F4 reference manual.
    ////        The DMA section has a table showing what TIMx_UP match with what DMA streams/channels
    // the RCC DMA clock should have already been enabled by system.c -- if not, enable it here

    // de-init disables the stream and resets a lot of DMA's important register values
    // that will be changed below
    DMA_DeInit(config_->tx_dma_stream);
        
    // now configure DMA:
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit(&DMA_InitStruct); // initialize the struct to the default values (basically all zeros)
    DMA_InitStruct.DMA_Channel            = config_->tx_dma_channel;           // channel is dependent on TIMx_UP connection to specified DMA stream
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &config_->tx_tim->DMAR; // the TIM DMAR is a register where DMA values are pre-loaded before being placed in their proper register
    DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t) out_buffer_;                 // point the base address for DMA transfers to internal dshot output buffer address
    DMA_InitStruct.DMA_DIR                = DMA_DIR_MemoryToPeripheral;             // moving stuff from our out_buffer_ -> the timer's CCR register(s)
    DMA_InitStruct.DMA_BufferSize         = DSHOT_OUT_BUFF_SIZE;                    // perform as many transfers as the size of our output buffer
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;                   // want to increment the memory address DMA is transfering as the transfer takes place
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;        // peripheral should be expecting 16 bit half-words
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;            // driver is transfering 16 bit half-words from memory (out_buffer_ is uint16)
    DMA_InitStruct.DMA_Priority           = DMA_Priority_VeryHigh;                  // getting commands to motors is important
    DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;             // this is just a default setting.. dont ask me why
    DMA_Init(config_->tx_dma_stream, &DMA_InitStruct); // apply all them settings!
    DMA_ITConfig(config_->tx_dma_stream, DMA_IT_TC, ENABLE); // enable the transfer_complete interrupt

    out_buffer_[DSHOT_OUT_BUFF_SIZE - 1] = 0; // the 17th value in our output buffer is always a 0 so the timer doesn't transfer continuous 0bits or 1bits, but holds a 0 after a DMA transfer is complete
    // --END DMA init--

    // TIM INIT:
    TIM_TypeDef* tim = config_->tx_tim;
    TIM_DeInit(tim); // resets the clock used by the Timer. The timer clock should have already been enabled in system.c

    tim->CR1  = 0;
    tim->CR2  = 0;
    tim->CNT  = 0;
    tim->CCR1 = 0;
    tim->CCR2 = 0;
    tim->CCR3 = 0;
    tim->CCR4 = 0;
    tim->CCER = 0;

    // NOTE: All the prescaler values for each DSHOT rate were precomputed for
    //      the F4 specifically. if you were going to use a different chip, you'd
    //      probably want to recompute them all to make sure you're within spec
    // HOW I calculated it:
    //      SystemCoreClock = 84000000 = about 11ns / cycle == clock period
    //      DSHOT600 = 1.67us / bit (this is the bit period)
    //              all the other protocols scale from this (ie: DSHOT1200 = DSHOT600/2, DSHOT300 = DSHOT600*2, etc)
    //      DSHOT_PERIOD_CYCLES_COUNT = 70
    //      Prescaler = bit period / clock period / dshot period cycles
    //      EX: DSHOT150 = 6680ns /  11ns         / 70 = 8 (truncating decimals)
    int prescaler = dshot_speed;
    if (tim == TIM9 || tim == TIM10 || tim == TIM11)
    {
        //For the F4 TIM9-11 have a max timer clk double that of all the other TIMs
        //compensate for this by doubling its prescaler
        prescaler = prescaler * 2;
    }

    TIM_TimeBaseInitTypeDef tim_init_struct;
    TIM_TimeBaseStructInit(&tim_init_struct);
    tim_init_struct.TIM_Period 		  = DSHOT_PERIOD_CYCLES_COUNT - 1; // how high to count before we restart counting. number is 0 based. (ie: 69 == count to 70)
    tim_init_struct.TIM_Prescaler 	  = prescaler - 1; // how many clock cycles should go by before we increment the TIM counter. Again, o-based
    TIM_TimeBaseInit(tim, &tim_init_struct);

    tim->CCMR1 = TIM_OCMode_PWM1 | (TIM_OCMode_PWM1 << 8) | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    tim->CCMR2 = TIM_OCMode_PWM1 | (TIM_OCMode_PWM1 << 8) | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
    tim->CCER |= TIM_CCER_CC3E; // JUST ENABLING CCR3 FOR NOW

    tim->DCR  = TIM_DMABase_CCR3 | TIM_DMABurstLength_1Transfer; // single transfers to CCR3. Since the revo's pwm out 1 is tied to TIM3-CHN3
    tim->DIER = TIM_DIER_UDE;

    tim->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
}

void DSHOT_OUT::write(float value) {

    if ((config_->tx_dma_stream->CR & DMA_SxCR_EN) == 0) {

        uint16_t packet = prepareDshotPacket(value);

        for (uint8_t i = 0; i < 16; i++) {
            // here we're converting each bit of the outgoing data packet into timer values
            // put timer values into our DMA out buffer. DMA will automatically 
            // feed these to the timer
            // Dshot is MSB first
            out_buffer_[i] = (packet & 0x8000) ? DSHOT_BIT_1_CYCLES_COUNT : DSHOT_BIT_0_CYCLES_COUNT;
            packet <<= 1; // get ready to process the next bit
        }

        config_->tx_dma_stream->NDTR = 17; // transferring 17 bytes to TIM

        //===========================================
        //TODO: this shift depends on the stream, and is a sucky way todo this
        // also the register used
        uint32_t currentFlags = config_->tx_dma_base->LISR;
        //0x3D 
        config_->tx_dma_base->LIFCR = currentFlags & (0x3D << 16U);
        //===========================================

        config_->tx_tim->EGR |= TIM_EGR_UG; // generate an update event for the counter. this reinitializes and updates the registers
        config_->tx_dma_stream->CR |= DMA_SxCR_EN; // enable DMA
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
    if ((DMA1->LISR & 0x200000) != 0) {  //DMA_GetITStatus(stream, DMA_FLAG_TCIF2)){
        DMA_ClearITPendingBit(stream, DMA_FLAG_TCIF2);
        DMA_Cmd(stream, DISABLE);
    } 
}
}