#include "dshot.h"

// The timer frequency is scaled so that these are always true
#define DSHOT_PERIOD_CYCLES_COUNT          70
#define DSHOT_BIT_0_CYCLES_COUNT           26
#define DSHOT_BIT_1_CYCLES_COUNT           53

DSHOT_OUT::DSHOT_OUT(){}

void DSHOT_OUT::init(int dshot_bitrate) {

    request_telemetry_ = false;
    max_throttle_val_ = 2047; // 11 bits of all 1s
    min_throttle_val_ = 48; // there are some lower values (0-47) that are reserved by the protocol

    GPIO_InitTypeDef gpio_init_struct;
    TIM_TimeBaseInitTypeDef tim_init_struct;
    TIM_OCInitTypeDef tim_oc_init_struct;

    TIM_TypeDef* TIMPtr = TIM3;

    ////////////////////////////
    // SETUP GPIO FOR OUTPUT //
    //////////////////////////
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

    ////////////////////////////
    //      SETUP DMA        //
    //////////////////////////
    // disable while we configure
    DMA_Cmd(DMA2_Stream0, DISABLE);
    DMA_DeInit(DMA2_Stream0);

    // set it up so it manages the packets going out to the ESC
    DMA_InitTypeDef  DMA_InitStructure_;
    DMA_InitStructure_.DMA_Channel = DMA_Channel_6; // TODO: not sure how much this matters yet
    DMA_InitStructure_.DMA_PeripheralBaseAddr = reinterpret_cast<uintptr_t>(&TIMPtr->DMAR); /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
    DMA_InitStructure_.DMA_Memory0BaseAddr    = reinterpret_cast<uintptr_t>(&out_buffer_[0]); // base address of your dma out array (address of first element)
    DMA_InitStructure_.DMA_DIR = DMA_DIR_MemoryToPeripheral; // going from memory to esc
    DMA_InitStructure_.DMA_BufferSize = DSHOT_OUT_BUFF_SIZE; // how many elements are in your array (aka how many elements in dshot's out_buff_)?
    DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // dont increment the address to the peripheral
    DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable; // should the memory address be incremented (after we send a block?)
    DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // peripheral datasize (what size of thing is peripheral expecting?) half-word == 16 bits
    DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // size of memory chunks in buffer we're sending
    DMA_InitStructure_.DMA_Priority = DMA_Priority_VeryHigh; // timely motor control is mildly important ;)
    // fifo mode
    // fifo threshold
    DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single; // burst transfer a single block
    DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;


    // figure out the output timer values
    // This is dependent on how fast the SystemCoreClock is. (ie will change between stm32fX models)
    // "The prescaler can divide the counter clock frequency by any factor between 1 and 65536"
    const uint16_t prescaler_default = 1; //GOTTA GO FASTTTTT
    uint32_t freq_prescale = prescaler_default * 2;

    // this is the base DSHOT12000 frequency --> fast as possible
    uint32_t timer_freq_hz = SystemCoreClock / freq_prescale; //84,000,000
    dshot_freq_hz = timer_freq_hz;

    // TODO: temp, change bitrate input param to an enum
    switch (dshot_bitrate) {
        case 600:
            timer_freq_hz /= 2; // cut dshot1200 freq in half for dshot600
            break;
        case 300:
            timer_freq_hz /= 4; // cut dshot1200 freq in 4 for dshot300
            break;
        case 150:
            timer_freq_hz /= 8; // cut dshot1200 freq in 8 for dshot150
            break;
    }

    // Setup timer prescaler for version of DSHOT we're using
    //      Objective: scale TIM properly (for each DSHOT speed) so that it is 
    //      always DSHOT_PERIOD_CYCLES_COUNT per bit
    uint32_t tim_prescaler = timer_freq_hz / dshot_freq_hz;

    if (TIMPtr == TIM9 || TIMPtr == TIM10 || TIMPtr == TIM11)
    {
        //For F4's (possibly others) TIM9-11 have a max timer clk double that of all the other TIMs
        //compensate for this by doubling its prescaler
        tim_prescaler = tim_prescaler * 2;
    }
    // prescaling is 0 based, so subtract 1
    tim_prescaler -= 1;

    // calculate how long the timer needs to run for a 0 bit and a 1 bit in DSHOT
    //   make sure we round properly before truncate with a cast. it's probably good to 
    //   make the pulse slightly longer than necessary
    //   Bit length (total timing period) is 1670 nanoseconds
    //   For a bit to be 0, the pulse width is 625 nanosecond
    //   For a bit to be 1, the pulse width is 1250 nanoseconds

    // setup the timer:
    // we need to tell the timer to trigger the DMA once it counts up to the value it was told
    TIM_TimeBaseStructInit(&tim_init_struct);
    tim_init_struct.TIM_Period 		  = DSHOT_PERIOD_CYCLES_COUNT - 1; // 0 indexed, goes into TIMx_ARR register
    tim_init_struct.TIM_Prescaler 	  = tim_prescaler;
    tim_init_struct.TIM_ClockDivision = TIM_CKD_DIV1; //0x0000
    tim_init_struct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMPtr, &tim_init_struct);

    // SETUP TIM OUTPUT COMPARE
    //    objective - tim runs for DSHOT_PERIOD_CYCLES_COUNT at which point it
    //      triggers DMA and gets the next time value and starts again
    TIM_OCStructInit(&tim_oc_init_struct);
    tim_oc_init_struct.TIM_OCMode 		= TIM_OCMode_PWM2;
    tim_oc_init_struct.TIM_OutputState 	= TIM_OutputState_Enable;
    tim_oc_init_struct.TIM_OutputNState = TIM_OutputNState_Disable;
    tim_oc_init_struct.TIM_Pulse 		= min_cyc_ - 1;
    tim_oc_init_struct.TIM_OCPolarity 	= TIM_OCPolarity_Low;
    tim_oc_init_struct.TIM_OCIdleState 	= TIM_OCIdleState_Set;

    // The structs presented by the StdPeriphDriver code dont expose functionality
    //      to all the weird stuff we need todo with the timer, so we've gotta
    //      directly setup certain registers

    // modify the timer dma control register (TIMx_DCR)
    //      I'm not positive why I'm setting DCR.DBA = 0xe here other than
    //      that's what ST does in their example on page 644 of "STM43F405 etc reference.pdf"
    TIM_DMAConfig(TIMPtr, 0xe, DSHOT_OUT_BUFF_SIZE);

    /*
    // Configure the period
    constexpr const auto bit_period_ = (TIM_RATE + 1200000/2) / 1200000;
    TIM1->ARR = bit_period_ - 1;
    // Configure the Timer prescaler
    const auto div = 1200000 / bitrate - 1;
    TIM1->PSC = div;
    // Configure pulse width
    TIM1->CCR1 = 0;
    // Enable auto-reload Preload
    TIM1->CR1 |= TIM_CR1_ARPE;
    //
    // Set PWM mode
    //
    // Select the output compare mode 1
    // Enable output compare 1 Preload
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) |
                  (1 << TIM_CCMR1_OC1PE_Pos);
    // Enable the TIM1 Main Output
    TIM1->BDTR = TIM_BDTR_MOE;
    // Enable CC1 output
    TIM1->CCER = TIM_CCER_CC1E;
    //
    // Setup Timer DMA settings
    //
    // Configure of the DMA Base register to CCR1 and the DMA Burst Length to 1
    TIM1->DCR = (0 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);
    // TIM1 DMA Update enable
    TIM1->DIER |= TIM_DIER_UDE;
    // Enable the TIM Counter
    TIM1->CR1 |= TIM_CR1_CEN;
    */

}

float DSHOT_OUT::getNSCyc() {
    return cycles_per_ns_;
}

uint16_t DSHOT_OUT::write(float value) {
    // DELETEME: currently just returning uint16_t for debugging.. change to void

    // you fill the dma with 32 bit values of pulse widths the timer should run.
    // remember that the different bits take different amounts of time in DSHOT:
    

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