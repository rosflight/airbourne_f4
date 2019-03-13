#include "dshot.h"

#include "printf.h" //todo: delete me

// The timer frequency is scaled so that these are always true
#define DSHOT_PERIOD_CYCLES_COUNT          70
#define DSHOT_BIT_0_CYCLES_COUNT           26
#define DSHOT_BIT_1_CYCLES_COUNT           53
#define DSHOT_MIN_THROTTLE                 48 // there are some lower values (0-47) that are reserved by the protocol
#define DSHOT_MAX_THROTTLE                 2047 // 11 bits of all 1s

DSHOT_OUT::DSHOT_OUT(){}

void DSHOT_OUT::init(int dshot_bitrate) {

    request_telemetry_ = false;

    GPIO_InitTypeDef gpio_init_struct;
    TIM_TimeBaseInitTypeDef tim_init_struct;
    TIM_OCInitTypeDef tim_oc_init_struct;

    TIMPtr = TIM1;

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
    gpio_init_struct.GPIO_Speed	= GPIO_Speed_100MHz; // aka "High Speed"
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;
    gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
    GPIO_Init(port_, &gpio_init_struct);
    GPIO_PinAFConfig(port_, pin_, GPIO_AF_TIM3);

    ////////////////////////////
    //      SETUP DMA        //
    //////////////////////////
    DMA_Cmd(DMA2_Stream0, DISABLE); // disable while we configure
    DMA_DeInit(DMA2_Stream0);

    // set it up so it manages the packets going out to the ESC
    DMA_InitTypeDef  DMA_InitStructure_;
    DMA_InitStructure_.DMA_Channel = DMA_Channel_6; // TODO: not sure how much this matters yet
    DMA_InitStructure_.DMA_PeripheralBaseAddr = reinterpret_cast<uintptr_t>(&TIMPtr->DMAR); /*!< TIM DMA address for full transfer,  */
    DMA_InitStructure_.DMA_Memory0BaseAddr    = reinterpret_cast<uintptr_t>(out_buffer_); // base address of your dma out array (address of first element)
    DMA_InitStructure_.DMA_DIR = DMA_DIR_MemoryToPeripheral; // going from memory to esc
    DMA_InitStructure_.DMA_BufferSize = DSHOT_OUT_BUFF_SIZE; // how many elements are in your array (aka how many elements in dshot's out_buff_)?
    DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // dont increment the address to the peripheral
    DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable; // should the memory address be incremented (after we send a block?)
    DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // peripheral datasize (what size of thing is peripheral expecting?) half-word == 16 bits
    DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // size of memory chunks in buffer we're sending
    DMA_InitStructure_.DMA_Priority = DMA_Priority_VeryHigh; // timely motor control is mildly important ;)
    // fifo mode
    // fifo threshold
    DMA_InitStructure_.DMA_MemoryBurst = DMA_MemoryBurst_Single; // burst transfer a single block
    DMA_InitStructure_.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure_);

    ////////////////////////////
    //      SETUP TIMER      //
    //////////////////////////
    // figure out the output timer values
    // This is dependent on how fast the SystemCoreClock is. (ie will change between stm32fX models)
    // "The prescaler can divide the counter clock frequency by any factor between 1 and 65536"
    const uint16_t prescaler_default = 1; //GOTTA GO FASTTTTT
    uint32_t freq_prescale = prescaler_default * 2;

    // this is the base DSHOT12000 frequency --> fast as possible
    uint32_t timer_freq_hz = SystemCoreClock / freq_prescale; //84,000,000
    uint16_t tim_prescaler = 0; // default to 0 prescaler
    printf("Sys clk:: %d\n", SystemCoreClock);
    printf("DSHOT1200 freq:: %d\n", timer_freq_hz);
    dshot_freq_hz = timer_freq_hz;

    // TODO: temp, change bitrate input param to an enum
    switch (dshot_bitrate) {
        case 600:
            timer_freq_hz /= 2; // cut dshot1200 freq in half for dshot600
            tim_prescaler = 2;
            break;
        case 300:
            timer_freq_hz /= 4; // cut dshot1200 freq in 4 for dshot300
            tim_prescaler = 4;
            break;
        case 150:
            timer_freq_hz /= 8; // cut dshot1200 freq in 8 for dshot150
            tim_prescaler = 8;
            break;
    }

    // Setup timer prescaler for version of DSHOT we're using
    //      Objective: scale TIM properly (for each DSHOT speed) so that it is 
    //      always DSHOT_PERIOD_CYCLES_COUNT per bit
    // uint32_t tim_prescaler = timer_freq_hz / dshot_freq_hz;

    // TODO: not worrying about timer double freq right now, may need to change
    // if (TIMPtr == TIM9 || TIMPtr == TIM10 || TIMPtr == TIM11)
    // {
    //     //For F4's (possibly others) TIM9-11 have a max timer clk double that of all the other TIMs
    //     //compensate for this by doubling its prescaler
    //     tim_prescaler = tim_prescaler * 2;
    // }
    // prescaling is 0 based, so subtract 1
    // tim_prescaler = tim_prescaler != 0 ? tim_prescaler-- : tim_prescaler;

    printf("init1 bit period:: %d\n", DSHOT_PERIOD_CYCLES_COUNT - 1);
    printf("init1 PSC div: %d\n", tim_prescaler);

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
    // PWM1 OCMode specifies that the channel outputs a 1 as long as the TIM counter is 
    //      < our reference value (which was moved by DMA into CCR1 to represent a bit length)
    //      Otherwise, its set to a 0
    tim_oc_init_struct.TIM_OCMode 		= TIM_OCMode_PWM1;
    // init TIM_CCRx to 0 -> this will be modified by DMA when we actually wanna transfer crap
    tim_oc_init_struct.TIM_Pulse 		= 0; 
    // sets CC1E (Capture-compare 1 enable bit) -> tells it to use the signal from OC1 as an output to our specified output pin
    tim_oc_init_struct.TIM_OutputState 	= TIM_OutputState_Enable; 
    // i'm not sure that these next 3 options matter all that much. But it's what we set 
    //      in the PWM code so do it here ¯\_(ツ)_/¯ 
    // tim_oc_init_struct.TIM_OutputNState = TIM_OutputNState_Disable;
    // tim_oc_init_struct.TIM_OCPolarity 	= TIM_OCPolarity_Low;
    // tim_oc_init_struct.TIM_OCIdleState 	= TIM_OCIdleState_Set;
    TIM_OC1Init(TIMPtr, &tim_oc_init_struct);

    // enable OC1 Preload. On each update event, the value is loaded into the active register
    //      afaik, this helps guarantee safe transitions between CCR, so that they occur precisely
    //          on update events
    TIM_OC1PreloadConfig(TIMPtr, TIM_OCPreload_Enable); 

    // The structs presented by the StdPeriphDriver code dont expose functionality
    //      to all the weird stuff we need todo with the timer, so we've gotta
    //      directly setup certain registers

    // modify the timer dma control register (TIMx_DCR)
    //      0x34 is the base address of TIMx_CCR1 - which is where we want to start
    //          the dma transfers. This goes into DCR.DBA - 'DMA Base Address'
    //      ST does an example on page 644 of "STM43F405 etc reference.pdf", but set it to 0xe for some reason
    //      17 Transfers since that is the size of our output buff: 
    //          16 for the actual packet, 1 to ensure proper spacing between packet transmission
    TIM_DMAConfig(TIMPtr, TIM_DMABase_CCR1, TIM_DMABurstLength_1Transfer);

    // set the UDE bit in DMA Interrupt Enable Register (DIER)
    // UDE is 'Update DMA request Enable' (turns on the DMA interrupt triggered by the timer)
    TIM_DMACmd(TIMPtr, TIM_DMA_Update, ENABLE);

    if (TIMPtr == TIM1 || TIMPtr == TIM8) {
        // TIM1 and 8 are 'advanced timers' with even MORE features (somehow..)
        // we need to tell them to enable the main output 
        // set the MOE (Main Output Enable) bit:
        //      "OC and OCN outputs are enabled if their respective enable bits are set"
        TIM_CtrlPWMOutputs(TIMPtr, ENABLE);
    }

    TIM_ARRPreloadConfig(TIMPtr, ENABLE); // enable auto-preload of TIMx_ARR register (TIM1->CR1)
    TIM_Cmd(TIMPtr, ENABLE); // ready to turn it on!
}

void DSHOT_OUT::init2(int dshot_bitrate) {
    request_telemetry_ = false;
    out_buffer_[16] = 0;
    printf("init2 stuff\n");

    port_ = GPIOB;
    pin_  = GPIO_Pin_0;


    // gpio stuff below is equivalent to this, but for GPIOB.pin0 instead of GPIOA.pin
    //    and sets up for TIM3 instead of TIM0
    // GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos);       // AF01
    // GPIOA->MODER |= (2 << GPIO_MODER_MODE8_Pos);        // Alternate function
    // GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED8_Pos);  // Fast mode

    // Configure GPIO as alternative function, fast speed (not high speed)
    GPIO_InitTypeDef gpio_init_struct;
    GPIO_PinAFConfig(port_, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(port_, pin_, GPIO_AF_TIM1);

    gpio_init_struct.GPIO_Pin 	= pin_;
    gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF; // AF is "alternative function"
    gpio_init_struct.GPIO_Speed	= GPIO_Speed_100MHz; // aka "High Speed"
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;
    gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
    GPIO_Init(port_, &gpio_init_struct);

    // following dma is equivalent to this::
    // DMA2_Stream0->CR = (6 << DMA_SxCR_CHSEL_Pos) | // set to channel 6
    //                    (1 << DMA_SxCR_DIR_Pos) | // mem to peripheral
    //                    (1 << DMA_SxCR_MINC_Pos) | //enable memory increment
    //                    (0 << DMA_SxCR_PINC_Pos) | //disable peripheral increment
    //                    (2 << DMA_SxCR_MSIZE_Pos) | // in this case we're doing half word for peripheral
    //                    (2 << DMA_SxCR_PSIZE_Pos) |// and memory size (how he does it in official firmware)
    //                    (3 << DMA_SxCR_PL_Pos);
    DMA_Cmd(DMA2_Stream5, DISABLE); // disable while we configure
    DMA_DeInit(DMA2_Stream5);
    DMA_InitTypeDef  DMA_InitStructure_;
    DMA_StructInit(&DMA_InitStructure_);

    DMA_InitStructure_.DMA_Channel = DMA_Channel_6; // TODO: not sure how much this matters yet
    DMA_InitStructure_.DMA_DIR = DMA_DIR_MemoryToPeripheral; // going from memory to esc
    DMA_InitStructure_.DMA_MemoryInc = DMA_MemoryInc_Enable; // should the memory address be incremented (after we send a block?)
    DMA_InitStructure_.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // dont increment the address to the peripheral
    DMA_InitStructure_.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // peripheral datasize (what size of thing is peripheral expecting?) half-word == 16 bits
    DMA_InitStructure_.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // size of memory chunks in buffer we're sending
    DMA_InitStructure_.DMA_Priority = DMA_Priority_VeryHigh; // timely motor control is mildly important ;)
    DMA_Init(DMA2_Stream5, &DMA_InitStructure_);



    // Write Timer DMAR address
    DMA2_Stream5->PAR = reinterpret_cast<uintptr_t>(&TIM1->DMAR);

    // Set the address to the memory buffer
    DMA2_Stream5->M0AR = reinterpret_cast<uintptr_t>(payload_.data());

    //
    // Setup timer PWM mode
    //

    // Reset settings
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;

    // Configure the period
    static const constexpr unsigned TIM_RATE = 100000000;
    uint16_t bit_period_ = (TIM_RATE + 1200000/2) / 1200000;
    TIM1->ARR = bit_period_ - 1;

    printf("init2 bit period:: %d\n", bit_period_);

    // Configure the Timer prescaler
    uint16_t div = 1200000 / dshot_bitrate - 1;
    TIM1->PSC = div;

    printf("init2 PSC div: %d\n", div);

    // Configure pulse width
    TIM1->CCR1 = 0;

    // Enable auto-reload Preload
    TIM1->CR1 |= TIM_CR1_ARPE;

    //
    // Set PWM mode
    //

    // Select the output compare mode 1
    // Enable output compare 1 Preload
    // equivalent to this:
    // TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) |
    //               (1 << TIM_CCMR1_OC1PE_Pos);

    TIM_OCInitTypeDef tim_oc_init_struct;
    TIM_OCStructInit(&tim_oc_init_struct);
    tim_oc_init_struct.TIM_OCMode 		= TIM_OCMode_PWM1;
    TIM_OC1Init(TIM1, &tim_oc_init_struct);
    TIM_OC1PreloadConfig(TIMPtr, TIM_OCPreload_Enable);

    // Enable the TIM1 Main Output
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Enable CC1 output
    TIM1->CCER |= TIM_CCER_CC1E;

    //
    // Setup Timer DMA settings
    //

    // Configure of the DMA Base register to CCR1 and the DMA Burst Length to 1
    // TIM1->DCR = (0 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);
    TIM_DMAConfig(TIM1, TIM_DMABase_CCR1, TIM_DMABurstLength_1Transfer);

    // TIM1 DMA Update enable
    TIM1->DIER |= TIM_DIER_UDE;

    // Enable the TIM Counter
    TIM1->CR1 |= TIM_CR1_CEN;
}

void DSHOT_OUT::init3() {

    // objective, just get the tim and gpio working with CCR
    //  THEN screw with DMA hookup
    printf("init 3 bb\n");
    request_telemetry_ = false;

    // GPIO_InitTypeDef gpio_init_struct;
    // TIM_TimeBaseInitTypeDef tim_init_struct;
    // TIM_OCInitTypeDef tim_oc_init_struct;

    // TIMPtr = TIM3;

    // ////////////////////////////
    // // SETUP GPIO FOR OUTPUT //
    // //////////////////////////
    // // TODO: parameterize. rn just going to pwm output 0
    // port_ = GPIOB;
    // pin_  = GPIO_Pin_0;

    // // Configure GPIO as alternative function, fast speed (not high speed)
    // gpio_init_struct.GPIO_Pin 	= pin_;
    // gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF; // AF is "alternative function"
    // gpio_init_struct.GPIO_Speed	= GPIO_Speed_100MHz; // aka "High Speed"
    // gpio_init_struct.GPIO_OType = GPIO_OType_PP;
    // gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_DOWN; //GPIO_PuPd_DOWN
    // GPIO_Init(port_, &gpio_init_struct);
    // GPIO_PinAFConfig(port_, GPIO_PinSource0, GPIO_AF_TIM3);

    // prescaler = tim_frequency / 

    // TIM_DeInit(TIMPtr);
    // TIMPtr->ARR = DSHOT_PERIOD_CYCLES_COUNT - 1; // period 
    // TIMPtr->PSC = 15; // 16(0 based index), prescaler for dshot150
    // // TIM_TimeBaseStructInit(&tim_init_struct);
    // // tim_init_struct.TIM_Period 		  = DSHOT_PERIOD_CYCLES_COUNT - 1; // 0 indexed, goes into TIMx_ARR register
    // // tim_init_struct.TIM_Prescaler 	  = 1; // (sys clk / dshot_freq) - 1
    // // TIM_TimeBaseInit(TIMPtr, &tim_init_struct);

    // TIMPtr->CCR1 = 0; // just reset the CCR register so we can put a new value in it later

    // TIMPtr->CCMR1 = TIM_OCMode_PWM1 | TIM_OCPreload_Enable;
    // TIMPtr->CCER = 0x1; // enable CC1E output compare 1

    // TIMPtr->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | 0x1; // enable ARPE, URS and enable TIM

//////////////////////////////////////

    GPIO_InitTypeDef gpio_init_struct;
  TIM_TimeBaseInitTypeDef tim_init_struct;
  TIM_OCInitTypeDef tim_oc_init_struct;

  port_ = GPIOB;
  pin_  = GPIO_Pin_0;

  GPIO_PinAFConfig(port_, GPIO_PinSource0, GPIO_AF_TIM3);

  gpio_init_struct.GPIO_Pin 	= pin_;
  gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF;
  gpio_init_struct.GPIO_Speed	= GPIO_Speed_100MHz;
  gpio_init_struct.GPIO_OType   = GPIO_OType_PP;
  gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
  GPIO_Init(port_, &gpio_init_struct);

  TIMPtr = TIM3;

  //init timer
  TIM_DeInit(TIMPtr);
  TIM_TimeBaseStructInit(&tim_init_struct);
  tim_init_struct.TIM_Period 		  = DSHOT_PERIOD_CYCLES_COUNT - 1; // period is the same for all dshot types - we change the prescaler accordingly
  tim_init_struct.TIM_Prescaler 	  = 2 - 1; //for dshot600
  tim_init_struct.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &tim_init_struct);
  TIMPtr->CR1 |= TIM_CR1_URS;

  //init output compare
  TIM_OCStructInit(&tim_oc_init_struct);
  tim_oc_init_struct.TIM_OCMode 		= TIM_OCMode_PWM1;
  tim_oc_init_struct.TIM_OutputState 	= TIM_OutputState_Enable;
  tim_oc_init_struct.TIM_OutputNState = TIM_OutputNState_Disable;
//   tim_oc_init_struct.TIM_Pulse 		    = min_cyc_ - 1;
  tim_oc_init_struct.TIM_OCPolarity 	= TIM_OCPolarity_High;
  tim_oc_init_struct.TIM_OCIdleState 	= TIM_OCIdleState_Reset;

  switch (TIM_Channel_3)
  {
  case TIM_Channel_1:
  default:
    TIM_OC1Init(TIMPtr, &tim_oc_init_struct);
    TIM_OC1PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
    CCR_ = &TIMPtr->CCR1;
    break;
  case TIM_Channel_2:
    TIM_OC2Init(TIMPtr, &tim_oc_init_struct);
    TIM_OC2PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
    CCR_ = &TIMPtr->CCR2;
    break;
  case TIM_Channel_3:
    TIM_OC3Init(TIMPtr, &tim_oc_init_struct);
    TIM_OC3PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
    CCR_ = &TIMPtr->CCR3;
    break;
  case TIM_Channel_4:
    TIM_OC4Init(TIMPtr, &tim_oc_init_struct);
    TIM_OC4PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
    CCR_ = &TIMPtr->CCR4;
    break;
  }

  TIM_ARRPreloadConfig(TIMPtr, ENABLE);
  TIM_Cmd(TIMPtr, ENABLE);

}

void DSHOT_OUT::write(float value) {
    
    uint16_t packet = prepareDshotPacket(value);

    for (uint8_t i = 0; i < 16; i++) {
        // here we're converting each bit of the outgoing data packet into timer values
        // put timer values into our DMA out buffer. DMA will automatically 
        // feed these to the timer
        // Dshot is MSB first
        out_buffer_[i] = (packet & 0x8000) ? DSHOT_BIT_1_CYCLES_COUNT : DSHOT_BIT_0_CYCLES_COUNT;
        packet <<= 1; // get ready to process the next bit
    }

    out_buffer_[16] = 0;

    printf("\nHere's the output buff::\n");
    for (int i= 0; i < 17; i++) {
        printf("%04x\n", out_buffer_[i]);
    }

    // the packet is now setup in our output buffer. now we just need to enable DMA
    // and tell it to send it out
    TIM1->CCR1 = out_buffer_[0];

    DMA_SetCurrDataCounter(DMA2_Stream0, DSHOT_OUT_BUFF_SIZE);
    DMA2->HIFCR = DMA2->HISR;
    DMA_Cmd(DMA2_Stream0, ENABLE);
    
    // restart the timer's counter so it properly handles the new packet:
    TIM1->EGR |= TIM_EGR_UG;
    // TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
    printf("wait...");
    printf("...");
    printf("...\n");

}

void DSHOT_OUT::write2(float value) {

    // if (*CCR_ == DSHOT_BIT_1_CYCLES_COUNT) {
        // *CCR_ = DSHOT_BIT_0_CYCLES_COUNT;
    // } else {
        *CCR_ = DSHOT_BIT_1_CYCLES_COUNT;
    // }


    // constexpr const auto bit_period_ = (TIM_RATE + 1200000/2) / 1200000;
    // constexpr const auto bit_0_ = (bit_period_ * 1 + 1) / 3 - 1;
    // constexpr const auto bit_1_ = (bit_period_ * 3 + 2) / 4 - 1;

    // uint16_t packet = prepareDshotPacket(value);

    // for (auto i = 0; i < 16; i++)
    // {
    //   // Dshot is MSB first
    //   payload_[i] = (packet & 0x8000) ? bit_1_ : bit_0_;
    //   packet <<= 1;
    // }

    // DMA2_Stream5->NDTR = payload_.size();
    // DMA2->HIFCR = DMA2->HISR;
    // DMA2_Stream5->CR |= DMA_SxCR_EN;

    // // Reset and update Timer registers
    // TIM1->EGR |= TIM_EGR_UG;
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