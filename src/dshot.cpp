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

    TIMPtr = TIM3;

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
    DMA_Cmd(DMA2_Stream0, DISABLE); // disable while we configure
    DMA_DeInit(DMA2_Stream0);

    // set it up so it manages the packets going out to the ESC
    DMA_InitTypeDef  DMA_InitStructure_;
    DMA_InitStructure_.DMA_Channel = DMA_Channel_6; // TODO: not sure how much this matters yet
    DMA_InitStructure_.DMA_PeripheralBaseAddr = reinterpret_cast<uintptr_t>(&TIMPtr->DMAR); /*!< TIM DMA address for full transfer,  */
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

    printf("\nHere's the output buff::\n");
    for (int i= 0; i < 17; i++) {
        printf("%04x\n", out_buffer_[i]);
    }

    // the packet is now setup in our output buffer. now we just need to enable DMA
    // and tell it to send it out

    DMA_Cmd(DMA2_Stream0, ENABLE);
    
    // restart the timer's counter so it properly handles the new packet:
    TIM_GenerateEvent(TIMPtr, TIM_EventSource_Update);
    printf("wait...");
    printf("...");
    printf("...\n");
    /*
    steps in simple example:
    DMA2->HIFCR = DMA2->HISR; // WHY IS THIS DONE?
    */

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