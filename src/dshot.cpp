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

    DMAPtr = DMA1_Stream2;
    TIMPtr = TIM3;
    DMABasePtr = DMA1;

    // TODO:: is this needed??
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_Init(&NVIC_InitStruct);
    /// GPIO INIT: (this is based off crect.. cant find in kfly
    GPIO_DeInit(GPIOB);

    GPIO_InitTypeDef gpio_init_struct;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);

    gpio_init_struct.GPIO_Pin 	= GPIO_Pin_0;
    gpio_init_struct.GPIO_Mode 	= GPIO_Mode_AF;
    gpio_init_struct.GPIO_Speed	= GPIO_Speed_100MHz;
    gpio_init_struct.GPIO_OType = GPIO_OType_PP;
    gpio_init_struct.GPIO_PuPd 	= GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &gpio_init_struct);
    // end gpio init

    //// Allocate DMA Stream
        // enable DMA1 rcc clock
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

        // disable stream: DMA1_Stream2
            // Clear CR bits: TCIE, HTIE, TEIE, DMEIE, EN
            DMAPtr->CR &= ~(DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | 0x1); 
            while ((DMAPtr->CR & 0x1) != 0); // wait until its truly disabled
            // clear all interrupt flags:
            DMA1->LIFCR = (0x3D) << 16;

        //set all of CR to 0:
        DMAPtr->CR  = 0x00000000U;
        DMAPtr->FCR = 0x00000021U; //todo: WHY?
    // now configure DMA:
    uint32_t dmaMode = DMA_Priority_VeryHigh | DMA_DIR_MemoryToPeripheral | 
        DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_HalfWord | DMA_MemoryInc_Enable | // pburst = mburst = single = 0
        DMA_IT_TC; // enable transfer complete interrupt

    DMAPtr->M0AR = (uint32_t) out_buffer_;
    DMAPtr->PAR  = (uint32_t) &TIMPtr->DMAR;
    DMAPtr->NDTR = (uint32_t) 0;
    DMAPtr->CR   = dmaMode | DMA_Channel_5;

    out_buffer_[16] = 0;
    // END DMA INIT

    // TIM INIT:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE); // enable and then disable the reset as is done in kfly
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);

    TIMPtr->CR1  = 0;
    TIMPtr->CR2  = 0;
    TIMPtr->CNT  = 0;
    TIMPtr->CCR1 = 0;
    TIMPtr->CCR2 = 0;
    TIMPtr->CCR3 = 0;
    TIMPtr->CCR4 = 0;
    TIMPtr->CCER = 0;

    TIMPtr->ARR = DSHOT_PERIOD_CYCLES_COUNT - 1;
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
        printf("lesss go..\n");

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