#include "rc_ppm.h"

RC_PPM* RC_PPM_Ptr = NULL;

void RC_PPM::init(const pwm_hardware_struct_t* conf)
{
  // Initialize Variables
  for (int i = 0; i < 8; i++)
    rc_raw_[i] = 0;
  chan_ = 0;
  current_capture_ = 0;
  last_capture_ = 0;
  last_pulse_ms_ = 0;

  TIM_ = conf->TIM;
  TIM_IT_ = conf->TIM_IT_CC;
  TIM_Channel_ = conf->TIM_Channel;

  // Connect the global pointer
  RC_PPM_Ptr = this;

  // Set up the Pin
  pin_.init(conf->GPIO, conf->GPIO_Pin, GPIO::PERIPH_IN);

  // Connect Pin to the timer peripherial
  GPIO_PinAFConfig(conf->GPIO, conf->GPIO_PinSource, conf->GIPO_AF_TIM);

  // Configure the timer
  TIM_TimeBaseInitTypeDef TIM_init_struct;
  TIM_init_struct.TIM_Period = 0xFFFF; // It'll get reset by the CC
  TIM_init_struct.TIM_ClockDivision = TIM_CKD_DIV1; // No clock division
  TIM_init_struct.TIM_Prescaler = (SystemCoreClock / (2000000)) - 1; // prescaler (0-indexed), set to 1 MHz
  TIM_init_struct.TIM_CounterMode = TIM_CounterMode_Up; // count up
  TIM_TimeBaseInit(TIM_, &TIM_init_struct);

  // Configure the Input Compare Peripheral
  TIM_ICInitTypeDef TIM_IC_init_struct;
  TIM_IC_init_struct.TIM_Channel = TIM_Channel_;
  TIM_IC_init_struct.TIM_ICFilter = 0x00;
  TIM_IC_init_struct.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_IC_init_struct.TIM_ICPrescaler =  TIM_ICPSC_DIV1;
  TIM_IC_init_struct.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInit(TIM_, &TIM_IC_init_struct);

  // Set up the interrupt for the Timer
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = conf->TIM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable the CC interrupt
  TIM_ITConfig(TIM_, TIM_IT_, ENABLE);

  // Start Counting!
  TIM_ARRPreloadConfig(TIM_, ENABLE);
  TIM_Cmd(TIM_, ENABLE);
}

float RC_PPM::read(uint8_t channel)
{
  return rc_raw_[channel];
}

bool RC_PPM::lost()
{
  return millis() > last_pulse_ms_ + 100;
}

void RC_PPM::pulse_callback()
{
  if(TIM_GetITStatus(TIM_, TIM_IT_))
  {
    TIM_ClearITPendingBit(TIM_, TIM_IT_);
    last_pulse_ms_ = millis();

    switch (TIM_Channel_)
    {
    case TIM_Channel_1:
        current_capture_ = TIM_GetCapture1(TIM_);
        break;
    case TIM_Channel_2:
        current_capture_ = TIM_GetCapture2(TIM_);
        break;
    case TIM_Channel_3:
        current_capture_ = TIM_GetCapture3(TIM_);
        break;
    case TIM_Channel_4:
        current_capture_ = TIM_GetCapture4(TIM_);
        break;

    }
    uint16_t diff = current_capture_ - last_capture_;
    last_capture_ = current_capture_;

    // We're on a new frame
    if(diff > 2500)
    {
      chan_ = 0;
    }
    else
    {
      // If it's a valid reading, then save it!
      if(diff > 750 && diff < 2250 && chan_ < 8)
      {
        rc_raw_[chan_] = diff;
      }
      chan_++;
    }
    if (chan_ > 8)
    {
      chan_ = 0;
      TIM_SetCounter(TIM_, 0);
    }
  }
}

extern "C"
{

void PPM_RC_IQRHandler(void)
{
    if(RC_PPM_Ptr != NULL)
    {
        RC_PPM_Ptr->pulse_callback();
    }
}

}
