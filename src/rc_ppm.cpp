#include "rc_ppm.h"

RC_PPM* RC_PPM_Ptr = NULL;

void RC_PPM::init()
{
  // Initialize Variables
  for (int i = 0; i < PWM_NUM_RC_INPUTS; i++)
    rc_raw_[i] = 0;
  chan_ = 0;
  current_capture_ = 0;
  last_capture_ = 0;
  last_pulse_ms_ = 0;

  // Connect the global pointer
  RC_PPM_Ptr = this;

  // Set up the Pin
  pin_.init(GPIOB, GPIO_Pin_0, GPIO::PERIPH_IN);

  // Connect Pin to the timer peripherial
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);

  // Configure the timer
  TIM_TimeBaseInitTypeDef TIM_init_struct;
  TIM_init_struct.TIM_Period = 0xFFFF; // It'll get reset by the CC
  TIM_init_struct.TIM_ClockDivision = TIM_CKD_DIV1; // No clock division
  TIM_init_struct.TIM_Prescaler = (SystemCoreClock / (2000000)) - 1; // prescaler (0-indexed), set to 1 MHz
  TIM_init_struct.TIM_CounterMode = TIM_CounterMode_Up; // count up
  TIM_TimeBaseInit(TIM3, &TIM_init_struct);

  // Configure the Input Compare Peripheral
  TIM_ICInitTypeDef TIM_IC_init_struct;
  TIM_IC_init_struct.TIM_Channel = TIM_Channel_3;
  TIM_IC_init_struct.TIM_ICFilter = 0x00;
  TIM_IC_init_struct.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_IC_init_struct.TIM_ICPrescaler =  TIM_ICPSC_DIV1;
  TIM_IC_init_struct.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInit(TIM3, &TIM_IC_init_struct);

  // Set up the interrupt for the Timer
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable the CC interrupt
  TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);

  // Start Counting!
  TIM_Cmd(TIM3, ENABLE);
}

float RC_PPM::read(uint8_t channel)
{
  return (float)(rc_raw_[channel] - 1000)/1000.0;
}

bool RC_PPM::lost()
{
  return millis() > last_pulse_ms_ + 100;
}

void RC_PPM::pulse_callback()
{
  if(TIM_GetITStatus(TIM3, TIM_IT_CC3))
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    last_pulse_ms_ = millis();

    current_capture_ = TIM_GetCapture3(TIM3);
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
      if(diff > 750 && diff < 2250 && chan_ < PWM_NUM_RC_INPUTS)
      {
        rc_raw_[chan_] = diff;
      }
      chan_++;
    }
    if (chan_ > PWM_NUM_RC_INPUTS)
    {
      TIM_SetCounter(TIM3, 0);
    }
  }
}

extern "C"
{

void TIM3_IRQHandler(void)
{
    if(RC_PPM_Ptr != NULL)
    {
        RC_PPM_Ptr->pulse_callback();
    }
}

}
