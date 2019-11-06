#include "analog.h"

void Analog::init(ADC_TypeDef *adc, GPIO_TypeDef *BasePort, uint16_t pin, uint8_t adc_channel)
{
  if (!Analog::is_adc_initialized(adc))
    Analog::init_adc(adc);
  this->gpio_.init(BasePort, pin, GPIO::ANALOG);
  // Relatively speaking, we don't need to read the ADC particularly fast
  ADC_RegularChannelConfig(adc, adc_channel, get_current_channel_count(adc)+1, ADC_SampleTime_480Cycles);
  Analog::increment_channel_count(adc);
  ADC_SoftwareStartConv(adc);
}

double Analog::read()
{
  uint16_t converted_value = ADC_GetConversionValue(ADC1);
  return converted_value * REFERENCE_VOLTAGE / UINT12_MAX;
}

uint16_t Analog::read_raw()
{
  return ADC_GetConversionValue(ADC1);
}
/*
void Analog::init_adc()
{
  ADC_InitTypeDef adc_init_struct;
  ADC_StructInit(&adc_init_struct);
  adc_init_struct.ADC_Resolution = ADC_Resolution_12b;
  adc_init_struct.ADC_ScanConvMode = ENABLE;
  adc_init_struct.ADC_ContinuousConvMode = ENABLE;
  adc_init_struct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
  adc_init_struct.ADC_NbrOfConversion = 1;
  ADC_Cmd(ADC1, ENABLE);
  ADC_ContinuousModeCmd(ADC1, ENABLE);
  ADC_Init(ADC1, &adc_init_struct);
  //adc_started_ = true;
}
*/
static uint8_t Analog::get_current_channel_count(ADC_TypeDef *adc)
{
  uint32_t length = adc->SQR1 & (SQR1_L_MASK);
  length >> Analog::SQR1_L_OFFSET;
  return length;
}
static bool Analog::is_adc_initialized(ADC_TypeDef *adc)
{
  if (adc->CR2 | 1)
    return true;
  else
    return false;
}
static void Analog::init_adc(ADC_TypeDef *adc)
{
  ADC_InitTypeDef adc_init_struct;
  ADC_StructInit(&adc_init_struct);
  adc_init_struct.ADC_Resolution = ADC_Resolution_12b;
  adc_init_struct.ADC_ScanConvMode = ENABLE;
  adc_init_struct.ADC_ContinuousConvMode = ENABLE;
  adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
  adc_init_struct.ADC_NbrOfConversion = 0;
  ADC_Init(adc,&adc_init_struct);
  ADC_ContinuousModeCmd(adc, ENABLE);
  ADC_Cmd(adc, ENABLE);
}
static void Analog::increment_channel_count(ADC_TypeDef *adc)
{
  uint8_t length = Analog::get_current_channel_count(adc)+1;
  adc->SQR1 &=(~SQR1_L_MASK);
  adc->SQR1 |=((length>>SQR1_L_OFFSET)|SQR1_L_MASK);
}
