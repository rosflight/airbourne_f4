#include "analog_pin.h"

void AnalogPin::init(AnalogDigitalConverter *adc, GPIO_TypeDef *BasePort, uint16_t pin, uint8_t adc_channel)
{
  this->adc_ = adc;
  this->gpio_.init(BasePort, pin, GPIO::ANALOG);
  this->rank = this->adc_->add_channel(adc_channel);
}

double AnalogPin::read()
{
  return this->read_raw() * AnalogDigitalConverter::REFERENCE_VOLTAGE / AnalogDigitalConverter::RAW_READING_MAX;
}

uint16_t AnalogPin::read_raw()
{
  return this->adc_->read(this->rank);
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
/*
static uint8_t AnalogPin::get_current_channel_count(ADC_TypeDef *adc)
{
  uint32_t length = adc->SQR1 & (SQR1_L_MASK);
  length >> AnalogPin::SQR1_L_OFFSET;
  return length;
}
static bool AnalogPin::is_adc_initialized(ADC_TypeDef *adc)
{
  if (adc->CR2 | 1)
    return true;
  else
    return false;
}
static void AnalogPin::init_adc(adc_hardware_struct_t *adc_def)
{
  ADC_TypeDef *adc = adc_def->adc;
  ADC_InitTypeDef adc_init_struct;
  ADC_StructInit(&adc_init_struct);
  adc_init_struct.ADC_Resolution = ADC_Resolution_12b;
  adc_init_struct.ADC_ScanConvMode = ENABLE;
  adc_init_struct.ADC_ContinuousConvMode = ENABLE;
  adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
  adc_init_struct.ADC_NbrOfConversion = 0;
  ADC_Init(adc,&adc_init_struct);
  AnalogPin::init_dma(adc_def);
  ADC_ContinuousModeCmd(adc, ENABLE);
  ADC_Cmd(adc, ENABLE);
}
static void AnalogPin::init_dma(adc_hardware_struct_t *adc_def)
{
  DMA_InitTypeDef dma_init_struct;
  DMA_StructInit(&dma_init_struct);
  dma_init_struct.DMA_Channel = adc_def->DMA_channel;
  dma_init_struct.DMA_PeripheralBaseAddr = adc_def->adc->DR;


}
static void AnalogPin::increment_channel_count(ADC_TypeDef *adc)
{
  uint8_t length = AnalogPin::get_current_channel_count(adc)+1;
  adc->SQR1 &=(~SQR1_L_MASK);
  adc->SQR1 |=((length>>SQR1_L_OFFSET)|SQR1_L_MASK);
}
*/
