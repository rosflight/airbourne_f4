#include "analog_digital_converter.h"

void AnalogDigitalConverter::init(const adc_hardware_struct_t *adc_def)
{
  this->current_channels = 0;
  this->adc_def_ = adc_def;
  ADC_TypeDef *adc = adc_def_->adc;
  for (size_t index =0; index< CHANNEL_COUNT; index++)
    this->buffer[index]=2*RAW_READING_MAX;

  ADC_CommonInitTypeDef adc_common_init_struct;
  adc_common_init_struct.ADC_Mode = ADC_Mode_Independent;
  adc_common_init_struct.ADC_Prescaler = ADC_Prescaler_Div2;
  adc_common_init_struct.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  adc_common_init_struct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  \
  ADC_CommonInit(&adc_common_init_struct);

  ADC_InitTypeDef adc_init_struct;
  ADC_StructInit(&adc_init_struct);
  adc_init_struct.ADC_Resolution = ADC_Resolution_12b;
  adc_init_struct.ADC_ScanConvMode = ENABLE;
  adc_init_struct.ADC_ContinuousConvMode = ENABLE;
  adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
  adc_init_struct.ADC_NbrOfConversion = 1; //This can't be less than 1
  ADC_Init(adc,&adc_init_struct);
  this->init_dma();
  ADC_ContinuousModeCmd(adc, ENABLE);
  this->is_initialized_ = true;
}

void AnalogDigitalConverter::init_dma()
{
  DMA_Cmd(this->adc_def_->DMA_Stream, DISABLE);
  DMA_DeInit(this->adc_def_->DMA_Stream);
  DMA_InitTypeDef dma_init_struct;
  DMA_StructInit(&dma_init_struct);

  dma_init_struct.DMA_Channel = this->adc_def_->DMA_channel;
  dma_init_struct.DMA_PeripheralBaseAddr = reinterpret_cast<uint32_t>(&(this->adc_def_->adc->DR));
  dma_init_struct.DMA_Memory0BaseAddr = reinterpret_cast<uint32_t>(&(this->buffer));
  dma_init_struct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dma_init_struct.DMA_BufferSize = this->get_current_channel_count();
  dma_init_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma_init_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  //DR is a uint32, even though it only has up to 12 bits of data, so word transfers?
  dma_init_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  dma_init_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  dma_init_struct.DMA_Mode = DMA_Mode_Circular;
  dma_init_struct.DMA_Priority = DMA_Priority_Medium;
  dma_init_struct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dma_init_struct.DMA_FIFOMode = DMA_FIFOStatus_1QuarterFull;
  dma_init_struct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  dma_init_struct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  DMA_Init(this->adc_def_->DMA_Stream, &dma_init_struct);

  ADC_DMACmd(this->adc_def_->adc, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(this->adc_def_->adc, ENABLE);
}

uint8_t AnalogDigitalConverter::add_channel(uint8_t channel)
{
  uint8_t rank = this->get_current_channel_count() + 1;
  if (this->current_channels==0)
    rank--;
  this->current_channels++;
  ADC_RegularChannelConfig(this->adc_def_->adc, channel, rank, ADC_SampleTime_480Cycles);

  //Increment the number of channels
  this->adc_def_->adc->SQR1 &=(~SQR1_L_MASK);
  this->adc_def_->adc->SQR1 |=((rank<<SQR1_L_OFFSET)&SQR1_L_MASK);

  this->init_dma(); // This sets up the DMA to use the correct buffer size
  this->start_dma();
  ADC_Cmd(this->adc_def_->adc, ENABLE);
  ADC_SoftwareStartConv(this->adc_def_->adc);
  return rank;
}
void AnalogDigitalConverter::start_dma()
{
  DMA_Cmd(this->adc_def_->DMA_Stream, ENABLE);
}
bool AnalogDigitalConverter::is_initialized()
{
  return this->is_initialized_;
}
uint16_t AnalogDigitalConverter::read(uint8_t rank)
{
  //return (this->adc_def_->adc->DR & 0xFFFF);
  return this->buffer[rank-1];
}


uint8_t AnalogDigitalConverter::get_current_channel_count()
{
  uint32_t length = this->adc_def_->adc->SQR1 & (AnalogDigitalConverter::SQR1_L_MASK);
  length >> AnalogDigitalConverter::SQR1_L_OFFSET;
  return length+1;
}
