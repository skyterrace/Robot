#ifndef __ANALOGIN_H
#define __ANALOGIN_H
#include "stm32f4xx.h"
#define ADCx                     ADC1
#define ADCx_CLK                 RCC_APB2Periph_ADC1
#define DMA_CHANNELx             DMA_Channel_0
#define DMA_STREAMx              DMA2_Stream0
#define ADCx_DR_ADDRESS          ((uint32_t)0x4001204C)
#define ADC_Channel_Num					10  //�ܹ��ж��ٸ�ͨ����ע��ADC_Config��˿ڳ�ʼ��Ҫ��Ӧ
#define ADC_Channel_SampleCount 5  //ÿ��ͨ�����ֵĲ���������5�����򻺳����Ĵ�СΪͨ����*5
#define ADC_Channel_VPOWER			ADC_Channel_10

void ADC_Config(void);
uint16_t ADC_GetPowerVoltage(void);

#endif
