#include "ADIN.h"

volatile uint16_t ADC_ConvertedValue[ADIN_CHN_COUNT];

//AD�˿ڳ�ʼ����PC1/PC2/PC3/PA4/PA5/PC4/PC5/PB0/PB1 ��9���˿�
void ADIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	//����PC1/PC2/PC3/PA4/PA5/PC4/PC5/PB0/PB1ģ������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);		
	
	//ʹ��DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //DMAͨ��1�ĵ�ַ 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue[0]; //DMA���͵�ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //���ͷ���
	DMA_InitStructure.DMA_BufferSize = ADIN_CHN_COUNT;           //�����ڴ��С��ADIN_CHN_COUNT��16λ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//�����ڴ��ַ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//ADC1ת����������16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//���͵�Ŀ�ĵ�ַ��16λ���
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	//ADC1��ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//ģ��ת��������ɨ��ģʽ����ͨ�������ǵ��Σ���ͨ����ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת��������ɨ��ģʽ����ͨ�������ǵ��Σ���ͨ����ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = ADIN_CHN_COUNT;               //�涨��˳����й���ת����ADCͨ������Ŀ�������Ŀ��ȡֵ��Χ��1��16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	//ADC1����ͨ������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5);	  //ͨ��11��ʱ�� 239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_239Cycles5);	  //ͨ��12��ʱ�� 239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_239Cycles5);	  //ͨ��13��ʱ�� 239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_239Cycles5);	  //ͨ��4��ʱ�� 239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_239Cycles5);	  //ͨ��5��ʱ�� 239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 6, ADC_SampleTime_239Cycles5);	  //ͨ��14��ʱ�� 239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 7, ADC_SampleTime_239Cycles5);	  //ͨ��15��ʱ�� 239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 8, ADC_SampleTime_239Cycles5);	  //ͨ��8��ʱ�� 239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 9, ADC_SampleTime_239Cycles5);	  //ͨ��9��ʱ�� 239.5����


	//ʹ��ADC1 DMA 
	ADC_DMACmd(ADC1, ENABLE);
	//ʹ��ADC1
	ADC_Cmd(ADC1, ENABLE);	

	//ADC1��У׼
	ADC_ResetCalibration(ADC1); // ��ʼ��ADC1У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1)); //���ADC1У׼�Ĵ�����ʼ���Ƿ����
	ADC_StartCalibration(ADC1); //��ʼУ׼ADC1
	while(ADC_GetCalibrationStatus(ADC1)); //����Ƿ����У׼

	
}

//ADת������/ֹͣ
void ADIN_Enable(FunctionalState NewState)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//ADC1ת������
}


//��ȡͨ��nChannel��AD����
uint16_t ADIN_GetValue(uint8_t nIndex)
{
	if(nIndex<1 || nIndex > ADIN_CHN_COUNT) return 0;
	return ADC_ConvertedValue[nIndex-1];
}

