#include "ADIN.h"

volatile uint16_t ADC_ConvertedValue[ADIN_CHN_COUNT];

//AD端口初始化，PC1/PC2/PC3/PA4/PA5/PC4/PC5/PB0/PB1 共9个端口
void ADIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	//配置PC1/PC2/PC3/PA4/PA5/PC4/PC5/PB0/PB1模拟输入
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
	
	//使能DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; //DMA通道1的地址 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue[0]; //DMA传送地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //传送方向
	DMA_InitStructure.DMA_BufferSize = ADIN_CHN_COUNT;           //传送内存大小，ADIN_CHN_COUNT个16位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//传送内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//ADC1转换的数据是16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//传送的目的地址是16位宽度
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	//ADC1初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC1工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//模数转换工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在扫描模式（多通道）还是单次（单通道）模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = ADIN_CHN_COUNT;               //规定了顺序进行规则转换的ADC通道的数目。这个数目的取值范围是1到16
	ADC_Init(ADC1, &ADC_InitStructure);
	
	//ADC1规则通道配置
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5);	  //通道11样时间 239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_239Cycles5);	  //通道12样时间 239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_239Cycles5);	  //通道13样时间 239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_239Cycles5);	  //通道4样时间 239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_239Cycles5);	  //通道5样时间 239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 6, ADC_SampleTime_239Cycles5);	  //通道14样时间 239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 7, ADC_SampleTime_239Cycles5);	  //通道15样时间 239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 8, ADC_SampleTime_239Cycles5);	  //通道8样时间 239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 9, ADC_SampleTime_239Cycles5);	  //通道9样时间 239.5周期


	//使能ADC1 DMA 
	ADC_DMACmd(ADC1, ENABLE);
	//使能ADC1
	ADC_Cmd(ADC1, ENABLE);	

	//ADC1自校准
	ADC_ResetCalibration(ADC1); // 初始化ADC1校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1)); //检测ADC1校准寄存器初始化是否完成
	ADC_StartCalibration(ADC1); //开始校准ADC1
	while(ADC_GetCalibrationStatus(ADC1)); //检测是否完成校准

	
}

//AD转换启动/停止
void ADIN_Enable(FunctionalState NewState)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//ADC1转换启动
}


//获取通道nChannel的AD数据
uint16_t ADIN_GetValue(uint8_t nIndex)
{
	if(nIndex<1 || nIndex > ADIN_CHN_COUNT) return 0;
	return ADC_ConvertedValue[nIndex-1];
}

