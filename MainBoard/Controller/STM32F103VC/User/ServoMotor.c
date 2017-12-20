#include "ServoMotor.h"

uint16_t nCHPulse[SERVOMOTOR_CHN_COUNT];  //������һ�����õ�ͨ��CH1��������

void ServoMotor_Init(uint8_t nServoCount) //��ʼ��TIM5��·PWMͨ��
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t i;
	
	if(nServoCount <1 || nServoCount > 8) return;
	
	for(i=0;i<SERVOMOTOR_CHN_COUNT;i++)
	{
		nCHPulse[i]=0;
	}
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	switch (nServoCount)
	{
		case 8:
		case 7:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
		
			GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
		
			//PrescalerValue�ļ��㷽����ϵͳʱ��Ƶ��/TIM5����ʱ��Ƶ�ʣ��̶�Ϊ1MHz�� - 1
			TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
			/* Time base configuration */
			//ʹTIM5���Ƶ�ʣ���PWMƵ�ʣ��ļ��㷽����TIM5����ʱ��Ƶ��(1MHz)/��ARR+1���������ARR����TIM_Period��ֵ�����19999�����TIM5����ʱ��Ƶ��Ϊ1M����PWMƵ��Ϊ50Hz
			TIM_TimeBaseStructure.TIM_Period = 19999;

			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

			TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

			/* PWM1 Mode configuration: Channel3 */
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC3Init(TIM2, &TIM_OCInitStructure);

			TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

			/* PWM1 Mode configuration: Channel4 */
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 0;

			TIM_OC4Init(TIM2, &TIM_OCInitStructure);

			TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);


			TIM_ARRPreloadConfig(TIM2, ENABLE);
			
			TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE); //�ر�����ж�

			/* TIM2 enable counter */
			TIM_Cmd(TIM2, ENABLE);	
			
		case 6:
		case 5:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, DISABLE);
		
			//PrescalerValue�ļ��㷽����ϵͳʱ��Ƶ��/TIM5����ʱ��Ƶ�ʣ��̶�Ϊ1MHz�� - 1
			TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
			/* Time base configuration */
			//ʹTIM5���Ƶ�ʣ���PWMƵ�ʣ��ļ��㷽����TIM5����ʱ��Ƶ��(1MHz)/��ARR+1���������ARR����TIM_Period��ֵ�����19999�����TIM5����ʱ��Ƶ��Ϊ1M����PWMƵ��Ϊ50Hz
			TIM_TimeBaseStructure.TIM_Period = 19999;

			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

			TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

			/* PWM1 Mode configuration: Channel1 */
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC1Init(TIM3, &TIM_OCInitStructure);

			TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

			/* PWM1 Mode configuration: Channel2 */
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 0;

			TIM_OC2Init(TIM3, &TIM_OCInitStructure);

			TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);


			TIM_ARRPreloadConfig(TIM3, ENABLE);
			
			TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE); //�ر�����ж�

			/* TIM3 enable counter */
			TIM_Cmd(TIM3, ENABLE);				
		case 4:
		case 3:
		case 2:
		case 1:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			/* Compute the prescaler value */
			//PrescalerValue�ļ��㷽����ϵͳʱ��Ƶ��/TIM5����ʱ��Ƶ�ʣ��̶�Ϊ1MHz�� - 1
			TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
			/* Time base configuration */
			//ʹTIM5���Ƶ�ʣ���PWMƵ�ʣ��ļ��㷽����TIM5����ʱ��Ƶ��(1MHz)/��ARR+1���������ARR����TIM_Period��ֵ�����19999�����TIM5����ʱ��Ƶ��Ϊ1M����PWMƵ��Ϊ50Hz
			TIM_TimeBaseStructure.TIM_Period = 19999;

			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

			TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

			/* PWM1 Mode configuration: Channel1 */
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 1000;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC1Init(TIM5, &TIM_OCInitStructure);

			TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

			/* PWM1 Mode configuration: Channel2 */
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 1000;

			TIM_OC2Init(TIM5, &TIM_OCInitStructure);

			TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

			/* PWM1 Mode configuration: Channel3 */
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 1000;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

			TIM_OC3Init(TIM5, &TIM_OCInitStructure);

			TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

			/* PWM1 Mode configuration: Channel4 */
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_Pulse = 1000;

			TIM_OC4Init(TIM5, &TIM_OCInitStructure);

			TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

			TIM_ARRPreloadConfig(TIM5, ENABLE);
			
			TIM_ITConfig(TIM5,TIM_IT_Update,DISABLE); //�ر�����ж�

			/* TIM5 enable counter */
			TIM_Cmd(TIM5, ENABLE);	
			TIM_CtrlPWMOutputs(TIM5, ENABLE);
	}
}

//void ServoMotor_Cmd(uint8_t nCH, FunctionalState NewState) //������ر�nCHͨ����PWM�����ͨ������ռ�ձ�Ϊ0���ر�PWM���
//{
//	if(NewState != DISABLE)
//	{
//		ServoMotor_SetPulse(nCH,nCHPulse[nCH-1]); //����ʱ����Ϊ�ϴε�ռ�ձ�
//	}
//	else
//	{
//		ServoMotor_SetPulse(nCH,0);
//	}
//}

uint8_t ServoMotor_SetPulse(uint8_t nCH, uint16_t nPulse) //����ͨ��nCH��ռ�ձȣ���ֵ��1000~2000���ֱ��Ӧ1/20~2/20�������Ϊ0����������������൱�ڹ��˶����������������Χ�ڷ���0�����򷵻�1
{
	if( (nPulse>0 && nPulse<1000) || nPulse>2000) return 0;
	switch(nCH)
	{
		case 1:
			TIM5->CCR1 = nPulse;  //����ͨ��һ��ռ�ձ�ΪnPulse/20000��
			nCHPulse[0] = nPulse;  //���浱ǰ����ֵ��������
			break;
		case 2:
			TIM5->CCR2 = nPulse;
			nCHPulse[1] = nPulse;
			break;
		case 3:
			TIM5->CCR3 = nPulse;
			nCHPulse[2] = nPulse;
			break;
		case 4:
			TIM5->CCR4 = nPulse;
			nCHPulse[3] = nPulse;
			break;
		case 5:
			TIM3->CCR1 = nPulse;
			nCHPulse[4] = nPulse;
			break;
		case 6:
			TIM3->CCR2 = nPulse;
			nCHPulse[5] = nPulse;
			break;
		case 7:
			TIM2->CCR3 = nPulse;
			nCHPulse[6] = nPulse;
			break;
		case 8:
			TIM2->CCR4 = nPulse;
			nCHPulse[7] = nPulse;
			break;		
		default:
			return 0;
	}
	return 1;		
}

uint16_t ServoMotor_GetPulse(uint8_t nCH) //��ȡͨ��nCH��ǰ���õ�ռ�ձ�ֵ
{
	return nCHPulse[nCH];
}
