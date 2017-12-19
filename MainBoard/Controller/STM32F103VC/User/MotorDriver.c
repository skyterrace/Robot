#include "MotorDriver.h"
static int16_t MotorDirver_Tim3_Update_Count=0;
static int16_t MotorDirver_Tim2_Update_Count=0;
static int16_t MotorDirver_Tim8_Update_Count=0;
static int16_t MotorDirver_Tim4_Update_Count=0;

void MotorDriver_Init(uint8_t nMotorCount)  //��ʼ�����������nMotor=1����ʼ�����A��nMotor=2����ʼ�����A��B���Դ����ƣ���1��4
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	if(nMotorCount < 1 || nMotorCount > 4) return;
	//ʹ��TIM1��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//��ʱ��1����


  /* Time base configuration */
	//ʹTIM1���Ƶ�ʣ���PWMƵ�ʣ��ļ��㷽����ϵͳʱ��Ƶ��/��ARR+1���������ARR����TIM1_ARR_VALUE��ֵ�����ϵͳʱ��Ƶ��Ϊ72MHz��������Ԥ��Ƶϵ��Ϊ0����PWMƵ��Ϊ7.2KHz
  TIM_TimeBaseStructure.TIM_Period = PWM_DUTY_LIMIT-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	switch (nMotorCount)
	{
		case 4:  //��ʼ�����D��IN1-PD8��IN2-PA8��TIM1-1����EF-PD9
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOD, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOD, &GPIO_InitStructure);

			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
		
				/* PWM1 Mode configuration: Channel1 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = PWM_DUTY_LIMIT/2;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

				TIM_OC1Init(TIM1, &TIM_OCInitStructure);
				TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
				MotorDriver_Stop(4,0);
		
		case 3:   //��ʼ�����C��IN1-PC8��IN2-PA9��TIM1-2����EF-PC9
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);				
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOC, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOC, &GPIO_InitStructure);

			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
		
				/* PWM1 Mode configuration: Channel2 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = PWM_DUTY_LIMIT/2;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

				TIM_OC2Init(TIM1, &TIM_OCInitStructure);
				TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);	
				MotorDriver_Stop(3,0);
		case 2:		//��ʼ�����B��IN1-PD10��IN2-PA10��TIM1-3����EF-PD11
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOD, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOD, &GPIO_InitStructure);

			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
		
				/* PWM1 Mode configuration: Channel3 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = PWM_DUTY_LIMIT/2;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

				TIM_OC3Init(TIM1, &TIM_OCInitStructure);
				TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);	
				MotorDriver_Stop(2,0);
		case 1:		//��ʼ�����A��IN1-PD14��IN2-PA11��TIM1-4����EF-PD15
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOD, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOD, &GPIO_InitStructure);

			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
		
				/* PWM1 Mode configuration: Channel4 */
				TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
				TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
				TIM_OCInitStructure.TIM_Pulse = PWM_DUTY_LIMIT/2;
				TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

				TIM_OC4Init(TIM1, &TIM_OCInitStructure);
				TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
				MotorDriver_Stop(1,0);				
		default:
				;
			
	}
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
void MotorDriver_Start(uint8_t nMotor, uint16_t nDuty)
{
	uint16_t nDutySet;
	if(nDuty>PWM_DUTY_LIMIT) nDutySet = PWM_DUTY_LIMIT;
	else nDutySet = nDuty;
	switch (nMotor)
	{
		case 4:
			GPIO_ResetBits(GPIOD,GPIO_Pin_8);
			TIM1->CCR1 = nDutySet;
			break;
		case 3:
			GPIO_ResetBits(GPIOC,GPIO_Pin_8);
			TIM1->CCR2 = nDutySet;
			break;
		case 2:
			GPIO_ResetBits(GPIOD,GPIO_Pin_10);
			TIM1->CCR3 = nDutySet;
			break;
		case 1:
			GPIO_ResetBits(GPIOD,GPIO_Pin_14);
			TIM1->CCR4 = nDutySet;
			break;
		default:
			;
	}	
}
void MotorDriver_SetPWMDuty(uint8_t nMotor, uint16_t nDuty)
{
	uint16_t nDutySet;
	if(nDuty>PWM_DUTY_LIMIT) nDutySet = PWM_DUTY_LIMIT;
	else nDutySet = nDuty;
	switch (nMotor)
	{
		case 1:
			TIM1->CCR4 = nDutySet;
			break;
		case 2:
			TIM1->CCR3 = nDutySet;
			break;
		case 3:
			TIM1->CCR2 = nDutySet;
			break;
		case 4:
			TIM1->CCR1 = nDutySet;
			break;
		default:
			;
	}
}
void MotorDriver_Stop(uint8_t nMotor, uint16_t nDuty)
{
	uint16_t nDutySet;
	if(nDuty>PWM_DUTY_LIMIT) nDutySet = PWM_DUTY_LIMIT;
	else nDutySet = nDuty;
	switch (nMotor)
	{
		case 1:
			GPIO_SetBits(GPIOD,GPIO_Pin_14);
			TIM1->CCR4 = nDutySet;
			break;
		case 2:
			GPIO_SetBits(GPIOD,GPIO_Pin_10);
			TIM1->CCR3 = nDutySet;
			break;
		case 3:
			GPIO_SetBits(GPIOC,GPIO_Pin_8);
			TIM1->CCR2 = nDutySet;
			break;
		case 4:
			GPIO_SetBits(GPIOD,GPIO_Pin_8);
			TIM1->CCR1 = nDutySet;
			break;
		default:
			;
	}	
}
uint8_t MotorDriver_GetMotorState(uint8_t nMotor)
{
	switch (nMotor)
	{
		case 1:
			return GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_14);
//			break;
		case 2:
			return GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_10);
//			break;
		case 3:
			return GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_8);
//			break;
		case 4:
			return GPIO_ReadOutputDataBit(GPIOD,GPIO_Pin_8);
//			break;
		default:
			return 1;
	}	
}
void Encoder_Init(uint8_t nEncoderCount)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //NVIC�ж������ṹ��
	
	if(nEncoderCount < 1 || nEncoderCount > 4) return;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	switch (nEncoderCount)
	{
		case 4:  //��ʼ��������D��A-PD12(TIM4-1)��B-PD13��TIM4-2������Ҫremap
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOD, &GPIO_InitStructure);
		
				GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
		
				TIM_TimeBaseStructure.TIM_Period = ENC_TIM_ARR-1;
				TIM_TimeBaseStructure.TIM_Prescaler = 0;
				TIM_TimeBaseStructure.TIM_ClockDivision = 0;
				TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

				TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		
				TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		
				/* Enable the TIM4 Update Interrupt */
				NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
				TIM_ClearFlag(TIM4, TIM_FLAG_Update);
				TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
				
				TIM4->CNT = 0;
				TIM_Cmd(TIM4, ENABLE);

		
		
		case 3:   //��ʼ��������C��A-PC6(TIM8-1)��B-PC7��TIM8-2��
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOC, &GPIO_InitStructure);
		
				TIM_TimeBaseStructure.TIM_Period = ENC_TIM_ARR-1;
				TIM_TimeBaseStructure.TIM_Prescaler = 0;
				TIM_TimeBaseStructure.TIM_ClockDivision = 0;
				TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
				TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //TIM8���ظ�������

				TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
		
				TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
				
				/* Enable the TIM8 Update Interrupt */
				NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
				TIM_ClearFlag(TIM8, TIM_FLAG_Update);
				TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
				
				TIM8->CNT = 0;
				TIM_Cmd(TIM8, ENABLE);			
		
		
		case 2:		//��ʼ��������B��A-PA15(TIM2-1)��B-PB3��TIM2-2������Ҫremap
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOB, &GPIO_InitStructure);
		
				GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
				GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
		
				TIM_TimeBaseStructure.TIM_Period = ENC_TIM_ARR-1;
				TIM_TimeBaseStructure.TIM_Prescaler = 0;
				TIM_TimeBaseStructure.TIM_ClockDivision = 0;
				TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
				TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 

				TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		
				TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
				
				/* Enable the TIM2 Update Interrupt */
				NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
				TIM_ClearFlag(TIM2, TIM_FLAG_Update);
				TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
				
				TIM2->CNT = 0;
				TIM_Cmd(TIM2, ENABLE);

		case 1:		//��ʼ��������A��A-PB4(TIM3-1)��B-PB5��TIM3-2������Ҫremap
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOB, &GPIO_InitStructure);
		
				GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
		
				TIM_TimeBaseStructure.TIM_Period = ENC_TIM_ARR-1;
				TIM_TimeBaseStructure.TIM_Prescaler = 0;
				TIM_TimeBaseStructure.TIM_ClockDivision = 0;
				TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

				TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		
				TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		
				/* Enable the TIM3 Update Interrupt */
				NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
				TIM_ClearFlag(TIM3, TIM_FLAG_Update);
				TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		
				
				TIM3->CNT = 0;
				TIM_Cmd(TIM3, ENABLE);
		default:
				;
			
	}
	
	MotorDirver_Tim3_Update_Count=0;
	MotorDirver_Tim2_Update_Count=0;
	MotorDirver_Tim8_Update_Count=0;
	MotorDirver_Tim4_Update_Count=0;
}
uint16_t Encoder_GetCNT(uint8_t nEncoder)
{
	switch (nEncoder)
	{
		case 1:
			return TIM3->CNT;
		case 2:
			return TIM2->CNT;
		case 3:
			return TIM8->CNT;
		case 4:
			return TIM4->CNT;
		default:
			return 0;
	}
}
int32_t Encoder_GetEncCount(uint8_t nEncoder)
{
	switch (nEncoder)
	{
		case 1:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim3_Update_Count+TIM3->CNT;
		case 2:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim2_Update_Count+TIM2->CNT;
		case 3:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim8_Update_Count+TIM8->CNT;
		case 4:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim4_Update_Count+TIM4->CNT;
		default:
			return 0;
	}
}

/*����Ϊ�жϷ������ע�ⲻҪ��stm32f10x_it.c�ļ��е��ظ�*/
void TIM3_IRQHandler(void)
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update); 
		//�����ж�
		if(TIM3->CNT>ENC_TIM_ARR/2) MotorDirver_Tim3_Update_Count--;
		else 	MotorDirver_Tim3_Update_Count++;
	}
	
}

void TIM2_IRQHandler(void)
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update); 
		//�����ж�
		if(TIM2->CNT>ENC_TIM_ARR/2) MotorDirver_Tim2_Update_Count--;
		else 	MotorDirver_Tim2_Update_Count++;
	}
	
}

void TIM8_UP_IRQHandler(void)
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update); 
		//�����ж�
		if(TIM8->CNT>ENC_TIM_ARR/2) MotorDirver_Tim8_Update_Count--;
		else 	MotorDirver_Tim8_Update_Count++;
	}
	
}

void TIM4_IRQHandler(void)
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
		//�����ж�
		if(TIM4->CNT>ENC_TIM_ARR/2) MotorDirver_Tim4_Update_Count--;  //�������
		else 	MotorDirver_Tim4_Update_Count++; //�������
	}
	
}

