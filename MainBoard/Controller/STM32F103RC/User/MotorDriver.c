#include "MotorDriver.h"
static int16_t MotorDirver_Tim4_Update_Count=0;
static int16_t MotorDirver_Tim8_Update_Count=0;
static int16_t MotorDirver_Tim3_Update_Count=0;
static int16_t MotorDirver_Tim5_Update_Count=0;

void MotorDriver_Init(uint8_t nMotorCount)  //初始化电机驱动，nMotor=1，初始化电机A，nMotor=2，初始化电机A和B，以此类推，从1到4
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	if(nMotorCount < 1 || nMotorCount > 4) return;
	//使能TIM1和GPIOA时钟
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//定时器1配置


  /* Time base configuration */
	//使TIM1输出频率（即PWM频率）的计算方法：系统时钟频率/（ARR+1），这里的ARR就是TIM1_ARR_VALUE的值，如果系统时钟频率为72MHz，计数器预分频系数为0，则PWM频率为7.2KHz
  TIM_TimeBaseStructure.TIM_Period = PWM_DUTY_LIMIT-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	switch (nMotorCount)
	{
		case 4:  //初始化电机D，IN1-PA5，IN2-PA8（TIM1-1），EF-PA4
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOA, &GPIO_InitStructure);

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
		
		case 3:   //初始化电机C，IN1-PC5，IN2-PA9（TIM1-2），EF-PC4
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);				
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOC, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
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
		case 2:		//初始化电机B，IN1-PB1，IN2-PA10（TIM1-3），EF-PB0
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOB, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOB, &GPIO_InitStructure);

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
		case 1:		//初始化电机A，IN1-PC9，IN2-PA11（TIM1-4），EF-PC8
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOC, &GPIO_InitStructure);
		
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOC, &GPIO_InitStructure);

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
			GPIO_ResetBits(GPIOA,GPIO_Pin_5);
			TIM1->CCR1 = nDutySet;
			break;
		case 3:
			GPIO_ResetBits(GPIOC,GPIO_Pin_5);
			TIM1->CCR2 = nDutySet;
			break;
		case 2:
			GPIO_ResetBits(GPIOB,GPIO_Pin_1);
			TIM1->CCR3 = nDutySet;
			break;
		case 1:
			GPIO_ResetBits(GPIOC,GPIO_Pin_9);
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
			GPIO_SetBits(GPIOC,GPIO_Pin_9);
			TIM1->CCR4 = nDutySet;
			break;
		case 2:
			GPIO_SetBits(GPIOB,GPIO_Pin_1);
			TIM1->CCR3 = nDutySet;
			break;
		case 3:
			GPIO_SetBits(GPIOC,GPIO_Pin_5);
			TIM1->CCR2 = nDutySet;
			break;
		case 4:
			GPIO_SetBits(GPIOA,GPIO_Pin_5);
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
			return GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_9);
//			break;
		case 2:
			return GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_1);
//			break;
		case 3:
			return GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_5);
//			break;
		case 4:
			return GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5);
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
	NVIC_InitTypeDef NVIC_InitStructure;  //NVIC中断向量结构体
	
	if(nEncoderCount < 1 || nEncoderCount > 4) return;

	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	switch (nEncoderCount)
	{
		case 4:  //初始化编码器D，A-PA0(TIM5-1)，B-PA1（TIM5-2）
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
		
				TIM_TimeBaseStructure.TIM_Period = ENC_TIM_ARR-1;
				TIM_TimeBaseStructure.TIM_Prescaler = 0;
				TIM_TimeBaseStructure.TIM_ClockDivision = 0;
				TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

				TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
		
				TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		
				/* Enable the TIM5 Update Interrupt */
				NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
				TIM_ClearFlag(TIM5, TIM_FLAG_Update);
				TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
				
				TIM5->CNT = 0;
				TIM_Cmd(TIM5, ENABLE);

		
		
		case 3:   //初始化编码器C，A-PA6(TIM3-1)，B-PA7（TIM3-2）
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
		
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

		
		case 2:		//初始化编码器B，A-PC6(TIM8-1)，B-PC7（TIM8-2）
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOC, &GPIO_InitStructure);
		
				TIM_TimeBaseStructure.TIM_Period = ENC_TIM_ARR-1;
				TIM_TimeBaseStructure.TIM_Prescaler = 0;
				TIM_TimeBaseStructure.TIM_ClockDivision = 0;
				TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

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

		case 1:		//初始化编码器A，A-PB6(TIM4-1)，B-PB7（TIM4-2）
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
			  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
				GPIO_Init(GPIOB, &GPIO_InitStructure);
		
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
		default:
				;
			
	}
}
uint16_t Encoder_GetCNT(uint8_t nEncoder)
{
	switch (nEncoder)
	{
		case 1:
			return TIM4->CNT;
		case 2:
			return TIM8->CNT;
		case 3:
			return TIM3->CNT;
		case 4:
			return TIM5->CNT;
		default:
			return 0;
	}
}
int32_t Encoder_GetEncCount(uint8_t nEncoder)
{
	switch (nEncoder)
	{
		case 1:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim4_Update_Count+TIM4->CNT;
		case 2:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim8_Update_Count+TIM8->CNT;
		case 3:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim3_Update_Count+TIM3->CNT;
		case 4:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim5_Update_Count+TIM5->CNT;
		default:
			return 0;
	}
}

/*以下为中断服务程序，注意不要和stm32f10x_it.c文件中的重复*/
void TIM4_IRQHandler(void)
{
	 //是否有更新中断
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
	{
		 //清除中断标志
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
		//处理中断
		if(TIM4->CNT>ENC_TIM_ARR/2) MotorDirver_Tim4_Update_Count--;  //向下溢出
		else 	MotorDirver_Tim4_Update_Count++; //向上溢出
	}
	
}
void TIM8_UP_IRQHandler(void)
{
	 //是否有更新中断
	if(TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)
	{
		 //清除中断标志
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update); 
		//处理中断
		if(TIM8->CNT>ENC_TIM_ARR/2) MotorDirver_Tim8_Update_Count--;
		else 	MotorDirver_Tim8_Update_Count++;
	}
	
}
void TIM3_IRQHandler(void)
{
	 //是否有更新中断
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)
	{
		 //清除中断标志
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update); 
		//处理中断
		if(TIM3->CNT>ENC_TIM_ARR/2) MotorDirver_Tim3_Update_Count--;
		else 	MotorDirver_Tim3_Update_Count++;
	}
	
}
void TIM5_IRQHandler(void)
{
	 //是否有更新中断
	if(TIM_GetITStatus(TIM5,TIM_IT_Update) != RESET)
	{
		 //清除中断标志
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update); 
		//处理中断
		if(TIM5->CNT>ENC_TIM_ARR/2) MotorDirver_Tim5_Update_Count--;
		else 	MotorDirver_Tim5_Update_Count++;
	}
	
}
