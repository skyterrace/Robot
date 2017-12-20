#include "MotorController.h"
#include "MotorDriver.h"
__IO uint16_t MotorController_EncoderResolution= 390;
__IO uint8_t MotorController_WheelDiameter = 64;
__IO uint16_t MotorController_Acc=0;
__IO int16_t MotorController_MotorA_SpeedSet,MotorController_MotorB_SpeedSet,MotorController_MotorC_SpeedSet,MotorController_MotorD_SpeedSet;
__IO int16_t MotorController_MotorA_SpeedCur,MotorController_MotorB_SpeedCur,MotorController_MotorC_SpeedCur,MotorController_MotorD_SpeedCur;
__IO uint16_t MotorController_MotorA_SpeedPWM,MotorController_MotorB_SpeedPWM,MotorController_MotorC_SpeedPWM,MotorController_MotorD_SpeedPWM;
__IO int32_t MotorController_MotorA_EncCnt,MotorController_MotorB_EncCnt,MotorController_MotorC_EncCnt,MotorController_MotorD_EncCnt;
__IO float MotorController_MotorA_SpeedErr1,MotorController_MotorB_SpeedErr1,MotorController_MotorC_SpeedErr1,MotorController_MotorD_SpeedErr1;
__IO float MotorController_MotorA_SpeedErr2,MotorController_MotorB_SpeedErr2,MotorController_MotorC_SpeedErr2,MotorController_MotorD_SpeedErr2;
__IO uint8_t MotorController_MotorEnabledCount;  //��Ҫ���ڵĵ������
__IO float MotorController_KP, MotorController_KI, MotorController_KD;  //PID����

//MotorController_Init() ��ʼ������
//nEncoderResolution�������ֱ��ʣ�����һȦ����������nWheelDiameter���ӵ�ֱ������λ��mm
//nMotorCount������������Ϊ2������A��B��������Ϊ4������A��B��C��D�ĸ����
void MotorController_Init(uint16_t nEncoderResolution, uint8_t nWheelDiameter,uint8_t nMotorCount) 
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_ClocksTypeDef RCC_Clocks;  //RCCʱ�ӽṹ��
	NVIC_InitTypeDef NVIC_InitStructure;  //NVIC�ж������ṹ��
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	
	RCC_GetClocksFreq(&RCC_Clocks);		//��ȡϵͳʱ��
	//Ԥ��Ƶֵ�ļ��㷽����ϵͳʱ��Ƶ��/TIM6����ʱ��Ƶ�� - 1
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (RCC_Clocks.SYSCLK_Frequency / 10000) - 1;  //����Ƶ��10KHz
	//ʹTIM6���Ƶ�ʵļ��㷽����TIM6����ʱ��Ƶ��/��ARR+1���������ARR����TIM_Period��ֵ�����9�����TIM6����ʱ��Ƶ��Ϊ10K�����������Ϊ1ms
	TIM_TimeBaseStructure.TIM_Period = (uint16_t) MOTOR_CONTROLLER_PERIOD*10 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	/* Enable the TIM6 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //������ж�
	
	MotorController_Enable(DISABLE);

	MotorController_EncoderResolution = nEncoderResolution;
	MotorController_WheelDiameter = nWheelDiameter;
	MotorController_Acc = 0;
	
	MotorController_MotorA_SpeedSet = 0;
	MotorController_MotorB_SpeedSet = 0;
	MotorController_MotorC_SpeedSet = 0;
	MotorController_MotorD_SpeedSet = 0;
	MotorController_MotorA_SpeedCur = 0;
	MotorController_MotorB_SpeedCur = 0;
	MotorController_MotorC_SpeedCur = 0;
	MotorController_MotorD_SpeedCur = 0;
	MotorController_MotorA_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorB_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorC_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorD_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorEnabledCount = nMotorCount;
	
	MotorController_MotorD_EncCnt = Encoder_GetEncCount(4);
	MotorController_MotorC_EncCnt = Encoder_GetEncCount(3);
	MotorController_MotorB_EncCnt = Encoder_GetEncCount(2);
	MotorController_MotorA_EncCnt = Encoder_GetEncCount(1);
	
	//��ʼ��PID����
	MotorController_KP = MOTOR_CONTROLLER_KP;
	MotorController_KI = MOTOR_CONTROLLER_KI;
	MotorController_KD = MOTOR_CONTROLLER_KD;
}
void MotorController_SetAcceleration(uint16_t nAcc) //�������ӵļ��ٶ�ֵ����λmm/s/s����Ϊ0�൱����Сֵ1��
{
	MotorController_Acc = nAcc * MOTOR_CONTROLLER_PERIOD / 1000 + 1;
}
void MotorController_SetSpeed(uint8_t nMotor, int16_t nSpeed) //��������ת�٣�nMotor�����ţ�nSpeed�������ٶȣ���λ��mm/s
{
	switch(nMotor)
	{
		case 1:
			MotorController_MotorA_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(1) == 1)  //����������ֹͣ״̬
			{
				MotorController_MotorA_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(1,MotorController_MotorD_SpeedPWM);
			}
			break;
		case 2:
			MotorController_MotorB_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(2) == 1)  //����������ֹͣ״̬
			{
				MotorController_MotorB_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(2,MotorController_MotorD_SpeedPWM);
			}		
			break;
		case 3:
			MotorController_MotorC_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(3) == 1)  //����������ֹͣ״̬
			{
				MotorController_MotorC_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(3,MotorController_MotorD_SpeedPWM);
			}		
			break;
		case 4:
			MotorController_MotorD_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(4) == 1)  //����������ֹͣ״̬
			{
				MotorController_MotorD_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(4,MotorController_MotorD_SpeedPWM);
			}		
			break;
		default:
			;
	}
}

void MotorController_Enable(FunctionalState NewState)
{
  assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	//�����ٶȵ�����ǰ�������м���������㡣
	MotorController_MotorA_EncCnt = Encoder_GetEncCount(1);
	MotorController_MotorA_SpeedErr2 = 0;
	MotorController_MotorA_SpeedErr1 = 0;
	
	MotorController_MotorB_EncCnt = Encoder_GetEncCount(2);
	MotorController_MotorB_SpeedErr2 = 0;
	MotorController_MotorB_SpeedErr1 = 0;

	MotorController_MotorC_EncCnt = Encoder_GetEncCount(3);
	MotorController_MotorC_SpeedErr2 = 0;
	MotorController_MotorC_SpeedErr1 = 0;
	
	MotorController_MotorD_EncCnt = Encoder_GetEncCount(4);
	MotorController_MotorD_SpeedErr2 = 0;
	MotorController_MotorD_SpeedErr1 = 0;
	
	MotorController_MotorA_SpeedSet = 0;
	MotorController_MotorB_SpeedSet = 0;
	MotorController_MotorC_SpeedSet = 0;
	MotorController_MotorD_SpeedSet = 0;
	MotorController_MotorA_SpeedCur = 0;
	MotorController_MotorB_SpeedCur = 0;
	MotorController_MotorC_SpeedCur = 0;
	MotorController_MotorD_SpeedCur = 0;
	MotorController_MotorA_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorB_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorC_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorD_SpeedPWM = PWM_DUTY_LIMIT/2;	
  
  if (NewState != DISABLE)
  {
		TIM_Cmd(TIM6,ENABLE);
	}
	else
	{
		TIM_Cmd(TIM6,DISABLE);
	}
}
void MotorController_SpeedTunner(void)
{
	int16_t nSpeedExpect;
	static int16_t pwmDelta = 0;
	int16_t pwmSet;
	float fError;
	int32_t nCnt;
	float fSpeedCur;
	switch(MotorController_MotorEnabledCount)
	{
		case 4:
			if(MotorController_MotorD_SpeedCur < MotorController_MotorD_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorD_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorD_SpeedSet) nSpeedExpect = MotorController_MotorD_SpeedSet;
			}
			else if(MotorController_MotorD_SpeedCur > MotorController_MotorD_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorD_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorD_SpeedSet) nSpeedExpect = MotorController_MotorD_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorD_SpeedSet;		
			}
			MotorController_MotorD_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(4);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorD_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorD_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorD_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorD_SpeedErr1+MotorController_MotorD_SpeedErr2);
			pwmSet = MotorController_MotorD_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorD_SpeedPWM = pwmSet;					
			MotorDriver_SetPWMDuty(4,MotorController_MotorD_SpeedPWM);
			MotorController_MotorD_SpeedErr2 = MotorController_MotorD_SpeedErr1;
			MotorController_MotorD_SpeedErr1 = fError;
			MotorController_MotorD_EncCnt = nCnt;
			
		case 3:
			if(MotorController_MotorC_SpeedCur < MotorController_MotorC_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorC_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorC_SpeedSet) nSpeedExpect = MotorController_MotorC_SpeedSet;
			}
			else if(MotorController_MotorC_SpeedCur > MotorController_MotorC_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorC_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorC_SpeedSet) nSpeedExpect = MotorController_MotorC_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorC_SpeedSet;		
			}
			MotorController_MotorC_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(3);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorC_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorC_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorC_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorC_SpeedErr1+MotorController_MotorC_SpeedErr2);

			pwmSet = MotorController_MotorC_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorC_SpeedPWM = pwmSet;					
			MotorDriver_SetPWMDuty(3,MotorController_MotorC_SpeedPWM);
			MotorController_MotorC_SpeedErr2 = MotorController_MotorC_SpeedErr1;
			MotorController_MotorC_SpeedErr1 = fError;
			MotorController_MotorC_EncCnt = nCnt;
		case 2:
			if(MotorController_MotorB_SpeedCur < MotorController_MotorB_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorB_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorB_SpeedSet) 
					nSpeedExpect = MotorController_MotorB_SpeedSet;
			}
			else if(MotorController_MotorB_SpeedCur > MotorController_MotorB_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorB_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorB_SpeedSet) 
					nSpeedExpect = MotorController_MotorB_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorB_SpeedSet;		
			}
			MotorController_MotorB_SpeedCur = nSpeedExpect;			
			nCnt = Encoder_GetEncCount(2);

			fSpeedCur = 3.14*(nCnt - MotorController_MotorB_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			
			fError = nSpeedExpect - fSpeedCur;

			pwmDelta = MotorController_KP * (fError - MotorController_MotorB_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorB_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorB_SpeedErr1+MotorController_MotorB_SpeedErr2);
			pwmSet = MotorController_MotorB_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorB_SpeedPWM = pwmSet;			
			MotorDriver_SetPWMDuty(2,MotorController_MotorB_SpeedPWM);

			MotorController_MotorB_SpeedErr2 = MotorController_MotorB_SpeedErr1;
			MotorController_MotorB_SpeedErr1 = fError;
			MotorController_MotorB_EncCnt = nCnt;
		case 1:
			if(MotorController_MotorA_SpeedCur < MotorController_MotorA_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorA_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorA_SpeedSet) nSpeedExpect = MotorController_MotorA_SpeedSet;
			}
			else if(MotorController_MotorA_SpeedCur > MotorController_MotorA_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorA_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorA_SpeedSet) nSpeedExpect = MotorController_MotorA_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorA_SpeedSet;		
			}
			MotorController_MotorA_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(1);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorA_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorA_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorA_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorA_SpeedErr1+MotorController_MotorA_SpeedErr2);
			pwmSet = MotorController_MotorA_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorA_SpeedPWM = pwmSet;			
			MotorDriver_SetPWMDuty(1,MotorController_MotorA_SpeedPWM);
			MotorController_MotorA_SpeedErr2 = MotorController_MotorA_SpeedErr1;
			MotorController_MotorA_SpeedErr1 = fError;
			MotorController_MotorA_EncCnt = nCnt;			
		default:
			;
	}
}
void MotorController_SetPIDParam(float Kp,float Ki,float Kd)
{
	MotorController_KP = Kp;
	MotorController_KI = Ki;
	MotorController_KD = Kd;	
}

/*����Ϊ�жϷ������ע�ⲻҪ��stm32f10x_it.c�ļ��е��ظ�*/
void TIM6_IRQHandler(void)
{
	 //�Ƿ��и����ж�
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
	{
		 //����жϱ�־
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update); 
		//�����ж�
		MotorController_SpeedTunner();

	}
	
}
