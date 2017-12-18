#include "stm32f4xx.h"
#include "misc.h"
#include "delay.h"
#include "KeyLed.h"
#include "MotorDriver.h"
#include "MotorController.h"
#include "BTModule.h"
#include "AnalogIn.h"
#include "camera/dcmi_ov2640.h"

/*�������Ҫ��MPU6050��������Ŀ�ļ��У���I2CRoutines.c��MPU6050DMP.cԴ�����ļ��Ƴ�*/
/*������Ԥ������Ŷ����н�MPU6050��EMPL_TARGET_STM32�ķ��Ŷ���ɾ��*/

#if defined MPU6050  //ֻ����MPU6050��ʱ�����Ҫ���Ӵ˲��ִ���
#include "MPU6050DMP.h"
struct dmpYawPitchRoll_s sMPU6050YawPitchRoll;
struct MPU6050_RawData_s sIMUVar;
#endif

//RCC_Configuration(), NVIC_Configuration(), GPIO_Config() ������ǰҪ����һ�¡�
//void RCC_Configuration(void); //ϵͳʱ������
void NVIC_Configuration(void); //�ж�����
void GPIO_Config(void); //ͨ����������˿�����

//ȫ�ֱ���
uint16_t n10msCount;
uint8_t b100msFlag;

int16_t nSpeedCnt;
int32_t nEncoderACount,nEncoderACount_Last;
int32_t nEncoderBCount,nEncoderBCount_Last;
int32_t nSpeed;
uint16_t nVPower;
extern uint16_t uhADCConvertedValue;
int main(void)
{

//	RCC_Configuration(); //ʱ�ӳ�ʼ��
	GPIO_Config(); //�˿ڳ�ʼ��
	NVIC_Configuration();  //�жϳ�ʼ��
	Delay_ms(100);
	Led_Init(1); //ֻʹ��LED2��LED3�����ſ�����������;
	Key_Init(2); //ֻʹ��Key1��Key2�����ſ�����������;
	USART2_Init();  //�������ڳ�ʼ��	
	
	while(Key_Released(1)==0);  //���Key1û�а��£���һֱ�ȴ�

	MotorDriver_Init(4);
	MotorDriver_Start(1,PWM_DUTY_LIMIT/2);
	MotorDriver_Start(2,PWM_DUTY_LIMIT/2);
	MotorDriver_Start(3,PWM_DUTY_LIMIT/2);
	MotorDriver_Start(4,PWM_DUTY_LIMIT/2);
	Encoder_Init(4);

	MotorController_Init(390,60,4);
	MotorController_Enable(ENABLE);
	MotorController_SetAcceleration(100);

	
#if defined MPU6050  //ֻ����MPU6050��ʱ�����Ҫ���Ӵ˲��ִ���
	MPU6050_InitDMP();
#endif

	Delay_ms(1000);
	ADC_Config();
	ADC_SoftwareStartConv(ADCx);
	nSpeed = 0;

/*��������ͷ*/

	OV2640_HW_Init();
	Delay_ms(100);
	OV2640_Reset();
	Delay_ms(100);
	OV2640_IDTypeDef OV2640ID;
	OV2640_ReadID(&OV2640ID);
	if(OV2640ID.PIDH  == 0x26)
  {
    printf("OV2640 Camera ID 0x%x", OV2640ID.PIDH);
  }	
	OV2640_Init(BMP_QQVGA);
	OV2640_QQVGAConfig();
	
  /* Enable DMA2 stream 1 and DCMI interface then start image capture */
  DMA_Cmd(DMA2_Stream1, ENABLE); 
  DCMI_Cmd(ENABLE); 

  /* Insert 100ms delay: wait 100ms */
  Delay_ms(200); 

  DCMI_CaptureCmd(ENABLE); 	
	OV2640_BrightnessConfig(0x20);

	/*��������ͷ����*/
	
	while(1)
	{
		/*
		if(nMotorDir == 0)
		{
			pwm+=1000;
			if(pwm>PWM_DUTY_LIMIT)
			{
				nMotorDir = 1;
				pwm-=1000;
				Delay_ms(1000);
				//MotorDriver_SetPWMDuty(1,5000);
				MotorDriver_Stop(1,0);
				Delay_ms(2000);
				//ֹͣ�������Ҫ����start����ת��
				MotorDriver_Start(1,PWM_DUTY_LIMIT/2);
			}
		}
		else
		{
			pwm-=1000;
			if(pwm<1000)
			{
				nMotorDir = 0;
				pwm+=1000;
			}			
		}
		MotorDriver_SetPWMDuty(1,pwm);
		MotorDriver_SetPWMDuty(2,pwm);
		*/
		
//		if(Key_Released(1)==1) nMotorDir = (nMotorDir == 1) ? 0 : 1;
//		if(nMotorDir==1) {MotorDriver_SetPWMDuty(2,10000);}
//		else {MotorDriver_SetPWMDuty(2,0);}
		if(Key_Released(1)==1) 
		{
			nSpeed += 100;
			MotorController_SetSpeed(1,nSpeed);
			MotorController_SetSpeed(2,nSpeed);
						MotorController_SetSpeed(3,nSpeed);
			MotorController_SetSpeed(4,nSpeed);
		}
		if(Key_Released(2)==1) 
		{
			nSpeed -= 100;
			MotorController_SetSpeed(1,nSpeed);
			MotorController_SetSpeed(2,nSpeed);
						MotorController_SetSpeed(3,nSpeed);
			MotorController_SetSpeed(4,nSpeed);
		}
		
		if(b10msFlag==1)
		{
			b10msFlag = 0;  //�� 10ms ��־λ����
			n10msCount++;
			
			//10ms��־ÿ10����������100ms
			if(n10msCount%10 == 0) b100msFlag = 1;
			
			if(n10msCount == 50) //500msʱ����LED
			{
				LED2_ON();
			}
			else if(n10msCount>=100)  //1000msʱ�ر�LED��ͬʱ�������㣬����һ���ٶ�ֵ
			{
				LED2_OFF();
				n10msCount = 0;
				
			}
			
#if defined MPU6050  //ֻ����MPU6050��ʱ�����Ҫ���Ӵ˲��ִ���
			if(n10msCount%2 == 1)  //������10msʱ����ȡ����ǣ�����ǣ�������
			{
				if(MPU6050_GetYawPitchRoll(&sMPU6050YawPitchRoll))
				{
					//�Ӵ�����������Ƕȣ�ת�������ͣ����Ŵ�10������122����12.2��
//					USART_OUT(USART2,"%d %d %d\r\n",(int)(sMPU6050YawPitchRoll.yaw*1800/3.1415926),
//						(int)(sMPU6050YawPitchRoll.pitch*1800/3.1415926),
//						(int)(sMPU6050YawPitchRoll.roll*1800/3.1415926));
					printf("%d %d %d\r\n",(int)(sMPU6050YawPitchRoll.yaw*1800/3.1415926),
						(int)(sMPU6050YawPitchRoll.pitch*1800/3.1415926),
						(int)(sMPU6050YawPitchRoll.roll*1800/3.1415926));						
				}
			}
			else if (n10msCount%2 == 0)  //ż����10msʱ����ȡ���ٶȼƺ����������ݲ�ͨ�����ڴ���
			{
				if(MPU6050_ReadRawData(&sIMUVar))
				{
					//�Ӵ���������ٶ�ax,ay,az�Լ����ٶ�gx,gy,gz
//					USART_OUT(USART2,"%d %d %d %d %d %d\r\n",sIMUVar.ax,sIMUVar.ay,sIMUVar.az,
//						sIMUVar.gx,sIMUVar.gy,sIMUVar.gz);					
				}
				nVPower = ADC_GetPowerVoltage();
				printf("Battery Voltage:%d mV\r\n",nVPower);
			}
#endif			
			
			if(b100msFlag == 1)
			{
				b100msFlag = 0;
				//�������ٶ�
				nEncoderACount = Encoder_GetEncCount(1);
				nEncoderBCount = Encoder_GetEncCount(2);
				nSpeedCnt = nEncoderBCount - nEncoderBCount_Last;
				
				//ͨ���������ڷ����ٶ�
				//USART_OUT(USART2,"%d %d\r\n",nEncoderACount - nEncoderACount_Last,nSpeedCnt);
				
				//ͨ���������ڷ����ۼ�����������̣�
				//USART_OUT(USART2,"%d  %d\r\n",nEncoderACount,nEncoderBCount);
				
				nEncoderACount_Last = nEncoderACount;
				nEncoderBCount_Last = nEncoderBCount;
				
				//�������������ڽ��յ�������
				USART_Process();
			
			}

		}
	}
	
}

/*ϵͳʱ�����ú���*/
//void RCC_Configuration(void)
//{
//	
////        SystemInit();//Դ��system_stm32f10x.c�ļ�,ֻ��Ҫ���ô˺���,������RCC��Ĭ������.�����뿴2_RCC
//	
//				//���ǻ����Լ������ð�
//	      RCC_DeInit();

//				RCC_HSEConfig(RCC_HSE_ON);  //ʹ���ⲿ8MHz����
//        
//        while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)        
//        {        
//        }

////        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

////        FLASH_SetLatency(FLASH_Latency_2);
//				
//        /* HCLK = SYSCLK */
//        RCC_HCLKConfig(RCC_SYSCLK_Div1);
//        //APB2
//        RCC_PCLK2Config(RCC_HCLK_Div1);
//        //APB1
//        RCC_PCLK1Config(RCC_HCLK_Div2);
//        //PLL ��Ƶ
//        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);        //�ⲿ����*9���ⲿ����8MHz������PLL����Ƶ9����PLL���72MHz
//        RCC_PLLCmd(ENABLE);                        										//ʹ�ܱ�Ƶ
//                                                                                                         
//				//�ȴ�ָ���� RCC ��־λ���óɹ� �ȴ�PLL��ʼ���ɹ�
//        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
//        {
//        }

//        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);        //��PLL�����Ϊϵͳʱ�ӡ�

//        /**************************************************
//        ��ȡRCC����Ϣ,������
//        ��ο�RCC_ClocksTypeDef�ṹ�������,��ʱ��������ɺ�,
//        ���������ֵ��ֱ�ӷ�ӳ�������������ֵ�����Ƶ��
//        ***************************************************/
//                
//        while(RCC_GetSYSCLKSource() != 0x08){}		//����0x08˵��ʹ��PLL��Ϊϵͳʱ�ӳɹ�
//					
//}

/*�ж����ú���*/
void NVIC_Configuration(void)
{
//  NVIC_InitTypeDef NVIC_InitStructure;  //NVIC�ж������ṹ��
//	RCC_ClocksTypeDef RCC_Clocks;  //RCCʱ�ӽṹ��
  /* Configure the NVIC Preemption Priority Bits */  
  /* Configure one bit for preemption priority */
  /* ���ȼ��� ˵������ռ���ȼ����õ�λ�����������ȼ����õ�λ����������2��2*/    
 		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  

//	  /* Enable the RTC Interrupt ʹ��ʵʱʱ���ж�*/
//  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;					//�����ⲿ�ж�Դ�����жϣ� 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);  
	
//	/*ʹ�ܶ�ʱ�ж�*/
//	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;												//ָ���ж�Դ
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;							
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;										//ָ����Ӧ���ȼ���
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	NVIC_Init(&NVIC_InitStructure); 
	
	//�������ʱ���ж�
  if (SysTick_Config(SystemCoreClock / 100000))
  { 
    /* Capture error */ 
    while (1);
  }
}

/*�˿����ú���*/
void GPIO_Config(void)
{
	//ʹ��GPIOA/GPIOC�����ߣ�����˿ڲ��ܹ����������������˿ڣ����Բ���ʹ�ܡ�
	
}
