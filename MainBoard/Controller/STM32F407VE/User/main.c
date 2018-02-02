#include "stm32f4xx.h"
#include "misc.h"
#include "delay.h"
#include "KeyLed.h"
#include "MotorDriver.h"
#include "MotorController.h"
#include "BTModule.h"
#include "AnalogIn.h"
#include "DRS3100.h"
#include "camera/dcmi_ov2640.h"

/*如果不需要用MPU6050，则在项目文件中，将I2CRoutines.c和MPU6050DMP.c源代码文件移除*/
/*并且在预编译符号定义中将MPU6050和EMPL_TARGET_STM32的符号定义删除*/

#if defined MPU6050  //只有用MPU6050的时候才需要增加此部分代码
//#define USE_MPU6050
#include "MPU6050DMP.h"
struct dmpYawPitchRoll_s sMPU6050YawPitchRoll;
struct MPU6050_RawData_s sIMUVar;
#endif

//RCC_Configuration(), NVIC_Configuration(), GPIO_Config() 在引用前要声明一下。
//void RCC_Configuration(void); //系统时钟配置
void NVIC_Configuration(void); //中断配置
void GPIO_Config(void); //通用输入输出端口配置

//全局变量
uint16_t n10msCount;
uint8_t b100msFlag;

int16_t nSpeedCnt;
int32_t nEncoderACount,nEncoderACount_Last;
int32_t nEncoderBCount,nEncoderBCount_Last;
int32_t nSpeed;
uint16_t nVPower;
extern uint16_t uhADCConvertedValue;

uint32_t JpegDataCnt = 0;
extern uint8_t JpegBuffer[1024*33];
#define JpegBufferLen (sizeof(JpegBuffer)/sizeof(char)) //计算JpegBuffer元素总个数
	
uint8_t DRS_Buff[18];
int main(void)
{
	uint32_t i = 0;

//	RCC_Configuration(); //时钟初始化
	GPIO_Config(); //端口初始化
	NVIC_Configuration();  //中断初始化
	Delay_ms(100);
	Led_Init(1); //只使用LED2，LED3的引脚可以做其他用途
	Key_Init(2); //只使用Key1，Key2的引脚可以做其他用途
	USART2_Init();  //蓝牙串口初始化	
	DRS3100_Init(DRS3100_I2C);
	while(Key_Released(1)==0);  //如果Key1没有按下，则一直等待

//	MotorDriver_Init(4);
//	MotorDriver_Start(1,PWM_DUTY_LIMIT/2);
//	MotorDriver_Start(2,PWM_DUTY_LIMIT/2);
//	MotorDriver_Start(3,PWM_DUTY_LIMIT/2);
//	MotorDriver_Start(4,PWM_DUTY_LIMIT/2);
//	Encoder_Init(4);

//	MotorController_Init(390,60,4);
//	MotorController_Enable(ENABLE);
//	MotorController_SetAcceleration(100);

	
#if defined USE_MPU6050  //只有用MPU6050的时候才需要增加此部分代码
	MPU6050_InitDMP();
#endif

	Delay_ms(1000);
//	ADC_Config();
//	ADC_SoftwareStartConv(ADCx);
	nSpeed = 0;

///*测试摄像头*/

//	OV2640_HW_Init();
//	Delay_ms(100);
//	OV2640_Reset();
//	Delay_ms(100);
//	OV2640_IDTypeDef OV2640ID;
//	OV2640_ReadID(&OV2640ID);
//	if(OV2640ID.PIDH  == 0x26)
//  {
////    printf("OV2640 Camera ID 0x%x", OV2640ID.PIDH);
//  }	
////	OV2640_Init(BMP_QQVGA);
////	OV2640_QQVGAConfig();
////	OV2640_Init(BMP_QVGA);
////	OV2640_QVGAConfig();	
//	OV2640_Init(JPEG_320x240);
//	OV2640_Reset();
//	Delay_ms(100);
//	OV2640_JPEGConfig(JPEG_320x240);
//	Delay_ms(100);
//		OV2640_BrightnessConfig(0x20);	
////		OV2640_AutoExposure(2);		
//  /* Enable DMA2 stream 1 and DCMI interface then start image capture */
//  DMA_Cmd(DMA2_Stream1, ENABLE); 
//  DCMI_Cmd(ENABLE); 

//  /* Insert 100ms delay: wait 100ms */
//  Delay_ms(200); 

//  DCMI_CaptureCmd(ENABLE); 	
////	OV2640_BrightnessConfig(0x20);

//	/*测试摄像头结束*/
	

	
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
				//停止电机后需要重新start才能转动
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
//		if(OV2640_jpg_flag == 1)
//		{
//			DCMI_Cmd(DISABLE); 
//			DCMI_CaptureCmd(DISABLE);
//			DMA_Cmd(DMA2_Stream1, DISABLE);
//			if( (JpegBuffer[0]==0xFF)&&(JpegBuffer[1]==0xD8) )
//			{
//					while ( !( (JpegBuffer[JpegBufferLen - JpegDataCnt-2]==0xFF) && (JpegBuffer[JpegBufferLen-JpegDataCnt-1]==0xD9) ) ) //从数据包的尾开始检索  
//				{		
//					JpegDataCnt++;
//				}				
//				 for(i = 0; i < (JpegBufferLen - JpegDataCnt); i++)	//sizeof(JpegBuffer)
//				{
//					USART_SendData(USART2, (unsigned char) JpegBuffer[i]);
//					while( USART_GetFlagStatus(USART2,USART_FLAG_TC)!= SET);
//				} 
//			}
//			JpegDataCnt = 0;
//				
//			OV2640_jpg_flag = 0;
////			printf("jpg flag\r\n");
//			/* Enable DMA2 stream 1 and DCMI interface then start image capture */
//			DMA_Cmd(DMA2_Stream1, ENABLE); 
//			DCMI_Cmd(ENABLE); 

//			/* Insert 100ms delay: wait 100ms */
//			Delay_ms(200); 

//			DCMI_CaptureCmd(ENABLE); 				
//			
//		}
		
		if(b10msFlag==1)
		{
			b10msFlag = 0;  //把 10ms 标志位清零
			n10msCount++;
			
			//10ms标志每10个计数等于100ms
			if(n10msCount%10 == 0) b100msFlag = 1;
			
			if(n10msCount == 50) //500ms时点亮LED
			{
				LED2_ON();
				
			}
			else if(n10msCount>=100)  //1000ms时关闭LED，同时计数清零，计算一次速度值
			{
				LED2_OFF();
				n10msCount = 0;
				
				
			}
			
#if defined USE_MPU6050  //只有用MPU6050的时候才需要增加此部分代码
			if(n10msCount%2 == 1)  //奇数个10ms时，读取航向角，横滚角，俯仰角
			{
				if(MPU6050_GetYawPitchRoll(&sMPU6050YawPitchRoll))
				{
					//从串口输出三个角度，转换成整型，并放大10倍，即122代表12.2度
//					USART_OUT(USART2,"%d %d %d\r\n",(int)(sMPU6050YawPitchRoll.yaw*1800/3.1415926),
//						(int)(sMPU6050YawPitchRoll.pitch*1800/3.1415926),
//						(int)(sMPU6050YawPitchRoll.roll*1800/3.1415926));
					printf("%d %d %d\r\n",(int)(sMPU6050YawPitchRoll.yaw*1800/3.1415926),
						(int)(sMPU6050YawPitchRoll.pitch*1800/3.1415926),
						(int)(sMPU6050YawPitchRoll.roll*1800/3.1415926));						
				}
			}
			else if (n10msCount%2 == 0)  //偶数个10ms时，读取加速度计和陀螺仪数据并通过串口传输
			{
				if(MPU6050_ReadRawData(&sIMUVar))
				{
					//从串口输出加速度ax,ay,az以及角速度gx,gy,gz
//					USART_OUT(USART2,"%d %d %d %d %d %d\r\n",sIMUVar.ax,sIMUVar.ay,sIMUVar.az,
//						sIMUVar.gx,sIMUVar.gy,sIMUVar.gz);					
				}
			}
#endif			
			
			if(b100msFlag == 1)
			{
				b100msFlag = 0;
				//计算下速度
				nEncoderACount = Encoder_GetEncCount(1);
				nEncoderBCount = Encoder_GetEncCount(2);
				nSpeedCnt = nEncoderBCount - nEncoderBCount_Last;
				
				//通过蓝牙串口发送速度
				//USART_OUT(USART2,"%d %d\r\n",nEncoderACount - nEncoderACount_Last,nSpeedCnt);
				
				//通过蓝牙串口发送累计脉冲数（里程）
				//USART_OUT(USART2,"%d  %d\r\n",nEncoderACount,nEncoderBCount);
				
				nEncoderACount_Last = nEncoderACount;
				nEncoderBCount_Last = nEncoderBCount;
				nVPower = ADC_GetPowerVoltage();
//				printf("Battery Voltage:%d mV\r\n",nVPower);
				//处理下蓝牙串口接收到的数据
				USART_Process();
				
				//测试DRS3100

				GPIO_SetBits(GPIOB,GPIO_Pin_2);
				DRS3100_GetPoint139(DRS_Buff);
				GPIO_ResetBits(GPIOB,GPIO_Pin_2);
				for(i=0;i<17;i++)
				{
					printf("%x,",DRS_Buff[i]);
				}
				printf("%x\r\n",DRS_Buff[17]);
			}
			

		}
	}
	
}

/*系统时钟配置函数*/
//void RCC_Configuration(void)
//{
//	
////        SystemInit();//源自system_stm32f10x.c文件,只需要调用此函数,则可完成RCC的默认配置.具体请看2_RCC
//	
//				//我们还是自己来配置吧
//	      RCC_DeInit();

//				RCC_HSEConfig(RCC_HSE_ON);  //使用外部8MHz晶振
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
//        //PLL 倍频
//        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);        //外部晶振*9，外部晶振8MHz，进入PLL，倍频9，则PLL输出72MHz
//        RCC_PLLCmd(ENABLE);                        										//使能倍频
//                                                                                                         
//				//等待指定的 RCC 标志位设置成功 等待PLL初始化成功
//        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
//        {
//        }

//        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);        //以PLL输出作为系统时钟。

//        /**************************************************
//        获取RCC的信息,调试用
//        请参考RCC_ClocksTypeDef结构体的内容,当时钟配置完成后,
//        里面变量的值就直接反映了器件各个部分的运行频率
//        ***************************************************/
//                
//        while(RCC_GetSYSCLKSource() != 0x08){}		//返回0x08说明使用PLL作为系统时钟成功
//					
//}

/*中断配置函数*/
void NVIC_Configuration(void)
{
//  NVIC_InitTypeDef NVIC_InitStructure;  //NVIC中断向量结构体
//	RCC_ClocksTypeDef RCC_Clocks;  //RCC时钟结构体
  /* Configure the NVIC Preemption Priority Bits */  
  /* Configure one bit for preemption priority */
  /* 优先级组 说明了抢占优先级所用的位数，和子优先级所用的位数。这里是2，2*/    
 		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  

//	  /* Enable the RTC Interrupt 使能实时时钟中断*/
//  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;					//配置外部中断源（秒中断） 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);  
	
//	/*使能定时中断*/
//	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;												//指定中断源
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;							
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;										//指定响应优先级别
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
//	NVIC_Init(&NVIC_InitStructure); 
	
	//设置嘀嗒时钟中断
  if (SysTick_Config(SystemCoreClock / 100000))
  { 
    /* Capture error */ 
    while (1);
  }
}

/*端口配置函数*/
void GPIO_Config(void)
{
	//使能GPIOA/GPIOC的总线，否则端口不能工作，如果不用这个端口，可以不用使能。
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				 //PB2	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}
