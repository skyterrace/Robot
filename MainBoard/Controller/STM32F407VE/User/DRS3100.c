#include "DRS3100.h"
#include "tm_stm32f4_i2c.h"
#include "delay.h"


__IO uint8_t DRS3100_Buff[DRS3100_FRAME_BYTE_LENGTH]; //���ջ�����
__IO uint8_t DRS3100_ComType; //ͨѶ����

void DRS3100_Init(DRS3100_COM_Type ComType)
{
	uint8_t i;
	DRS3100_ComType = ComType;
	if(ComType == DRS3100_USART)  //����ģʽ�����210ms���յ�һ��139��������ݡ�
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;  //NVIC�ж������ṹ��
		DMA_InitTypeDef DMA_InitStructure;
		/* config USART1 clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		// Enable DMA1 Controller.
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	
			// Clear USART1 RX DMA Channel config.
		DMA_DeInit(DMA2_Stream5);
		// Initialize USART1 RX DMA Channel:
		DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR; // USART1 RX Data Register.
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DRS3100_Buff; // Copy data to RxBuffer.
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Peripheral as source, memory as destination.
		DMA_InitStructure.DMA_BufferSize = DRS3100_FRAME_BYTE_LENGTH; // Defined above.
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // No increment on RDR address.
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // Increment memory address.
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Byte-wise copy.
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // Byte-wise copy.
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		// Initialize USART1 RX DMA Channel.
		DMA_Init(DMA2_Stream5, &DMA_InitStructure);
		// Enable Transfer Complete, Half Transfer and Transfer Error interrupts.
		DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
		// Enable USART1 RX DMA Channel.
		DMA_Cmd(DMA2_Stream5, ENABLE);
		
		 /* Connect USART pins to AF7 */
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
		
		/* USART1 GPIO config */ 
		/* Configure USART1 Tx (PB.06) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_UP;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		/* Configure USART1 Rx (PB.07) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_UP;	
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		/* Enable the DMA Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		/* Enable the USART OverSampling by 8 */
		USART_OverSampling8Cmd(USART1, ENABLE);  	

		/* USART2 mode config */
		USART_InitStructure.USART_BaudRate = 115200;
	//	USART_InitStructure.USART_BaudRate = 19200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure);
		
		/* Enable USART2 Receive and Transmit interrupts */
	//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
	//	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 

			//ʹ��USART2�ж�
	//  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//  NVIC_Init(&NVIC_InitStructure);		

		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
		USART_Cmd(USART1, ENABLE);
	}
	else if (ComType == DRS3100_I2C)
	{
		i=0x01;
		while (i==0x01)
		{
		
			GPIO_InitTypeDef GPIO_InitStructure;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_8;				 //PB9,SDA
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//�������
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			
			for(i=0;i<100;i++)
			{
				GPIO_SetBits(GPIOB,GPIO_Pin_9);
				Delay_ms(1);
				GPIO_SetBits(GPIOB,GPIO_Pin_8);
				Delay_ms(1);
				GPIO_ResetBits(GPIOB,GPIO_Pin_9);
				Delay_ms(1);
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				Delay_ms(1);
			}
			
			TM_I2C_Init(I2C1, TM_I2C_PinsPack_2, 100000);
			Delay_ms(10);
			DRS3100_GetPoint8(&i);  //������������ݵ���1����˵��û�н���I2CͨѶ�����ظ�һ�Ρ�
		}
	}
}

/*8 ��������� ռ��1 ���ֽ��е�8 ��λ
ÿλ�� 0���׵ף�1������
���λ���Ҹ�λ
������𣬵��ұ�ֹ*/
void DRS3100_GetPoint8(uint8_t *p)
{
	*p = TM_I2C_Read(DRS3100_I2Cx, DRS3100_I2C_Address, 0);
}
/*24 ��������� ռ��3 ���ֽ��е�24 ��λ
ÿλ�� 0���׵ף�1������
���λ���Ҹ�λ
������𣬵��ұ�ֹ*/
void DRS3100_GetPoint24(uint8_t *p)
{
	TM_I2C_ReadMulti(DRS3100_I2Cx,DRS3100_I2C_Address,1,p,3);	

}
/*139 ��������ݣ�ռ��18���ֽ��е�ǰ139��λ
ÿλ�� 0���׵ף�1������
���λ���Ҹ�λ
������𣬵��ұ�ֹ*/
void DRS3100_GetPoint139(uint8_t *p)
{
	TM_I2C_ReadMulti(DRS3100_I2Cx,DRS3100_I2C_Address,4,p,18);	
}
/*139 ��ѹ������ 
��ַ22����ƽ��ת����
��ַ23~32��10 �η�ת��λ��
�������ת10 �Σ����ɶ���5�����ߵ�λ�úͿ��*/
void DRS3100_GetTurn(uint8_t *p)
{
}
/*�жϷ�ֵ��־ 0�����жϣ�1���ҷ��ж� ֻ��¼���Ȳ����жϵ���Դ*/
void DRS3100_GetFlagLimit(uint8_t *p)
{
	*p = TM_I2C_Read(DRS3100_I2Cx, DRS3100_I2C_Address, 35);
}


/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream5_IRQHandler(void)      //�жϷ���
{
	if(DMA_GetITStatus(DMA2_Stream5,DMA_IT_TCIF5)!=RESET) 
	{
		/*���Բ��������� ��ʼ*/
		if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_2) == 1)
			GPIO_ResetBits(GPIOB,GPIO_Pin_2);
		else
			GPIO_SetBits(GPIOB,GPIO_Pin_2);
		/*���Բ��������� ����*/
    DMA_ClearITPendingBit(DMA2_Stream5,DMA_IT_TCIF5);
	}
}
