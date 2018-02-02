#include "BTModule.h"
#include "MotorController.h"
__IO uint8_t nRx2Counter=0; //�����ֽ���
__IO uint8_t USART_Rx2Buff[FRAME_BYTE_LENGTH]; //���ջ�����
__IO uint8_t USART_FrameFlag = 0; //������������֡��־��1������0������
void USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //NVIC�ж������ṹ��
	/* config USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* USART2 GPIO config */ 
	/* Configure USART2 Tx (PD.05) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/* Configure USART2 Rx (PD.06) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

	/* USART2 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
//	USART_InitStructure.USART_BaudRate = 19200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	/* Enable USART2 Receive and Transmit interrupts */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 

	USART_Cmd(USART2, ENABLE);
	
		//ʹ��USART2�ж�
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
}

/******************************************************
		��ʽ�������������
        "\r"	�س���	   USART_OUT(USART1, "abcdefg\r")   
		"\n"	���з�	   USART_OUT(USART1, "abcdefg\r\n")
		"%s"	�ַ���	   USART_OUT(USART1, "�ַ����ǣ�%s","abcdefg")
		"%d"	ʮ����	   USART_OUT(USART1, "a=%d",10)
**********************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 
	const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //�ж��Ƿ񵽴��ַ���������
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);	   
					Data++;
					break;
				case 'n':							          //���з�
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				
				default:
					Data++;
				    break;
			}
	
			 
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //�ַ���
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //ʮ����
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}
/******************************************************
		��������ת�ַ�������
        char *itoa(int value, char *string, int radix)
		radix=10 ��ʾ��10����	��ʮ���ƣ�ת�����Ϊ0;  

	    ����d=-379;
		ִ��	itoa(d, buf, 10); ��
		
		buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

//�ض���printf������
int fputc(int ch, FILE *f)
{
 	USART_SendData(USART2, (unsigned char) ch);
 	while( USART_GetFlagStatus(USART2,USART_FLAG_TC)!= SET);
	return (ch);
}

void USART_GetChar(uint8_t nChar) //���ڽ��յ�һ���ֽ�
{
//	if(USART_FrameFlag == 1) return;   //����ϴε�����֡��û��������򷵻�
//	
//	USART_Rx2Buff[nRx2Counter++]=nChar;  //���浽������
//	if(nChar == CMD_BYTE_END)   //�����֡������־
//	{
//		if(nRx2Counter == CMD_BYTE_LENGTH)  //���֡������־���ˣ����ҽ��յ��ֽ������õ���֡Ҫ���ֽ���������ճɹ����ñ�־Ϊ1
//		{
//			USART_FrameFlag = 1;
//		}
//		nRx2Counter=0;
//	}
//	else if(nRx2Counter>=CMD_BYTE_LENGTH)
//	{
//		nRx2Counter = 0;
//	}
	if(USART_FrameFlag == 1) return;   //����ϴε�����֡��û��������򷵻�
	
	if(nRx2Counter==0 && nChar == FRAME_START)
	{
		USART_Rx2Buff[nRx2Counter++]=nChar;  //���浽������
	}
	else if(nRx2Counter>0) //���յ�֡ͷ�Ժ�ż�������
	{
		USART_Rx2Buff[nRx2Counter++]=nChar;  //���浽������
		if(nRx2Counter>=FRAME_BYTE_LENGTH)  //���յ�һ֡����
		{
			nRx2Counter = 0;
			if(USART_Rx2Buff[FRAME_BYTE_LENGTH-1] == FRAME_END) //������һ���ֽ���֡β��������֡����
			{
				USART_FrameFlag=1;
			}
		}
	}	
}

void USART_Process(void) //��������֡
{
//	uint8_t i;
	uint16_t nVal;
	float fKp,fKi,fKd;
	if(USART_FrameFlag == 1)
	{
		//������ԭ�ⲻ�����ͻ�ȥ
//		for(i=0;i<FRAME_BYTE_LENGTH;i++)
//		{
//			USART_SendData(USART2,USART_Rx2Buff[i]);
//			while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
//		}
		
		if(USART_Rx2Buff[1] == 0x11) //��������ֽڵ���0x11����������PID����ָ���ЩЭ������Լ�����
		{
			//Kpֵ
			nVal = (USART_Rx2Buff[2]<<8) + USART_Rx2Buff[3];
			fKp = (float)nVal/100.0;   //����ʱ�Ŵ�100����Ʃ��Kp=2.3������ʱΪ230����
			//Kiֵ
			nVal = (USART_Rx2Buff[4]<<8) + USART_Rx2Buff[5];
			fKi = (float)nVal/100.0;
			//Kpֵ
			nVal = (USART_Rx2Buff[6]<<8) + USART_Rx2Buff[7];
			fKp = (float)nVal/100.0;
			MotorController_SetPIDParam(fKp,fKi,fKd);
		}
		else if(USART_Rx2Buff[1] == 0x12) //��������ֽڵ���0x12���������ü��ٶȲ���ָ���ЩЭ������Լ�����
		{
			//2��3�ֽ��Ǽ��ٶ�ֵ
			nVal = (USART_Rx2Buff[2]<<8) + USART_Rx2Buff[3];
			MotorController_SetAcceleration(nVal);
		}
		//������ϣ�����־��0
		USART_FrameFlag = 0; 
	}
}


/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)      //����2�жϷ���
{
	uint8_t getchar;
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)	   //�ж϶��Ĵ����Ƿ�Ϊ��
  {	
    /* Read one byte from the receive data register */
    getchar = USART_ReceiveData(USART2);   //�����Ĵ��������ݻ��浽���ջ�������
	
    USART_GetChar(getchar);
  }
  
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                   //�����Ϊ�˱���STM32 USART��һ���ֽڷ��Ͳ���ȥ��BUG 
  { 
     USART_ITConfig(USART2, USART_IT_TXE, DISABLE);					     //��ֹ���ͻ��������ж�
  }	
  
}
