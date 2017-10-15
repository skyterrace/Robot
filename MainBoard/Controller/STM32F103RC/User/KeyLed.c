#include "KeyLed.h"
#include "delay.h"
void Key_Init(uint8_t nKey)  //nKey=1��ֻʹ��Key1��nKey=2��ʹ��Key1��Key2
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	switch(nKey)
	{
		case 2:
			//����PC10ΪKey2
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //PC10	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//�������
			GPIO_Init(GPIOC, &GPIO_InitStructure);
		case 1:
			//����PC11ΪKey1
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				     
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
		default:
			;
	}		
}
uint8_t Key_Pressed(uint8_t nKey)  //���а�������ʱ����1�����򷵻�0��nKeyΪ����������1=Key1��2=Key2
{
	switch(nKey)
	{
		case 2:
			//����PC10ΪKey2
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0)
					return 1;
			}
			break;
		case 1:
			//����PC11ΪKey1
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)==0)
					return 1;
			}
			break;
		default:
			;
	}
	return 0;
}
uint8_t Key_Released(uint8_t nKey) //���а������²��ͷ�ʱ����1�����򷵻�0��nKeyΪ����������1=Key1��2=Key2
{
	switch(nKey)
	{
		case 2:
			//����PC10ΪKey2
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0)
				{
					while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0); //���һֱ������һֱ����
					return 1;
				}
			}
			break;
		case 1:
			//����PC11ΪKey1
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)==0)
				{
					while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)==0); //���һֱ������һֱ����
					return 1;
				}
			}
			break;
		default:
			;
	}
	return 0;	
}

void Led_Init(uint8_t nLed)  //nLed=1��ֻʹ��Led2��nLed=2��ʹ��Led2��Led3
{
	GPIO_InitTypeDef GPIO_InitStructure;
	switch(nLed)
	{
		case 2:
			//����PC12ΪLED3
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PC12	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;				//�������
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
			GPIO_Init(GPIOC, &GPIO_InitStructure);
		case 1:
			//����PA12ΪLED2
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				     
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //���߷�ת�ٶ�Ϊ50MHz
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		default:
			;
	}		
}
