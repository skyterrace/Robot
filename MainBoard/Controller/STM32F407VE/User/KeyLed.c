#include "KeyLed.h"
#include "delay.h"
void Key_Init(uint8_t nKey)  //nKey=1��ֻʹ��Key1��nKey=2��ʹ��Key1��Key2
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
	switch(nKey)
	{
		case 2:
			//����PE3ΪKey2
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;				//�������
//			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		
			GPIO_Init(GPIOE, &GPIO_InitStructure);
		case 1:
			//����PE2ΪKey1
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				     
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			
			GPIO_Init(GPIOE, &GPIO_InitStructure);
		default:
			;
	}		
}
uint8_t Key_Pressed(uint8_t nKey)  //���а�������ʱ����1�����򷵻�0��nKeyΪ����������1=Key1��2=Key2
{
	switch(nKey)
	{
		case 2:
			//����PE3ΪKey2
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
					return 1;
			}
			break;
		case 1:
			//����PE2ΪKey1
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
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
			//����PE3ΪKey2
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
				{
					while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0); //���һֱ������һֱ����
					return 1;
				}
			}
			break;
		case 1:
			//����PE2ΪKey1
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
				{
					while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0); //���һֱ������һֱ����
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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	switch(nLed)
	{
		case 2:
			//����PD7ΪLED3

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //PC12	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//�������
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);
		case 1:
			//����PD4ΪLED2
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				     
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);
		default:
			;
	}		
}
