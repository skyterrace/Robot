#include "KeyLed.h"
#include "delay.h"
void Key_Init(uint8_t nKey)  //nKey=1，只使能Key1，nKey=2，使能Key1和Key2
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
	switch(nKey)
	{
		case 2:
			//配置PC10为Key2
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //PC10	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				//推挽输出
			GPIO_Init(GPIOC, &GPIO_InitStructure);
		case 1:
			//配置PC11为Key1
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				     
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
		default:
			;
	}		
}
uint8_t Key_Pressed(uint8_t nKey)  //当有按键按下时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2
{
	switch(nKey)
	{
		case 2:
			//配置PC10为Key2
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0)
					return 1;
			}
			break;
		case 1:
			//配置PC11为Key1
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
uint8_t Key_Released(uint8_t nKey) //当有按键按下并释放时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2
{
	switch(nKey)
	{
		case 2:
			//配置PC10为Key2
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0)
				{
					while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_10)==0); //如果一直按着则一直等着
					return 1;
				}
			}
			break;
		case 1:
			//配置PC11为Key1
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)==0)
				{
					while(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_11)==0); //如果一直按着则一直等着
					return 1;
				}
			}
			break;
		default:
			;
	}
	return 0;	
}

void Led_Init(uint8_t nLed)  //nLed=1，只使能Led2，nLed=2，使能Led2和Led3
{
	GPIO_InitTypeDef GPIO_InitStructure;
	switch(nLed)
	{
		case 2:
			//配置PC12为LED3
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PC12	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;				//推挽输出
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
			GPIO_Init(GPIOC, &GPIO_InitStructure);
		case 1:
			//配置PA12为LED2
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				     
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			 //口线翻转速度为50MHz
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		default:
			;
	}		
}
