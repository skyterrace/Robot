#ifndef __KEYLED_H
#define __KEYLED_H
#include "stm32f10x.h"
void Key_Init(uint8_t nKey);  //nKey=1��ֻʹ��Key1��nKey=2��ʹ��Key1��Key2
uint8_t Key_Pressed(uint8_t nKey);  //���а�������ʱ����1�����򷵻�0��nKeyΪ����������1=Key1��2=Key2
uint8_t Key_Released(uint8_t nKey); //���а������²��ͷ�ʱ����1�����򷵻�0��nKeyΪ����������1=Key1��2=Key2


void Led_Init(uint8_t nLed);  //nLed=1��ֻʹ��Led2��nLed=2��ʹ��Led2��Led3
#define LED2_ON(); GPIO_ResetBits(GPIOD,GPIO_Pin_4);
#define LED2_OFF(); GPIO_SetBits(GPIOD,GPIO_Pin_4);
#define LED3_ON(); GPIO_ResetBits(GPIOD,GPIO_Pin_7);
#define LED3_OFF(); GPIO_SetBits(GPIOD,GPIO_Pin_7);
#endif
