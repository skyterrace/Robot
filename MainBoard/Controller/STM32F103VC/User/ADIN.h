#ifndef __ADIN_H
#define __ADIN_H
#include "stm32f10x.h"
#define ADIN_CHN_COUNT 9 //ADͨ������
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
//AD�˿ڳ�ʼ����PC1/PC2/PC3/PA4/PA5/PC4/PC5/PB0/PB1 ��9���˿�
void ADIN_Init(void);

//ADת������/ֹͣ
void ADIN_Enable(FunctionalState NewState);

//��ȡ����nIndex��AD����
uint16_t ADIN_GetValue(uint8_t nIndex);
#endif
