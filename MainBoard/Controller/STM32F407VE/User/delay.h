#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f4xx.h"
void TimingDelay_Decrement(void);
void Delay_10us(__IO uint32_t nTime);//��ʱ����λ��10΢��
void Delay_ms(__IO uint32_t nTime);//��ʱ����λ������
extern __IO uint8_t b10msFlag; //ÿ��1ms�ɵδ��ж���1�����ú��뼰ʱ���㡣
#if defined MPU6050    //���ʹ��MPU6050������Ҫ���д���
int stm32_get_clock_ms(unsigned long *count); //�ú������ϵͳ����������MPU6050������Ҫ�õ�
#endif
#endif
