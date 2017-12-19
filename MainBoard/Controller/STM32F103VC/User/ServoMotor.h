#ifndef __SERVOMOTOR_H__
#define __SERVOMOTOR_H__
#include "stm32f10x.h"

#define SERVOMOTOR_CHN_COUNT  8  //���8·
//1~4·���PWMͨ�����õ�PA0/1/2/3��Ӧ��TIM5-1, TIM5-2, TIM5-3, TIM5-4
//���4·��������Ҫ�õ���������TIM��
//5~6·���õ�PA6/7��Ӧ��TIM3-1/2���������A��Ӧ��TIM3-1/2�����á�
//7~8·���õ�PB10/11��Ӧ��TIM2-3/4���������B��Ӧ��TIM2-1/2�����á�

//��Ҫ˼·��ʹ��TIM5�����·50Hz��PWM��ռ�ձ�1/20 ~ 2/20

void ServoMotor_Init(uint8_t nServoCount); //��ʼ�����PWMͨ��
//void ServoMotor_Cmd(uint8_t nCH, FunctionalState NewState); //������ر�nCHͨ����PWM�����ͨ������ռ�ձ�Ϊ0���ر�PWM���
uint8_t ServoMotor_SetPulse(uint8_t nCH, uint16_t nPulse); //����ͨ��nCH��ռ�ձȣ���ֵ��1000~2000���ֱ��Ӧ1/20~2/20�������Ϊ0����������������൱�ڹ��˶����������������Χ�ڷ���0�����򷵻�1
uint16_t ServoMotor_GetPulse(uint8_t nCH); //��ȡͨ��nCH��ǰ���õ�ռ�ձ�ֵ

#endif
