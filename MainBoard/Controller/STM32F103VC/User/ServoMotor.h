#ifndef __SERVOMOTOR_H__
#define __SERVOMOTOR_H__
#include "stm32f10x.h"

#define SERVOMOTOR_CHN_COUNT  8  //最多8路
//1~4路舵机PWM通道。用到PA0/1/2/3对应的TIM5-1, TIM5-2, TIM5-3, TIM5-4
//如果4路不够，则要用到编码器的TIM。
//5~6路，用到PA6/7对应的TIM3-1/2，则编码器A对应的TIM3-1/2不可用。
//7~8路，用到PB10/11对应的TIM2-3/4，则编码器B对应的TIM2-1/2不可用。

//主要思路：使用TIM5输出四路50Hz的PWM，占空比1/20 ~ 2/20

void ServoMotor_Init(uint8_t nServoCount); //初始化舵机PWM通道
//void ServoMotor_Cmd(uint8_t nCH, FunctionalState NewState); //开启或关闭nCH通道的PWM输出，通过设置占空比为0来关闭PWM输出
uint8_t ServoMotor_SetPulse(uint8_t nCH, uint16_t nPulse); //设置通道nCH的占空比，数值在1000~2000，分别对应1/20~2/20。如果设为0，则无脉冲输出，相当于关了舵机。如果不在这个范围内返回0，否则返回1
uint16_t ServoMotor_GetPulse(uint8_t nCH); //获取通道nCH当前设置的占空比值

#endif
