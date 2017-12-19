#ifndef __ADIN_H
#define __ADIN_H
#include "stm32f10x.h"
#define ADIN_CHN_COUNT 9 //AD通道数量
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
//AD端口初始化，PC1/PC2/PC3/PA4/PA5/PC4/PC5/PB0/PB1 共9个端口
void ADIN_Init(void);

//AD转换启动/停止
void ADIN_Enable(FunctionalState NewState);

//获取引脚nIndex的AD数据
uint16_t ADIN_GetValue(uint8_t nIndex);
#endif
