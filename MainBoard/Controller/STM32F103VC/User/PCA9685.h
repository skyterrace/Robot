#ifndef __PCA9685_H
#define __PCA9685_H
#include "stm32f10x.h"
#define PCA9685_I2C I2C1
#define PCA9685_Addr 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09


#include "I2CRoutines.h"
Status PCA9685_Init(void);
void PCA9685_Reset(void);
void PCA9685_SetPWMFreq(float freq);
void PCA9685_SetPWM(uint8_t num,uint16_t on,uint16_t off);
uint8_t PCA9685_read(uint8_t startAddress);
void PCA9685_write(uint8_t startAddress, uint8_t buffer);


#endif
