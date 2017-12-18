#ifndef __MOTORCONTROLLER_H
#define __MOTORCONTROLLER_H
#include "stm32f10x.h"

//ʹ���˶�ʱ��6
#define MOTOR_CONTROLLER_PERIOD 20  //�������ڣ���λ��ms
//PID������ʼֵ�������п�ͨ�����ú����ı�
#define MOTOR_CONTROLLER_KP 10
#define MOTOR_CONTROLLER_KI 2
#define MOTOR_CONTROLLER_KD 3
//MotorController_Init() ��ʼ������
//nEncoderResolution�������ֱ��ʣ�����һȦ����������nWheelDiameter���ӵ�ֱ������λ��mm
//nMotorCount������������Ϊ2������A��B��������Ϊ4������A��B��C��D�ĸ����
void MotorController_Init(uint16_t nEncoderResolution, uint8_t nWheelDiameter,uint8_t nMotorCount); 
void MotorController_SetAcceleration(uint16_t nAcc); //�������ӵļ��ٶ�ֵ����λmm/s/s����Ϊ0�൱����Сֵ1��
void MotorController_SetSpeed(uint8_t nMotor, int16_t nSpeed); //��������ת�٣�nMotor�����ţ�nSpeed�������ٶȣ���λ��mm/s
void MotorController_Enable(FunctionalState NewState); //nEnable=1���ÿ�������=0 ֹͣ������
void MotorController_SpeedTunner(void); //���ٺ������ɶ�ʱ�����տ������ڶ�ʱ���á�
void MotorController_SetPIDParam(float Kp,float Ki,float Kd); //����PID����
#endif
