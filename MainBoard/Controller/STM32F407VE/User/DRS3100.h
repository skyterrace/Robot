#ifndef __DRS3100_H
#define __DRS3100_H
#include "stm32f4xx.h"


/*BT04��������͸��ģ��*/

//ʹ��USART2����ӦPA2/TX��PA3/RX��

typedef enum {DRS3100_USART = 0, DRS3100_I2C = 1} DRS3100_COM_Type;
#define DRS3100_I2Cx I2C1
#define DRS3100_I2C_Address 0x54
#define DRS3100_FRAME_BYTE_LENGTH 19 //����ͨѶһ֡���ݵ��ֽ�������֡ͷ��֡β����Ʃ��20���ֽ�Ϊһ������������֡����1���ֽ�֡ͷ����2���ֽڴ����������ͣ���3~6�ֽ��������������7���ֽ�Ϊ֡β
#define DRS3100_FRAME_START 0x55 //֡ͷ


//���������֡���ձ�־�ͽ��ջ���������ɿɹ�BTModule.c�����Դ����ֱ��ʹ�ã�ֻҪinclude��BTModule.h�ļ���
extern __IO uint8_t DRS3100_Buff[DRS3100_FRAME_BYTE_LENGTH]; //���ջ�����
extern __IO uint8_t DRS3100_FrameFlag; //������������֡��־��1������0������

void DRS3100_Init(DRS3100_COM_Type ComType);

void DRS3100_GetPoint8(uint8_t *p);
void DRS3100_GetPoint24(uint8_t *p);
void DRS3100_GetPoint139(uint8_t *p);
void DRS3100_GetTurn(uint8_t *p);
void DRS3100_GetFlagLimit(uint8_t *p);

#endif
