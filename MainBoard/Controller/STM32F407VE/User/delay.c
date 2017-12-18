#include "delay.h"
static __IO uint32_t TimingDelay; //SysTick��������
__IO uint8_t b10msFlag; //ÿ��1ms�ɵδ��ж���1�����ú��뼰ʱ���㡣

#if defined MPU6050    //���ʹ��MPU6050������Ҫ���д���
__IO uint32_t TimeStamp_ms=0; //�������ֵ
uint8_t TimeStamp_ms_counter=0;
#endif

/*ȫ�ֱ���TimingDelay��һ������ֱ��0Ϊֹ*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
	
#if defined MPU6050    //���ʹ��MPU6050������Ҫ���д���
	TimeStamp_ms_counter++;
	if(TimeStamp_ms_counter > 99)
	{
		TimeStamp_ms++;
		TimeStamp_ms_counter = 0;
	}
#endif
	
}

/*��ʱnTime*10΢��*/
void Delay_10us(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;
  while(TimingDelay != 0);   //һֱ�ȵ�TimingDelay����0Ϊֹ
}
/*��ʱnTime����*/
void Delay_ms(__IO uint32_t nTime)
{ 
	uint32_t i;
	i=nTime;
	while(i--)            //һֱ�ȵ�i����0Ϊֹ��iÿDelay_10us(100)��1�����һ
		Delay_10us(100);
}
#if defined MPU6050    //���ʹ��MPU6050������Ҫ���д���
int stm32_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = TimeStamp_ms;
    return 0;
}
#endif
