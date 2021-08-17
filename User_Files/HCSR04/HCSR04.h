#ifndef _HCSR04_
#define _HCSR04_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#define Average_count 3		//���ƾ�ֵ�˲���ȡ����

//���峬��������
enum
{
	HCSR_A = 0,
	HCSR_B,
	HCSR_C,
	HCSR_D,
	Total_HCSR
};

//���峬�����ṹ��
typedef struct
{
	
	uint32_t Output_Distance;				//��������ľ���
	uint8_t statu_flag;							//�������½���״̬��־λ
	uint8_t count_flag;							//������־λ�����ڽ���ƽ��ֵ�˲���
	float Temp_Distance;						//����ֵ���棨���ڽ���ƽ��ֵ�˲���
	float Temp_Array[Average_count];//����ֵ�������飨���ڽ���ƽ��ֵ�˲���
	float Average_Distance;					//ȡƽ��ֵ֮��ľ���
	uint32_t time;									//�����صĳ���ʱ��
	
}HCSR_Sructure;

extern HCSR_Sructure HCSR_Object[Total_HCSR];

float Distance_Calculate(uint32_t time);
void HCSR_Process(HCSR_Sructure *HCSR_x , TIM_TypeDef *TIM_x , TIM_HandleTypeDef htim_x);
void Ultrasonic_Trig_A(void);
void Ultrasonic_Trig_B(void);
void Ultrasonic_Trig_C(void);
void Ultrasonic_Trig_D(void);
void Ultrasonic_Trig_ALL(void);

#endif

