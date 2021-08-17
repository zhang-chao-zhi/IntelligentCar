#ifndef _HCSR04_
#define _HCSR04_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#define Average_count 3		//控制均值滤波的取样数

//定义超声波个数
enum
{
	HCSR_A = 0,
	HCSR_B,
	HCSR_C,
	HCSR_D,
	Total_HCSR
};

//定义超声波结构体
typedef struct
{
	
	uint32_t Output_Distance;				//最终输出的距离
	uint8_t statu_flag;							//上升沿下降沿状态标志位
	uint8_t count_flag;							//计数标志位（用于进行平均值滤波）
	float Temp_Distance;						//距离值缓存（用于进行平均值滤波）
	float Temp_Array[Average_count];//距离值缓存数组（用于进行平均值滤波）
	float Average_Distance;					//取平均值之后的距离
	uint32_t time;									//上升沿的持续时间
	
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

