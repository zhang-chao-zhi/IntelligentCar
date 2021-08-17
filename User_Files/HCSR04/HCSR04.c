#include "HCSR04.h"

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

extern osSemaphoreId HCSRA_SemHandle;
extern osSemaphoreId HCSRB_SemHandle;
extern osSemaphoreId HCSRC_SemHandle;
extern osSemaphoreId HCSRD_SemHandle;

HCSR_Sructure HCSR_Object[Total_HCSR];

void Ultrasonic_Trig_A(void)
{
	HAL_GPIO_WritePin(GPIOF, Trig_A_Pin, GPIO_PIN_SET);
	for(int i=0;i<256;i++);
	HAL_GPIO_WritePin(GPIOF, Trig_A_Pin, GPIO_PIN_RESET);
}
void Ultrasonic_Trig_B(void)
{
	HAL_GPIO_WritePin(GPIOF, Trig_B_Pin, GPIO_PIN_SET);
	for(int i=0;i<256;i++);
	HAL_GPIO_WritePin(GPIOF, Trig_B_Pin, GPIO_PIN_RESET);
}
void Ultrasonic_Trig_C(void)
{
	HAL_GPIO_WritePin(GPIOF, Trig_C_Pin, GPIO_PIN_SET);
	for(int i=0;i<256;i++);
	HAL_GPIO_WritePin(GPIOF, Trig_C_Pin, GPIO_PIN_RESET);
}
void Ultrasonic_Trig_D(void)
{
	HAL_GPIO_WritePin(GPIOF, Trig_D_Pin, GPIO_PIN_SET);
	for(int i=0;i<256;i++);
	HAL_GPIO_WritePin(GPIOF, Trig_D_Pin, GPIO_PIN_RESET);
}

void Ultrasonic_Trig_ALL(void)
{
	Ultrasonic_Trig_A();
	Ultrasonic_Trig_B();
	Ultrasonic_Trig_C();
	Ultrasonic_Trig_D();
}

float Distance_Calculate(uint32_t time)
{
    float Distance = 0;
    Distance = ((float)time / 58) ;
		return Distance;
}

void HCSR_Process(HCSR_Sructure *HCSR_x , TIM_TypeDef *TIM_x , TIM_HandleTypeDef htim_x)
{
	if(HCSR_x->statu_flag == 0)//当标志位为0，表示此时捕获到了上升沿
		{				
			HCSR_x->statu_flag = 1;//将标志位置1，便于接下来捕获下降沿
			TIM_x->CNT = 0;//清空计数值
		  __HAL_TIM_SET_CAPTUREPOLARITY(&htim_x,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);//翻转捕获电平，接下来捕获下降沿时再进入中断
		}		 
		else//当标志位为1，表示此时捕获到了下降沿
		{
			HCSR_x->statu_flag = 0;//将标志位置0，便于接下来捕获上升沿
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim_x,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);//翻转捕获电平，接下来捕获上升沿时再进入中断
			
			HCSR_x->time = HAL_TIM_ReadCapturedValue(&htim_x, TIM_CHANNEL_1);//获取高电平持续时间		
			HCSR_x->Temp_Distance = Distance_Calculate(HCSR_x->time);//解算距离
			HCSR_x->Temp_Array[HCSR_x->count_flag++] = HCSR_x->Temp_Distance;//存入数组，便于做均值滤波算法
			
			if(HCSR_x->count_flag==Average_count)//当记录到了Average_count个数，进行均值滤波算法处理
			{
				
				for(int i=0;i<Average_count;i++)//将缓存的距离相加
				{
					HCSR_x->Average_Distance = HCSR_x->Average_Distance + HCSR_x->Temp_Array[i];
				}
				HCSR_x->Average_Distance = HCSR_x->Average_Distance/Average_count;//求平均值
				HCSR_x->Output_Distance = (uint32_t)HCSR_x->Average_Distance;//将均值滤波算法处理之后的数据赋值给全局变量
				HCSR_x->Average_Distance = 0;//清空数据，便于下次进行均值处理
				HCSR_x->count_flag=0;//清空数据，重新存入
			}
		}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//输入捕获中断回调
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	
	
	if((htim->Instance==TIM10)&&(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1))//判断定时器与通道
	{
		osSemaphoreRelease(HCSRA_SemHandle);
	}
	
	
		if((htim->Instance==TIM11)&&(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1))//判断定时器与通道
	{
		osSemaphoreRelease(HCSRB_SemHandle);
	}
	
	
		if((htim->Instance==TIM13)&&(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1))//判断定时器与通道
	{
//		osSemaphoreRelease(HCSRC_SemHandle);
		//此任务被删除了，要在中断里面写处理函数
		HCSR_Process(&HCSR_Object[HCSR_C],TIM13,htim13);
	}
	
	
		if((htim->Instance==TIM14)&&(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1))//判断定时器与通道
	{
		osSemaphoreRelease(HCSRD_SemHandle);
	}
	
	
}
