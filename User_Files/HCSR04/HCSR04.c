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
	if(HCSR_x->statu_flag == 0)//����־λΪ0����ʾ��ʱ������������
		{				
			HCSR_x->statu_flag = 1;//����־λ��1�����ڽ����������½���
			TIM_x->CNT = 0;//��ռ���ֵ
		  __HAL_TIM_SET_CAPTUREPOLARITY(&htim_x,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);//��ת�����ƽ�������������½���ʱ�ٽ����ж�
		}		 
		else//����־λΪ1����ʾ��ʱ�������½���
		{
			HCSR_x->statu_flag = 0;//����־λ��0�����ڽ���������������
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim_x,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);//��ת�����ƽ������������������ʱ�ٽ����ж�
			
			HCSR_x->time = HAL_TIM_ReadCapturedValue(&htim_x, TIM_CHANNEL_1);//��ȡ�ߵ�ƽ����ʱ��		
			HCSR_x->Temp_Distance = Distance_Calculate(HCSR_x->time);//�������
			HCSR_x->Temp_Array[HCSR_x->count_flag++] = HCSR_x->Temp_Distance;//�������飬��������ֵ�˲��㷨
			
			if(HCSR_x->count_flag==Average_count)//����¼����Average_count���������о�ֵ�˲��㷨����
			{
				
				for(int i=0;i<Average_count;i++)//������ľ������
				{
					HCSR_x->Average_Distance = HCSR_x->Average_Distance + HCSR_x->Temp_Array[i];
				}
				HCSR_x->Average_Distance = HCSR_x->Average_Distance/Average_count;//��ƽ��ֵ
				HCSR_x->Output_Distance = (uint32_t)HCSR_x->Average_Distance;//����ֵ�˲��㷨����֮������ݸ�ֵ��ȫ�ֱ���
				HCSR_x->Average_Distance = 0;//������ݣ������´ν��о�ֵ����
				HCSR_x->count_flag=0;//������ݣ����´���
			}
		}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//���벶���жϻص�
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	
	
	if((htim->Instance==TIM10)&&(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1))//�ж϶�ʱ����ͨ��
	{
		osSemaphoreRelease(HCSRA_SemHandle);
	}
	
	
		if((htim->Instance==TIM11)&&(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1))//�ж϶�ʱ����ͨ��
	{
		osSemaphoreRelease(HCSRB_SemHandle);
	}
	
	
		if((htim->Instance==TIM13)&&(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1))//�ж϶�ʱ����ͨ��
	{
//		osSemaphoreRelease(HCSRC_SemHandle);
		//������ɾ���ˣ�Ҫ���ж�����д������
		HCSR_Process(&HCSR_Object[HCSR_C],TIM13,htim13);
	}
	
	
		if((htim->Instance==TIM14)&&(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1))//�ж϶�ʱ����ͨ��
	{
		osSemaphoreRelease(HCSRD_SemHandle);
	}
	
	
}
