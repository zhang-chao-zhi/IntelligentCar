#include "MOTION_CONTROL.h"

#define Confine_Value 12000//����PWM�޷�ֵ

//-----------------------------motor_test������---------------------------
int16_t E1,E2,E3,E4;
uint16_t PWM1=3000,PWM2=3000,PWM3=3000,PWM4=3000;
int8_t Select=0;
//-----------------------------motor_test������---------------------------


//�������ƣ�motor_test
//�������ܣ����Ա�����������Ƿ�����
//�����������
//������������
//ע�����ʹ��Ӳ�����棬
//					�۲�ֵΪE1,E2,E3,E4
//					PWM����ֵΪPWM1,PWM2,PWM3,PWM4
//					ģʽ����ֵΪSelect
//					0�����ֹͣ
//					1�����A��ת
//					2�����B��ת
//					3�����C��ת
//					4�����D��ת
//					5�����е����ת
void motor_test(void)
{
		if(Select==0)
		{
			HAL_GPIO_WritePin(GPIOG,A1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,A2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,B1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,B2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,C1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,C2_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOG,D1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,D2_Pin,GPIO_PIN_RESET);
		}
		if(Select==1)
		{
			HAL_GPIO_WritePin(GPIOG,A1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG,A2_Pin,GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(GPIOG,B1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,B2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,C1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,C2_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOG,D1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,D2_Pin,GPIO_PIN_RESET);
		}
		if(Select==2)
		{
			HAL_GPIO_WritePin(GPIOG,B1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG,B2_Pin,GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(GPIOG,A1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,A2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,C1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,C2_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOG,D1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,D2_Pin,GPIO_PIN_RESET);
		}
		if(Select==3)
		{
			HAL_GPIO_WritePin(GPIOG,C1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG,C2_Pin,GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(GPIOG,A1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,A2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,B1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,B2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,D1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,D2_Pin,GPIO_PIN_RESET);
		}
		if(Select==4)
		{
			HAL_GPIO_WritePin(GPIOG,D1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,D2_Pin,GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(GPIOG,A1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,A2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,C1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,C2_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOG,B1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOG,B2_Pin,GPIO_PIN_RESET);
		}
		if(Select==5)
		{
			HAL_GPIO_WritePin(GPIOG,D1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,D2_Pin,GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(GPIOG,A1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG,A2_Pin,GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(GPIOG,C1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG,C2_Pin,GPIO_PIN_RESET);	
			
			HAL_GPIO_WritePin(GPIOG,B1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOG,B2_Pin,GPIO_PIN_RESET);
		}
		
		TIM1->CCR1=PWM1;
		TIM1->CCR2=PWM2;
		TIM1->CCR3=PWM3;
		TIM1->CCR4=PWM4;
		
		E1 =-(int16_t)TIM2->CNT ;
		E2 =-(int16_t)TIM3->CNT ;
		E3 = (int16_t)TIM4->CNT ;
		E4 = (int16_t)TIM5->CNT ;
		
		HAL_Delay(5);
}

//�������ƣ�amp_confine
//�������ܣ�����pwm����
//��������������˶�ѧ��PID���������ĸ����PWMֵ
//�����������޷��������ָ�룬����������������޷�����ĸ����PWMֵ
//ע��������ڵ����16V����������ѹ��12V���������Ե��޷����Ϊ10000���ң�����ᵼ��debugģʽ�ѻ�
int16_t *amp_confine(int16_t MA, int16_t MB, int16_t MC, int16_t MD)
{
	static int16_t Confine_Motor[4];
	
	if(MA>=Confine_Value) MA=Confine_Value;
	if(MA<=-Confine_Value) MA=-Confine_Value;
	
	if(MB>=Confine_Value) MB=Confine_Value;
	if(MB<=-Confine_Value) MB=-Confine_Value;
	
	if(MC>=Confine_Value) MC=Confine_Value;
	if(MC<=-Confine_Value) MC=-Confine_Value;
	
	if(MD>=Confine_Value) MD=Confine_Value;
	if(MD<=-Confine_Value) MD=-Confine_Value;
	
	Confine_Motor[0]=MA;
	Confine_Motor[1]=MB;
	Confine_Motor[2]=MC;
	Confine_Motor[3]=MD;
	
	return Confine_Motor;
}

//�������ƣ�set_ccr
//�������ܣ��趨CCR�Ĵ��������µ���ٶ�
//��������������˶�ѧ��PID���������ĸ����PWMֵ
//������������
//ע������Ѿ���������ֵ�������˵������������ת
void set_ccr(int16_t MA, int16_t MB, int16_t MC, int16_t MD)
{
	int16_t *Confine_Motor;
	Confine_Motor=amp_confine(MA, MB, MC, MD);
	
	if(Confine_Motor[0]>=0)
	{
		HAL_GPIO_WritePin(GPIOG,A1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG,A2_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG,A1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG,A2_Pin,GPIO_PIN_SET);
		Confine_Motor[0]=-Confine_Motor[0];
	}
	
	if(Confine_Motor[1]>=0)
	{
		HAL_GPIO_WritePin(GPIOG,B1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG,B2_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG,B1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG,B2_Pin,GPIO_PIN_SET);
		Confine_Motor[1]=-Confine_Motor[1];
	}
	
	if(Confine_Motor[2]>=0)
	{
		HAL_GPIO_WritePin(GPIOG,C1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG,C2_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG,C1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG,C2_Pin,GPIO_PIN_SET);
		Confine_Motor[2]=-Confine_Motor[2];
	}
	
	if(Confine_Motor[3]>=0)
	{
		HAL_GPIO_WritePin(GPIOG,D1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,D2_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOG,D1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,D2_Pin,GPIO_PIN_SET);
		Confine_Motor[3]=-Confine_Motor[3];
	}
	
	
	TIM1->CCR1=Confine_Motor[0];
	TIM1->CCR2=Confine_Motor[1];
	TIM1->CCR3=Confine_Motor[2];
	TIM1->CCR4=Confine_Motor[3];
	
}

/**************************************************************************
�������ƣ�kinematic_analysis
�������ܣ������˶���ѧģ��
���������X Y Z �����ٶ�
��������������ָ�룬����洢���ĸ����Ŀ���ٶ�ֵ
ע�����z����С����ֱ������ת���ٶ�
**************************************************************************/

int16_t *kinematic_analysis(float Vx,float Vy,float Vz)
{
	static int16_t Target[4];
	Target[0]   = -Vx+Vy-Vz*2;
	Target[1]   = +Vx+Vy-Vz*2;
	Target[2]   = -Vx+Vy+Vz*2;
	Target[3]   = +Vx+Vy+Vz*2;
	return Target;
}

/**************************************************************************
�������ƣ�read_encoder
�������ܣ���ȡ������ֵ
�����������
��������������ָ�룬����洢���ĸ��������ٶ�ֵ
ע�������
**************************************************************************/
int16_t *read_encoder(void)
{
	static int16_t Encoder[4];
	
	Encoder[0]=-(int16_t)TIM2->CNT;
	Encoder[1]=-(int16_t)TIM3->CNT;
	Encoder[2]=(int16_t)TIM4->CNT;
	Encoder[3]=(int16_t)TIM5->CNT;
	
	TIM2->CNT=0;
	TIM3->CNT=0;
	TIM4->CNT=0;
	TIM5->CNT=0;
	
	return Encoder;
}

