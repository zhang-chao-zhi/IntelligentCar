#include "MOTION_CONTROL.h"

#define Confine_Value 12000//定义PWM限幅值

//-----------------------------motor_test参数表---------------------------
int16_t E1,E2,E3,E4;
uint16_t PWM1=3000,PWM2=3000,PWM3=3000,PWM4=3000;
int8_t Select=0;
//-----------------------------motor_test参数表---------------------------


//函数名称：motor_test
//函数功能：测试编码器及电机是否在线
//传入参数：无
//传出参数：无
//注意事项：使用硬件仿真，
//					观察值为E1,E2,E3,E4
//					PWM调试值为PWM1,PWM2,PWM3,PWM4
//					模式设置值为Select
//					0：电机停止
//					1：电机A正转
//					2：电机B正转
//					3：电机C正转
//					4：电机D正转
//					5：所有电机正转
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

//函数名称：amp_confine
//函数功能：限制pwm幅度
//传入参数：经过运动学和PID计算过后的四个电机PWM值
//传出参数：限幅后的数组指针，其中数组里面包括限幅后的四个电机PWM值
//注意事项：由于电池有16V，而电机额定电压是12V，经过测试得限幅最好为10000左右，否则会导致debug模式脱机
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

//函数名称：set_ccr
//函数功能：设定CCR寄存器，更新电机速度
//传入参数：经过运动学和PID计算过后的四个电机PWM值
//传出参数：无
//注意事项：已经根据正负值，调整了电机驱动的正反转
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
函数名称：kinematic_analysis
函数功能：麦轮运动数学模型
传入参数：X Y Z 三轴速度
传出参数：数组指针，数组存储有四个电机目标速度值
注意事项：z轴是小车竖直方向旋转的速度
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
函数名称：read_encoder
函数功能：读取编码器值
传入参数：无
传出参数：数组指针，数组存储有四个编码器速度值
注意事项：无
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

