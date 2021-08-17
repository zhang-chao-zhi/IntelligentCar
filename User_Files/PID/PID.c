#include "PID.h"

#define Motor_outPLimit 5500			//P输出限幅
#define Motor_iLimit 10000				//积分限幅
#define Motor_outputLimit 14000		//输出限幅

#define HWT101_outPLimit 2000				//P输出限幅
#define HWT101_iLimit 10000				//积分限幅
#define HWT101_outputLimit 12000		//输出限幅

#define HCSR_outPLimit 4000				//P输出限幅
#define HCSR_iLimit 10000				//积分限幅
#define HCSR_outputLimit 12000		//输出限幅

#define LASER_outPLimit 4000				//P输出限幅
#define LASER_iLimit 10000				//积分限幅
#define LASER_outputLimit 12000		//输出限幅

PID_Value_Structure PID_Value[PID_TotalNum];//定义PID参数结构体
Pid_Structure PID_Object[PID_TotalNum];//定义PID对象

//初始化一个PID参数
void PIDInit(Pid_Structure* pid, PID_Value_Structure* value, float outPLimit, float iLimit, float outputLimit)
{
	pid->desired   = 0; //目标值
	pid->error     = 0; //这一次的差
	pid->prevError = 0;	//上一次的差
	pid->integ     = 0; //积分
	pid->deriv     = 0; //微分
	pid->kp 	   	 = value->kp;
	pid->ki        = value->ki;
	pid->kd        = value->kd;
	pid->outP      = 0;
	pid->outI      = 0;
	pid->outD      = 0;
	pid->outPLimit = outPLimit; //P输出限幅
	pid->iLimit    = iLimit; //积分限幅
	pid->outputLimit = outputLimit; //输出限幅
}

//初始化所有PID参数
void ALL_PID_Init(void)
{
	PID_Value[Motor_A].kp = 20;
	PID_Value[Motor_A].ki = 1.35;
	PID_Value[Motor_A].kd = 0;
	
	PID_Value[Motor_B].kp = 20;
	PID_Value[Motor_B].ki = 1.5;
	PID_Value[Motor_B].kd = 0;
	
	PID_Value[Motor_C].kp = 20;
	PID_Value[Motor_C].ki = 1.5;
	PID_Value[Motor_C].kd = 0;
	
	PID_Value[Motor_D].kp = 20;
	PID_Value[Motor_D].ki = 1.5;
	PID_Value[Motor_D].kd = 0;
	
	PID_Value[HWT101].kp = 6;
	PID_Value[HWT101].ki = 0;
	PID_Value[HWT101].kd = 0;
	
	PID_Value[NEWHWT180].kp = 5;
	PID_Value[NEWHWT180].ki = 0;
	PID_Value[NEWHWT180].kd = 0;
	
//	PID_Value[HCPID_A].kp = 10;
//	PID_Value[HCPID_A].ki = 0;
//	PID_Value[HCPID_A].kd = 0;
//	
//	PID_Value[HCPID_B].kp = -10;
//	PID_Value[HCPID_B].ki = 0;
//	PID_Value[HCPID_B].kd = 0;
//	
//	PID_Value[HCPID_C].kp = -1;
//	PID_Value[HCPID_C].ki = 0;
//	PID_Value[HCPID_C].kd = 0;
	
	PID_Value[LAPID_A].kp = 35;
	PID_Value[LAPID_A].ki = 0;
	PID_Value[LAPID_A].kd = 0;
	
	PID_Value[LAPID_B].kp = -40;
	PID_Value[LAPID_B].ki = 0;
	PID_Value[LAPID_B].kd = 0;
	
	PID_Value[HCNEW_A].kp = 40;
	PID_Value[HCNEW_A].ki = 0;
	PID_Value[HCNEW_A].kd = 0;
	
	PID_Value[HCNEW_B].kp = -40;
	PID_Value[HCNEW_B].ki = 0;
	PID_Value[HCNEW_B].kd = 0;
	
	PID_Value[HCNEW_C].kp = -40;
	PID_Value[HCNEW_C].ki = 0;
	PID_Value[HCNEW_C].kd = 0;
	
	PID_Value[OpenMV].kp = 1;
	PID_Value[OpenMV].ki = 0.01;
	PID_Value[OpenMV].kd = 0;
	
	PIDInit(&PID_Object[Motor_A],&PID_Value[Motor_A],Motor_outPLimit,Motor_iLimit,Motor_outputLimit);
	PIDInit(&PID_Object[Motor_B],&PID_Value[Motor_B],Motor_outPLimit,Motor_iLimit,Motor_outputLimit);
	PIDInit(&PID_Object[Motor_C],&PID_Value[Motor_C],Motor_outPLimit,Motor_iLimit,Motor_outputLimit);
	PIDInit(&PID_Object[Motor_D],&PID_Value[Motor_D],Motor_outPLimit,Motor_iLimit,Motor_outputLimit);
	
	PIDInit(&PID_Object[HWT101],&PID_Value[HWT101],HWT101_outPLimit,HWT101_iLimit,HWT101_outputLimit);
	PIDInit(&PID_Object[NEWHWT180],&PID_Value[NEWHWT180],HWT101_outPLimit,HWT101_iLimit,HWT101_outputLimit);
	
//	PIDInit(&PID_Object[HCPID_A],&PID_Value[HCPID_A],HCSR_outPLimit,HCSR_iLimit,HCSR_outputLimit);
//	PIDInit(&PID_Object[HCPID_B],&PID_Value[HCPID_B],HCSR_outPLimit,HCSR_iLimit,HCSR_outputLimit);
	
//	
//	PIDInit(&PID_Object[HCPID_C],&PID_Value[HCPID_C],HCSR_outPLimit,HCSR_iLimit,HCSR_outputLimit);
//	PIDInit(&PID_Object[HCPID_D],&PID_Value[HCPID_D],HCSR_outPLimit,HCSR_iLimit,HCSR_outputLimit);
	
	PIDInit(&PID_Object[LAPID_A],&PID_Value[LAPID_A],LASER_outPLimit,LASER_iLimit,LASER_outputLimit);
	PIDInit(&PID_Object[LAPID_B],&PID_Value[LAPID_B],LASER_outPLimit,LASER_iLimit,LASER_outputLimit);
	
	PIDInit(&PID_Object[HCNEW_A],&PID_Value[HCNEW_A],HCSR_outPLimit,HCSR_iLimit,HCSR_outputLimit);
	PIDInit(&PID_Object[HCNEW_B],&PID_Value[HCNEW_B],HCSR_outPLimit,HCSR_iLimit,HCSR_outputLimit);
	PIDInit(&PID_Object[HCNEW_C],&PID_Value[HCNEW_C],HCSR_outPLimit,HCSR_iLimit,HCSR_outputLimit);

	
	PIDInit(&PID_Object[OpenMV],&PID_Value[OpenMV],HCSR_outPLimit,HCSR_iLimit,HCSR_outputLimit);
}



//位置式PID
int PID_Position(int Encoder, int Target, Pid_Structure *PID)
{
  static int pwm;                                  
 
  PID->error = Encoder - Target;	//获取偏差
	PID->integ+=PID->error;					//获取积分
	
	if(PID->integ >  PID->iLimit)  PID->integ =  PID->iLimit;//积分限幅
	if(PID->integ < -PID->iLimit)  PID->integ = -PID->iLimit;//积分限幅
	
	PID->outP = PID->kp * PID->error;//计算比例部分的输出
	PID->outI = PID->ki * PID->integ;//计算积分部分的输出
	
	if(PID->outP >=  PID->outPLimit) PID->outP =  PID->outPLimit;//P输出限幅
	if(PID->outP <= -PID->outPLimit) PID->outP = -PID->outPLimit;//P输出限幅
	
	pwm = PID->outP + PID->outI + PID->kd*(PID->error - PID->prevError);//计算输出
	

	PID->prevError = PID->error;//记录上次偏差
	
	return -pwm;
}

int PID_Position_float(float Angle, float Target, Pid_Structure *PID)
{
  static int pwm;                                  
 
  PID->error = Angle - Target;	//获取偏差
	PID->integ+=PID->error;					//获取积分
	
	if(PID->integ >  PID->iLimit)  PID->integ =  PID->iLimit;//积分限幅
	if(PID->integ < -PID->iLimit)  PID->integ = -PID->iLimit;//积分限幅
	
	PID->outP = PID->kp * PID->error;//计算比例部分的输出
	PID->outI = PID->ki * PID->integ;//计算积分部分的输出
	
	if(PID->outP >=  PID->outPLimit) PID->outP =  PID->outPLimit;//P输出限幅
	if(PID->outP <= -PID->outPLimit) PID->outP = -PID->outPLimit;//P输出限幅
	
	pwm = PID->outP + PID->outI + PID->kd*(PID->error - PID->prevError);//计算输出
	
	PID->prevError = PID->error;//记录上次偏差
	
	return -pwm;
}



