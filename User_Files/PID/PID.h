#ifndef _PID_
#define _PID_
#include "main.h"
enum pid_number
{
	Motor_A=0,
	Motor_B,
	Motor_C,
	Motor_D,
	HWT101,
	HCPID_A,
	HCPID_B,
	HCPID_C,
	HCPID_D,
	LAPID_A,
	LAPID_B,
	HCNEW_A,
	HCNEW_B,
	HCNEW_C,
	NEWHWT180,
	OpenMV,
	PID_TotalNum//前面还可以放超声波等项，PID_TotalNum此项一定要放最后面
};

typedef struct 
{
	float kp;
	float ki;
	float kd;
} PID_Value_Structure;

typedef struct
{
	float desired;		  //< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float outPLimit;
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	
} Pid_Structure;

void PIDInit(Pid_Structure* pid, PID_Value_Structure* value, float outPLimit, float iLimit, float outputLimit);
extern Pid_Structure PID_Object[PID_TotalNum];

void ALL_PID_Init(void);
int PID_Position(int Encoder, int Target, Pid_Structure *PID);
int PID_Position_float(float Angle, float Target, Pid_Structure *PID);
#endif
