#ifndef _RUN_
#define _RUN_
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"

#define USART2_BUFF_SIZE 10 
extern uint8_t USART2_Rx_Buff[];
extern uint8_t cnt;
extern int16_t OpenMV_dis;
extern uint8_t OpenMV_Process_Busy;
extern UART_HandleTypeDef huart2;
void OpenMV_Init(void);
//列举步骤
enum Step_Type
{
	GoIn_Step = 0,
	Turn_90To0_Step,
	Turn_0To90_Step,
	Turn_0ToI90_Step,
	Turn_I90To0_Step,
	Forward_Step,
	Backward_Step,
	Shift_Step_Head,
	Shift_Step_Tail,
	GoOut_step,
	Stop,
	Shift_Final_Step,
	Backward_Final_Step,
	Turn_0To180_Step
};


//HWT通讯的角度结构体
typedef struct 
{
	short Angle[3];
	short T;
}SAngle;

//HWT通讯的函数及变量
extern uint8_t SingleData;
extern float angle;
void CopeSerial2Data(unsigned char ucData);

//步骤管理
extern uint8_t Step;
extern uint8_t Step_Count;
extern uint8_t Hold_On;

//编码器、目标值、位置值、车的三轴速度
extern int16_t *Encoder;
extern int16_t *Target;
extern int32_t	Position[4];
extern double Displacement_X,Displacement_Y;	//以小车中心为原点，建立的X轴，Y轴坐标系，X轴，Y轴方向上的位移大小
extern int16_t x,y,z;

//计算位置值
void Calculate_Position(void);
	
//计算实际位移值
void Calculate_Displacement(void);

//清空实际位移
void Clear_Displacement(void);

////方案A的路径规划
//void Step_Sequence_PlanA(void);

//方案A的路径规划
void Step_Sequence_PlanB(void);

#endif
