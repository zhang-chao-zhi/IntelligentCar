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
//�оٲ���
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


//HWTͨѶ�ĽǶȽṹ��
typedef struct 
{
	short Angle[3];
	short T;
}SAngle;

//HWTͨѶ�ĺ���������
extern uint8_t SingleData;
extern float angle;
void CopeSerial2Data(unsigned char ucData);

//�������
extern uint8_t Step;
extern uint8_t Step_Count;
extern uint8_t Hold_On;

//��������Ŀ��ֵ��λ��ֵ�����������ٶ�
extern int16_t *Encoder;
extern int16_t *Target;
extern int32_t	Position[4];
extern double Displacement_X,Displacement_Y;	//��С������Ϊԭ�㣬������X�ᣬY������ϵ��X�ᣬY�᷽���ϵ�λ�ƴ�С
extern int16_t x,y,z;

//����λ��ֵ
void Calculate_Position(void);
	
//����ʵ��λ��ֵ
void Calculate_Displacement(void);

//���ʵ��λ��
void Clear_Displacement(void);

////����A��·���滮
//void Step_Sequence_PlanA(void);

//����A��·���滮
void Step_Sequence_PlanB(void);

#endif
