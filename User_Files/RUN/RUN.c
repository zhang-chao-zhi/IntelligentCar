#include "RUN.h"
//串口2处理

uint8_t USART2_Rx_Buff[USART2_BUFF_SIZE]={0};
uint8_t cnt=0;
int16_t OpenMV_dis=0;
uint8_t OpenMV_Process_Busy=0;

//步骤管理
uint8_t Step = GoIn_Step;	//用于决定进入哪个步骤
uint8_t Step_Count=0;			//记录执行步骤的次数
uint8_t Hold_On=0;				//是否让步骤保持执行的标志

//编码器、目标值、位置值、车的三轴速度
int16_t *Encoder;
int16_t *Target;
int32_t	Position[4]={0};		//记录编码器位置
double Displacement_X=0.0,Displacement_Y=0.0;	//以小车中心为原点，建立的X轴，Y轴坐标系，X轴，Y轴方向上的位移大小
int16_t x=0,y=0,z=0;				//定义小车速度

//计算位置值
void Calculate_Position(void)
{
	Position[0]+=Encoder[0];
	Position[1]+=Encoder[1];
	Position[2]+=Encoder[2];
	Position[3]+=Encoder[3];
}

//计算实际位移值
void Calculate_Displacement(void)
{
	Displacement_X=(Position[1]+Position[3]-Position[0]-Position[2])/4.0/33689.6*23.55;
	Displacement_Y=(Position[0]+Position[1]+Position[2]+Position[3])/4.0/33689.6*23.55;
}

//清空实际位移
void Clear_Displacement(void)
{
	Displacement_Y=0;
	Displacement_X=0;
	Position[0]=0;
	Position[1]=0;
	Position[2]=0;
	Position[3]=0;
}
//方案A的路径规划

//Turn_0To90_Step   Forward_Step   Turn_90To0_Step Shift_Step    Backward_Step 
//void Step_Sequence_PlanA(void)
//{
//			switch(Step_Count)
//		{
//			case 1:Step = GoIn_Step; break;
//			
//			case 2:Step = Forward_Step;break;
//			case 3:Step = Turn_0To90_Step;break;
//			case 4:Step = Turn_90To0_Step;break;
//			case 5:Step = Shift_Step;break;
//			
//			case 6:Step = Turn_0To90_Step;break;
//			case 7:Step = Backward_Step;break;
//			case 8:Step = Turn_90To0_Step;break;
//			case 9:Step = Shift_Step;break;
//			
//			case 10:Step = Turn_0To90_Step;break;
//			case 11:Step = Forward_Step;break;
//			case 12:Step = Turn_90To0_Step;break;
//			case 13:Step = Shift_Step;break;
//			
//			case 14:Step = Turn_0To90_Step;break;
//			case 15:Step = Backward_Step;break;
//			case 16:Step = Turn_90To0_Step;break;
//			case 17:Step = Shift_Step;break;
//			
//			case 18:Step = Turn_0To90_Step;break;
//			case 19:Step = Forward_Step;break;
//			case 20:Step = Turn_90To0_Step;break;
//			case 21:Step = Shift_Step;break;
//			
//			case 22:Step = Turn_0To90_Step;break;
//			case 23:Step = Backward_Step;break;
//			case 24:Step = Turn_90To0_Step;break;
//			case 25:Step = GoOut_step;break;
//		}
//}
void Step_Sequence_PlanB(void)
{
			switch(Step_Count)
		{
			case 1:Step = GoIn_Step; break;
			
			case 2:Step = Backward_Step;break;
			case 3:Step = Turn_0To90_Step;break;
			case 4:Step = Shift_Step_Head;break;
			case 5:Step = Turn_90To0_Step;break;
			
			case 6:Step = Forward_Step;break;
			case 7:Step = Turn_0ToI90_Step;break;
			case 8:Step = Shift_Step_Tail;break;
			case 9:Step = Turn_I90To0_Step;break;
			
			case 10:Step = Backward_Step;break;
			case 11:Step = Turn_0To90_Step;break;
			case 12:Step = Shift_Step_Head;break;
			case 13:Step = Turn_90To0_Step;break;
			
			case 14:Step = Forward_Step;break;
			case 15:Step = Turn_0ToI90_Step;break;
			case 16:Step = Shift_Step_Tail;break;
			case 17:Step = Turn_I90To0_Step;break;
			
			case 18:Step = Backward_Step;break;
			//////这边要转180度
			case 19:Step = Turn_0To180_Step;break;
			//180度任务还没有写
			case 20:Step = Shift_Final_Step;break;
			case 21:Step = Backward_Final_Step;break;
	
			case 22:Step = GoOut_step;break;
//			case 21:Step = Backward_Step;break;
//			case 23:Step = Backward_Step;break;
//			case 24:Step = Turn_90To0_Step;break;
//			case 25:Step = GoOut_step;break;
		}
}

///* USER CODE END Variables */
//osThreadId Run_TaskHandle;
//osThreadId Step_ManagerHandle;
//osThreadId Go_InHandle;
//osThreadId Turn_0To90Handle;
//osThreadId Turn_90To0Handle;
//osThreadId ForwardHandle;
//osThreadId BackwardHandle;
//osThreadId Shift_HeadHandle;
//osThreadId Go_OutHandle;
//osThreadId Read_HWT101Handle;
//osThreadId Start_ISRHandle;
//osThreadId HCSRA_ProcessHandle;
//osThreadId HCSRB_ProcessHandle;
//osThreadId Shift_TailHandle;
//osThreadId Turn_Zero_To_I9Handle;
//osThreadId Turn_I90_To_ZerHandle;
//osThreadId Turn_0To_180Handle;
//osThreadId Shift_FinalHandle;
//osThreadId Backward_FinalHandle;
//osThreadId ServoHandle;
//osThreadId OpenMV_ProcessHandle;

//HWT通讯的函数及变量
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern osSemaphoreId ISR_ReadHWT_SemHandle;
uint8_t SingleData;
SAngle 	hwt101Angle;
float angle;
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	

	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			
			case 0x53:	memcpy(&hwt101Angle,&ucRxBuffer[2],8);break;
			
		}
		angle=(float)hwt101Angle.Angle[2]/32768*180;
		ucRxCnt=0;//清空缓存区
		
	}
}


//中断处理程序，用于释放信号量来进行同步
//切记，使用中断前，需要把freertos里面的library_max_syscall_interupt_priority里面的值调小,初始值为5，最小为1
//并且把NVIC里面的中断优先级数字调高一点
//尽量把中断处理的同步任务的优先级提高，避免被打断从而影响数据
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		osSemaphoreRelease(ISR_ReadHWT_SemHandle);
	}
}

void OpenMV_Init(void)
{
		__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart2,USART2_Rx_Buff,USART2_BUFF_SIZE);
}

