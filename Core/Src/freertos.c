/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MOTION_CONTROL.h"
#include "PID.h"
#include "RUN.h"
#include "HCSR04.h"
#include "stdio.h"
#include	"bsp_laser.h"
#include  "bsp_gyro.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Turn_Range 3.0
#define Forward_Dis 30
#define Backward_Dis 10
#define AlongWall_Dis 230
#define GoIn_Dis 50
#define GoOut_Dis 50

#define Shift_Dis 57


#define fast_speed 1500
#define middle_speed 800
#define shift_speed 1200

uint8_t Servo_Flag=0;
//1200 中等
//800  最慢

uint16_t up_servo=2400;

uint16_t down_servo=900;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;


/* USER CODE END Variables */
osThreadId Run_TaskHandle;
osThreadId Step_ManagerHandle;
osThreadId Go_InHandle;
osThreadId Turn_0To90Handle;
osThreadId Turn_90To0Handle;
osThreadId ForwardHandle;
osThreadId BackwardHandle;
osThreadId Shift_HeadHandle;
osThreadId Go_OutHandle;
osThreadId Read_HWT101Handle;
osThreadId Start_ISRHandle;
osThreadId HCSRA_ProcessHandle;
osThreadId HCSRB_ProcessHandle;
osThreadId Shift_TailHandle;
osThreadId Turn_Zero_To_I9Handle;
osThreadId Turn_I90_To_ZerHandle;
osThreadId Turn_0To_180Handle;
osThreadId Shift_FinalHandle;
osThreadId Backward_FinalHandle;
osThreadId ServoHandle;
osThreadId OpenMV_ProcessHandle;
osThreadId openmv_walkHandle;
osSemaphoreId GoIn_SemHandle;
osSemaphoreId Turn_0To90_SemHandle;
osSemaphoreId Turn_90To0_SemHandle;
osSemaphoreId Shift_Head_SemHandle;
osSemaphoreId GoOut_SemHandle;
osSemaphoreId Forward_SemHandle;
osSemaphoreId Backward_SemHandle;
osSemaphoreId Step_Manager_SemHandle;
osSemaphoreId ISR_ReadHWT_SemHandle;
osSemaphoreId HCSRA_SemHandle;
osSemaphoreId HCSRB_SemHandle;
osSemaphoreId HCSRC_SemHandle;
osSemaphoreId HCSRD_SemHandle;
osSemaphoreId Shift_Tail_SemHandle;
osSemaphoreId Turn_0_I90_SemHandle;
osSemaphoreId Turn_I90_To_0_SemHandle;
osSemaphoreId Turn_0To_180_SemHandle;
osSemaphoreId Shift_Final_SemHandle;
osSemaphoreId Backward_Final_SemHandle;
osSemaphoreId ServoSemHandle;
osSemaphoreId OpenMV_SemHandle;
osSemaphoreId openmvSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void RunTask(void const * argument);
void StepManager(void const * argument);
void GoIn(void const * argument);
void Turn0_To_90(void const * argument);
void Turn90_To_0(void const * argument);
void Forward_(void const * argument);
void Backward_(void const * argument);
void Shift_Head_Task(void const * argument);
void GoOut(void const * argument);
void ReadHWT101(void const * argument);
void StartISR(void const * argument);
void HCSRAProcess(void const * argument);
void HCSRBProcess(void const * argument);
void Shift_Tail_Task(void const * argument);
void Turn_Zero_To_I90_Task(void const * argument);
void Turn_I90_To_Zero(void const * argument);
void Turn_0To_180_Task(void const * argument);
void Shift_Final_Task(void const * argument);
void Backward_Final_Task(void const * argument);
void Servo_Task(void const * argument);
void OpenMV_Process_Task(void const * argument);
void openmv_walk_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	 printf("任务：%s堆栈溢出\r\n", pcTaskName);
}

float absolute(float ang)
{
	if(ang>=0) return ang;
	else return 360+ang;
}
/* USER CODE END 4 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of GoIn_Sem */
  osSemaphoreDef(GoIn_Sem);
  GoIn_SemHandle = osSemaphoreCreate(osSemaphore(GoIn_Sem), 1);

  /* definition and creation of Turn_0To90_Sem */
  osSemaphoreDef(Turn_0To90_Sem);
  Turn_0To90_SemHandle = osSemaphoreCreate(osSemaphore(Turn_0To90_Sem), 1);

  /* definition and creation of Turn_90To0_Sem */
  osSemaphoreDef(Turn_90To0_Sem);
  Turn_90To0_SemHandle = osSemaphoreCreate(osSemaphore(Turn_90To0_Sem), 1);

  /* definition and creation of Shift_Head_Sem */
  osSemaphoreDef(Shift_Head_Sem);
  Shift_Head_SemHandle = osSemaphoreCreate(osSemaphore(Shift_Head_Sem), 1);

  /* definition and creation of GoOut_Sem */
  osSemaphoreDef(GoOut_Sem);
  GoOut_SemHandle = osSemaphoreCreate(osSemaphore(GoOut_Sem), 1);

  /* definition and creation of Forward_Sem */
  osSemaphoreDef(Forward_Sem);
  Forward_SemHandle = osSemaphoreCreate(osSemaphore(Forward_Sem), 1);

  /* definition and creation of Backward_Sem */
  osSemaphoreDef(Backward_Sem);
  Backward_SemHandle = osSemaphoreCreate(osSemaphore(Backward_Sem), 1);

  /* definition and creation of Step_Manager_Sem */
  osSemaphoreDef(Step_Manager_Sem);
  Step_Manager_SemHandle = osSemaphoreCreate(osSemaphore(Step_Manager_Sem), 1);

  /* definition and creation of ISR_ReadHWT_Sem */
  osSemaphoreDef(ISR_ReadHWT_Sem);
  ISR_ReadHWT_SemHandle = osSemaphoreCreate(osSemaphore(ISR_ReadHWT_Sem), 1);

  /* definition and creation of HCSRA_Sem */
  osSemaphoreDef(HCSRA_Sem);
  HCSRA_SemHandle = osSemaphoreCreate(osSemaphore(HCSRA_Sem), 1);

  /* definition and creation of HCSRB_Sem */
  osSemaphoreDef(HCSRB_Sem);
  HCSRB_SemHandle = osSemaphoreCreate(osSemaphore(HCSRB_Sem), 1);

  /* definition and creation of HCSRC_Sem */
  osSemaphoreDef(HCSRC_Sem);
  HCSRC_SemHandle = osSemaphoreCreate(osSemaphore(HCSRC_Sem), 1);

  /* definition and creation of HCSRD_Sem */
  osSemaphoreDef(HCSRD_Sem);
  HCSRD_SemHandle = osSemaphoreCreate(osSemaphore(HCSRD_Sem), 1);

  /* definition and creation of Shift_Tail_Sem */
  osSemaphoreDef(Shift_Tail_Sem);
  Shift_Tail_SemHandle = osSemaphoreCreate(osSemaphore(Shift_Tail_Sem), 1);

  /* definition and creation of Turn_0_I90_Sem */
  osSemaphoreDef(Turn_0_I90_Sem);
  Turn_0_I90_SemHandle = osSemaphoreCreate(osSemaphore(Turn_0_I90_Sem), 1);

  /* definition and creation of Turn_I90_To_0_Sem */
  osSemaphoreDef(Turn_I90_To_0_Sem);
  Turn_I90_To_0_SemHandle = osSemaphoreCreate(osSemaphore(Turn_I90_To_0_Sem), 1);

  /* definition and creation of Turn_0To_180_Sem */
  osSemaphoreDef(Turn_0To_180_Sem);
  Turn_0To_180_SemHandle = osSemaphoreCreate(osSemaphore(Turn_0To_180_Sem), 1);

  /* definition and creation of Shift_Final_Sem */
  osSemaphoreDef(Shift_Final_Sem);
  Shift_Final_SemHandle = osSemaphoreCreate(osSemaphore(Shift_Final_Sem), 1);

  /* definition and creation of Backward_Final_Sem */
  osSemaphoreDef(Backward_Final_Sem);
  Backward_Final_SemHandle = osSemaphoreCreate(osSemaphore(Backward_Final_Sem), 1);

  /* definition and creation of ServoSem */
  osSemaphoreDef(ServoSem);
  ServoSemHandle = osSemaphoreCreate(osSemaphore(ServoSem), 1);

  /* definition and creation of OpenMV_Sem */
  osSemaphoreDef(OpenMV_Sem);
  OpenMV_SemHandle = osSemaphoreCreate(osSemaphore(OpenMV_Sem), 1);

  /* definition and creation of openmvSem */
  osSemaphoreDef(openmvSem);
  openmvSemHandle = osSemaphoreCreate(osSemaphore(openmvSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	
////////////////	//这里不懂

	//清除信号量创建时自动释放的一次信号量		如果添加了新的信号量 		务必在此加上相应信号量
	osSemaphoreWait(GoIn_SemHandle, osWaitForever);
	osSemaphoreWait(Turn_0To90_SemHandle, osWaitForever);
	osSemaphoreWait(Turn_90To0_SemHandle, osWaitForever);
	osSemaphoreWait(Turn_0_I90_SemHandle, osWaitForever);
	osSemaphoreWait(Turn_I90_To_0_SemHandle, osWaitForever);
	osSemaphoreWait(Shift_Head_SemHandle, osWaitForever);
	osSemaphoreWait(Shift_Tail_SemHandle, osWaitForever);
	osSemaphoreWait(GoOut_SemHandle, osWaitForever);
	osSemaphoreWait(Forward_SemHandle, osWaitForever);
	osSemaphoreWait(Backward_SemHandle, osWaitForever);
	osSemaphoreWait(Step_Manager_SemHandle, osWaitForever);
	osSemaphoreWait(ISR_ReadHWT_SemHandle, osWaitForever);
	osSemaphoreWait(HCSRA_SemHandle, osWaitForever);
	osSemaphoreWait(HCSRB_SemHandle, osWaitForever);
	osSemaphoreWait(HCSRC_SemHandle, osWaitForever);
	osSemaphoreWait(HCSRD_SemHandle, osWaitForever);
	osSemaphoreWait(Turn_0To_180_SemHandle, osWaitForever);
	osSemaphoreWait(Shift_Final_SemHandle, osWaitForever);
	osSemaphoreWait(Backward_Final_SemHandle, osWaitForever);
	osSemaphoreWait(ServoSemHandle, osWaitForever);
	osSemaphoreWait(OpenMV_SemHandle, osWaitForever);
	osSemaphoreWait(openmvSemHandle, osWaitForever);
	//清除信号量创建时自动释放的一次信号量		如果添加了新的信号量 		务必在此加上相应信号量
	
	
	
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Run_Task */
  osThreadDef(Run_Task, RunTask, osPriorityNormal, 0, 128);
  Run_TaskHandle = osThreadCreate(osThread(Run_Task), NULL);

  /* definition and creation of Step_Manager */
  osThreadDef(Step_Manager, StepManager, osPriorityLow, 0, 128);
  Step_ManagerHandle = osThreadCreate(osThread(Step_Manager), NULL);

  /* definition and creation of Go_In */
  osThreadDef(Go_In, GoIn, osPriorityBelowNormal, 0, 128);
  Go_InHandle = osThreadCreate(osThread(Go_In), NULL);

  /* definition and creation of Turn_0To90 */
  osThreadDef(Turn_0To90, Turn0_To_90, osPriorityBelowNormal, 0, 128);
  Turn_0To90Handle = osThreadCreate(osThread(Turn_0To90), NULL);

  /* definition and creation of Turn_90To0 */
  osThreadDef(Turn_90To0, Turn90_To_0, osPriorityBelowNormal, 0, 128);
  Turn_90To0Handle = osThreadCreate(osThread(Turn_90To0), NULL);

  /* definition and creation of Forward */
  osThreadDef(Forward, Forward_, osPriorityBelowNormal, 0, 128);
  ForwardHandle = osThreadCreate(osThread(Forward), NULL);

  /* definition and creation of Backward */
  osThreadDef(Backward, Backward_, osPriorityBelowNormal, 0, 128);
  BackwardHandle = osThreadCreate(osThread(Backward), NULL);

  /* definition and creation of Shift_Head */
  osThreadDef(Shift_Head, Shift_Head_Task, osPriorityBelowNormal, 0, 128);
  Shift_HeadHandle = osThreadCreate(osThread(Shift_Head), NULL);

  /* definition and creation of Go_Out */
  osThreadDef(Go_Out, GoOut, osPriorityBelowNormal, 0, 128);
  Go_OutHandle = osThreadCreate(osThread(Go_Out), NULL);

  /* definition and creation of Read_HWT101 */
  osThreadDef(Read_HWT101, ReadHWT101, osPriorityHigh, 0, 128);
  Read_HWT101Handle = osThreadCreate(osThread(Read_HWT101), NULL);

  /* definition and creation of Start_ISR */
  osThreadDef(Start_ISR, StartISR, osPriorityRealtime, 0, 128);
  Start_ISRHandle = osThreadCreate(osThread(Start_ISR), NULL);

  /* definition and creation of HCSRA_Process */
  osThreadDef(HCSRA_Process, HCSRAProcess, osPriorityAboveNormal, 0, 128);
  HCSRA_ProcessHandle = osThreadCreate(osThread(HCSRA_Process), NULL);

  /* definition and creation of HCSRB_Process */
  osThreadDef(HCSRB_Process, HCSRBProcess, osPriorityAboveNormal, 0, 128);
  HCSRB_ProcessHandle = osThreadCreate(osThread(HCSRB_Process), NULL);

  /* definition and creation of Shift_Tail */
  osThreadDef(Shift_Tail, Shift_Tail_Task, osPriorityBelowNormal, 0, 128);
  Shift_TailHandle = osThreadCreate(osThread(Shift_Tail), NULL);

  /* definition and creation of Turn_Zero_To_I9 */
  osThreadDef(Turn_Zero_To_I9, Turn_Zero_To_I90_Task, osPriorityBelowNormal, 0, 128);
  Turn_Zero_To_I9Handle = osThreadCreate(osThread(Turn_Zero_To_I9), NULL);

  /* definition and creation of Turn_I90_To_Zer */
  osThreadDef(Turn_I90_To_Zer, Turn_I90_To_Zero, osPriorityBelowNormal, 0, 128);
  Turn_I90_To_ZerHandle = osThreadCreate(osThread(Turn_I90_To_Zer), NULL);

  /* definition and creation of Turn_0To_180 */
  osThreadDef(Turn_0To_180, Turn_0To_180_Task, osPriorityBelowNormal, 0, 128);
  Turn_0To_180Handle = osThreadCreate(osThread(Turn_0To_180), NULL);

  /* definition and creation of Shift_Final */
  osThreadDef(Shift_Final, Shift_Final_Task, osPriorityBelowNormal, 0, 128);
  Shift_FinalHandle = osThreadCreate(osThread(Shift_Final), NULL);

  /* definition and creation of Backward_Final */
  osThreadDef(Backward_Final, Backward_Final_Task, osPriorityBelowNormal, 0, 128);
  Backward_FinalHandle = osThreadCreate(osThread(Backward_Final), NULL);

  /* definition and creation of Servo */
  osThreadDef(Servo, Servo_Task, osPriorityBelowNormal, 0, 128);
  ServoHandle = osThreadCreate(osThread(Servo), NULL);

  /* definition and creation of OpenMV_Process */
  osThreadDef(OpenMV_Process, OpenMV_Process_Task, osPriorityRealtime, 0, 128);
  OpenMV_ProcessHandle = osThreadCreate(osThread(OpenMV_Process), NULL);

  /* definition and creation of openmv_walk */
  osThreadDef(openmv_walk, openmv_walk_task, osPriorityRealtime, 0, 128);
  openmv_walkHandle = osThreadCreate(osThread(openmv_walk), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_RunTask */
/**
  * @brief  Function implementing the Run_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RunTask */
void RunTask(void const * argument)
{
  /* USER CODE BEGIN RunTask */
	uint32_t wakeuptime;
//	uint8_t alongwall_end=0;
  /* Infinite loop */
  for(;;)
  {
		wakeuptime=osKernelSysTick();//定期获取时间
		
		Ultrasonic_Trig_ALL();
		Encoder=read_encoder();	
		
		Calculate_Position();
		Calculate_Displacement();
		
//		TIM12->CCR1=up_servo;
//		TIM12->CCR2=down_servo;
		
//		x=600;
//		z=PID_Position_float(angle, 0, &PID_Object[HWT101]);
//		
//			y=PID_Position(LASER_Data_Buff[Laser_One_Distance],10, &PID_Object[LAPID_A]);
//			y=PID_Position(HCSR_Object[HCSR_B].Output_Distance,10,&PID_Object[HCNEW_B]);
//		x=PID_Position(HCSR_Object[HCSR_A].Output_Distance,10,&PID_Object[HCNEW_A]);
//		x=PID_Position(HCSR_Object[HCSR_C].Output_Distance,10,&PID_Object[HCNEW_C]);
		
//		for(int i=0;i<16;i++)
//		{
//				if(x<=800)
//				{
//						x=y+50;
//				}
//		}
//		y=PID_Position(HCSR_Object[HCSR_B].Output_Distance,12,&PID_Object[HCNEW_B]);
//		
//		y=PID_Position(LASER_Data_Buff[Laser_Two_Distance],30, &PID_Object[LAPID_B]);
//		x=PID_Position(HCSR_Object[HCSR_A].Output_Distance,12, &PID_Object[HCNEW_A]);
		
//		z=PID_Position_float(angle, 0, &PID_Object[HWT101]);
		
//		y=PID_Position(LASER_Data_Buff[Laser_One_Distance],12,&PID_Object[LAPID_A]);

		Target=kinematic_analysis(x,y,z);
//	
		set_ccr
		(
				PID_Position(Encoder[0],Target[0], &PID_Object[Motor_A]),
				PID_Position(Encoder[1],Target[1], &PID_Object[Motor_B]),
				PID_Position(Encoder[2],Target[2], &PID_Object[Motor_C]),
				PID_Position(Encoder[3],Target[3], &PID_Object[Motor_D])
		);
		
		
		if(Servo_Flag==1)
		{
			x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,6,&PID_Object[HCNEW_C]);
			if(HCSR_Object[HCSR_C].Output_Distance==8)
			{
				Servo_Flag=0;
				osSemaphoreRelease(ServoSemHandle);
				
			}
		}
//		x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,15,&PID_Object[HCNEW_C]);
		
		
//		if(OpenMV_Process_Busy==1)
//		{
//			if(alongwall_end==0)
//			{
//				x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,15,&PID_Object[HCNEW_C]);
//			}
//			if(HCSR_Object[HCSR_C].Output_Distance==15)
//			{
//				x=0;
//				y=PID_Position(OpenMV_dis,-15,&PID_Object[OpenMV]);
//				alongwall_end=1;
//				if(OpenMV_dis<=3-15 && OpenMV_dis>=-3-15)
//				 {
//					 x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,8,&PID_Object[HCNEW_C]);
//					 y=0;
//			//		 z=PID_Position(angle,0,&PID_Object[HWT101]);
//					 if(HCSR_Object[HCSR_C].Output_Distance==8)
//					 {
//							x=0;
//						 alongwall_end=0;
//						 //此处应写释放舵机的信号量
//							osSemaphoreRelease(ServoSemHandle);
//					 }
//				 }
//			 }
//		 }
		
		 
		osDelayUntil(&wakeuptime,10);//固定10ms时间中断一次
		
  }
  /* USER CODE END RunTask */
}

/* USER CODE BEGIN Header_StepManager */
/**
* @brief Function implementing the Step_Manager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StepManager */
void StepManager(void const * argument)
{
  /* USER CODE BEGIN StepManager */
  /* Infinite loop */
  for(;;)
  {
		
		Step_Count++;//此变量具有计数的作用，用于判断车子执行了几个步骤，进而通过此变量来规划下一个步骤
		Step_Sequence_PlanB();
//		Step = Forward_Step;
    switch(Step)
		{
			case GoIn_Step: 			osSemaphoreRelease(GoIn_SemHandle); 			break;
			case Turn_90To0_Step: osSemaphoreRelease(Turn_90To0_SemHandle); break;
			case Turn_0To90_Step: osSemaphoreRelease(Turn_0To90_SemHandle); break;
			case Forward_Step: 		osSemaphoreRelease(Forward_SemHandle); 		break;
			case Backward_Step: 	osSemaphoreRelease(Backward_SemHandle); 	break;
			case Shift_Step_Head: 			osSemaphoreRelease(Shift_Head_SemHandle); 			break;
			case Shift_Step_Tail: 			osSemaphoreRelease(Shift_Tail_SemHandle); 			break;
			case GoOut_step: 			osSemaphoreRelease(GoOut_SemHandle); 			break;
			case Turn_0ToI90_Step: osSemaphoreRelease(Turn_0_I90_SemHandle);break;
			case Turn_I90To0_Step: osSemaphoreRelease(Turn_I90_To_0_SemHandle); break;
			case Turn_0To180_Step: osSemaphoreRelease(Turn_0To_180_SemHandle); break;
			case	Shift_Final_Step:osSemaphoreRelease(Shift_Final_SemHandle);break;
			case	Backward_Final_Step:osSemaphoreRelease(Backward_Final_SemHandle);break;
			default : break;
		}
		
		osSemaphoreWait(Step_Manager_SemHandle, osWaitForever);

  }
  /* USER CODE END StepManager */
}

/* USER CODE BEGIN Header_GoIn */
/**
* @brief Function implementing the Go_In thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GoIn */
void GoIn(void const * argument)
{
  /* USER CODE BEGIN GoIn */
  /* Infinite loop */
  for(;;)
  {
			
		if(Hold_On==0)
		{
			osSemaphoreWait(GoIn_SemHandle, osWaitForever);
			Clear_Displacement();
			Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
		}
		if(x<=800) x=x+5;//加速
		z=PID_Position_float(angle, 0, &PID_Object[HWT101]);
		

		osDelay(10);
		
		
		if(Displacement_X>=51)
		{
			x=0;
			y=0;
			Clear_Displacement();
			Hold_On=0;//退出任务
		  osSemaphoreRelease(Step_Manager_SemHandle);


		}
		
  }
  /* USER CODE END GoIn */
}

/* USER CODE BEGIN Header_Turn0_To_90 */
/**
* @brief Function implementing the Turn_0To90 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Turn0_To_90 */
void Turn0_To_90(void const * argument)
{
  /* USER CODE BEGIN Turn0_To_90 */
  /* Infinite loop */
  for(;;)
  {
    if(Hold_On==0)
		{
			osSemaphoreWait(Turn_0To90_SemHandle, osWaitForever);
			Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
		}
		z=PID_Position_float(angle, 90, &PID_Object[HWT101]);
//		osDelay(10);
		if( (angle>=90-Turn_Range) && (angle<=90+Turn_Range) )
		{
			x=0;
			y=0;
			z=0;
			Clear_Displacement();
			Hold_On=0;//退出任务

			osSemaphoreRelease(Step_Manager_SemHandle);
		}
		
  }
  /* USER CODE END Turn0_To_90 */
}

/* USER CODE BEGIN Header_Turn90_To_0 */
/**
* @brief Function implementing the Turn_90To0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Turn90_To_0 */
void Turn90_To_0(void const * argument)
{
  /* USER CODE BEGIN Turn90_To_0 */
  /* Infinite loop */
  for(;;)
  {
		if(Hold_On==0)
		{
			osSemaphoreWait(Turn_90To0_SemHandle, osWaitForever);
			Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
		}
		z=PID_Position_float(angle, 0, &PID_Object[HWT101]);
	
		if( (angle>=0-Turn_Range) && (angle<=0+Turn_Range) )
		{
			z=0;
			Clear_Displacement();
			Hold_On=0;//退出任务

			osSemaphoreRelease(Step_Manager_SemHandle);
		}
		
  }
 
  /* USER CODE END Turn90_To_0 */
}

/* USER CODE BEGIN Header_Forward_ */
/**
* @brief Function implementing the Forward thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Forward_ */
void Forward_(void const * argument)
{
  /* USER CODE BEGIN Forward_ */
  /* Infinite loop */
  for(;;)
  {
		
		
//		if(Hold_On==0)
//		{
//			osSemaphoreWait(Forward_SemHandle, osWaitForever);
//			Clear_Displacement();
//			Hold_On=1;   //接收到信号量之后，让任务处于保持运行状态	
//		}
//		y=600;
//		x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,15,&PID_Object[HCNEW_C]);
//		z=PID_Position_float(angle,0, &PID_Object[HWT101]);
		
		
		
		
    if(Hold_On==0)
		{
			osSemaphoreWait(Forward_SemHandle, osWaitForever);
			Clear_Displacement();
			Hold_On=1;   //接收到信号量之后，让任务处于保持运行状态	
		}
		
		if(Displacement_Y<30)
		{
				x=0;
				y=middle_speed;
				z=PID_Position_float(angle,0, &PID_Object[HWT101]);
		}
	
		
		if((Displacement_Y>=30)&&(Displacement_Y<=230))
		{
			 x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,11,&PID_Object[HCNEW_C]);	
//			 y=fast_speed;
				y=middle_speed;
			 z=PID_Position_float(angle,0, &PID_Object[HWT101]);
		}
		
		if(Displacement_Y>230)
		{
				x=0;
				y=PID_Position(HCSR_Object[HCSR_B].Output_Distance,9,&PID_Object[HCNEW_B]);
				z=PID_Position_float(angle,0, &PID_Object[HWT101]);
			
				if( (HCSR_Object[HCSR_B].Output_Distance<=9) && (HCSR_Object[HCSR_B].Output_Distance>=8) )
				{		
					x=0;
					y=PID_Position(HCSR_Object[HCSR_B].Output_Distance,9,&PID_Object[HCNEW_B]);
					z=0;
					Clear_Displacement();
					Hold_On=0;//退出任务
					osSemaphoreRelease(Step_Manager_SemHandle);
				}
		}
  }
  /* USER CODE END Forward_ */
}

/* USER CODE BEGIN Header_Backward_ */
/**
* @brief Function implementing the Backward thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Backward_ */
void Backward_(void const * argument)
{
  /* USER CODE BEGIN Backward_ */
  /* Infinite loop */
   for(;;)
  {
    if(Hold_On==0)
		{
			osSemaphoreWait(Backward_SemHandle, osWaitForever);
			Clear_Displacement();
			Hold_On=1;   //接收到信号量之后，让任务处于保持运行状态	
		}
		
		if(Displacement_Y>-30)
		{
				x=0;
				y=-middle_speed;
				z=PID_Position_float(angle,0, &PID_Object[HWT101]);
		}
	
		
	else	if((Displacement_Y<=-30)&&(Displacement_Y>=-230))
		{
			 x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,11,&PID_Object[HCNEW_C]);	
			 y=-middle_speed;
			 z=PID_Position_float(angle,0, &PID_Object[HWT101]);
		}
		

		if(Displacement_Y<-230)
		{
				x=0;
				y=PID_Position(LASER_Data_Buff[Laser_One_Distance],5,&PID_Object[LAPID_A]);
				z=PID_Position_float(angle,0, &PID_Object[HWT101]);
			
				if( (LASER_Data_Buff[Laser_One_Distance]<=10) && (LASER_Data_Buff[Laser_One_Distance]>=6) )
				{		
					x=0;
					y=PID_Position(LASER_Data_Buff[Laser_One_Distance],8,&PID_Object[LAPID_A]);
//(LASER_Data_Buff[Laser_Two_Distance]
					z=0;
					Clear_Displacement();
					Hold_On=0;//退出任务
					osSemaphoreRelease(Step_Manager_SemHandle);
				}
		}
  }
  /* USER CODE END Backward_ */
}

/* USER CODE BEGIN Header_Shift_Head_Task */
/**
* @brief Function implementing the Shift_Head thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shift_Head_Task */
void Shift_Head_Task(void const * argument)
{
  /* USER CODE BEGIN Shift_Head_Task */
  /* Infinite loop */
  for(;;)
  {
			if(Hold_On==0)
			{
					osSemaphoreWait(Shift_Head_SemHandle, osWaitForever);
					Clear_Displacement();
					Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
			}
			
			if(Displacement_Y>=-Shift_Dis)
			{
					x=PID_Position_float(HCSR_Object[HCSR_A].Output_Distance,7,&PID_Object[HCNEW_A]);
					y=-shift_speed;
					z=PID_Position_float(angle,90, &PID_Object[HWT101]);
			}
			
			else
			{
					x=0;
					y=0;
					z=PID_Position_float(angle,90, &PID_Object[HWT101]);
				
					Clear_Displacement();
					Hold_On=0;//退出任务
					osSemaphoreRelease(Step_Manager_SemHandle);
			}
  }
  /* USER CODE END Shift_Head_Task */
}

/* USER CODE BEGIN Header_GoOut */
/**
* @brief Function implementing the Go_Out thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GoOut */
void GoOut(void const * argument)
{
  /* USER CODE BEGIN GoOut */
  /* Infinite loop */
  for(;;)
  {
    if(Hold_On==0)
		{
			osSemaphoreWait(GoOut_SemHandle, osWaitForever);
			Clear_Displacement();
			Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
		}
		
		if(Displacement_X<=415)
		{
		x=1500;
		y=PID_Position(LASER_Data_Buff[Laser_One_Distance],7,&PID_Object[LAPID_A]);
		z=PID_Position_float( absolute(angle), 180, &PID_Object[NEWHWT180]);
		}
		
		
		else if(Displacement_X>445)
		{
			x=0;
			y=0;
			z=0;
			Clear_Displacement();
			Hold_On=0;//退出任务
			
//			Stop
//			Step = Turn_0To90_Step;
//			osSemaphoreRelease(Step_Manager_SemHandle);
		}
  }
  /* USER CODE END GoOut */
}

/* USER CODE BEGIN Header_ReadHWT101 */
/**
* @brief Function implementing the Read_HWT101 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadHWT101 */
void ReadHWT101(void const * argument)
{
  /* USER CODE BEGIN ReadHWT101 */
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(ISR_ReadHWT_SemHandle, osWaitForever);
    CopeSerial2Data(SingleData);
		HAL_UART_Receive_IT(&huart1, &SingleData, 1);
  }
  /* USER CODE END ReadHWT101 */
}

/* USER CODE BEGIN Header_StartISR */
/**
* @brief Function implementing the Start_ISR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartISR */
void StartISR(void const * argument)
{
  /* USER CODE BEGIN StartISR */
  /* Infinite loop */
  for(;;)
  {
			HAL_UART_Receive_IT(&huart1, &SingleData, 1);
			HAL_TIM_IC_Start_IT(&htim10,TIM_CHANNEL_1);
			HAL_TIM_IC_Start_IT(&htim11,TIM_CHANNEL_1);
			HAL_TIM_IC_Start_IT(&htim13,TIM_CHANNEL_1);
//		HAL_TIM_IC_Start_IT(&htim14,TIM_CHANNEL_1);
		
			OpenMV_Init();
			TIM12->CCR1 = 2050;//上舵机闭合
			TIM12->CCR2 = 1000;//下舵机闭

			LASER_1_Init();
			LASER_2_Init();
		
		
//			vTaskSuspend(Step_ManagerHandle);
//			vTaskSuspend(OpenMV_ProcessHandle);
			osThreadTerminate(Start_ISRHandle);//清除任务
  }
  /* USER CODE END StartISR */
}

/* USER CODE BEGIN Header_HCSRAProcess */
/**
* @brief Function implementing the HCSRA_Process thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HCSRAProcess */
void HCSRAProcess(void const * argument)
{
  /* USER CODE BEGIN HCSRAProcess */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(HCSRA_SemHandle, osWaitForever);
		HCSR_Process(&HCSR_Object[HCSR_A],TIM10,htim10);
  }
  /* USER CODE END HCSRAProcess */
}

/* USER CODE BEGIN Header_HCSRBProcess */
/**
* @brief Function implementing the HCSRB_Process thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HCSRBProcess */
void HCSRBProcess(void const * argument)
{
  /* USER CODE BEGIN HCSRBProcess */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(HCSRB_SemHandle, osWaitForever);
		HCSR_Process(&HCSR_Object[HCSR_B],TIM11,htim11);
  }
  /* USER CODE END HCSRBProcess */
}

/* USER CODE BEGIN Header_Shift_Tail_Task */
/**
* @brief Function implementing the Shift_Tail thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shift_Tail_Task */
void Shift_Tail_Task(void const * argument)
{
  /* USER CODE BEGIN Shift_Tail_Task */
  /* Infinite loop */
  for(;;)
  {
    if(Hold_On==0)
			{
					osSemaphoreWait(Shift_Tail_SemHandle, osWaitForever);
					Clear_Displacement();
					Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
			}
			
			if(Displacement_Y<=Shift_Dis)
			{
					x=PID_Position_float(HCSR_Object[HCSR_A].Output_Distance,9,&PID_Object[HCNEW_A]);
					y=shift_speed;
					z=PID_Position_float(angle,-90, &PID_Object[HWT101]);
			}
			
			else
			{
					x=0;
					y=0;
					z=PID_Position_float(angle,-90, &PID_Object[HWT101]);
				
					Clear_Displacement();
					Hold_On=0;//退出任务
					osSemaphoreRelease(Step_Manager_SemHandle);
			}
  }
  /* USER CODE END Shift_Tail_Task */
}

/* USER CODE BEGIN Header_Turn_Zero_To_I90_Task */
/**
* @brief Function implementing the Turn_Zero_To_I9 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Turn_Zero_To_I90_Task */
void Turn_Zero_To_I90_Task(void const * argument)
{
  /* USER CODE BEGIN Turn_Zero_To_I90_Task */
  /* Infinite loop */
  for(;;)
  {
    if(Hold_On==0)
		{
			osSemaphoreWait(Turn_0_I90_SemHandle, osWaitForever);
			Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
		}
		z=PID_Position_float(angle, -90, &PID_Object[HWT101]);
	
		if( (angle>=-90-Turn_Range) && (angle<=-90+Turn_Range) )
		{
			z=0;
			Clear_Displacement();
			Hold_On=0;//退出任务

			osSemaphoreRelease(Step_Manager_SemHandle);
		}
  }
  /* USER CODE END Turn_Zero_To_I90_Task */
}

/* USER CODE BEGIN Header_Turn_I90_To_Zero */
/**
* @brief Function implementing the Turn_I90_To_Zer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Turn_I90_To_Zero */
void Turn_I90_To_Zero(void const * argument)
{
  /* USER CODE BEGIN Turn_I90_To_Zero */
  /* Infinite loop */
  for(;;)
  {
    if(Hold_On==0)
		{
			osSemaphoreWait(Turn_I90_To_0_SemHandle, osWaitForever);
			Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
		}
		z=PID_Position_float(angle, 0, &PID_Object[HWT101]);
	
		if( (angle>=0-Turn_Range) && (angle<=0+Turn_Range) )
		{
			x=0;
			z=0;
			y=0;
			Clear_Displacement();
			Hold_On=0;//退出任务
			osSemaphoreRelease(Step_Manager_SemHandle);
		}
  }
  /* USER CODE END Turn_I90_To_Zero */
}

/* USER CODE BEGIN Header_Turn_0To_180_Task */
/**
* @brief Function implementing the Turn_0To_180 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Turn_0To_180_Task */
void Turn_0To_180_Task(void const * argument)
{
  /* USER CODE BEGIN Turn_0To_180_Task */
  /* Infinite loop */
	
  for(;;)
  {
   if(Hold_On==0)
		{
			osSemaphoreWait(Turn_0To_180_SemHandle, osWaitForever);
			Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
		}
			z=PID_Position_float( absolute(angle), 180, &PID_Object[NEWHWT180]);
			if( ( absolute(angle) >= 180-Turn_Range ) && ( absolute(angle) <= 180+Turn_Range ) )
			{
				x=0;
				y=0;
				Clear_Displacement();
				Hold_On=0;//退出任务
				osSemaphoreRelease(Step_Manager_SemHandle);
			}
	}
  
  /* USER CODE END Turn_0To_180_Task */
}

/* USER CODE BEGIN Header_Shift_Final_Task */
/**
* @brief Function implementing the Shift_Final thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shift_Final_Task */
void Shift_Final_Task(void const * argument)
{
  /* USER CODE BEGIN Shift_Final_Task */
	int	flag=0;
  /* Infinite loop */
  for(;;)
  {
    if(Hold_On==0)
			{
					osSemaphoreWait(Shift_Final_SemHandle, osWaitForever);
					Clear_Displacement();
					Hold_On=1;//接收到信号量之后，让任务处于保持运行状态
			}
			
			y=PID_Position_float(HCSR_Object[HCSR_B].Output_Distance,6,&PID_Object[HCNEW_B]);
			if((HCSR_Object[HCSR_B].Output_Distance>=6)&&(HCSR_Object[HCSR_B].Output_Distance<=7))
			{
					flag=1;
					Clear_Displacement();
			}
			
			if((Displacement_X>=-60)&&(flag==1))
			{
					y=PID_Position_float(HCSR_Object[HCSR_B].Output_Distance,7,&PID_Object[HCNEW_B]);
					x=-shift_speed;
					z=PID_Position_float( absolute(angle), 180, &PID_Object[NEWHWT180]);
			}
			
			else if((Displacement_X<-Shift_Dis)&&(flag==1))
			{
					x=0;
					y=0;
					z=PID_Position_float( absolute(angle), 180, &PID_Object[NEWHWT180]);
				
					Clear_Displacement();
					flag=0;
					Hold_On=0;//退出任务

					osSemaphoreRelease(Step_Manager_SemHandle);
			}
  }
  /* USER CODE END Shift_Final_Task */
}

/* USER CODE BEGIN Header_Backward_Final_Task */
/**
* @brief Function implementing the Backward_Final thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Backward_Final_Task */
void Backward_Final_Task(void const * argument)
{
  /* USER CODE BEGIN Backward_Final_Task */
  /* Infinite loop */
  for(;;)
  {
     if(Hold_On==0)
		{
			osSemaphoreWait(Backward_Final_SemHandle, osWaitForever);
			Clear_Displacement();
			Hold_On=1;   //接收到信号量之后，让任务处于保持运行状态	
		}
	
//		if(Displacement_Y>-30)
//		{
//				x=0;
//				y=-middle_speed;
//				z=PID_Position_float( absolute(angle), 180, &PID_Object[NEWHWT180]);
//		}
	
		
	if(Displacement_Y>-230)
		{
			 x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,11,&PID_Object[HCNEW_C]);
			 y=-fast_speed;
				z=PID_Position_float( absolute(angle), 180, &PID_Object[NEWHWT180]);
		}
		
		if(Displacement_Y<-230)
		{
				x=0;
				y=PID_Position(LASER_Data_Buff[Laser_One_Distance],7,&PID_Object[LAPID_A]);
				z=PID_Position_float( absolute(angle), 180, &PID_Object[NEWHWT180]);
			
				if( (LASER_Data_Buff[Laser_One_Distance]<=8) && (LASER_Data_Buff[Laser_One_Distance]>=6) )
				{		
					x=0;
					y=PID_Position(LASER_Data_Buff[Laser_One_Distance],6,&PID_Object[LAPID_A]);
					z=0;
					Clear_Displacement();
					Hold_On=0;//退出任务
					osSemaphoreRelease(Step_Manager_SemHandle);
				}
		}
  }
  /* USER CODE END Backward_Final_Task */
}

/* USER CODE BEGIN Header_Servo_Task */
/**
* @brief Function implementing the Servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Task */
void Servo_Task(void const * argument)
{
  /* USER CODE BEGIN Servo_Task */
  /* Infinite loop */
  for(;;)
  {
			osSemaphoreWait(ServoSemHandle, osWaitForever);
			
//			x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,8,&PID_Object[HCNEW_C]);
//			if(HCSR_Object[HCSR_C].Output_Distance==8)
//			{
//				x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,8,&PID_Object[HCNEW_C]);
			TIM12->CCR1 = 2050;//上舵机闭合
			TIM12->CCR2 = 600;//下舵机张开
			osDelay(400);
			TIM12->CCR2 = 1000;//下舵机闭
		
			osSemaphoreRelease(openmvSemHandle);
//		if(Step_Count==2)
//		{
//			y=-300;
//		}
//		if(Step==Forward_Step)
//		{
//			y=300;
//		}
//			osSemaphoreRelease(openmvSemHandle);
			

			//等到直走到openmv看不到的地方，恢复所有任务及openmv任务
			vTaskResume(OpenMV_ProcessHandle);

			TIM12->CCR1 = 2400;//上舵机张开			
			osDelay(1000);
			TIM12->CCR1 = 2050;//上舵机闭合
			
//			osDelay(3000);
			//OpenMV_Process_Busy置0
			OpenMV_Process_Busy=0;
				
//			}
		
//			TIM12->CCR1 = 2000;//上舵机闭合
//			TIM12->CCR2 = 600;//下舵机张开
//			osDelay(1000);
//			TIM12->CCR2 = 900;//下舵机闭合
			
			//恢复之前挂起的任务
			vTaskResume(Step_ManagerHandle);
			vTaskResume(Go_InHandle);
			vTaskResume(Turn_0To90Handle);
			vTaskResume(Turn_90To0Handle);

			vTaskResume(ForwardHandle);
			
			vTaskResume(BackwardHandle);
			vTaskResume(Shift_HeadHandle);
			vTaskResume(Go_OutHandle);
			vTaskResume(Shift_TailHandle);
			vTaskResume(Turn_Zero_To_I9Handle);
			vTaskResume(Turn_I90_To_ZerHandle);
			vTaskResume(Turn_0To_180Handle);
			vTaskResume(Shift_FinalHandle);
			vTaskResume(Backward_FinalHandle);
			vTaskResume(Step_ManagerHandle);
			
//			osDelay(1000);
//			TIM12->CCR1 = 2400;//上舵机张开
//			osDelay(1000);
//			TIM12->CCR1 = 2000;//上舵机闭合
//			
//			//OpenMV_Process_Busy置0
//			OpenMV_Process_Busy=0;
		
  }
  /* USER CODE END Servo_Task */
}

/* USER CODE BEGIN Header_OpenMV_Process_Task */
/**
* @brief Function implementing the OpenMV_Process thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OpenMV_Process_Task */
void OpenMV_Process_Task(void const * argument)
{
  /* USER CODE BEGIN OpenMV_Process_Task */
  /* Infinite loop */
	static uint8_t zero_count=0;
  for(;;)
  {
		osSemaphoreWait(OpenMV_SemHandle, osWaitForever);
		
		if(cnt==5)//OpenMV一次发过来的字节数是5
		{
			if(USART2_Rx_Buff[0]==0x55 && USART2_Rx_Buff[3]==0x0d && USART2_Rx_Buff[4]==0x0a)//检查包头&包尾是否正确
			{
				if(OpenMV_Process_Busy==0 && USART2_Rx_Buff[1]==1)
				{
					x=0;
					y=0;
					z=0;
					
					vTaskSuspend(Step_ManagerHandle);
					vTaskSuspend(Go_InHandle);
					vTaskSuspend(Turn_0To90Handle);
					vTaskSuspend(Turn_90To0Handle);
					vTaskSuspend(ForwardHandle);
					vTaskSuspend(BackwardHandle);
					vTaskSuspend(Shift_HeadHandle);
					vTaskSuspend(Go_OutHandle);
					vTaskSuspend(Shift_TailHandle);
					vTaskSuspend(Turn_Zero_To_I9Handle);
					vTaskSuspend(Turn_I90_To_ZerHandle);
					vTaskSuspend(Turn_0To_180Handle);
					vTaskSuspend(Shift_FinalHandle);
					vTaskSuspend(Backward_FinalHandle);

					zero_count=0;
					OpenMV_Process_Busy=1;
				}
				
//				if(OpenMV_Process_Busy==1 && USART2_Rx_Buff[1]==0 )
//				{
//					zero_count++;
//					if(zero_count>=10)
//					{
//						x=0;
//						y=0;
//						z=0;
//						
//						vTaskResume(Step_ManagerHandle);
//						vTaskResume(Go_InHandle);
//						vTaskResume(Turn_0To90Handle);
//						vTaskResume(Turn_90To0Handle);
//						vTaskResume(ForwardHandle);
//						vTaskResume(BackwardHandle);
//						vTaskResume(Shift_HeadHandle);
//						vTaskResume(Go_OutHandle);
//						vTaskResume(Shift_TailHandle);
//						vTaskResume(Turn_Zero_To_I9Handle);
//						vTaskResume(Turn_I90_To_ZerHandle);
//						vTaskResume(Turn_0To_180Handle);
//						vTaskResume(Shift_FinalHandle);
//						vTaskResume(Backward_FinalHandle);
//						zero_count=0;
//						OpenMV_Process_Busy=0;
//					}
//				}
				
				if(USART2_Rx_Buff[2]<176)
				{
					OpenMV_dis=USART2_Rx_Buff[2];
				}
				else
				{
					OpenMV_dis=USART2_Rx_Buff[2]-256;//负数处理
				}
				
				if(OpenMV_Process_Busy==1)
				{
					x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,11,&PID_Object[HCNEW_C]);
					y=PID_Position(OpenMV_dis,-15,&PID_Object[OpenMV]);
				}
//				z=PID_Position(angle,0,&PID_Object[HWT101]);
			}
		}
   if(OpenMV_dis<=3-15 && OpenMV_dis>=-3-15 && OpenMV_Process_Busy==1 )
	 {
		 x=0;
		 y=0;
////		 z=PID_Position(angle,0,&PID_Object[HWT101]);
//		 if(HCSR_Object[HCSR_C].Output_Distance==8)
//		 {
//				x=0;
		 Servo_Flag=1;
//		 osSemaphoreRelease(ServoSemHandle);
		 vTaskSuspend(OpenMV_ProcessHandle);
			 //此处应写释放舵机的信号量
			
		
		 }
//		 
//		 
//		 //当下舵机放苗完成之后，上舵机准备预装载下一个苗的时候，把小车行走的任务全部恢复，并且舵机动作全部完成之后OpenMV_Process_Busy置0等待下一次接收
//	 }
	 //将接收缓冲区数据清零
		memset(USART2_Rx_Buff,0,USART2_BUFF_SIZE);
		//再次使能DMA接收
		HAL_UART_Receive_DMA(&huart2,USART2_Rx_Buff,USART2_BUFF_SIZE);
//	 osDelay(10);
  }
  /* USER CODE END OpenMV_Process_Task */
}

/* USER CODE BEGIN Header_openmv_walk_task */
/**
* @brief Function implementing the openmv_walk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_openmv_walk_task */
void openmv_walk_task(void const * argument)
{
  /* USER CODE BEGIN openmv_walk_task */
  /* Infinite loop */
	static int delay_count;
	static int keep_walk=0;
  for(;;)
  {
		
		if(keep_walk==0)
		{
			osSemaphoreWait(openmvSemHandle, osWaitForever);
			keep_walk=1;
		}
		if(Step==Backward_Step)
		{
			y=-300;
		}
		if(Step==Forward_Step)
		{
			y=300;
		}
		x=PID_Position_float(HCSR_Object[HCSR_C].Output_Distance,11,&PID_Object[HCNEW_C]);
		z=PID_Position_float(angle,0, &PID_Object[HWT101]);
		vTaskSuspend(ServoHandle);
		osDelay(10);
		delay_count++;
		if(delay_count>=150)
		{
			keep_walk=0;
			vTaskResume(ServoHandle);
			x=0;
			y=0;z=0;
		}
  }
	
		
  /* USER CODE END openmv_walk_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
//{
//    printf("任务：%s堆栈溢出\r\n", pcTaskName);
//}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
