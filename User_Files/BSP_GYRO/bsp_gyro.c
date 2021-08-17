#include "bsp_gyro.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

static uint8_t USART1_Rx_Buff[USART_BUFF_SIZE]={0};
short    WT101_DATA_BUFF[1]={0};
float       WT101_Angle=0;

//开启串口1DMA接收和空闲中断
void   GYRO_Data_Init(void)
{
			__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
			HAL_UART_Receive_DMA(&huart1,USART1_Rx_Buff,USART_BUFF_SIZE);
}

/**
  * @brief This function handles USART1 global interrupt.
  */
//void USART1_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART1_IRQn 0 */
//	uint16_t cnt=0;
//	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)
//	{
//			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//			HAL_UART_DMAStop(&huart1);
//			cnt=USART_BUFF_SIZE-hdma_usart1_rx.Instance->NDTR;
//		
//			GYRO_RX_Calc(USART1_Rx_Buff,cnt);
//		
//			memset(USART1_Rx_Buff,0,USART_BUFF_SIZE);
//		
//			HAL_UART_Receive_DMA(&huart1,USART1_Rx_Buff,USART_BUFF_SIZE);
//			
//	}
//  /* USER CODE END USART1_IRQn 0 */
//  HAL_UART_IRQHandler(&huart1);
//  /* USER CODE BEGIN USART1_IRQn 1 */

//  /* USER CODE END USART1_IRQn 1 */
//}

void GYRO_RX_Calc(uint8_t *rx_buff,uint16_t len)
{

		uint8_t 	 i=0;
		uint8_t	 checksum_gyro=0;
		uint8_t 	 checksum_angle=0;
		if(len==WT101_DATA_LEN)
		{
				if((rx_buff[0]==WT101_DATA_HEAD)&&(rx_buff[11]==WT101_DATA_HEAD)&&(rx_buff[1]==0x52)&&(rx_buff[12]==0x53))
				{
						for(i=0;i<10;i++)
						{
								checksum_gyro+=rx_buff[i];
								checksum_angle+=rx_buff[i+11];
						}
						if(checksum_gyro==rx_buff[10]&&checksum_angle==rx_buff[21])
						{
								WT101_DATA_BUFF[0]=(short)(((short)rx_buff[18]<<8)|rx_buff[17]);
								WT101_Angle=(float)WT101_DATA_BUFF[0]/32768*180;
						}
				}
		}
}



