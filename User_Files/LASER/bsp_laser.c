#include "bsp_laser.h"



extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

 uint8_t  USART3_Rx_Buff[USART_BUFF_SIZE]={0};
static uint8_t  USART6_Rx_Buff[USART_BUFF_SIZE]={0};
uint16_t  			LASER_Data_Buff[4]={0};

void LASER1_RX_Calc(uint8_t *rx_buff,uint16_t len);
void LASER2_RX_Calc(uint8_t *rx_buff,uint16_t len);


//��������3�ʹ���6��һ��DMA����
void LASER_1_Init(void)
{
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart3,USART3_Rx_Buff,USART_BUFF_SIZE);
}

void LASER_2_Init(void)
{
		__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart6,USART6_Rx_Buff,USART_BUFF_SIZE);
}


/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	
	uint16_t cnt=0;
	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET)
	{
			//��������жϱ�־λ
			__HAL_UART_CLEAR_IDLEFLAG(&huart3);
			//�ر�DMA����
			HAL_UART_DMAStop(&huart3);
			//����˴ν��յ�������
			cnt=USART_BUFF_SIZE-hdma_usart3_rx.Instance->NDTR;
			//���ݽ��մ�����
			LASER1_RX_Calc(USART3_Rx_Buff,cnt);
			//�����ջ�������������
			memset(USART3_Rx_Buff,0,USART_BUFF_SIZE);
			//�ٴ�ʹ��DMA����
			HAL_UART_Receive_DMA(&huart3,USART3_Rx_Buff,USART_BUFF_SIZE);
	}
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}


/**
  * @brief This function handles USART3 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	
	uint16_t cnt=0;
	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)
	{
			//��������жϱ�־λ
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);
			//�ر�DMA����
			HAL_UART_DMAStop(&huart6);
			//����˴ν��յ�������
			cnt=USART_BUFF_SIZE-hdma_usart6_rx.Instance->NDTR;
			//���ݽ��մ�����
			LASER2_RX_Calc(USART6_Rx_Buff,cnt);
			//�����ջ�������������
			memset(USART6_Rx_Buff,0,USART_BUFF_SIZE);
			//�ٴ�ʹ��DMA����
			HAL_UART_Receive_DMA(&huart6,USART6_Rx_Buff,USART_BUFF_SIZE);
	}
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}


void LASER1_RX_Calc(uint8_t *rx_buff,uint16_t len)
{
			uint8_t 	i=0;
			uint8_t 	checksum=0;
			if(len==LASER_DATA_LEN)
			{
					if((rx_buff[0]==LASER_DATA_HEAD)&&(rx_buff[1]==LASER_DATA_HEAD))
					{
							for(i=0;i<len-1;i++)
							{
										checksum+=rx_buff[i];
							}
							if(checksum==rx_buff[LASER_DATA_LEN-1])
							{
									
									LASER_Data_Buff[Laser_One_Strength]=rx_buff[4]|rx_buff[5]<<8;
									if((LASER_Data_Buff[Laser_One_Strength]>100)&&(LASER_Data_Buff[Laser_One_Strength]!=65535))
									{
											LASER_Data_Buff[Laser_One_Distance]=rx_buff[2]|rx_buff[3]<<8;
											LASER_Data_Buff[Laser_One_Distance]=LASER_Data_Buff[Laser_One_Distance];
									}
									else
									{
											LASER_Data_Buff[Laser_One_Distance]=0;
									}			
							}
					}
			}
}

void LASER2_RX_Calc(uint8_t *rx_buff,uint16_t len)
{
			uint8_t 	i=0;
			uint8_t 	checksum=0;
			if(len==LASER_DATA_LEN)
			{
				if((rx_buff[0]==LASER_DATA_HEAD)&&(rx_buff[1]==LASER_DATA_HEAD))
				{
					for(i=0;i<len-1;i++)
					{
								checksum+=rx_buff[i];
					}
					if(checksum==rx_buff[LASER_DATA_LEN-1])
					{
							LASER_Data_Buff[Laser_Two_Strength]=rx_buff[4]|rx_buff[5]<<8;
							if((LASER_Data_Buff[Laser_Two_Strength]>100)&&(LASER_Data_Buff[Laser_Two_Strength]!=65535))
							{
									LASER_Data_Buff[Laser_Two_Distance]=rx_buff[2]|rx_buff[3]<<8;
									LASER_Data_Buff[Laser_Two_Distance]=LASER_Data_Buff[Laser_Two_Distance];
							}
							else
									LASER_Data_Buff[Laser_Two_Distance]=0;
					}
				}
			}
}


