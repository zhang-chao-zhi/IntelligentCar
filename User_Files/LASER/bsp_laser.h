#ifndef _BSP_LASER_H
#define _BSP_LASER_H

#include "main.h"
#include "usart.h"
#include	"stdio.h"
#include	"string.h"

#define LASER_DATA_LEN  9
#define LASER_DATA_HEAD 0x59
#define USART_BUFF_SIZE 64


enum  LASER_DATA
{
		Laser_One_Distance,
		Laser_One_Strength,
		Laser_Two_Distance,
		Laser_Two_Strength
};
void LASER_1_Init(void);
void LASER_2_Init(void);
void USART3_IRQHandler(void);
void USART6_IRQHandler(void);
void LASER1_RX_Calc(uint8_t *rx_buff,uint16_t len);
void LASER2_RX_Calc(uint8_t *rx_buff,uint16_t len);

extern uint16_t  			LASER_Data_Buff[4];
#endif


