#ifndef _BSP_GYRO_H
#define _BSP_GYRO_H

#include "main.h"
#include "usart.h"
#include "bsp_laser.h"

#define  WT101_DATA_LEN  22
#define  WT101_DATA_HEAD 0x55

extern short    WT101_DATA_BUFF[1];
extern float       WT101_Angle;

void GYRO_RX_Calc(uint8_t *rx_buff,uint16_t len);
void   GYRO_Data_Init(void);
#endif

