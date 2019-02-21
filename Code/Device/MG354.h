#ifndef MG354_H_201811121722
#define MG354_H_201811121722
/**
 * @file MG354.c
 * @author liun@xlnav.com
 * @date 2018-11-12
 * @brief EPSON MG354 Çý¶¯³ÌÐò
 * Copyright (c) 2018 XiLang Co. Ltd.
 * All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Global.h"
// Window ID 0
#define BURST 0x00 //Burst mode
#define MODE_CTRL_H 0x03//Operation mode control
#define MODE_CTRL_L 0x02
#define DIAG_STAT 0x04//Diagnostic result
#define FLAG 0x06//ND flag/.EA flag
#define GPIO_H 0x09//GPIO
#define GPIO_L 0x08
#define COUNT 0x0A//Sampling count value
#define TEMP_HIGH 0x0E//Temperature sensor value High
#define TEMP_LOW  0x10//Temperature sensorvalue Low
#define XGYRO_HIGH 0x12//X gyroscope sensor value High
#define XGYRO_LOW  0x14//X gyroscope sensor value Low
#define YGYRO_HIGH 0x16//Y gyroscope sensor value High
#define YGYRO_LOW  0x18//Y gyroscope sensor value Low
#define ZGYRO_HIGH 0x1A//Z gyroscope sensor value High
#define ZGYRO_LOW  0x1C//Z gyroscope sensor value Low
#define XACCL_HIGH 0x1E//X acceleration sensor value High
#define XACCL_LOW  0x20//X acceleration sensor value Low
#define YACCL_HIGH 0x22//Y acceleration sensor value High
#define YACCL_LOW  0x24//Y acceleration sensor value High
#define ZACCL_HIGH 0x26//X acceleration sensor value High
#define ZACCL_LOW  0x28//X acceleration sensor value High
//Window ID 1
#define SIG_CTRL_H 0x01//DataReady signal & polarity control
#define SIG_CTRL_L 0x00
#define MSC_CTRL_H 0x03//Other control
#define MSC_CTRL_L 0x02
#define SMPL_CTRL_H 0x05//Sampling control
#define SMPL_CTRL_L 0x04
#define FILTER_CTRL_H 0x07//Filter control
#define FILTER_CTRL_L 0x06
#define UART_CTRL_H 0x09//UART control
#define UART_CTRL_L 0x08
#define GLOB_CMD_H 0x0B//System control
#define GLOB_CMD_L 0x0A
#define BURST_CTRL1_H 0x0D//Burst control 1
#define BURST_CTRL1_L 0x0C
#define BURST_CTRL2_H 0x0F//Burst control 2
#define BURST_CTRL2_L 0x0E
#define POL_CTRL_H 0x11//Polarity control
#define POL_CTRL_L 0x10
#define PROD_ID1 0x6A//Product ID
#define PROD_ID2 0x6C
#define PROD_ID3 0x6E
#define PROD_ID4 0x70
#define VERSION 0x72//Version
#define SERIAL_NUM1 0x74//Serial Number
#define SERIAL_NUM2 0x76
#define SERIAL_NUM3 0x78
#define SERIAL_NUM4 0x7A
//Window ID 0/1
#define WIN_CTRL_H 0x7F//Register window control
#define WIN_CTRL_L 0x7E


/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/ 
typedef struct
{
	uint16_t u16Buffer[20];
	//int16_t s16Buffer[18];
	float fGyrX;
	float fGyrY;
	float fGyrZ;
	float fAccX;
	float fAccY;
	float fAccZ;
	float fTempretrue;
	unsigned char ucFrameNumber;
	uint32_t nProductID;	
	
} MG354_HandleTypeDef;
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void MG354_Init();
void Read_IMU_Start();
void ParaseIMU(MG354_HandleTypeDef *pBuf);
#endif 