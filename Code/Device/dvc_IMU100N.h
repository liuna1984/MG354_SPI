/**
 * @file dvc_ADI16495.c
 * @author Qiaoyy@xlnav.com
 * @date 2018-04-27
 * @brief  ADI 16495 驱动程序
 * Copyright (c) 2018 XiLang Co. Ltd.
 * All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Global.h"
/* Exported macro ------------------------------------------------------------*/
#define ADI_USE_SPI

// 16477 寄存器表
//  Reserved  0x00 to 0x01 // Reserved N/A 
#define  IMU100N_PAGE_ID    0x00//03、04
//Page ID 00
#define  IMU100N_TEMP_OUT    0x0E00 // Output, temperature R 
#define  IMU100N_X_GYRO_LOW  0x1000 // Output, x-axis gyroscope, low word R 
#define  IMU100N_X_GYRO_OUT  0x1200 // Output, x-axis gyroscope, high word R 
#define  IMU100N_Y_GYRO_LOW  0x1400 // Output, y-axis gyroscope, low word R 
#define  IMU100N_Y_GYRO_OUT  0x1600 // Output, y-axis gyroscope, high word R 
#define  IMU100N_Z_GYRO_LOW  0x1800 // Output, z-axis gyroscope, low word R 
#define  IMU100N_Z_GYRO_OUT  0x1A00 // Output, z-axis gyroscope, high word R 
#define  IMU100N_X_ACCL_LOW  0x1C00 // Output, x-axis accelerometer, low word R 
#define  IMU100N_X_ACCL_OUT  0x1E00 // Output, x-axis accelerometer, high word R 
#define  IMU100N_Y_ACCL_LOW  0x2000 // Output, y-axis accelerometer, low word R 
#define  IMU100N_Y_ACCL_OUT  0x2200 // Output, y-axis accelerometer, high word R 
#define  IMU100N_Z_ACCL_LOW  0x2400 // Output, z-axis accelerometer, low word R 
#define  IMU100N_Z_ACCL_OUT  0x2600 // Output, z-axis accelerometer, high word R 


//Page ID 03
#define IMU100N_FNCTIO_CTRL 0x0600//控制IO
#define IMU100N_GPIO_CTRL   0x0800//控制IO
//Page ID 04
#define SERIAL_NUM          0x2000 //序列号

/* Exported types ------------------------------------------------------------*/ 
typedef struct
{
	uint16_t usBuffer[20];
	//int16_t usBuffer[14];
	float fGyrX;
	float fGyrY;
	float fGyrZ;
	float fAccX;
	float fAccY;
	float fAccZ;
	int16_t fTempretrue;
	unsigned char ucFrameNumber;
	uint16_t nProductID;
#ifdef ADI_USE_SPI
	SPI_HandleTypeDef *phspi;
	GPIO_TypeDef* NSS_GPIOx;
	uint16_t NSS_GPIO_Pin;
#endif	
	

} IMU100N_HandleTypeDef;
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int IMU100N_ReadImu(IMU100N_HandleTypeDef* pImu);
int IMU100N_ReadDeviceNumber(IMU100N_HandleTypeDef* pImu);
int IMU100N_ParseImu(IMU100N_HandleTypeDef* pImu);
int IMU100N_Readreg(IMU100N_HandleTypeDef* pImu);


/************************ (C) COPYRIGHT XiLang CO. Ltd. *******END OF FILE*****/
