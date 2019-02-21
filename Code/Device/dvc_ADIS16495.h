#ifndef DVC_ADI16495_H_201804271518
#define DVC_ADI16495_H_201804271518
/**
 * @file dvc_ADI16495.c
 * @author Qiaoyy@xlnav.com
 * @date 2018-04-27
 * @brief  ADI 16495 Çý¶¯³ÌÐò
 * Copyright (c) 2018 XiLang Co. Ltd.
 * All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Global.h"
/* Exported macro ------------------------------------------------------------*/
#define ADI_USE_SPI

// 16477 ¼Ä´æÆ÷±í
//  Reserved  0x00 to 0x01 // Reserved N/A 
#define  ADI_16495_PAGE_ID0    0x0000
#define  ADI_16495_DIAG_STAT  0x0A0B // Output, system error flags R 
#define  ADI_16495_X_GYRO_LOW  0x1011 // Output, x-axis gyroscope, low word R 
#define  ADI_16495_X_GYRO_OUT  0x1213 // Output, x-axis gyroscope, high word R 
#define  ADI_16495_Y_GYRO_LOW  0x1415 // Output, y-axis gyroscope, low word R 
#define  ADI_16495_Y_GYRO_OUT  0x1617 // Output, y-axis gyroscope, high word R 
#define  ADI_16495_Z_GYRO_LOW  0x1819 // Output, z-axis gyroscope, low word R 
#define  ADI_16495_Z_GYRO_OUT  0x1A1B // Output, z-axis gyroscope, high word R 
#define  ADI_16495_X_ACCL_LOW  0x1C1D // Output, x-axis accelerometer, low word R 
#define  ADI_16495_X_ACCL_OUT  0x1E1F // Output, x-axis accelerometer, high word R 
#define  ADI_16495_Y_ACCL_LOW  0x2021 // Output, y-axis accelerometer, low word R 
#define  ADI_16495_Y_ACCL_OUT  0x2223 // Output, y-axis accelerometer, high word R 
#define  ADI_16495_Z_ACCL_LOW  0x2425 // Output, z-axis accelerometer, low word R 
#define  ADI_16495_Z_ACCL_OUT  0x2627 // Output, z-axis accelerometer, high word R 
#define  ADI_16495_TEMP_OUT    0x0E0F // Output, temperature R 
#define  ADI_16495_TIME_STAMP  0x2829 // Output, time stamp R 

#define  ADI_16495_DATA_CNTR  0x0405 // New data counter R 
#define  ADI_16495_X_DELTANG_LOW  0x4041 // Output, x-axis delta angle, low word R 
#define  ADI_16495_X_DELTANG_OUT  0x4243 // Output, x-axis delta angle, high word R 
#define  ADI_16495_Y_DELTANG_LOW  0x4445 // Output, y-axis delta angle, low word R 
#define  ADI_16495_Y_DELTANG_OUT  0x4647 // Output, y-axis delta angle, high word R 
#define  ADI_16495_Z_DELTANG_LOW  0x4849 // Output, z-axis delta angle, low word R 
#define  ADI_16495_Z_DELTANG_OUT  0x4A4B // Output, z-axis delta angle, high word R 
#define  ADI_16495_X_DELTVEL_LOW  0x4C4D // Output, x-axis delta velocity, low word R 
#define  ADI_16495_X_DELTVEL_OUT  0x4E4F // Output, x-axis delta velocity, high word R 
#define  ADI_16495_Y_DELTVEL_LOW  0x5051 // Output, y-axis delta velocity, low word R 
#define  ADI_16495_Y_DELTVEL_OUT  0x5253 // Output, y-axis delta velocity, high word R 
#define  ADI_16495_Z_DELTVEL_LOW  0x5455 // Output, z-axis delta velocity, low word R 
#define  ADI_16495_Z_DELTVEL_OUT  0x5657 // Output, z-axis delta velocity, high word R 
#define  ADI_16495_PROD_ID        0x7E7F// Identification, part number R 0x4071

#define  ADI_16495_PAGE_ID2      0x0001
#define  ADI_16495_XG_BIAS_LOW   0x1011 // Calibration, offset, gyroscope, x-axis, low word R/W 
#define  ADI_16495_XG_BIAS_HIGH  0x1213 // Calibration, offset, gyroscope, x-axis, high word R/W 
#define  ADI_16495_YG_BIAS_LOW   0x1415// Calibration, offset, gyroscope, y-axis, low word R/W 
#define  ADI_16495_YG_BIAS_HIGH  0x1617// Calibration, offset, gyroscope, y-axis, high word R/W 
#define  ADI_16495_ZG_BIAS_LOW   0x1819 // Calibration, offset, gyroscope, z-axis, low word R/W 
#define  ADI_16495_ZG_BIAS_HIGH  0x1A1B // Calibration, offset, gyroscope, z-axis, high word R/W 
#define  ADI_16495_XA_BIAS_LOW   0x1C1D // Calibration, offset, accelerometer, x-axis, low word R/W 
#define  ADI_16495_XA_BIAS_HIGH  0x1E1F // Calibration, offset, accelerometer, x-axis, high word R/W 
#define  ADI_16495_YA_BIAS_LOW   0x2021 // Calibration, offset, accelerometer, y-axis, low word R/W 
#define  ADI_16495_YA_BIAS_HIGH  0x2223 // Calibration, offset, accelerometer, y-axis, high word R/W 
#define  ADI_16495_ZA_BIAS_LOW   0x2425 // Calibration, offset, accelerometer, z-axis, low word R/W 
#define  ADI_16495_ZA_BIAS_HIGH  0x2627 // Calibration, offset, accelerometer, z-axis, high word R/W 
 
#define  ADI_16495_PAGE_ID3  0x0001
#define  ADI_16495_DEC_RATE  0x0C0D // Control, output sample rate decimation
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
	

} ADI_16495_HandleTypeDef;
/* Exported constants --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int ADI_16495_ReadImu(ADI_16495_HandleTypeDef* pImu);
int ADI_16495_ReadDeviceNumber(ADI_16495_HandleTypeDef* pImu);
int ADI_16495_ParseImu(ADI_16495_HandleTypeDef* pImu);
int ADI_16495_Readreg(ADI_16495_HandleTypeDef* pImu);
int ADI_16495_DEC_SET(ADI_16495_HandleTypeDef* pImu);
int make_crc32_table();
int ADI_16495_ReadburstImu(ADI_16495_HandleTypeDef* pImu);
int ADI_16495_Filter_SET(ADI_16495_HandleTypeDef* pImu);

#endif
/************************ (C) COPYRIGHT XiLang CO. Ltd. *******END OF FILE*****/
