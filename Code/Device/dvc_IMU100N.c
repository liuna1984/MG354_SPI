/**
 * @file dvc_ADI16495.c
 * @author Qiaoyy@xlnav.com
 * @version V1.0
 * @date 2018-04-27
 * @brief  ADI 16495 Çý¶¯³ÌÐò
 * Copyright (c) 2018 XiLang Co. Ltd.
 * All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "dvc_IMU100N.h"
#include "stdio.h"
#include "usart.h"
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  
 * @param
 * @return 
*/

void mydelay(uint32_t timecount);
//int IMU100N_ReadDeviceNumber(IMU100N_HandleTypeDef* pImu)
//{
//	unsigned short buffer[2]={ IMU100N_PROD_ID,0};
//	unsigned short PAGE_ID[2]={0x8000,0x8100};
//	
//	
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
// 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID[0],1,100);
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	mydelay(50);
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
// 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID[1],1,100);
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	mydelay(50);
//	
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
// 	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[0], 1,0xFFFF);
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	mydelay(50);
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[1], 1,0xFFFF);
//	pImu->nProductID = buffer[1];
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	mydelay(50);
//	return 1;
//}
int IMU100N_software_reset(IMU100N_HandleTypeDef* pImu)
{
	unsigned short buffer[2]={0x8280,0x8300};
	unsigned short PAGE_ID[2]={0x8003,0x8100};
	
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID[0],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID[1],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[0], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[1], 1,0xFFFF);
	pImu->nProductID = buffer[1];
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	return 1;
}
//int IMU100N_ReadDeviceNumber(IMU100N_HandleTypeDef* pImu)
//{
//	unsigned short cmd[2]={ IMU100N_PROD_ID,0};
//	unsigned short buffer[2]={ IMU100N_PROD_ID,0};
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive(pImu->phspi, (uint8_t*)&cmd[0], (uint8_t*)&pImu->usBuffer[0], 1,100);
//	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	mydelay(50);
////	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
////	HAL_SPI_TransmitReceive(pImu->phspi, (uint8_t*)&cmd[1], (uint8_t*)&pImu->usBuffer[1], 1,100);
////	pImu->nProductID = pImu->usBuffer[1];
////	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	return 1;
//}
unsigned short outbuffer[20];
int IMU100N_ReadburstImu(IMU100N_HandleTypeDef* pImu)
{
	unsigned short cmd;
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	cmd=0x7C00;
    HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&cmd,1,100);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&pImu->usBuffer[0], 20,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	return 1;
}
int IMU100N_ReadImu(IMU100N_HandleTypeDef* pImu)
{
	//HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	unsigned short cmd[14];
	int i;
	
	
	pImu->usBuffer[0] = cmd[0] = IMU100N_X_GYRO_LOW;
	pImu->usBuffer[6] = cmd[1] = IMU100N_X_GYRO_OUT;

	pImu->usBuffer[1] = cmd[2] = IMU100N_Y_GYRO_LOW;
	pImu->usBuffer[7] = cmd[3] = IMU100N_Y_GYRO_OUT;

	pImu->usBuffer[2] = cmd[4] = IMU100N_Z_GYRO_LOW;
	pImu->usBuffer[8] = cmd[5] = IMU100N_Z_GYRO_OUT;

	pImu->usBuffer[3] = cmd[6] = IMU100N_X_ACCL_LOW;
	pImu->usBuffer[9] = cmd[7] = IMU100N_X_ACCL_OUT;

	pImu->usBuffer[4] = cmd[8] = IMU100N_Y_ACCL_LOW;
	pImu->usBuffer[10] = cmd[9] = IMU100N_Y_ACCL_OUT;

	pImu->usBuffer[5] = cmd[10] = IMU100N_Z_ACCL_LOW;
	pImu->usBuffer[11] = cmd[11] = IMU100N_Z_ACCL_OUT;

	pImu->usBuffer[12] = cmd[12] = IMU100N_TEMP_OUT;
    pImu->usBuffer[13] = cmd[13] = IMU100N_TEMP_OUT;
  for(i=0;i<14;i++)
	{
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(pImu->phspi, (uint8_t*)&cmd[i], (uint8_t*)&pImu->usBuffer[i], 1,100);	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	}


	return 1;
}
void mydelay(uint32_t timecount)
{int i,j;
	for(i=0;i<timecount;i++)
	for(j=0;j<timecount;j++)
       {;}	
}
int count16495=0;

int IMU100N_ParseImu(IMU100N_HandleTypeDef* pImu)
{
	
	
    
	pImu->fGyrX = (float)((short)pImu->usBuffer[2]) * 0.02f;
	pImu->fGyrY = (float)((short)pImu->usBuffer[4]) * 0.02f;
	pImu->fGyrZ = (float)((short)pImu->usBuffer[6]) * 0.02f;
	pImu->fAccX = (float)(((long)pImu->usBuffer[8]) * 65536 + (float)((long)pImu->usBuffer[7]))*0.00001220703125*0.001*9.8;
	pImu->fAccY = (float)(((long)pImu->usBuffer[10]) * 65536 + (float)((long)pImu->usBuffer[9]))*0.00001220703125*0.001*9.8;
	pImu->fAccZ = (float)(((long)pImu->usBuffer[12]) * 65536 + (float)((long)pImu->usBuffer[11]))*0.00001220703125*0.001*9.8;
	pImu->fTempretrue = (int16_t)(((float)((short)pImu->usBuffer[13])*0.00565+25)*100);


 
// printf("%f %f %f %f %f %f %d\n",pImu->fGyrX,pImu->fGyrY,pImu->fGyrZ,pImu->fAccX,pImu->fAccY,pImu->fAccZ,pImu->fTempretrue);

	//{
	//HAL_UART_Transmit(&huart2,(uint8_t *)&send_data_buf[0],30,0xffff);
	//}
 		
	return 1;
}

/************************ (C) COPYRIGHT XiLang CO. Ltd. ****** END OF FILE ****/
