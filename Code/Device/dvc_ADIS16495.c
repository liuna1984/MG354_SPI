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
#include "dvc_ADIS16495.h"
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
#define POLY        0x04C11DB7 
unsigned long crc32_table[256];  
int make_crc32_table()  
{  
    unsigned long c;  
    int i = 0;  
    int bit = 0;  
      
    for(i = 0; i < 256; i++)  
    {  
        c  = (unsigned long)i;  
          
        for(bit = 0; bit < 8; bit++)  
        {  
            if(c&1)  
            {  
				//0xEDB88320//0x4C11DB7
                c = (c >> 1)^(0xEDB88320);  
            }  
            else  
            {  
                c =  c >> 1;  
            }  
              
        }  
        crc32_table[i] = c;  
    }  
      
  return 0;
}  
  
long make_crc(unsigned long crc, unsigned char *string, long size)  
{  
  
    while(size--)  
        crc = (crc >> 8)^(crc32_table[(crc ^ *string++)&0xff]);  
  
    return crc^0xFFFFFFFF;  
} 
int ADI_16495_ReadDeviceNumber(ADI_16495_HandleTypeDef* pImu)
{
	unsigned short buffer[2]={ ADI_16495_PROD_ID,0};
	unsigned short PAGE_ID[2]={0x8000,0x8100};
	
	
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
int ADI_16495_software_reset(ADI_16495_HandleTypeDef* pImu)
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
int ADI_16495_DEC_SET(ADI_16495_HandleTypeDef* pImu)
{
	unsigned short buffer[2]={0x8C2A,0x8D00};
	unsigned short PAGE_ID3[2]={0x8003,0x8100};
	unsigned short PAGE_ID0[2]={0x8000,0x8100};
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID3[0],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID3[1],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&buffer[0], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&buffer[1], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID0[0],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID0[1],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	return 1;
}
int ADI_16495_Filter_SET(ADI_16495_HandleTypeDef* pImu)
{
	unsigned short buffer1[2]={0x966D,0x975B};
	unsigned short buffer2[2]={0x9805,0x9900};
	unsigned short PAGE_ID3[2]={0x8003,0x8100};
	unsigned short PAGE_ID0[2]={0x8000,0x8100};
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID3[0],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID3[1],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&buffer1[0], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&buffer1[1], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&buffer2[0], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&buffer2[1], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID0[0],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 	 HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&PAGE_ID0[1],1,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	return 1;
}
//int ADI_16495_ReadDeviceNumber(ADI_16495_HandleTypeDef* pImu)
//{
//	unsigned short cmd[2]={ ADI_16495_PROD_ID,0};
//	unsigned short buffer[2]={ ADI_16495_PROD_ID,0};
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
uint16_t Buffer[20];
uint8_t crcOK=1;
unsigned long crcRes,crcRec;
int ADI_16495_ReadburstImu(ADI_16495_HandleTypeDef* pImu)
{
	unsigned short cmd[20] = {0x0000};
//	uint16_t Buffer[20];
	uint8_t crcdata[30];
	int i;
	cmd[0]=0x7C00;
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	for(i=0;i<19;i++)
	{
		HAL_SPI_TransmitReceive(pImu->phspi, (uint8_t*)&cmd[i], (uint8_t*)&Buffer[i],1,0xFFFF);
	}
	//HAL_SPI_Transmit(pImu->phspi, (uint8_t*)&cmd[0],1,100);
	//HAL_SPI_Receive(pImu->phspi, (uint8_t*)&Buffer[0],19,100);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
//	HAL_Delay(1000);
	for(i=0;i<15;i++)
	{
		crcdata[2*i]=Buffer[i+2];
		crcdata[2*i+1]=Buffer[i+2]>>8;
		
	}
	crcRec=Buffer[18]<<16|Buffer[17];
	crcRes=make_crc(0xFFFFFFFF,crcdata,30);
	if(crcRec==crcRes)
	{
		crcOK=1;
	}
	else
	{
		crcOK=0;
	}
	for(i=1;i<13;i++)
	{
		pImu->usBuffer[i]=Buffer[i+3];
	}
	pImu->usBuffer[13]=Buffer[3];
	return 1;
}
int ADI_16495_ReadImu(ADI_16495_HandleTypeDef* pImu)
{
	//HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	unsigned short cmd[14];
	int i;
	
	pImu->usBuffer[0] = cmd[0] = ADI_16495_X_GYRO_LOW;
	pImu->usBuffer[6] = cmd[1] = ADI_16495_X_GYRO_OUT;

	pImu->usBuffer[1] = cmd[2] = ADI_16495_Y_GYRO_LOW;
	pImu->usBuffer[7] = cmd[3] = ADI_16495_Y_GYRO_OUT;

	pImu->usBuffer[2] = cmd[4] = ADI_16495_Z_GYRO_LOW;
	pImu->usBuffer[8] = cmd[5] = ADI_16495_Z_GYRO_OUT;

	pImu->usBuffer[3] = cmd[6] = ADI_16495_X_ACCL_LOW;
	pImu->usBuffer[9] = cmd[7] = ADI_16495_X_ACCL_OUT;

	pImu->usBuffer[4] = cmd[8] = ADI_16495_Y_ACCL_LOW;
	pImu->usBuffer[10] = cmd[9] = ADI_16495_Y_ACCL_OUT;

	pImu->usBuffer[5] = cmd[10] = ADI_16495_Z_ACCL_LOW;
	pImu->usBuffer[11] = cmd[11] = ADI_16495_Z_ACCL_OUT;

	pImu->usBuffer[12] = cmd[12] = ADI_16495_TEMP_OUT;
    pImu->usBuffer[13] = cmd[13] = ADI_16495_PROD_ID;
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

int ADI_16495_ParseImu(ADI_16495_HandleTypeDef* pImu)
{
	
	pImu->fAccY = (float)((short)pImu->usBuffer[10]) * 0.00025f*9.8 + (float)((short)pImu->usBuffer[9])/65536.0f * 0.00025f*9.8;
	pImu->fAccZ = (float)((short)pImu->usBuffer[12]) * 0.00025f*9.8 + (float)((short)pImu->usBuffer[11])/65536.0f * 0.00025f*9.8;
	pImu->fTempretrue = (int16_t)(((float)((short)pImu->usBuffer[13])/80+25)*100);
    
	pImu->fGyrX = (float)((short)pImu->usBuffer[2]) * 0.025f + (float)((short)pImu->usBuffer[1])/65536.0f * 0.025f;
	pImu->fGyrY = (float)((short)pImu->usBuffer[4]) * 0.025f + (float)((short)pImu->usBuffer[3])/65536.0f * 0.025f;
	pImu->fGyrZ = (float)((short)pImu->usBuffer[6]) * 0.025f + (float)((short)pImu->usBuffer[5])/65536.0f * 0.025f;
	pImu->fAccX = (float)((short)pImu->usBuffer[8]) * 0.00025f*9.8 + (float)((short)pImu->usBuffer[7])/65536.0f * 0.00025f*9.8;



 
// printf("%f %f %f %f %f %f %d\n",pImu->fGyrX,pImu->fGyrY,pImu->fGyrZ,pImu->fAccX,pImu->fAccY,pImu->fAccZ,pImu->fTempretrue);

	//{
	//HAL_UART_Transmit(&huart2,(uint8_t *)&send_data_buf[0],30,0xffff);
	//}
 		
	return 1;
}
unsigned short ErrSYS;
int ADI_16495_Readreg(ADI_16495_HandleTypeDef* pImu)
{
 	unsigned short buffer[2] = {ADI_16495_DIAG_STAT,0};
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
 // while(HAL_GPIO_ReadPin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin) == GPIO_PIN_SET);
//	while(HAL_GPIO_ReadPin(DR_16495_GPIO_Port, DR_16495_Pin) == GPIO_PIN_RESET);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[0], 1,0xFFFF);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	mydelay(50);
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(pImu->phspi, (uint8_t*)&buffer[1], 1,0xFFFF);
	ErrSYS= buffer[1];
	HAL_GPIO_WritePin(pImu->NSS_GPIOx, pImu->NSS_GPIO_Pin, GPIO_PIN_SET);
	return 1;
}

/************************ (C) COPYRIGHT XiLang CO. Ltd. ****** END OF FILE ****/
