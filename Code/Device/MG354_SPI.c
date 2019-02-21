/**
  ******************************************************************************
  * @file    MG354_SPI.c
  * @author  liun@xlnav.com
  * @version V1.0
  * @date    21-June-2018
  * @brief   This file includes the MG354 driver functions
  
  ******************************************************************************
  */

//#include "Debug.h"
//#include "i2c.h"
#include "MG354.h"
#include "spi.h"
/**
  * @brief  Read bytes by SPI
  * @param  REG_Add:register address will be writen
	*         *pData: the point to the result of reading
	*				  Size:the zize of the reading bytes
  * @retval None
  */
//void MG354_DevieceSPI_ReadBytes(uint8_t REG_Add, uint8_t *pData, uint16_t Size)
//{
//  REG_Add=REG_Add|0x80;
//	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi1,&REG_Add,1,100);
//	HAL_SPI_Receive(&hspi1,pData,Size,100);
//  HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_SET);	
//}
/**
  * @brief  Write bytes by SPI
  * @param  REG_Add:register address will be writen
	*        *pData: the point to the date which will be writen
	* 				Size:the size of the reading bytes
  * @retval None
  */

//void MG354_DevieceSPI_WriteBytes(uint8_t REG_Add, uint8_t *pData, uint16_t Size)
//{
//	REG_Add=REG_Add&0x7F;
//	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi1,&REG_Add,1,100);
//	HAL_SPI_Transmit(&hspi1,pData,Size,100);
//	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_SET);
//	
//}
/**
  * @brief  Read one byte by SPI
  * @param  REG_Add:register address will be writen
	*         *pData: the point to the redult of reading
  * @retval None
  */
void mydelay(uint32_t timecount)
{int i,j;
	for(i=0;i<timecount;i++)
	for(j=0;j<timecount;j++)
       {;}	
}
void MG354_DevieceSPI_ReadOneByte(uint8_t REG_Add, uint16_t* pData)
{
	uint16_t SPIRead=0;
	REG_Add=REG_Add&0x7F;
	SPIRead=REG_Add<<8;
	//while(HAL_GPIO_ReadPin(DRDY_MG354_GPIO_Port, DRDY_MG354_Pin)==GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&SPIRead, (uint8_t*)pData, 1,100);	
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&SPIRead,1,100);
//	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_SET);
//	mydelay(50);
//	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Receive(&hspi1,(uint8_t*)pData,1,100);
	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_SET);
	mydelay(100);
}
/**
  * @brief  Write one byte by SPI
  * @param  REG_Add:register address will be writen
	*         *pData: the point to the date which will be writen
  * @retval None
  */
void MG354_DevieceSPI_WriteOneByte(uint8_t REG_Add, uint8_t Data)
{
	uint16_t SPIWrite=0;
	REG_Add=REG_Add|0x80;
	SPIWrite=REG_Add<<8|Data;
	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,(uint8_t*)&SPIWrite,1,100);
	HAL_GPIO_WritePin(CS_MG354_GPIO_Port, CS_MG354_Pin, GPIO_PIN_SET);
	mydelay(100);
}

/**
  * @brief  Initializes MG354
  * @param  None
  * @retval None
  */
void MG354_Init_SPI(void)
{
	uint16_t buffer[2]={0,0};
	HAL_Delay(1000);
	MG354_DevieceSPI_WriteOneByte(WIN_CTRL_L,0x01);
	do
	{
		MG354_DevieceSPI_ReadOneByte(GLOB_CMD_L,&buffer[0]);
		MG354_DevieceSPI_ReadOneByte(0x0000,&buffer[1]);
	}
	while((buffer[1]&0x0400)==1);
	MG354_DevieceSPI_WriteOneByte(WIN_CTRL_L,0x00);
	MG354_DevieceSPI_ReadOneByte(DIAG_STAT,&buffer[0]);
	MG354_DevieceSPI_ReadOneByte(0x0000,&buffer[1]);
	if((buffer[1]&0x0060)==0x00)
	{
		printf("IMU is OK!\n");
	}
	else
	{
		printf("IMU is faulty!\n");
	}
	//filter setting
	MG354_DevieceSPI_WriteOneByte(WIN_CTRL_L,0x01);
	MG354_DevieceSPI_WriteOneByte(FILTER_CTRL_L,0x09);
	do
	{
	  MG354_DevieceSPI_ReadOneByte(WIN_CTRL_L,&buffer[0]);
		MG354_DevieceSPI_ReadOneByte(0x0000,&buffer[1]);
	}
	while((buffer[1]&0x0020)==1);
	
}
void Read_IMU_Start_SPI()
{
	MG354_DevieceSPI_WriteOneByte(WIN_CTRL_L,0x01);/* WINDOW=1 */
	MG354_DevieceSPI_WriteOneByte(SMPL_CTRL_H,0x0A);/* 100SPS */
  MG354_DevieceSPI_WriteOneByte(UART_CTRL_L,0x00);/*disable UART auto mode, just in case*/
	MG354_DevieceSPI_WriteOneByte(BURST_CTRL1_L,0x07); /* GPIO=on,COUNT=on,CheckSum=on */
	MG354_DevieceSPI_WriteOneByte(BURST_CTRL1_H,0xF0);/* FLAG=on,TEMP=on,Gyro=on,ACCL=on */
	MG354_DevieceSPI_WriteOneByte(BURST_CTRL2_H,0x70);/* TEMP=32bit,Gyro=32bit,ACCL=32bit */
	//MG354_DevieceSPI_WriteOneByte(MSC_CTRL_L,0x00);/* DRDY set GPIO */
	MG354_DevieceSPI_WriteOneByte(WIN_CTRL_L,0x00);/* WINDOW=0 */
	MG354_DevieceSPI_WriteOneByte(MODE_CTRL_H,0x01);/* move to Sampling mode */
}
/**
  * @brief Get accelerometer and gyroscope datas
  * @param  accel: the value of accelerometer
  *         gyro:the value of gyroscope
  *         temp:the pointer to temperature
  * @retval None
  */
void Read_IMU_SPI(MG354_HandleTypeDef *pBuf)
{
 // while(HAL_GPIO_ReadPin(DRDY_MG354_GPIO_Port, DRDY_MG354_Pin)==GPIO_PIN_RESET);	
	MG354_DevieceSPI_ReadOneByte(FLAG,&(pBuf->u16Buffer[0])); /* FLAG read command */
	MG354_DevieceSPI_ReadOneByte(TEMP_HIGH,&(pBuf->u16Buffer[1])); /* TEMP_HIGH read command */
	MG354_DevieceSPI_ReadOneByte(TEMP_LOW,&(pBuf->u16Buffer[2])); /* TEMP_LOW read command */
	MG354_DevieceSPI_ReadOneByte(XGYRO_HIGH,&(pBuf->u16Buffer[3])); /* XGYRO_HIGH read command */
	MG354_DevieceSPI_ReadOneByte(XGYRO_LOW,&(pBuf->u16Buffer[4])); /* XGYRO_LOW read command */
	MG354_DevieceSPI_ReadOneByte(YGYRO_HIGH,&(pBuf->u16Buffer)[5]); /* YGYRO_HIGH read command */
	MG354_DevieceSPI_ReadOneByte(YGYRO_LOW,&(pBuf->u16Buffer[6])); /* YGYRO_LOW read command */
	MG354_DevieceSPI_ReadOneByte(ZGYRO_HIGH,&(pBuf->u16Buffer[7]));/* ZGYRO_HIGH read command */
	MG354_DevieceSPI_ReadOneByte(ZGYRO_LOW,&(pBuf->u16Buffer[8])); /* ZGYRO_LOW read command */
	MG354_DevieceSPI_ReadOneByte(XACCL_HIGH,&(pBuf->u16Buffer[9])); /* XACCL_HIGH read command */
	MG354_DevieceSPI_ReadOneByte(XACCL_LOW,&(pBuf->u16Buffer[10])); /* XACCL_LOW read command */
	MG354_DevieceSPI_ReadOneByte(YACCL_HIGH,&(pBuf->u16Buffer[11])); /* YACCL_HIGH read command */
	MG354_DevieceSPI_ReadOneByte(YACCL_LOW,&(pBuf->u16Buffer[12])); /* YACCL_LOW read command */
	MG354_DevieceSPI_ReadOneByte(ZACCL_HIGH,&(pBuf->u16Buffer[13]));/* ZACCL_HIGH read command */
	MG354_DevieceSPI_ReadOneByte(ZACCL_LOW,&(pBuf->u16Buffer[14])); /* ZACCL_LOW read command */
	MG354_DevieceSPI_ReadOneByte(GPIO_L,&(pBuf->u16Buffer[15]));  /* GPIO read command */
	MG354_DevieceSPI_ReadOneByte(COUNT,&(pBuf->u16Buffer[16])); /* COUNT read command */
	MG354_DevieceSPI_ReadOneByte(0x00,&(pBuf->u16Buffer[17]));
}
//uint16_t CheckSum16N(uint8_t* pBuf,uint16_t length)
//{
//	int i;
//	uint16_t Checksum=0;
//	uint16_t Checkdata[length/2];
//	
//	for(i=0;i<length/2;i++)
//	{
//		Checkdata[i]=(pBuf[2*i]<<8)|(pBuf[2*i+1]);
//		Checksum=Checksum+Checkdata[i];
//	}
//	return Checksum;
//}
uint16_t CheckSum16NN(uint16_t* pBuf,uint16_t length)
{
	int i;
	uint16_t Checksum=0;
	
	for(i=0;i<length;i++)
	{
		Checksum=Checksum+pBuf[i];
	}
	return Checksum;
}
uint8_t RecFlagSPI=1;
void Read_IMU_SPI_Burst(MG354_HandleTypeDef *pBuf)
{
	int i;
	uint16_t crcRec,crcRes;
	MG354_DevieceSPI_WriteOneByte(BURST,0x00);
	for(i=0;i<18;i++)
	{
		MG354_DevieceSPI_ReadOneByte(0x00,&(pBuf->u16Buffer)[i+1]);
	}
	crcRec=(pBuf->u16Buffer)[18];
  crcRes=CheckSum16NN(&(pBuf->u16Buffer)[1],17);
	if(crcRec==crcRes)
	{
		RecFlagSPI=1;
	}
	else 
	{
		RecFlagSPI=0;
	}

}
void ParaseIMU_SPI(MG354_HandleTypeDef *pBuf)
{
	int32_t Temp,XGyro,YGyro,ZGyro,XAccel,YAccel,ZAccel;
	Temp=(pBuf->u16Buffer)[2]<<16|(pBuf->u16Buffer)[3];
	XGyro=(pBuf->u16Buffer)[4]<<16|(pBuf->u16Buffer)[5];
	YGyro=(pBuf->u16Buffer)[6]<<16|(pBuf->u16Buffer)[7];
	ZGyro=(pBuf->u16Buffer)[8]<<16|(pBuf->u16Buffer)[9];
	XAccel=(pBuf->u16Buffer)[10]<<16|(pBuf->u16Buffer)[11];
	YAccel=(pBuf->u16Buffer)[12]<<16|(pBuf->u16Buffer)[13];
	ZAccel=(pBuf->u16Buffer)[14]<<16|(pBuf->u16Buffer)[15];
	pBuf->fTempretrue=(float)(-0.0037918/65536) * ( Temp -172621824 ) + 25;
	pBuf->fGyrX=(float)(0.016/65536) * XGyro;
	pBuf->fGyrY=(float)(0.016/65536) * YGyro;
	pBuf->fGyrZ=(float)(0.016/65536) * ZGyro;
	pBuf->fAccX=(float)(0.2/65536) * XAccel/1000*9.8;
	pBuf->fAccY=(float)(0.2/65536) * YAccel/1000*9.8;
	pBuf->fAccZ=(float)(0.2/65536) * ZAccel/1000*9.8;
}


