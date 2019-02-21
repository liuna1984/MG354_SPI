#include "MG354.h"
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern uint8_t usart3_rxbuf[100] ;
extern uint16_t usart3_rxlen;
extern uint8_t RecFlag;
extern uint8_t CheckSumenable;
extern uint8_t usart3_Rx_Data;
void UART_Write_Command(uint8_t RegAddr,uint8_t data)
{
	unsigned char Txdata[3];
	Txdata[0]=0x80|RegAddr;
	Txdata[1]=data;
	Txdata[2]=0x0D;
	//HAL_UART_Transmit_IT(&huart3,(uint8_t*)&Txdata[0],3);
	//HAL_Delay(1);
	HAL_UART_Transmit(&huart5,(uint8_t*)&Txdata[0],3,100);
}
uint8_t UART_Read_Command(uint8_t RegAddr)
{
	unsigned char Txdata[3];
	uint16_t delaycount=0;
	Txdata[0]=RegAddr;
	Txdata[1]=0x00;
	Txdata[2]=0x0D;
	//HAL_UART_Transmit_IT(&huart3,(uint8_t*)&Txdata[0],3);
	//HAL_Delay(1);
	HAL_UART_Transmit(&huart5,(uint8_t*)&Txdata[0],3,100);
	while(RecFlag==0)
	{
		delaycount++;
		HAL_Delay(1);
		if(delaycount>100)
		{
			delaycount=0;
			printf("IMU has no response!\n");
			return 0;
		}
		
	}
	RecFlag=0;
	return 1;
}
void MG354_Init()
{
	uint8_t ReadRes;
	//Power on sequence
	HAL_Delay(1000);
	UART_Write_Command(0x7E,0x01);
	do
	{
		ReadRes=UART_Read_Command(GLOB_CMD_L);
	}
	while((usart3_rxbuf[1]&0x04)==1&&ReadRes==1);
	UART_Write_Command(0x7E,0x0);
	ReadRes=UART_Read_Command(DIAG_STAT);
	if(ReadRes)
	{
		if((usart3_rxbuf[2]&0x60)==0x00)
		{
			printf("IMU is OK!\n");
		}
		else
		{
			printf("IMU is faulty!\n");
		}
    };
	// Filter setting
	UART_Write_Command(0x7E,0x01);
	UART_Write_Command(FILTER_CTRL_L,0x09);/*FIR Kaiser filter (parameter=8) TAP=32 and fc=100*/
	do
	{
		ReadRes=UART_Read_Command(FILTER_CTRL_L);
	}
	while((usart3_rxbuf[2]&0x20)==1&&ReadRes==1);
	
}
void Read_IMU_Start()
{
	uint8_t ReadRes;
	//Configulation mode
	UART_Write_Command(0x7E,0x00);
	UART_Write_Command(MODE_CTRL_H,0x02);/*return to Configulation mode */
	UART_Write_Command(0x7E,0x1);/* WINDOW=1 */
	UART_Write_Command(SMPL_CTRL_H,0x0A);/* 100SPS */
  UART_Write_Command(UART_CTRL_L,0x01);/*UART Auto mode*/
//	UART_Write_Command(UART_CTRL_L,0x03);/*UART Auto mode£¬Auto start*/
	UART_Write_Command(BURST_CTRL1_L,0x07); /* GPIO=on,COUNT=on,CheckSum=on */
//	CheckSumenable=1;
	UART_Write_Command(BURST_CTRL1_H,0xF0);/* FLAG=on,TEMP=on,Gyro=on,ACCL=on */
	UART_Write_Command(BURST_CTRL2_H,0x70);/* TEMP=32bit,Gyro=32bit,ACCL=32bit */
	UART_Write_Command(GLOB_CMD_L,0x08);/* Flash backup */
	UART_Write_Command(0x7E,0x0);/* WINDOW=0 */
	UART_Write_Command(MODE_CTRL_H,0x01);/* move to Sampling mode */
//	HAL_UART_Receive_IT(&huart3,&usart3_Rx_Data, 1);
}
/*0x80, FLAG_Hi, FLAG_Lo,
TEMP_HIGH_Hi, TEMP_HIGH_Lo, TEMP_LOW_Hi, TEMP_LOW_Lo,
XGYRO_HIGH_Hi, XGYRO_HIGH_Lo, XGYRO_LOW_Hi, XGYRO_LOW_Lo,
YGYRO_HIGH_Hi, YGYRO_HIGH_Lo, YGYRO_LOW_Hi, YGYRO_LOW_Lo,
ZGYRO_HIGH_Hi, ZGYRO_HIGH_Lo, ZGYRO_LOW_Hi, ZGYRO_LOW_Lo,
XACCL_HIGH_Hi, XACCL_HIGH_Lo, XACCL_LOW_Hi, XACCL_LOW_Lo,
YACCL_HIGH_Hi, YACCL_HIGH_Lo, YACCL_LOW_Hi, YACCL_LOW_Lo,
ZACCL_HIGH_Hi, ZACCL_HIGH_Lo, ZACCL_LOW_Hi, ZACCL_LOW_Lo,
GPIO_Hi, GPIO_Lo, COUNT_Hi, COUNT_Lo, 0x0d*/
void ParaseIMU(MG354_HandleTypeDef *pBuf)
{
	int32_t Temp,XGyro,YGyro,ZGyro,XAccel,YAccel,ZAccel;
	Temp=usart3_rxbuf[3]<<24|usart3_rxbuf[4]<<16|usart3_rxbuf[5]<<8|usart3_rxbuf[6];
	XGyro=usart3_rxbuf[7]<<24|usart3_rxbuf[8]<<16|usart3_rxbuf[9]<<8|usart3_rxbuf[10];
	YGyro=usart3_rxbuf[11]<<24|usart3_rxbuf[12]<<16|usart3_rxbuf[13]<<8|usart3_rxbuf[14];
	ZGyro=usart3_rxbuf[15]<<24|usart3_rxbuf[16]<<16|usart3_rxbuf[17]<<8|usart3_rxbuf[18];
	XAccel=usart3_rxbuf[19]<<24|usart3_rxbuf[20]<<16|usart3_rxbuf[21]<<8|usart3_rxbuf[22];
	YAccel=usart3_rxbuf[23]<<24|usart3_rxbuf[24]<<16|usart3_rxbuf[25]<<8|usart3_rxbuf[26];
	ZAccel=usart3_rxbuf[27]<<24|usart3_rxbuf[28]<<16|usart3_rxbuf[29]<<8|usart3_rxbuf[30];
	pBuf->fTempretrue=(float)(-0.0037918/65536) * ( Temp -172621824 ) + 25;
	pBuf->fGyrX=(float)(0.016/65536) * XGyro;
	pBuf->fGyrY=(float)(0.016/65536) * YGyro;
	pBuf->fGyrZ=(float)(0.016/65536) * ZGyro;
	pBuf->fAccX=(float)(0.2/65536) * XAccel/1000*9.8;
	pBuf->fAccY=(float)(0.2/65536) * YAccel/1000*9.8;
	pBuf->fAccZ=(float)(0.2/65536) * ZAccel/1000*9.8;
}
