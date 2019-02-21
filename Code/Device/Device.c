#include "main.h"
#include "dvc_Led.h"
#include "Device.h"
#include "EnhancedSTM32.h"
#include "Device.h"
#include "Message.h"
#include "MG354.h"
//#include "dvc_IMU100N.h"
//#include "dvc_ADIS16495.h"
extern TIM_HandleTypeDef htim6;

extern UART_HandleTypeDef huart2; //!< <-> Main Port
extern UART_HandleTypeDef huart3; //!< <-> Debug Port
extern UART_HandleTypeDef huart5; //!< <-> M-G354 Port
//extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi4;

//ADI_16495_HandleTypeDef g_hImu16495;

LED_DEFINE_TABLE_BEGIN
{ "Live", GPIO_PIN_10,GPIOE },
{ "IN",   GPIO_PIN_9,GPIOE },
{ "OUT",  GPIO_PIN_8,GPIOE },
LED_DEFINE_TABLE_END

extern ParseParam g_Usart5Parse;
XL_UART_IT_InputStreamTypeDef g_Usart1InputStream;
XL_UART_IT_InputStreamTypeDef g_Usart3InputStream;
//XL_UART_IT_InputStreamTypeDef g_Usart5InputStream;
XL_UART_DMA_InputStreamTypeDef g_Usart5InputStream;
XL_UART_DMA_OutputStreamTypeDef g_Usart3OutputStream;

CONSTRUCT_CIRCULAR_BUFFER_MEMORY_POOL(g_USART1_Buffer, 1024);
CONSTRUCT_CIRCULAR_BUFFER_MEMORY_POOL(g_USART3_Buffer, 128);
//CONSTRUCT_CIRCULAR_BUFFER_MEMORY_POOL(g_USART5_Buffer, 128);
unsigned char g_USART5_Buffer[128];

CONSTRUCT_DOUBLE_BUFFER_MEMORY_POOL(g_Usart1OutputBuffer, 1024);
CONSTRUCT_DOUBLE_BUFFER_MEMORY_POOL(g_Usart3OutputBuffer, 1024);

MG354_HandleTypeDef MG354;

DMA_CircularBuffer g_CircularBuffer_DMA_UART5;

void InitDevice(void)
{	
//	g_Usart5InputStream.huart = &huart5;
//	g_Usart5InputStream.pCircularBuffer = &g_USART5_Buffer;
//	g_Usart5InputStream.pParseParam = &g_Usart5Parse;
//	UART_LinkInputStream_IT(&g_Usart5InputStream);
//	g_Usart5InputStream.circularbuffer = &g_CircularBuffer_DMA_UART5;
//	g_Usart5InputStream.circularbuffer->husart = &huart5;
//	g_Usart5InputStream.circularbuffer->buffer = g_USART5_Buffer;
//	g_Usart5InputStream.circularbuffer->maxSize = 128;
//	g_Usart5InputStream.pParseParam = &g_Usart5Parse;
//	UART_LinkInputStream_DMA(&g_Usart5InputStream);
	
	g_Usart3OutputStream.huart = &huart3;
	g_Usart3OutputStream.doublebuffer = &g_Usart3OutputBuffer;
//	g_hImu16495.phspi = &hspi1;
//	g_hImu16495.NSS_GPIOx = CS_100N_GPIO_Port;
//	g_hImu16495.NSS_GPIO_Pin = CS_100N_Pin;
//	
//	//	HAL_InitTick(1); // SYSTick 计时器1ms}	
//	//LED_TurnOnAll();
	HAL_GPIO_WritePin(RST_MG354_GPIO_Port, RST_MG354_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
  HAL_GPIO_WritePin(RST_MG354_GPIO_Port, RST_MG354_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(RST_MG354_GPIO_Port, RST_MG354_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
//	//ADI_16495_software_reset(&g_hImu16495);
//	//HAL_Delay(1000);
//	//do
//   {
//	    ADI_16495_ReadDeviceNumber(&g_hImu16495);

//	}
//  //  while(g_hImu16495.nProductID != 0x4071);
//	if (g_hImu16495.nProductID == 0x406F)
//	{
//		LED_Toggle(LED2);
//	}	
//	ADI_16495_DEC_SET(&g_hImu16495);
//	ADI_16495_Filter_SET(&g_hImu16495);
//	//crc校验
//	make_crc32_table();
//  Read_IMU_Start();
	HAL_TIM_Base_Start_IT(&htim6);
	MG354_Init_SPI();
	Read_IMU_Start_SPI();
}

unsigned char f10ms = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		///@todo 在这里补充输出函数
		f10ms = 1;

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t crcRec,crcRes;
	if (huart->Instance == USART1)
	{
		RecieveInputStream_IT(huart, &g_Usart1InputStream);
	}
	else if (huart->Instance == USART3)
	{
		RecieveInputStream_IT(huart, &g_Usart3InputStream);
	}
	else if (huart->Instance == UART5)
	{
		DmaCircularBufferIrqHandle(g_Usart5InputStream.circularbuffer);
		//RecieveInputStream_IT(huart, &g_Usart5InputStream);
	}
	else
	{
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (HAL_UART_GetError(huart) == HAL_UART_ERROR_ORE)
	{
		huart->RxState = HAL_UART_STATE_READY;
	}
	else if (HAL_UART_GetError(huart) == HAL_UART_ERROR_DMA)
	{
		huart->RxState = HAL_UART_STATE_READY;
	}
	else
	{
	}

	if (huart->Instance == USART1)
	{
	}
	else if (huart->Instance == USART3)
	{
	}
	else
	{
	}
}

int fputc(int ch, FILE* f)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, 100);
	return ch;
}

unsigned char rxCpltFlag = 0;
void SampleImuDataMG354(MSG_QSENS_DatagramImuTypeDef* imu)
{
	//ParaseIMU(&MG354);
	ParaseIMU_SPI(&MG354);
	imu->data.lAccX =(long)(MG354.fAccX/9.8*2048);
	imu->data.lAccY =(long)(MG354.fAccY/9.8*2048);
	imu->data.lAccZ =(long)(MG354.fAccZ/9.8*2048);
	imu->data.lGyroX =(long)(MG354.fGyrX*131);
	imu->data.lGyroY =(long)(MG354.fGyrY*131);
	imu->data.lGyroZ =(long)(MG354.fGyrZ*131);
	imu->data.sTempretureAccX = (int16_t)(MG354.fTempretrue*100);
	imu->data.sTempretureAccY = 0x0000;
	imu->data.sTempretureAccZ = 0x0000;
	imu->data.sTempretureGyroX = 0x0000;
	imu->data.sTempretureGyroY = 0x0000;
	imu->data.sTempretureGyroZ = 0x0000;			
}