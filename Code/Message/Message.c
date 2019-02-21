/**
 * @file     Com.c
 * @author   Qiaoyy
 * @date     2017/12/03
 * @version  V1.0
 * @brief    Comunication Level
 */

/* Includes ------------*/
#include "Global.h"
#include "ParseFrame.h"
#include "Device.h"
#include "msg_Qsens.h"
#include "Message.h"
//#include "dvc_ADI16470.h"
#include "msg_Qsens.h"
//#include "bsp_mpu9255.h"
#include "EnhancedSTM32.h"
/* Private typedef ------------*/

/* Private macro ------------*/

/* Private variables ---------*/

//extern ADI_16470_HandleTypeDef g_hImu16470;
extern XL_UART_DMA_InputStreamTypeDef g_Usart5InputStream;
extern XL_UART_DMA_OutputStreamTypeDef g_Usart3OutputStream;

MSG_QSENS_DatagramImuTypeDef Imu;

TRANSPORT_LAYER_DEFINE_BEGIN(USART1_FRAME)
	QSENS_BIN_PROTOCOL_MAPPING_DEFINE
TRANSPORT_LAYER_DEFINE_END

TRANSPORT_LAYER_DEFINE_BEGIN(USART3_FRAME)
//	QSENS_BIN_PROTOCOL_MAPPING_DEFINE
    MG354_MAPPING_DEFINE
TRANSPORT_LAYER_DEFINE_END

TRANSPORT_LAYER_DEFINE_BEGIN(USART5_FRAME)
//	QSENS_BIN_PROTOCOL_MAPPING_DEFINE
    MG354_MAPPING_DEFINE
TRANSPORT_LAYER_DEFINE_END
extern CircularBuffer g_USART1_Buffer;
extern CircularBuffer g_USART3_Buffer;
extern CircularBuffer g_USART5_Buffer;

CONSTRUCT_PARSE_PARAM_MEMORY_POOL(g_Usart1Parse, 128);
CONSTRUCT_PARSE_PARAM_MEMORY_POOL(g_Usart3Parse, 128);
CONSTRUCT_PARSE_PARAM_MEMORY_POOL(g_Usart5Parse, 128);

/* Private function prototypes ------*/

/* Private functions ------------*/

void InitMessage()
{
	g_Usart1Parse.TransportFrameDefine = USART1_FRAME;
	g_Usart1Parse.TransportFrameNumber = TRANSPORT_NUMBER(USART1_FRAME);
	
	g_Usart3Parse.TransportFrameDefine = USART3_FRAME;
	g_Usart3Parse.TransportFrameNumber = TRANSPORT_NUMBER(USART3_FRAME);
	
	g_Usart5Parse.TransportFrameDefine = USART5_FRAME;
	g_Usart5Parse.TransportFrameNumber = TRANSPORT_NUMBER(USART5_FRAME);
	
	InitFrameParse(&g_Usart1Parse, 0, 0);
	InitFrameParse(&g_Usart3Parse, 0, 0);
	InitFrameParse(&g_Usart5Parse, 0, 0);
}

void ParseFrameUsart1(void)
{
	ParseFrame(&g_USART1_Buffer, &g_Usart1Parse, 1024);
}

void ParseFrameUsart3(void)
{
	ParseFrame(&g_USART3_Buffer, &g_Usart3Parse, 1024);
}

void ParseFrameUsart5(void)
{
	ParseFrame(&g_USART5_Buffer, &g_Usart5Parse, 1024);
}
void ParseFrameUsart5DMA(void)
{
	ParseFrame_DMA(&g_Usart5InputStream, 1024);
}
void SampleAndSendImuData()
{
//	SampleImuData(&Imu);
	SampleImuDataMG354(&Imu);
	QSENS_MakeImuFrame(&Imu, g_Usart3OutputStream.doublebuffer);
	SendDoubleBuffer(g_Usart3OutputStream.doublebuffer, g_Usart3OutputStream.huart);
}


