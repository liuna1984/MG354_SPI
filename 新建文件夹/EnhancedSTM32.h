#ifndef STM32_H_201803151748
#define STM32_H_201803151748
/**
 * @file 
 * @author Qiaoyy@xlnav.com
 * @date 
 * @brief  
 * Copyright (c) 2018 XiLang Co. Ltd.
 * All rights reserved.
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Global.h"
#include "ParseFrame.h"
/* Exported types ------------------------------------------------------------*/ 
typedef struct 
{
	volatile unsigned int readCounter;
	volatile unsigned int writeCounter;
	volatile unsigned int readCircle;
	volatile unsigned int writeCircle;
	volatile unsigned int lastWriteCircle;
	volatile unsigned int length;
	volatile unsigned int maxSize;
	volatile unsigned int head;
	volatile unsigned int tail;
	volatile unsigned int maxLength;
	unsigned char* buffer;
	UART_HandleTypeDef *husart;
	volatile unsigned char overLapFlag;
	volatile unsigned char protact;
} DMA_CircularBuffer;

typedef struct
{
	unsigned char ucTemp;
	UART_HandleTypeDef * huart;
	CircularBuffer *pCircularBuffer;
	ParseParam *pParseParam;
} XL_UART_IT_InputStreamTypeDef;

typedef struct
{
	DMA_CircularBuffer *circularbuffer;
	ParseParam *pParseParam;
} XL_UART_DMA_InputStreamTypeDef;

typedef struct
{
	UART_HandleTypeDef * huart;
	DoubleBuffer *doublebuffer;
} XL_UART_DMA_OutputStreamTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int XL_PopDmaCircularBuffer(uint8_t *value, DMA_CircularBuffer* buffer);
void XL_DmaCircularBufferIrqHandle(DMA_CircularBuffer* buffer);
void XL_InitDmaCircularBuffer(DMA_CircularBuffer* buffer);
void DmaCircularBufferIrqHandle(DMA_CircularBuffer* buffer);

void SendDoubleBuffer(DoubleBuffer *pBuf, UART_HandleTypeDef* huart);
void UART_LinkInputStream_IT(XL_UART_IT_InputStreamTypeDef* stream);
void UART_LinkInputStream_DMA(XL_UART_DMA_InputStreamTypeDef* stream);
void RecieveInputStream_IT(UART_HandleTypeDef *huart, XL_UART_IT_InputStreamTypeDef* stream);
void ParseFrame_DMA(XL_UART_DMA_InputStreamTypeDef* stream, unsigned int nMaxReadTimes);

#endif
/************************ (C) COPYRIGHT XiLang CO. Ltd. *******END OF FILE*****/

