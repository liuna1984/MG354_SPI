/**
 * @file
 * @author Qiaoyy@xlnav.com
 * @version
 * @date
 * @brief
 * Copyright (c) 2018 XiLang Co. Ltd.
 * All rights reserved.
 */

 /* Includes ------------------------------------------------------------------*/
#include "Global.h"
#include "EnhancedSTM32.h"
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
void InitDmaCircularBuffer(DMA_CircularBuffer* buffer)
{
	buffer->readCounter = 0;
	buffer->writeCounter = 0;
	buffer->readCircle = 0;
	buffer->writeCircle = 0;
	buffer->length = 0;
	buffer->head = 0;
	buffer->tail = 0;
	buffer->overLapFlag = 0;
	buffer->maxLength = 0;
	buffer->protact = 0;
	buffer->lastWriteCircle = 0;
	HAL_UART_Receive_DMA(buffer->husart, buffer->buffer, buffer->maxSize);
}

/**
 * @brief
 * @param
 * @return
*/
int PopDmaCircularBuffer(uint8_t *value, DMA_CircularBuffer* buffer)
{
	volatile unsigned int writeCircle;
	volatile unsigned int head;
	// 防止写入字节数溢出并处理过零问题
	writeCircle = buffer->writeCircle;
#ifdef STM32F429xx
	head = buffer->husart->hdmarx->Instance->NDTR;
#endif
#ifdef STM32F1
	head = buffer->husart->hdmarx->Instance->CNDTR;
#endif
	if (writeCircle == buffer->lastWriteCircle
		&& (head < 1 || head > buffer->maxSize - 1))
	{
		//		printf("protact1 %d %d %d %d %d\n", buffer->maxLength, buffer->head, buffer->tail, buffer->writeCounter, buffer->readCounter);
		return 0;
	}
	if (head > 1 && head < buffer->maxSize - 1)
	{
		buffer->lastWriteCircle = writeCircle;
	}


	if ((ULONG_MAX / buffer->maxSize) < (writeCircle + 10))
	{
		if (buffer->readCounter < writeCircle * buffer->maxSize)
		{
			buffer->writeCircle -= buffer->readCounter / buffer->maxSize;
			buffer->readCounter = buffer->readCounter % buffer->maxSize;
		}
		else
		{
			buffer->writeCircle -= writeCircle;
			buffer->readCounter -= writeCircle * buffer->maxSize;
		}
	}
	// 计算头部位置和总写入字节数
	buffer->head = buffer->maxSize - head;
	buffer->writeCounter = writeCircle * buffer->maxSize + buffer->head;

	// 计算缓存区中未读出的字节数
	buffer->length = buffer->writeCounter - buffer->readCounter;

	if (buffer->writeCounter < buffer->readCounter)
	{
		//		printf("Error %d %d %d %d %d %d %d %d %d\n", buffer->maxLength, buffer->head, buffer->tail, buffer->writeCounter, buffer->readCounter, head, buffer->lastWriteCircle, buffer->writeCircle, writeCircle);
	}

	// 检查是否溢出并重定位队尾位置
	if (buffer->length > buffer->maxSize)
	{
		buffer->overLapFlag = 1;
		buffer->length = buffer->maxSize;
		buffer->readCounter = buffer->writeCounter - buffer->length;
		if (buffer->head == buffer->maxSize - 1)
		{
			buffer->tail = 0;
		}
		else
		{
			buffer->tail = buffer->head + 1;
		}
		//		printf("Overlap %d %d %d %d %d %d\n", buffer->maxLength, buffer->head, buffer->tail, buffer->writeCounter, buffer->readCounter, head);

	}

	// 执行推送动作
	if (buffer->length > 0)
	{
		if (buffer->length >= buffer->maxLength)
		{

			buffer->maxLength = buffer->length;
			//			printf("Max: %d %d %d %d %d %d\n", buffer->maxLength, buffer->head, buffer->tail, buffer->writeCounter, buffer->readCounter, head);
		}
		*value = buffer->buffer[buffer->tail];
		buffer->readCounter++;
		buffer->tail++;
		buffer->length--;
		if (buffer->tail >= buffer->maxSize)
		{
			buffer->tail = 0;
			buffer->readCircle++;
		}
		return 1;
	}
	else
	{
		*value = 0x00;
		return 0;
	}
}

/**
 * @brief
 * @param
 * @return
*/
void DmaCircularBufferIrqHandle(DMA_CircularBuffer* buffer)
{
	buffer->protact = 1;
	buffer->writeCircle++;
	buffer->protact = 0;
}

/**
 * @brief
 * @param
 * @return
*/
void SendDoubleBuffer(DoubleBuffer *pBuf, UART_HandleTypeDef* huart)
{
	if (huart->gState != HAL_UART_STATE_BUSY_TX && pBuf->m_nBufferLength[pBuf->m_nBufferNumber] > 0)
	{
		if (pBuf->m_nBufferNumber == 1)
		{
			pBuf->m_nBufferNumber = 0;
		}
		else if (pBuf->m_nBufferNumber == 0)
		{
			pBuf->m_nBufferNumber = 1;
		}
		else
		{
			pBuf->m_nBufferNumber = 0;
		}

		{
			HAL_UART_Transmit_DMA(huart, pBuf->m_ucDataBuffer[1 - pBuf->m_nBufferNumber],
				pBuf->m_nBufferLength[1 - pBuf->m_nBufferNumber]);
		}
		pBuf->m_nBufferLength[1 - pBuf->m_nBufferNumber] = 0;
	}
}

void UART_LinkInputStream_IT(XL_UART_IT_InputStreamTypeDef* stream)
{
	//assert(stream->huart != NULL);
	//assert(stream->pCircularBuffer != NULL);
	//assert(stream->pParseParam != NULL);
	HAL_UART_Receive_IT(stream->huart, &(stream->ucTemp), 1);
	ClearCircularBuffer(stream->pCircularBuffer);
	ClearParseParam(stream->pParseParam);
}

void RecieveInputStream_IT(UART_HandleTypeDef *huart, XL_UART_IT_InputStreamTypeDef* stream)
{
	if(huart->Instance == stream->huart->Instance)
	{
		PushCircularBuffer(stream->ucTemp, stream->pCircularBuffer);
		HAL_UART_Receive_IT(stream->huart, &(stream->ucTemp), 1);
	}
}

void UART_LinkInputStream_DMA(XL_UART_DMA_InputStreamTypeDef* stream)
{
	InitDmaCircularBuffer(stream->circularbuffer);
	ClearParseParam(stream->pParseParam);
}

void ParseFrame_DMA(XL_UART_DMA_InputStreamTypeDef* stream, unsigned int nMaxReadTimes)
{
	unsigned char byte;
	uint32_t times = 0;
	unsigned char flag = 0x00;
	while ((PopDmaCircularBuffer(&byte, stream->circularbuffer) > 0) && (times < nMaxReadTimes))
	{
		times++;
		flag = ParseFrameByte(byte, stream->pParseParam);
		if (flag!=0)
		{
			break;
		}
	}
}

/************************ (C) COPYRIGHT XiLang CO. Ltd. ****** END OF FILE ****/

