#include "Global.h"
#include "Device.h"
#include "MainEntry.h"
#include "usart.h"
#include "stdio.h"
#include "MG354.h"
//#include "dvc_IMU100N.h"
//#include "dvc_ADIS16495.h"

extern uint8_t RecFlag;
extern unsigned char f10ms;
extern MG354_HandleTypeDef MG354;
extern uint8_t RecFlagSPI;

void HeartPulse(void);

//extern ADI_16495_HandleTypeDef g_hImu16495;

unsigned short FramLength=0;

extern uint8_t crcOK;

void MainEntry(void)
{
	InitDevice();
	//InitMessage();
	
    while (1)
    {
		HeartPulse();
		//ParseFrameUsart3();
		//ParseFrameUsart5();
		//ParseFrameUsart5DMA();
		if (f10ms==1)
		{
			f10ms = 0;
//    normal
//		Read_IMU_SPI(&MG354);
//		SampleAndSendImuData();
//    burst
			if(RecFlagSPI==1)
			{
				Read_IMU_SPI_Burst(&MG354);
				SampleAndSendImuData();
			}
		}
//		if(RecFlag==1)
//		{
//			RecFlag=0;
//			SampleAndSendImuData();
//		}
    }
}


void HeartPulse()
{
	static long counter = 0;
	counter++;
	if (counter>1000000)
	{
		counter = 0;
		LED_Toggle(LED1);
	}
}
