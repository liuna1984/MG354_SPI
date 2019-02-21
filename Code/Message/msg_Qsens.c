#include "msg_Qsens.h"
#include "ParseFrame.h"
#include "Message.h"

#if defined(MSG_QSENS_SEND_EPH) || defined(MSG_QSENS_RECIEVE_EPH)
GNSS_EphemDATA g_Eph;
#endif
#if defined(MSG_QSENS_SEND_GPS) || defined(MSG_QSENS_RECIEVE_GPS)
MSG_QSENS_DatagramGpsTypeDef g_GpsData;
#endif
#if defined(MSG_QSENS_SEND_NAV) || defined(MSG_QSENS_RECIEVE_NAV)
MSG_QSENS_DatagramNavTypeDef g_stNavResult;
#endif
#if defined(MSG_QSENS_SEND_OBS) || defined(MSG_QSENS_RECIEVE_OBS)
GNSS_EphemDATA g_Eph;
#endif
#if defined(MSG_QSENS_SEND_IMU) || defined(MSG_QSENS_RECIEVE_IMU) || defined(MSG_QSENS_SEND_IMU_FULL) || defined(MSG_QSENS_RECIEVE_IMU_FULL)
MSG_QSENS_DatagramImuTypeDef g_Imu;
#endif
#if defined(MSG_QSENS_SEND_MAG) || defined(MSG_QSENS_RECIEVE_MAG)
MSG_QSENS_DatagramMagTypeDef g_Eph;
#endif
#if defined(MSG_QSENS_SEND_ALT) || defined(MSG_QSENS_RECIEVE_ALT)
MSG_QSENS_DatagramAltTypeDef g_Eph;
#endif
#if defined(MSG_QSENS_SEND_DR) || defined(MSG_QSENS_RECIEVE_DR)
MSG_QSENS_DatagramDrTypeDef g_Eph;
#endif
FrameConstructor frameConstr;

int QSENS_BinInit(unsigned char* pBuffer, int nLength)
{
#if defined(MSG_QSENS_SEND_EPH) || defined(MSG_QSENS_RECIEVE_EPH)
	memset(&g_Eph, 0x00, sizeof(GNSS_EphemDATA));
#endif
#if defined(MSG_QSENS_SEND_GPS) || defined(MSG_QSENS_RECIEVE_GPS)
	memset(&g_GpsData, 0x00, sizeof(MSG_QSENS_DatagramGpsTypeDef));
#endif
#if defined(MSG_QSENS_SEND_NAV) || defined(MSG_QSENS_RECIEVE_NAV)
	memset(&g_stNavResult, 0x00, sizeof(MSG_QSENS_DatagramNavTypeDef));
#endif
#if defined(MSG_QSENS_SEND_OBS) || defined(MSG_QSENS_RECIEVE_OBS)
	memset(&g_Eph, 0x00, sizeof(GNSS_EphemDATA));
#endif
#if defined(MSG_QSENS_SEND_IMU) || defined(MSG_QSENS_RECIEVE_IMU) || defined(MSG_QSENS_SEND_IMU_FULL) || defined(MSG_QSENS_RECIEVE_IMU_FULL)
	memset(&g_Imu, 0x00, sizeof(MSG_QSENS_DatagramImuTypeDef));
#endif
#if defined(MSG_QSENS_SEND_MAG) || defined(MSG_QSENS_RECIEVE_MAG)
	memset(&g_Eph, 0x00, sizeof(MSG_QSENS_DatagramMagTypeDef));
#endif
#if defined(MSG_QSENS_SEND_ALT) || defined(MSG_QSENS_RECIEVE_ALT)
	memset(&g_Eph, 0x00, sizeof(MSG_QSENS_DatagramAltTypeDef));
#endif
#if defined(MSG_QSENS_SEND_DR) || defined(MSG_QSENS_RECIEVE_DR)
	memset(&g_Eph, 0x00, sizeof(MSG_QSENS_DatagramDrTypeDef));
#endif
	memset(&frameConstr, 0x00, sizeof(FrameConstructor));
    return 1;
}
int MG354_InitParse(unsigned char* pBuffer, int nLength)
{
	return 1;
}
int MG354_BinFrameLength(unsigned char* pBuffer, int nLength)
{
 	return 0;
}
uint16_t CheckSum16N(uint8_t* pBuf,uint16_t length)
{
	int i;
	uint16_t Checksum=0;
	uint16_t Checkdata[length/2];
	
	for(i=0;i<length/2;i++)
	{
		Checkdata[i]=(pBuf[2*i]<<8)|(pBuf[2*i+1]);
		Checksum=Checksum+Checkdata[i];
	}
	return Checksum;
}
uint8_t RecFlag=0;
int MG354_BinFrameCheckCRC32(unsigned char* pBuffer, int nLength)
{
    uint16_t crcRec,crcRes;
	crcRec=pBuffer[nLength-1]|(pBuffer[nLength-2]<<8);
    crcRes=CheckSum16N(&pBuffer[1],nLength-3);
	if(crcRec==crcRes)
	{
		RecFlag=1;
		return 1;
	}
	else 
	{
		RecFlag=0;
		return 0;
	}
	
}
#define USART3_RXBUFF_SIZE 100
uint8_t usart3_rxbuf[USART3_RXBUFF_SIZE] = {0};
int MG354_BinFrameParseUM(unsigned char* pBuffer, int nLength)
{
	int i;
	for(i=0;i<nLength;i++)
	{
		usart3_rxbuf[i]=pBuffer[i];
	}
	return 1;
}

int QSENS_BinFrameLength(unsigned char* pBuffer, int nLength)
{
	unsigned short nFrameLength = 0x00;
	memcpy(&nFrameLength, &pBuffer[4], 2);
	return nFrameLength+3;
}

int QSENS_BinFrameCheck(unsigned char* pBuffer, int nLength)
{
    int i;
    unsigned char crc = 0x00;
	for (i=2;i<nLength-1;i++)
	{
		crc += pBuffer[i];
	}
	if (crc==pBuffer[nLength-1])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

int QSENS_BinFrameParse(unsigned char* pBuffer, int nLength)
{
    unsigned short FrameID;
	int nCounter;

    memcpy(&FrameID, &pBuffer[2], 2);
	nCounter = 6;
    switch (FrameID)
    {
#ifdef MSG_QSENS_RECIEVE_NAV 
	case MSG_QSENS_DATAGRAM_NAV:
		break;
#endif
#ifdef MSG_QSENS_RECIEVE_IMU_FULL
	case MSG_QSENS_DATAGRAM_IMU_FULL:
		g_Imu.counter.current = GetUShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.ulFullGyroXCurrent = GetULongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.ulFullGyroYCurrent = GetULongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.ulFullGyroZCurrent = GetULongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.ulFullAccXCurrent = GetULongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.ulFullAccYCurrent = GetULongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.ulFullAccZCurrent = GetULongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.TempretureGyroX = GetShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		if (g_Imu.counter.first == 1)
		{
				// 计算陀螺加表增量并赋值
			g_Imu.data.ulGyroX = g_Imu.data.ulFullGyroXCurrent - g_Imu.data.ulFullGyroXLast;
			g_Imu.data.ulGyroY = g_Imu.data.ulFullGyroYCurrent - g_Imu.data.ulFullGyroYLast;
			g_Imu.data.ulGyroZ = g_Imu.data.ulFullGyroXCurrent - g_Imu.data.ulFullGyroZLast;
			g_Imu.data.ulAccX = g_Imu.data.ulFullAccXCurrent - g_Imu.data.ulFullAccXLast;
			g_Imu.data.ulAccY = g_Imu.data.ulFullAccYCurrent - g_Imu.data.ulFullAccYLast;
			g_Imu.data.ulAccZ = g_Imu.data.ulFullAccZCurrent - g_Imu.data.ulFullAccZLast;
			g_Imu.data.ulFullGyroXCurrent = g_Imu.data.ulFullGyroXLast;
			g_Imu.data.ulFullGyroYCurrent = g_Imu.data.ulFullGyroYLast;
			g_Imu.data.ulFullGyroXCurrent = g_Imu.data.ulFullGyroZLast;
			g_Imu.data.ulFullAccXCurrent = g_Imu.data.ulFullAccXLast;
			g_Imu.data.ulFullAccYCurrent = g_Imu.data.ulFullAccYLast;
			g_Imu.data.ulFullAccZCurrent = g_Imu.data.ulFullAccZLast;
				// 收到有效数据，置标志位
			g_Imu.recive.Status = MESSAGE_STATUS_AVALABEL;
		}
		else
		{
			g_Imu.data.ulFullGyroXCurrent = g_Imu.data.ulFullGyroXLast;
			g_Imu.data.ulFullGyroYCurrent = g_Imu.data.ulFullGyroYLast;
			g_Imu.data.ulFullGyroXCurrent = g_Imu.data.ulFullGyroZLast;
			g_Imu.data.ulFullAccXCurrent = g_Imu.data.ulFullAccXLast;
			g_Imu.data.ulFullAccYCurrent = g_Imu.data.ulFullAccYLast;
			g_Imu.data.ulFullAccZCurrent = g_Imu.data.ulFullAccZLast;
			g_Imu.recive.Status = MESSAGE_STATUS_INIT;
		}
		CalculateDeltaCounterUint16(&g_Imu.counter);
		break;
#endif
#ifdef MSG_QSENS_RECIEVE_IMU
	case MSG_QSENS_DATAGRAM_IMU:
		g_Imu.counter.current = GetUShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		CalculateDeltaCounterUint16(&g_Imu.counter);
		g_Imu.data.lGyroX = GetLongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.lGyroY = GetLongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.lGyroZ = GetLongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.lAccX = GetLongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.lAccY = GetLongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_Imu.data.lAccZ = GetLongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);	
		break;
#endif
#ifdef MSG_QSENS_RECIEVE_MAG 
	case MSG_QSENS_DATAGRAM_MAG:
		break;
#endif
#ifdef MSG_QSENS_RECIEVE_ALT 
	case MSG_QSENS_DATAGRAM_ALT:
		break;
#endif
#ifdef MSG_QSENS_RECIEVE_DR 
	case MSG_QSENS_DATAGRAM_DR:
		break;
#endif

#ifdef MSG_QSENS_RECIEVE_GPS
	case MSG_QSENS_DATAGRAM_GPS:
		g_GpsData.counter.current = GetUShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		CalculateDeltaCounterUint16(&g_GpsData.counter);
		g_GpsData.data.ucGGA_Status = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucSvNum = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucBaseAge = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucGPGGA_UpdateFlag = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucGPRMC_UpdateFlag = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucPSAT_UpdateFlag = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucYear = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucMonth = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucDay = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucHour = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.ucMinite = pBuffer[nCounter];
		nCounter++;
		g_GpsData.data.fSecond = GetFloatContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.lLongitude_GPS = GetLongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.lLatitude_GPS = GetLongContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.fHightSea_GPS = GetFloatContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.fHightGeo_GPS = GetFloatContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.fCourse = GetFloatContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.fRate = GetFloatContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.fMagYaw = GetFloatContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.fDoubleYaw = GetFloatContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.fHDOP = GetFloatContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.us1PPS_Counter = GetUShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.usGPGGA_Counter = GetUShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.usGPRMC_Counter = GetUShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.usPSAT_Counter = GetUShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.data.usRecieverStatus = GetUShortContinue((unsigned char*)&pBuffer[nCounter], &nCounter);
		g_GpsData.recive.Status = MESSAGE_STATUS_AVALABEL;
		break;
#endif

#ifdef MSG_QSENS_RECIEVE_EPH
	case MSG_QSENS_DATAGRAM_EPH:
		break;	
#endif

#ifdef MSG_QSENS_RECIEVE_OBS 
	case MSG_QSENS_DATAGRAM_OBS:
		break;
#endif

	default:
		break;
	}
	return 1;
}

#ifdef MSG_QSENS_SEND_GPS
MSG_QSENS_DatagramGpsTypeDef* QSENS_GetGpsData()
{
	return &g_GpsData;
}

void QSENS_MakeGpsFrame(MSG_QSENS_DatagramGpsTypeDef* gps, DoubleBuffer *buf)
{
	static unsigned short usFrameCounter = 0;
	FrameBegin(buf, &frameConstr);
	PushDoubleBuffer(0xEA, buf);
	PushDoubleBuffer(0x51, buf);
	CheckSectionBegin(buf, &frameConstr);
	LengthSectionBegin(buf, &frameConstr);
	PushWordDoubleBuffer(MSG_QSENS_DATAGRAM_GPS, buf);
	MarkLengthPosition(buf, &frameConstr, 2);
	PUSH_VAR_DOUBLEBUFFER(usFrameCounter, buf);
	usFrameCounter++;
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucGGA_Status, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucSvNum, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucBaseAge, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucGPGGA_UpdateFlag, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucGPRMC_UpdateFlag, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucPSAT_UpdateFlag, buf);
    PUSH_VAR_DOUBLEBUFFER(gps->data.ucYear, buf);
    PUSH_VAR_DOUBLEBUFFER(gps->data.ucMonth, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucDay, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucHour, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.ucMinite, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.fSecond, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.lLongitude_GPS, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.lLatitude_GPS, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.fHightSea_GPS, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.fHightGeo_GPS, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.fCourse, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.fRate, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.fMagYaw, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.fDoubleYaw, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.fHDOP, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.us1PPS_Counter, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.usGPGGA_Counter, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.usGPRMC_Counter, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.usPSAT_Counter, buf);
	PUSH_VAR_DOUBLEBUFFER(gps->data.usRecieverStatus, buf);
	LengthSectionEnd(buf, &frameConstr);
	CheckSectionEnd(buf, &frameConstr);
	FrameEnd(buf, &frameConstr);
}   
#endif

#ifdef MSG_QSENS_SEND_NAV 
MSG_QSENS_DatagramNavTypeDef* QSENS_GetNavData(void)
{
	return &g_stNavResult;
}

void QSENS_MakeNavResultFrame(MSG_QSENS_DatagramNavTypeDef* nav, DoubleBuffer *buf)
{
	static unsigned short usFrameCounter = 0;
	FrameBegin(buf, &frameConstr);
	PushDoubleBuffer(0xEA, buf);
	PushDoubleBuffer(0x51, buf);
	CheckSectionBegin(buf, &frameConstr);
	LengthSectionBegin(buf, &frameConstr);
	PushWordDoubleBuffer(MSG_QSENS_DATAGRAM_NAV, buf);
	MarkLengthPosition(buf, &frameConstr, 2);
	PUSH_VAR_DOUBLEBUFFER(usFrameCounter, buf);
	usFrameCounter++;
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.dLongitude, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.dLatitude, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.fHight, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.fVe, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.fVn, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.fVv, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.fYaw, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.fPitch, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.fRoll, buf);
	PUSH_VAR_DOUBLEBUFFER(g_stNavResult.data.uStatus, buf);
	LengthSectionEnd(buf, &frameConstr);
	CheckSectionEnd(buf, &frameConstr);
	FrameEnd(buf, &frameConstr);
}
#endif

#ifdef MSG_QSENS_SEND_IMU
void QSENS_MakeImuFrame(MSG_QSENS_DatagramImuTypeDef* imu, DoubleBuffer *buf)
{
	static unsigned short usFrameCounter = 0;
	FrameBegin(buf, &frameConstr);
	PushDoubleBuffer(0xEA, buf);
	PushDoubleBuffer(0x51, buf);
	CheckSectionBegin(buf, &frameConstr);
	LengthSectionBegin(buf, &frameConstr);
	PushWordDoubleBuffer(MSG_QSENS_DATAGRAM_IMU, buf);
	MarkLengthPosition(buf, &frameConstr, 2);
	PUSH_VAR_DOUBLEBUFFER(usFrameCounter, buf);
	usFrameCounter++;
	PUSH_VAR_DOUBLEBUFFER(imu->data.lGyroX, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.lGyroY, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.lGyroZ, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.lAccX, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.lAccY, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.lAccZ, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.sTempretureGyroX, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.sTempretureGyroY, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.sTempretureGyroZ, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.sTempretureAccX, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.sTempretureAccY, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.sTempretureAccZ, buf);
	LengthSectionEnd(buf, &frameConstr);
	CheckSectionEnd(buf, &frameConstr);
	FrameEnd(buf, &frameConstr);
}
#endif

#ifdef MSG_QSENS_SEND_IMU_FULL
void QSENS_MakeImuFullFrame(MSG_QSENS_DatagramImuTypeDef* imu, DoubleBuffer *buf)
{
	FrameBegin(buf, &frameConstr);
	PushDoubleBuffer(0xEA, buf);
	PushDoubleBuffer(0x51, buf);
	CheckSectionBegin(buf, &frameConstr);
	PushWordDoubleBuffer(MSG_QSENS_DATAGRAM_IMU, buf);
	MarkLengthPosition(buf, &frameConstr, 2);
	LengthSectionBegin(buf, &frameConstr);
	imu->data.ulFullGyroX += imu->data.ulGyroX;
	imu->data.ulFullGyroY += imu->data.ulGyroY;
	imu->data.ulFullGyroZ += imu->data.ulGyroZ;
	imu->data.ulFullAccX += imu->data.ulAccX;
	imu->data.ulFullAccY += imu->data.ulAccY;
	imu->data.ulFullAccZ += imu->data.ulAccZ;
	PUSH_VAR_DOUBLEBUFFER(imu->data.ulFullGyroX, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.ulFullGyroY, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.ulFullGyroZ, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.ulFullAccX, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.ulFullAccY, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.ulFullAccZ, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.TempretureGyroX, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.TempretureGyroY, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.TempretureGyroZ, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.TempretureAccX, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.TempretureAccY, buf);
	PUSH_VAR_DOUBLEBUFFER(imu->data.TempretureAccZ, buf);
	LengthSectionEnd(buf, &frameConstr);
	CheckSectionEnd(buf, &frameConstr);
	FrameEnd(buf, &frameConstr);
}
#endif

#ifdef MSG_QSENS_SEND_MAG 
#endif

#ifdef MSG_QSENS_SEND_ALT 
#endif

#ifdef MSG_QSENS_SEND_DR 
#endif

#ifdef MSG_QSENS_SEND_GPS
#endif

#ifdef MSG_QSENS_SEND_EPH
#endif

#ifdef MSG_QSENS_SEND_OBS 
#endif




