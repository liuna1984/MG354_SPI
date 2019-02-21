#ifndef HARDWARE_H_201705031049
#define HARDWARE_H_201705031049

#include "Global.h"
#include "dvc_Led.h"
#include "msg_Qsens.h"
//#include "dvc_ADI16470.h"

void InitDevice(void);
void SampleImuDataMG354(MSG_QSENS_DatagramImuTypeDef* imu);
#endif
