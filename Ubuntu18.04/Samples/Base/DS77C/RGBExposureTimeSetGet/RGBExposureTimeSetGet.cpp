﻿#include <thread>
#include <iostream>
#include "VzenseNebula_api.h"
#define frameSpace 10
using namespace std;

int main() {
	cout << "---RGBExposureTimeSetGet---"<< endl;

	//about dev
	uint32_t deviceCount;
	VzDeviceInfo* pDeviceListInfo = NULL;
	VzDeviceHandle deviceHandle = 0;
	VzReturnStatus status = VzRetOthers;

	//SDK Initialize
	status = VZ_Initialize();
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VzInitialize failed status:" <<status << endl;
		system("pause");
		return -1;
	}

	//1.Search and notice the count of devices.
	//2.get infomation of the devices. 
	//3.open devices accroding to the info.
GET:
	status = VZ_GetDeviceCount(&deviceCount);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VzGetDeviceCount failed! make sure the DCAM is connected" << endl;
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
	}
	cout << "Get device count: " << deviceCount << endl;
	if (0 == deviceCount)
	{
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
	}

	pDeviceListInfo = new VzDeviceInfo[deviceCount];
	status = VZ_GetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "GetDeviceListInfo failed status:" << status << endl;
		return -1;
	}
	else
	{
		if (VzConnected != pDeviceListInfo[0].status)
		{
			cout << "connect statu" << pDeviceListInfo[0].status << endl;
			cout << "Call VZ_OpenDevice with connect status :" << VzConnected << endl;
			return -1;
		}
	}

	cout << "uri:" << pDeviceListInfo[0].uri << endl
	<< "alias:" << pDeviceListInfo[0].alias << endl
	<< "ip:" << pDeviceListInfo[0].ip << endl
	<< "connectStatus:" << pDeviceListInfo[0].status << endl;

	status = VZ_OpenDeviceByUri(pDeviceListInfo[0].uri, &deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "OpenDevice failed status:" <<status << endl;
		return false;
	}

    cout << "open device successful,status :" << status << endl;

	//Starts capturing the image stream
	status = VZ_StartStream(deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_StartStream failed status:" <<status<< endl;
		return -1;
	}

	//switch exposure mode to manual
	status = VZ_SetExposureControlMode(deviceHandle, VzColorSensor, VzExposureControlMode_Manual);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetExposureControlMode failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "switch to manual" << endl;
	}

	VzExposureTimeParams params = {VzExposureControlMode_Manual, 4000};
	status = VZ_SetExposureTime(deviceHandle, VzColorSensor, params);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetExposureTime failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "SetExposureTime:"<< params.exposureTime << endl;
	}

	params.exposureTime = 0;
	status = VZ_GetExposureTime(deviceHandle, VzColorSensor, &params);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetExposureTime failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "GetExposureTime:"<< params.exposureTime << endl;
	}

	//switch exposure mode to auto
	status = VZ_SetExposureControlMode(deviceHandle, VzColorSensor, VzExposureControlMode_Auto);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetExposureControlMode failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "switch to auto" << endl;
	}

	//Stop capturing the image stream
	status = VZ_StopStream(deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_StopStream failed status:" <<status<< endl;
		return -1;
	}

	//1.close device
	//2.SDK shutdown
	status = VZ_CloseDevice(&deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_CloseDevice failed status:" <<status<< endl;
		return -1;
	}
	status = VZ_Shutdown();
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_Shutdown failed status:" <<status<< endl;
		return -1;
	}
	cout<< "---end---"<< endl;

	return 0;
}