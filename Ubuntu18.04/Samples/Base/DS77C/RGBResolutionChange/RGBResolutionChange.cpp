﻿#include <thread>
#include <iostream>
#include "VzenseNebula_api.h"
#define frameSpace 10
using namespace std;

int main() {
	cout << "---RGBResolutionChange---"<< endl;

	//about dev
	uint32_t deviceCount;
	VzDeviceInfo* pDeviceListInfo = NULL;
	VzDeviceHandle deviceHandle = 0;
	VzReturnStatus status = VzRetOthers;
	
	//about frame
	
	VzFrameReady FrameReady = {0};
	VzFrame RGBFrame = { 0 };

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

	//switch RGBResolution
	int resolution_w = 640;
	int resolution_h = 480;
	//1.640_480
	status = VZ_SetColorResolution(deviceHandle, resolution_w, resolution_h);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetColorResolution failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "set to 640_480" << endl;
	}

    //Wait for the device to upload image data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	for (int i = 0; i < frameSpace; i++)
	{
		status = VZ_GetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "VZ_GetFrameReady failed status:" <<status<< endl;
			continue;
		}

		//cout resolution
		if (1 == FrameReady.color)
		{
			status = VZ_GetFrame(deviceHandle, VzColorFrame, &RGBFrame);
			if (status == VzReturnStatus::VzRetOK && RGBFrame.pFrameData != NULL
			&& RGBFrame.width==640 )
			{
				cout << "get Frame successful,status:" << status << "  "
					 << "resolution: "<< RGBFrame.width  <<"x"<<RGBFrame.height<< endl;	
			}
		}

	}

	resolution_w = 1600;
	resolution_h = 1200;
	//2.1600_1200
	status = VZ_SetColorResolution(deviceHandle, resolution_w, resolution_h);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetColorResolution failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "set to 1600_1200" << endl;
	}

    //Wait for the device to upload image data
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

	for (int i = 0; i < frameSpace; i++)
	{
		status = VZ_GetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "VZ_GetFrameReady failed status:" <<status<< endl;
			continue;
		}

		//cout resolution
		if (1 == FrameReady.color)
		{
			status = VZ_GetFrame(deviceHandle, VzColorFrame, &RGBFrame);
			if (status == VzReturnStatus::VzRetOK && RGBFrame.pFrameData != NULL
						&& RGBFrame.width==1600 )

			{
				cout << "get Frame successful,status:" << status << "  "
					<< "resolution: "<< RGBFrame.width  <<"x"<<RGBFrame.height<< endl;	
			}
		}

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
