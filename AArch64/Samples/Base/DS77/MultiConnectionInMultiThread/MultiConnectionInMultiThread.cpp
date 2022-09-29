﻿#include <thread>
#include <iostream>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "VzenseNebula_api.h"
#define frameSpace 10
using namespace std;

class Device
{
public:
	void TestDevice(VzDeviceInfo * pDeviceListInfo)
	{
        lock_guard<mutex> lk(mutex_);
		cout << "uri:" << pDeviceListInfo->uri << endl
			 << "alias:" << pDeviceListInfo->alias << endl
			 << "connectStatus:" << pDeviceListInfo->status << endl;

		VzReturnStatus status = VZ_OpenDeviceByUri(pDeviceListInfo->uri, &device_);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "OpenDevice "<<pDeviceListInfo->serialNumber<<" failed status:" << status << endl;
		}

		// Starts capturing the image stream
		status = VZ_StartStream(device_);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "VZ_StartStream "<<pDeviceListInfo->serialNumber<<" failed status:" << status << endl;
		}

        //Wait for the device to upload image data
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		// 1.ReadNextFrame.
		// 2.Get depth Frame acoording to Ready flag.
		for (int j = 0; j < frameSpace; j++)
		{
			VzFrame depthFrame = {0};
			VzFrameReady frameReady = {0};
			status = VZ_GetFrameReady(device_, 1200, &frameReady);

			if (status != VzRetOK)
			{
				cout << pDeviceListInfo->serialNumber << " VZ_GetFrameReady failed status:" << status << endl;
				continue;
			}

			// Get depth frame, depth frame only output in following data mode
			if (1 == frameReady.depth)
			{
				status = VZ_GetFrame(device_, VzDepthFrame, &depthFrame);

				if (status == VzRetOK && depthFrame.pFrameData != NULL)
				{
					cout << pDeviceListInfo->serialNumber << " frameIndex :" << depthFrame.frameIndex << endl;
				}
				else
				{
					cout << pDeviceListInfo->serialNumber << "VZ_GetFrame VzDepthFrame status:" << status << endl;
				}
			}
		}

		// 1.close device
		// 2.SDK shutdown
		status = VZ_StopStream(device_);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "VZ_StopStream failed status:" << status << endl;
		}
		status = VZ_CloseDevice(&device_);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "VZ_CloseDevice failed status:" << status << endl;
		}

        isTestDone_ = true;
        condition_.notify_one();

		return;
	}

    bool WaitForTestDone()
    {
        unique_lock<mutex> lk(mutex_);
        condition_.wait(lk, [this] {return isTestDone_;});
        return isTestDone_;
    }

private:
	VzDeviceHandle device_;
    bool isTestDone_ = false;
    mutex mutex_;
    condition_variable condition_;
};

int main() {
	cout << "---MultiConnection---"<< endl;

	//about dev
	uint32_t deviceCount = 0;
	VzReturnStatus status = VzRetOthers;
	VzDeviceHandle *deviceHandle = nullptr;
	VzDeviceInfo* pDeviceListInfo = nullptr;
	//about Frame
	
	VzFrameReady FrameReady = { 0 };
	VzFrame Depth = { 0 };

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
	if (deviceCount < 2)
	{
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
	}

	deviceHandle = new VzDeviceHandle[deviceCount];
	pDeviceListInfo = new VzDeviceInfo[deviceCount];


	status = VZ_GetDeviceInfoList(deviceCount, pDeviceListInfo);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "GetDeviceListInfo failed status:" << status << endl;
		delete[] pDeviceListInfo;
		delete[] deviceHandle;
		return -1;
	}
	else
	{
		if (VzConnected != pDeviceListInfo[0].status)
		{
			cout << "connect statu" << pDeviceListInfo[0].status << endl;
			cout << "Call VZ_OpenDevice with connect status :" << VzConnected << endl;
			delete[] pDeviceListInfo;
			delete[] deviceHandle;
			return -1;
		}
		Device* pDevice = new Device[deviceCount];

		for(uint32_t i = 0; i < deviceCount; i++)
		{
            thread(&Device::TestDevice, std::ref(pDevice[i]), &pDeviceListInfo[i]).detach();
		}

		for(uint32_t i = 0; i < deviceCount; i++)
		{
            pDevice[i].WaitForTestDone();
		}

		delete[] pDevice;
		delete[] pDeviceListInfo;
		delete[] deviceHandle;
	}

	status = VZ_Shutdown();
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_Shutdown failed status:" <<status<< endl;
		return -1;
	}

	cout << "--end--"<< endl;

	return 0;
}
