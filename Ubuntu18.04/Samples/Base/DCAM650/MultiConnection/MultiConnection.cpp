#include <thread>
#include <iostream>
#include "VzenseNebula_api.h"
#define frameSpace 10
using namespace std;

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
	for (int i = 0; i < deviceCount; i++)
	{
		cout << "CameraIndex: " << i << endl;
		cout << "uri:" << pDeviceListInfo[i].uri << endl
			<< "alias:" << pDeviceListInfo[i].alias << endl
			<< "connectStatus:" << pDeviceListInfo[i].status << endl;
	}
	for(int i = 0;i < deviceCount; i++)
	{
		status = VZ_OpenDeviceByUri(pDeviceListInfo[i].uri, &deviceHandle[i]);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "OpenDevice failed status:" <<status << endl;
			return -1;
		}
	}

    for (int i = 0; i < deviceCount; i++)
    {
        //Starts capturing the image stream
        status = VZ_StartStream(deviceHandle[i]);
        if (status != VzReturnStatus::VzRetOK)
        {
            cout << "VZ_StartStream failed status:" << status << endl;
            return -1;
        }
    }

    //Wait for the device to upload image data
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	//1.ReadNextFrame.
	//2.Get depth Frame acoording to Ready flag.
	for (int j = 0; j < frameSpace; j++)
	{
		for (int i = 0; i < deviceCount; i++)
		{
			if (deviceHandle[i])
			{
				VzFrame depthFrame = { 0 };
				VzFrameReady frameReady = { 0 };
				status = VZ_GetFrameReady(deviceHandle[i], 1200, &frameReady);

				if (status != VzRetOK)
				{
					cout << pDeviceListInfo[i].alias <<"  VZ_GetFrameReady failed status:" << status << endl;
					continue;
				}

				//Get depth frame, depth frame only output in following data mode
				if (1 == frameReady.depth)
				{
					status = VZ_GetFrame(deviceHandle[i], VzDepthFrame, &depthFrame);

					if (status == VzRetOK && depthFrame.pFrameData != NULL)
					{
						cout << pDeviceListInfo[i].alias << " frameIndex :" << depthFrame.frameIndex << endl;
					}
					else
					{
						cout << pDeviceListInfo[i].alias << "VZ_GetFrame VzDepthFrame status:" << status  << endl;
					}
				}
			}

		}
	}
 
	//1.close device
	//2.SDK shutdown
	for (int i = 0; i < deviceCount; i++)
	{
		status = VZ_StopStream(deviceHandle[i]);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "VZ_StopStream failed status:" <<status<< endl;
		}
		status = VZ_CloseDevice(&deviceHandle[i]);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "VZ_CloseDevice failed status:" <<status<< endl;
		}
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
