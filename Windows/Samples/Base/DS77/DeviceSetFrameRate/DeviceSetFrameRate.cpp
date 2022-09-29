#include <thread>
#include <iostream>
#include <thread>
#include "VzenseNebula_api.h"

using namespace std;

time_t GetTimeStampMS();

int main() {
	cout << "---DeviceSetFrameRate---"<< endl;

	//about dev
	uint32_t deviceCount;
	VzDeviceInfo* pDeviceListInfo = NULL;
	VzDeviceHandle deviceHandle = 0;
	VzReturnStatus status = VzRetOthers;

	//SDK Initialize
	status = VZ_Initialize();
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VzInitialize failed status:" << status << endl;
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

	status = VZ_OpenDeviceByUri(pDeviceListInfo[0].uri, &deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "OpenDevice failed status:" << status << endl;
		return -1;
	}
    cout << "open device successful,status :" << status << endl;

	//Starts capturing the image stream
	status = VZ_StartStream(deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_StartStream failed status:" << status << endl;
		return -1;
	}

	int frameRate = 15;
	status = VZ_SetFrameRate(deviceHandle, frameRate);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetFrameRate failed status:" << status << endl;
		return -1;
	}
	cout << "set frame rate :" << frameRate <<" is OK."<< endl;

    //Wait for the device to upload image data
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    cout << "Start testing the average frame rate for 30 seconds, Please wait patiently" << endl;
	//statistical frame rate
	for (;;)
	{
		VzFrameReady FrameReady = {};
		status = VZ_GetFrameReady(deviceHandle, 1200, &FrameReady);
		if (status != VzReturnStatus::VzRetOK)
		{ 
			cout << "VZ_GetFrameReady failed status:" << status << endl;
			continue;
		}

		if (1 == FrameReady.depth) 
		{
			VzFrame depthFrame = {};
			status = VZ_GetFrame(deviceHandle, VzDepthFrame, &depthFrame);
			if (depthFrame.pFrameData != NULL)
			{
                const int TESTPERIOD = 30;//30 senconds
				static int index = 0;
                static time_t start = GetTimeStampMS();

				time_t diff = GetTimeStampMS() - start;
                index++;
                if (diff > (TESTPERIOD * 1000))
                {
                    float fps = (index * TESTPERIOD * 1000.0f / diff) / TESTPERIOD;
                    index = 0;
					cout << fps << endl;
					break;
                }
			}
		}

	}

	status = VZ_StopStream(deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_StopStream failed status:" << status << endl;
		return -1;
	}
	//1.close device
	//2.SDK shutdown
	status = VZ_CloseDevice(&deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_CloseDevice failed status:" << status << endl;
		return -1;
	}
	status = VZ_Shutdown();
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_Shutdown failed status:" << status << endl;
		return -1;
	}
	cout << "---end---"<< endl;

	return 0;
}

time_t GetTimeStampMS()
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    time_t timestamp = tp.time_since_epoch().count();
    return timestamp;
}