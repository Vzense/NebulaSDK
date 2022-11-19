#include <thread>
#include <iostream>
#include "VzenseNebula_api.h"
using namespace std;

int main() {
	cout << "---- TOFExposureTimeSetGet ----"<< endl;

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
	//2.Get infomation of the devices. 
	//3.Open devices accroding to the info.
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

    cout << "Open device successful,status :" << status << endl;

	//Starts capturing the image stream
	status = VZ_StartStream(deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_StartStream failed status:" <<status<< endl;
		return -1;
	}

	//1.Default FrameRate
	//2.Set new ExposureTime
	//3.Change FrameRate to 15
	//4.Set new ExposureTime

	cout << endl << "---- Default FrameRate ----" << endl;
	//Set Control mode to manual
	status = VZ_SetExposureControlMode(deviceHandle, VzToFSensor, VzExposureControlMode_Manual);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetExposureControlMode failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "Set control mode to manual." << endl;
	}

	//Get default frame rate
	int defaultframeRate = 20; 
	status = VZ_GetFrameRate(deviceHandle, &defaultframeRate);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetFrameRate failed status:" << status << endl;
		return -1;
	}
	cout << "Get dfault frame rate: " << defaultframeRate  << endl;

	//Get the range of the TOF exposure time 
	VzExposureTimeParams maxExposureTime = { VzExposureControlMode_Manual,0 };
	status = VZ_GetProperty(deviceHandle, "Py_ToFExposureTimeMax", &maxExposureTime, sizeof(maxExposureTime));
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetProperty get Py_ToFExposureTimeMax failed status:" << status << endl;
		return -1;
	}
	cout <<"Recommended scope: 58 - " << maxExposureTime.exposureTime << endl;

	//Set new ExposureTime
	VzExposureTimeParams params = {VzExposureControlMode_Manual, 400};
	status = VZ_SetExposureTime(deviceHandle, VzToFSensor, params);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetExposureTime failed status:" <<status<< endl;
		return -1;
	}
	else
	{
		cout << "Set exposure time "<< params.exposureTime << " is OK." << endl;
	}

	cout << endl << "---- Set FrameRate to 15 ----" << endl;
	//Set new FrameRate
	int frameRate = 15;  //New frame rate, can change it
	status = VZ_SetFrameRate(deviceHandle, frameRate);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetFrameRate set new FrameRate failed status:" << status << endl;
		return -1;
	}
	cout << "Set frame rate " << frameRate << " is OK." << endl;

	//Need to get new ExposureTime Range due to FrameRate change.
	params.exposureTime = 0;
	status = VZ_GetProperty(deviceHandle, "Py_ToFExposureTimeMax", &params, sizeof(params));
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetProperty get Py_ToFExposureTimeMax failed status:" << status << endl;
		return -1;
	}
	cout << "Recommended scope: 58 - " << params.exposureTime << endl;

	//Set new ExposureTime 500	
	params.exposureTime = 500;
 	status = VZ_SetExposureTime(deviceHandle, VzToFSensor, params);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetExposureTime failed status:" << status << endl;
		return -1;
	}
	else
	{
		cout << "Set exposure time " << params.exposureTime << " is OK." << endl;
	}

	//Stop capturing the image stream
	status = VZ_StopStream(deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_StopStream failed status:" <<status<< endl;
		return -1;
	}

	//1.Close device
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

	cout << endl << "--- Test end, please reboot camera to restore the default settings ---" << endl;
	return 0;
}
