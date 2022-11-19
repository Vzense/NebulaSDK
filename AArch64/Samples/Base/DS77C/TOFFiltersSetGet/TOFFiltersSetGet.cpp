#include <thread>
#include <iostream>
#include "VzenseNebula_api.h"
using namespace std;

int main() {
	cout << "--------------TOFFiltersSetGet-------------"<< endl;

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

	//The parameters of TimeFilter and ConfidenceFilter are stored in camera
	//The parameters of FlyingPixelFilter, FillHoleFilter and SpatialFilter are stored in SDK

	cout << endl << "-------------1------------ test TimeFilter --------------------------" << endl;

	VzTimeFilterParams TimeFilterParams = { true, 1 };
	status = VZ_GetTimeFilterParams(deviceHandle, &TimeFilterParams);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetTimeFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "The default TimeFilter switch is " << boolalpha << TimeFilterParams.enable << endl;

	TimeFilterParams.enable = !TimeFilterParams.enable;
 	status = VZ_SetTimeFilterParams(deviceHandle, TimeFilterParams);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetTimeFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "Set TimeFilter switch to " << boolalpha << TimeFilterParams.enable <<" is Ok."<< endl;

	cout << endl << "-------------2--------- test ConfidenceFilter -----------------------" << endl;

	VzConfidenceFilterParams confidenceFilterParams = { true, 15 };
	status = VZ_GetConfidenceFilterParams(deviceHandle, &confidenceFilterParams);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetConfidenceFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "The default ConfidenceFilter switch is " << boolalpha << confidenceFilterParams.enable << endl;
	
	confidenceFilterParams.enable = !confidenceFilterParams.enable;
	status = VZ_SetConfidenceFilterParams(deviceHandle, confidenceFilterParams);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetConfidenceFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "Set ConfidenceFilter switch to " << boolalpha << confidenceFilterParams.enable << " is Ok." << endl;


	cout << endl << "-------------3--------- test FlyingPixelFilter ----------------------" << endl;

	VzFlyingPixelFilterParams flyingPixelFilterParams = { true, 15 };
	status = VZ_GetFlyingPixelFilterParams(deviceHandle, &flyingPixelFilterParams);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetFlyingPixelFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "The default FlyingPixelFilter switch is " << boolalpha << flyingPixelFilterParams.enable << endl;
	
	flyingPixelFilterParams.enable = !flyingPixelFilterParams.enable;
	status = VZ_SetFlyingPixelFilterParams(deviceHandle,flyingPixelFilterParams);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetFlyingPixelFilterParams failed status:" << status << endl;
		return -1;
	}
	cout << "Set FlyingPixelFilter switch to " << boolalpha << flyingPixelFilterParams.enable << " is Ok." << endl;
	
	cout << endl << "-------------4---------- test FillHoleFilter ------------------------" << endl;

	bool bFillHoleFilter= true;
	status = VZ_GetFillHoleFilterEnabled(deviceHandle, &bFillHoleFilter);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetFillHoleFilterEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "The default FillHoleFilter switch is " << boolalpha << bFillHoleFilter << endl;
	
	bFillHoleFilter = !bFillHoleFilter;
	status = VZ_SetFillHoleFilterEnabled(deviceHandle, bFillHoleFilter);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetFillHoleFilterEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "Set FillHoleFilter switch to " << boolalpha << bFillHoleFilter << " is Ok." << endl;

	cout << endl << "-------------5---------- test SpatialFilter -------------------------" << endl;

	bool bSpatialFilter = true;
	status = VZ_GetSpatialFilterEnabled(deviceHandle, &bSpatialFilter);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_GetSpatialFilterEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "The default SpatialFilter switch is " << boolalpha << bSpatialFilter << endl;
	
	bSpatialFilter = !bSpatialFilter;
	status = VZ_SetSpatialFilterEnabled(deviceHandle, bSpatialFilter);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_SetSpatialFilterEnabled failed status:" << status << endl;
		return -1;
	}
	cout << "Set SpatialFilter switch to " << boolalpha << bSpatialFilter << " is Ok." << endl;

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

	cout << endl << "---Test end, please reboot camera to restore the default settings.----" << endl;
	return 0;
}
