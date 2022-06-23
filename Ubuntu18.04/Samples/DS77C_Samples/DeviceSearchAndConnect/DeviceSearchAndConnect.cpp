#include <thread>
#include <iostream>
#include "DS77C/VzenseDS77C_api.h"

using namespace std;

int main() {
	cout << "---DeviceSearchAndConnect---\n";

	uint32_t deviceCount;
	VzDeviceInfo* pDeviceInfo = NULL;
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

	pDeviceInfo = new VzDeviceInfo;
	uint32_t deviceIndex = 0;

	//VZ_GetDeviceInfo is similar to VZ_GetDeviceInfoList,but hasn't list.
	status = VZ_GetDeviceInfo(deviceIndex, pDeviceInfo);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "GetDeviceListInfo failed status:" <<status<< endl;
		return -1;
	}

	cout << "uri:" << pDeviceInfo->uri << endl
		<< "alias:" << pDeviceInfo->alias << endl
		<< "connectStatus:" << pDeviceInfo->status << endl;

	status = VZ_OpenDeviceByUri(pDeviceInfo->uri, &deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "OpenDevice failed status:" <<status << endl;
		return -1;
	}
	cout << "open device successful,status :" << status << endl;

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
	cout << "--end--";

	return 0;
}
