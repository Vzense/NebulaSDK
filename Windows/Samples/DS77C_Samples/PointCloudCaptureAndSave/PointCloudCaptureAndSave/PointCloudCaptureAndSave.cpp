#include <thread>
#include <iostream>
#include "DS77C/VzenseDS77C_api.h"
#include <fstream>
#define frameSpace 20

using namespace std;

int main() {
	cout << "---PointCloudCaptureAndSave---\n";

	//about dev
	uint32_t deviceCount;
	VzDeviceInfo* pDeviceListInfo = NULL;
	VzDeviceHandle deviceHandle = 0;
	VzReturnStatus status = VzRetOthers;
	
	//about frame
	
	VzFrameReady FrameReady = { 0 };
	VzFrame depthFrame = { 0 };
	ofstream PointCloudWriter;

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
		if (Connected != pDeviceListInfo[0].status)
		{
			cout << "connect statu" << pDeviceListInfo[0].status << endl;
			cout << "Call VZ_OpenDevice with connect status :" << Connected << endl;
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
		return -1;
	}

	//Starts capturing the image stream
	status = VZ_StartStream(deviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VZ_StartStream failed status:" <<status<< endl;
		return -1;
	}

	//1.ReadNextFrame.
	//2.GetFrame acoording to Ready flag and Frametype.
	//3.save points.
	for (int i = 0; i < frameSpace; i++)
	{
		status = VZ_GetFrameReady(deviceHandle, 80, &FrameReady);
		if (status != VzReturnStatus::VzRetOK)
		{
			cout << "VZ_GetFrameReady failed status:" <<status<< endl;
			continue;
		}

		//depthFrame only.
		if (1 == FrameReady.depth) {
			status = VZ_GetFrame(deviceHandle, VzDepthFrame, &depthFrame);
			if (depthFrame.pFrameData != NULL)
			{
				// once save
				
				PointCloudWriter.open("PointCloud.txt");
				VzFrame &srcFrame = depthFrame;
				const int len = srcFrame.width * srcFrame.height;
				VzVector3f* worldV = new VzVector3f[len];

				VZ_ConvertDepthFrameToPointCloudVector(deviceHandle, &srcFrame, worldV); //Convert Depth frame to World vectors.

				for (int i = 0; i < len; i++)
				{
					PointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << std::endl;
				}
				delete[] worldV;
				worldV = NULL;
				std::cout << "Save point cloud successful in PointCloud.txt" << std::endl;
				PointCloudWriter.close();
				break;
			}
		}

	}


	//StoVz capturing the image stream
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
	cout << endl << "---end---";

	return 0;
}
