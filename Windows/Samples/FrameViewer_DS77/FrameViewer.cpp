#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "VzenseDS77_api.h"
#include <thread>

using namespace std;
using namespace cv;

PsDeviceInfo* g_pDeviceListInfo = NULL;
PsDeviceHandle g_DeviceHandle = 0;
Point g_Pos(320, 240);
int g_Slope = 4500;

bool InitDevice(const int deviceCount);
void ShowMenu();
static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg);

void HotPlugStateCallback(const PsDeviceInfo* pInfo, int state, void* pUserData);

void on_MouseHandle(int event, int x, int y, int flags, void * param)
{
	if (EVENT_RBUTTONDOWN == event)
	{
		g_Pos.x = x;
		g_Pos.y = y;
	}
}

int main(int argc, char *argv[])
{
	uint32_t deviceCount = 0;
	
	PsReturnStatus status = VZCT_Initialize();
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsInitialize failed!" << endl;
		system("pause");
		return -1;
	}

GET:
	status = VZCT_GetDeviceCount(&deviceCount);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsGetDeviceCount failed!" << endl;
		system("pause");
		return -1;
	}
	cout << "Get device count: " << deviceCount << endl;
	if (0 == deviceCount)
	{
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
	}

	InitDevice(deviceCount);

	ShowMenu();

	cv::Mat imageMat;
	const string irImageWindow = "IR Image";
	const string depthImageWindow = "Depth Image";
    const string rgbImageWindow = "RGB Image";
	cv::namedWindow(depthImageWindow, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(irImageWindow, cv::WINDOW_AUTOSIZE);
	setMouseCallback(depthImageWindow, on_MouseHandle, nullptr);
	setMouseCallback(irImageWindow, on_MouseHandle, nullptr);

	for (;;)
	{
		PsFrame depthFrame = { 0 };
		PsFrame irFrame = { 0 };
		PsFrame rgbFrame = { 0 };
		PsFrame transformedDepthFrame = { 0 };
		PsFrame transformedRgbFrame = { 0 };

		// Read one frame before call PsGetFrame
		PsFrameReady frameReady = {0};
		status = VZCT_ReadNextFrame(g_DeviceHandle, &frameReady);
		
		//Get depth frame, depth frame only output in following data mode
		if (1 == frameReady.depth)
		{
			status = VZCT_GetFrame(g_DeviceHandle, PsDepthFrame, &depthFrame);

			if (depthFrame.pFrameData != NULL)
			{
                static int index = 0;
                static float fps = 0;
                static int64 start = cv::getTickCount();

                int64 current = cv::getTickCount();
                int64 diff = current - start;
                index++;
                if (diff > cv::getTickFrequency())
                {
                    fps = index * cv::getTickFrequency() / diff;
                    index = 0;
                    start = current;
                }

				//Display the Depth Image
				Opencv_Depth(g_Slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);
                char text[30] = "";
                sprintf(text, "%.2f", fps);
                putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));
				cv::imshow(depthImageWindow, imageMat);
			}
			else
			{
				cout << "VZCT_GetFrame PsDepthFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}
		if (1 == frameReady.transformedDepth)
		{
			status = VZCT_GetFrame(g_DeviceHandle, PsTransformDepthImgToColorCameraFrame, &transformedDepthFrame);

			if (transformedDepthFrame.pFrameData != NULL)
			{
				static int index = 0;
				static float fps = 0;
				static int64 start = cv::getTickCount();

				int64 current = cv::getTickCount();
				int64 diff = current - start;
				index++;
				if (diff > cv::getTickFrequency())
				{
					fps = index * cv::getTickFrequency() / diff;
					index = 0;
					start = current;
				}

				//Display the Depth Image
				Opencv_Depth(g_Slope, transformedDepthFrame.height, transformedDepthFrame.width, transformedDepthFrame.pFrameData, imageMat);
				char text[30] = "";
				sprintf(text, "%.2f", fps);
				putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));
				cv::imshow("TransformedDepth", imageMat);
			}
			else
			{
				cout << "VZCT_GetFrame PsDepthFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

		//Get IR frame, IR frame only output in following data mode
		if (1 == frameReady.ir)
		{
			status = VZCT_GetFrame(g_DeviceHandle, PsIRFrame, &irFrame);

			if (irFrame.pFrameData != NULL)
			{
				//Display the IR Image
                char text[30] = "";
                imageMat = cv::Mat(irFrame.height, irFrame.width, CV_8UC1, irFrame.pFrameData);
                sprintf(text, "%d", imageMat.at<uint8_t>(g_Pos));

				Scalar color = Scalar(0, 0, 0);
                if (imageMat.at<uint8_t>(g_Pos) > 128)
                {
					color = Scalar(0, 0, 0);
                }
                else
                {
					color = Scalar(255, 255, 255);
                }

				circle(imageMat, g_Pos, 4, color, -1, 8, 0);
				putText(imageMat, text, g_Pos, FONT_HERSHEY_DUPLEX, 2, color);
				cv::imshow(irImageWindow, imageMat);
			}
			else
			{
				cout << "VZCT_GetFrame PsIRFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

        //Get RGB frame, RGB frame only output in following data mode
        if (1 == frameReady.color)
        {
            status = VZCT_GetFrame(g_DeviceHandle, PsColorFrame, &rgbFrame);

            if (rgbFrame.pFrameData != NULL)
            {

                static int index = 0;
                static float fps = 0;
                static int64 start = cv::getTickCount();

                int64 current = cv::getTickCount();
                int64 diff = current - start;
                index++;
                if (diff > cv::getTickFrequency())
                {
                    fps = index * cv::getTickFrequency() / diff;
                    index = 0;
                    start = current;
                }

                //Display the RGB Image
                imageMat = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);

                char text[30] = "";
                sprintf(text, "%.2f", fps);
                putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

                cv::imshow(rgbImageWindow, imageMat);
            }
            else
            {
                cout << "VZCT_GetFrame PsRGBFrame status:" << status << " pFrameData is NULL " << endl;
            }
        }
		if (1 == frameReady.transformedColor)
		{
			status = VZCT_GetFrame(g_DeviceHandle, PsTransformColorImgToDepthCameraFrame, &transformedRgbFrame);

			if (transformedRgbFrame.pFrameData != NULL)
			{

				static int index = 0;
				static float fps = 0;
				static int64 start = cv::getTickCount();

				int64 current = cv::getTickCount();
				int64 diff = current - start;
				index++;
				if (diff > cv::getTickFrequency())
				{
					fps = index * cv::getTickFrequency() / diff;
					index = 0;
					start = current;
				}

				//Display the RGB Image
				imageMat = cv::Mat(transformedRgbFrame.height, transformedRgbFrame.width, CV_8UC3, transformedRgbFrame.pFrameData);

				char text[30] = "";
				sprintf(text, "%.2f", fps);
				putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

				cv::imshow("TransformedColor", imageMat);
			}
			else
			{
				cout << "VZCT_GetFrame PsRGBFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

	KEY:
		unsigned char key = waitKey(1);
        if (key == 'R' || key == 'r')
        {
            cout << "please select RGB resolution to set: 0:640*480; 1:800*600; 2:1600*1200" << endl;
            int index = 0;
            cin >> index;
            if (cin.fail())
            {
                std::cout << "Unexpected input\n";
                cin.clear();
                cin.ignore(1024, '\n');
                continue;
            }
            else
            {
                cin.clear();
                cin.ignore(1024, '\n');
            }

            switch (index)
            {
            case 0:
                VZCT_SetColorResolution(g_DeviceHandle, PsColor_Resolution_640_480);
                break;
            case 1:
                VZCT_SetColorResolution(g_DeviceHandle, PsColor_Resolution_800_600);
                break;
            case 2:
                VZCT_SetColorResolution(g_DeviceHandle, PsColor_Resolution_1600_1200);
                break;
            default:
                cout << "input is invalid." << endl;
                break;
            }
            
        }
		else if (key == 'P' || key == 'p')
		{
			//Save the pointcloud
			if (depthFrame.pFrameData != NULL)
			{
                ofstream PointCloudWriter;
				PointCloudWriter.open("PointCloud.txt");
				PsFrame &srcFrame = depthFrame;
				const int len = srcFrame.width * srcFrame.height;
				PsVector3f* worldV = new PsVector3f[len];

				VZCT_ConvertDepthFrameToPointCloudVector(g_DeviceHandle, srcFrame, worldV); //Convert Depth frame to World vectors.

				for (int i = 0; i < len; i++)
				{ 
                    if (0 < worldV[i].z && worldV[i].z < g_Slope)
                    {
                        PointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << std::endl;
                    }
				}
				delete[] worldV;
				worldV = NULL;
				std::cout << "Save point cloud successful in PointCloud.txt" << std::endl;
				PointCloudWriter.close();
			}
			else
			{
				std::cout << "Current Depth Frame is NULL" << endl;
			}
		}
        else if (key == 'Q' || key == 'q')
        {
			static bool isTransformColorImgToDepthCameraEnabled = true;
			VZCT_SetTransformColorImgToDepthCameraEnabled(g_DeviceHandle, isTransformColorImgToDepthCameraEnabled);
			cout << "SetTransformColorImgToDepthCameraEnabled " << ((true == isTransformColorImgToDepthCameraEnabled) ? "enable" : "disable") << endl;
			isTransformColorImgToDepthCameraEnabled = !isTransformColorImgToDepthCameraEnabled;
        }
        else if (key == 'L' || key == 'l')
        {
			static bool isTransformDepthImgToColorCameraEnabled = true;
			VZCT_SetTransformDepthImgToColorCameraEnabled(g_DeviceHandle, isTransformDepthImgToColorCameraEnabled);
			cout << "SetTransformDepthImgToColorCameraEnabled " << ((true == isTransformDepthImgToColorCameraEnabled) ? "enable" : "disable") << endl;
			isTransformDepthImgToColorCameraEnabled = !isTransformDepthImgToColorCameraEnabled;
        }		
		else if (key == 27)	//ESC Pressed
		{
			break;
		}
	}

    status = VZCT_CloseDevice(&g_DeviceHandle);
    cout << "CloseDevice status: " << status << endl;

    status = VZCT_Shutdown();
    cout << "Shutdown status: " << status << endl;
	cv::destroyAllWindows();

	delete[] g_pDeviceListInfo;
	g_pDeviceListInfo = NULL;

    return 0;
}

bool InitDevice(const int deviceCount)
{
	VZCT_SetHotPlugStatusCallback(HotPlugStateCallback, nullptr);

	g_pDeviceListInfo = new PsDeviceInfo[deviceCount];
	PsReturnStatus status = VZCT_GetDeviceListInfo(g_pDeviceListInfo, deviceCount);
	g_DeviceHandle = 0;
	status = VZCT_OpenDevice(g_pDeviceListInfo[0].uri, &g_DeviceHandle);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "OpenDevice failed!" << endl;
		system("pause");
		return false;
	}
        
	PsCameraParameters cameraParameters;
	status = VZCT_GetCameraParameters(g_DeviceHandle, PsToFSensor, &cameraParameters);

	cout << "Get PsGetCameraParameters status: " << status << endl;
	cout << "Depth Camera Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "Depth Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "K4: " << cameraParameters.k4 << endl;
	cout << "K5: " << cameraParameters.k5 << endl;
	cout << "K6: " << cameraParameters.k6 << endl;

    const int BufLen = 63;
	char sn[BufLen] = {0};
	VZCT_GetSerialNumber(g_DeviceHandle, sn, BufLen);
	cout << "sn  ==  " << sn << endl;

	char fw[BufLen] = { 0 };
	VZCT_GetFirmwareVersionNumber(g_DeviceHandle, fw, BufLen);
	cout << "fw  ==  " << fw << endl;

 	return true;
}

void ShowMenu()
{
	cout << "\n--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "Press following key to set corresponding feature:" << endl;
    cout << "R/r: Change the RGB resolution: input corresponding index in terminal:" << endl;
    cout << "                             0: 640 * 480" << endl;
    cout << "                             1: 800 * 600" << endl;
    cout << "                             2: 1600 * 1200" << endl;
	cout << "P/p: Save point cloud data into PointCloud.txt in current directory" << endl;
    cout << "Q/q: Enables or disables transforms a color image into the geometry of the depth camera" << endl;
    cout << "L/l: Enables or disables transforms the depth map into the geometry of the color camera" << endl;
	cout << "Esc: Program quit " << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------\n" << endl;
}

static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	Point2d pointxy = g_Pos;
	int val = dispImg.at<ushort>(pointxy);
	char text[20];
#ifdef _WIN32
	sprintf_s(text, "%d", val);
#else
	snprintf(text, sizeof(text), "%d", val);
#endif
	dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
	applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
	int color;
	if (val > 2500)
		color = 0;
	else
		color = 4096;
	circle(dispImg, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
	putText(dispImg, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
}

void HotPlugStateCallback(const PsDeviceInfo* pInfo, int state, void* pUserData)
{
	cout << pInfo->uri<<" " << (state ==0? "add":"remove" )<<endl ;
}