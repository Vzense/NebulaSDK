#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "VzenseNebula_api.h"
#include <thread>

using namespace std;
using namespace cv;

VzDeviceInfo* g_pDeviceListInfo = NULL;
VzDeviceHandle g_DeviceHandle = 0;
Point g_Pos(320, 240);
Point g_TransPos(800, 600);
int g_Slope = 7495;
bool g_IsSavePointCloud = false;

static const cv::Point2i TransformedDepthPoint[4] = {{160, 120}, {480, 120}, {160, 360},{480, 360}};

bool InitDevice(const int deviceCount);
void ShowMenu();
static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg, Point point);
void HotPlugStateCallback(const VzDeviceInfo* pInfo, int state, void* pUserData);
void SavePointCloud(const char* pFileName, const VzFrame& depthFrame);

void on_MouseHandle(int event, int x, int y, int flags, void * param)
{
	if (EVENT_RBUTTONDOWN == event)
	{
		g_Pos.x = x;
		g_Pos.y = y;		
	}
}
void on_TransMouseHandle(int event, int x, int y, int flags, void * param)
{
	if (EVENT_RBUTTONDOWN == event)
	{
		g_TransPos.x = x;
		g_TransPos.y = y;
	}
}

int main(int argc, char *argv[])
{
	uint32_t deviceCount = 0;
	
    VzReturnStatus status = VZ_Initialize();
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VzInitialize failed!" << endl;
		system("pause");
		return -1;
	}

GET:
	status = VZ_GetDeviceCount(&deviceCount);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "VzGetDeviceCount failed!" << endl;
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
	const string transformedDepthWindow = "TransformedDepth";
	cv::namedWindow(depthImageWindow, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(irImageWindow, cv::WINDOW_AUTOSIZE);
	setMouseCallback(depthImageWindow, on_MouseHandle, nullptr);
	setMouseCallback(irImageWindow, on_MouseHandle, nullptr);
	bool isTransformedDepthPointToColorPointEnable = false;
	VzVector2u16 rgbFrameWH = {640,480};

	for (;;)
	{
		VzFrame depthFrame = { 0 };
		VzFrame irFrame = { 0 };
		VzFrame rgbFrame = { 0 };
		VzFrame transformedDepthFrame = { 0 };
		VzFrame transformedRgbFrame = { 0 };
		VzDepthVector3 TransformedDepthDepthVector[4] = {};

		// Read one frame before call VzGetFrame
		VzFrameReady frameReady = {0};
		status = VZ_GetFrameReady(g_DeviceHandle, 1200, &frameReady);
		
		//Get depth frame, depth frame only output in following data mode
		if (1 == frameReady.depth)
		{
			status = VZ_GetFrame(g_DeviceHandle, VzDepthFrame, &depthFrame);

			if (depthFrame.pFrameData != NULL)
			{
				if (true == g_IsSavePointCloud)
				{
					g_IsSavePointCloud = (1 == frameReady.transformedDepth) ? true : false;
					SavePointCloud("pointCloud.txt", depthFrame);
				}

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

				if(true == isTransformedDepthPointToColorPointEnable)
				{
					cv::Mat depthMat = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
					for (size_t i = 0; i < sizeof(TransformedDepthDepthVector)/sizeof(TransformedDepthDepthVector[0]); i++)
					{
						TransformedDepthDepthVector[i].depthX = TransformedDepthPoint[i].x;
						TransformedDepthDepthVector[i].depthY = TransformedDepthPoint[i].y;
						TransformedDepthDepthVector[i].depthZ = depthMat.at<uint16_t>(TransformedDepthPoint[i]);
					}
				}

				//Display the Depth Image
				Opencv_Depth(g_Slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat,g_Pos);
                char text[30] = "";
                sprintf(text, "%.2f", fps);
                putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

				if(true == isTransformedDepthPointToColorPointEnable)
				{
					cv::rectangle(imageMat, TransformedDepthPoint[0], TransformedDepthPoint[3], Scalar(255, 255, 255));
				}
				cv::imshow(depthImageWindow, imageMat);
			}
			else
			{
				cout << "VZ_GetFrame VzDepthFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}
		if (1 == frameReady.transformedDepth)
		{
			status = VZ_GetFrame(g_DeviceHandle, VzTransformDepthImgToColorSensorFrame, &transformedDepthFrame);

			if (transformedDepthFrame.pFrameData != NULL)
			{
				if (true == g_IsSavePointCloud)
				{
					g_IsSavePointCloud = false;
					SavePointCloud("transformedPointCloud.txt", transformedDepthFrame);
				}

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
				Opencv_Depth(g_Slope, transformedDepthFrame.height, transformedDepthFrame.width, transformedDepthFrame.pFrameData, imageMat,g_TransPos);
				char text[30] = "";
				sprintf(text, "%.2f", fps);
				putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));
				cv::imshow(transformedDepthWindow, imageMat);				
			}
			else
			{
				cout << "VZ_GetFrame VzDepthFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

		//Get IR frame, IR frame only output in following data mode
		if (1 == frameReady.ir)
		{
			status = VZ_GetFrame(g_DeviceHandle, VzIRFrame, &irFrame);

			if (irFrame.pFrameData != NULL)
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

                memset(text, 0, sizeof(text));
                sprintf(text, "%.2f", fps);
                putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

				cv::imshow(irImageWindow, imageMat);
			}
			else
			{
				cout << "VZ_GetFrame VzIRFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

        //Get RGB frame, RGB frame only output in following data mode
        if (1 == frameReady.color)
        {
            status = VZ_GetFrame(g_DeviceHandle, VzColorFrame, &rgbFrame);

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
				rgbFrameWH.x = rgbFrame.width;
				rgbFrameWH.y = rgbFrame.height;
                char text[30] = "";
                sprintf(text, "%.2f", fps);
                putText(imageMat, text, Point(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255));

				if(true == isTransformedDepthPointToColorPointEnable)
				{
					VzVector2u16 posInRGB[4] = {};

					for (size_t i = 0; i < sizeof(TransformedDepthDepthVector)/sizeof(TransformedDepthDepthVector[0]); i++)
					{
						VZ_TransformedDepthPointToColorPoint(g_DeviceHandle, TransformedDepthDepthVector[i],
															 VzVector2u16{rgbFrame.width, rgbFrame.height}, &posInRGB[i]);
						if (0 != posInRGB[i].x && 0 != posInRGB[i].y)
						{
							circle(imageMat, cv::Point(posInRGB[i].x, posInRGB[i].y), 4, cv::Scalar(0, 0, 255), -1, 8, 0);
						}
					}

					if (0 != posInRGB[0].x && 0 != posInRGB[0].y 
						&& 0 != posInRGB[1].x && 0 != posInRGB[1].y 
						&& 0 != posInRGB[2].x && 0 != posInRGB[2].y 
						&& 0 != posInRGB[3].x && 0 != posInRGB[3].y)
					{
						cv::line(imageMat, cv::Point(posInRGB[0].x, posInRGB[0].y), cv::Point(posInRGB[1].x, posInRGB[1].y), Scalar(0, 0, 255));
						cv::line(imageMat, cv::Point(posInRGB[1].x, posInRGB[1].y), cv::Point(posInRGB[3].x, posInRGB[3].y), Scalar(0, 0, 255));
						cv::line(imageMat, cv::Point(posInRGB[2].x, posInRGB[2].y), cv::Point(posInRGB[0].x, posInRGB[0].y), Scalar(0, 0, 255));
						cv::line(imageMat, cv::Point(posInRGB[3].x, posInRGB[3].y), cv::Point(posInRGB[2].x, posInRGB[2].y), Scalar(0, 0, 255));
					}
				}

                cv::imshow(rgbImageWindow, imageMat);
            }
            else
            {
                cout << "VZ_GetFrame VzRGBFrame status:" << status << " pFrameData is NULL " << endl;
            }
        }
		if (1 == frameReady.transformedColor)
		{
			status = VZ_GetFrame(g_DeviceHandle, VzTransformColorImgToDepthSensorFrame, &transformedRgbFrame);

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
				cout << "VZ_GetFrame VzRGBFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

		unsigned char key = waitKey(1);
        if (key == 'R' || key == 'r')
        {
            cout << "please select RGB resolution to set: 0:640*480; 1:800*600; 2:1600*1200" << endl;
            int index = 0;
            cin >> index;
            if (cin.fail())
            {
                std::cout << "Unexpected input"<< endl;
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
				g_TransPos.x = g_TransPos.x *640 / rgbFrameWH.x;
				g_TransPos.y = g_TransPos.y* 480 /rgbFrameWH.y;
                VZ_SetColorResolution(g_DeviceHandle, 640, 480);
                break;
            case 1:
				g_TransPos.x = g_TransPos.x * 800 / rgbFrameWH.x;
				g_TransPos.y = g_TransPos.y * 600 / rgbFrameWH.y;
                VZ_SetColorResolution(g_DeviceHandle, 800, 600);
                break;
            case 2:
				g_TransPos.x = g_TransPos.x * 1600 / rgbFrameWH.x;
				g_TransPos.y = g_TransPos.y * 1200 / rgbFrameWH.y;
                VZ_SetColorResolution(g_DeviceHandle, 1600, 1200);
                break;
            default:
                cout << "input is invalid." << endl;
                break;
            }
            
        }
		else if (key == 'P' || key == 'p')
		{
			g_IsSavePointCloud = true;
		}
        else if (key == 'Q' || key == 'q')
        {
			static bool isTransformColorImgToDepthSensorEnabled = true;
			VZ_SetTransformColorImgToDepthSensorEnabled(g_DeviceHandle, isTransformColorImgToDepthSensorEnabled);
			cout << "SetTransformColorImgToDepthSensorEnabled " << ((true == isTransformColorImgToDepthSensorEnabled) ? "enable" : "disable") << endl;
			isTransformColorImgToDepthSensorEnabled = !isTransformColorImgToDepthSensorEnabled;
        }
        else if (key == 'L' || key == 'l')
        {
			cv::namedWindow(transformedDepthWindow, cv::WINDOW_AUTOSIZE);
			setMouseCallback(transformedDepthWindow, on_TransMouseHandle, nullptr);
			if (800 == g_TransPos.x && 600 == g_TransPos.y)
			{
				if (rgbFrameWH.x <= 800 && rgbFrameWH.y <= 600)
				{
					g_TransPos.x = rgbFrameWH.x / 2;
					g_TransPos.y = rgbFrameWH.y / 2;
				}
			}
			static bool isTransformDepthImgToColorSensorEnabled = true;			
			VZ_SetTransformDepthImgToColorSensorEnabled(g_DeviceHandle, isTransformDepthImgToColorSensorEnabled);
			cout << "SetTransformDepthImgToColorSensorEnabled " << ((true == isTransformDepthImgToColorSensorEnabled) ? "enable" : "disable") << endl;
			isTransformDepthImgToColorSensorEnabled = !isTransformDepthImgToColorSensorEnabled;
        }
		else if (key == 'T' || key == 't')
        {
			isTransformedDepthPointToColorPointEnable = !isTransformedDepthPointToColorPointEnable;
			cout << "isTransformedDepthPointToColorPointEnable:" << ((true == isTransformedDepthPointToColorPointEnable) ? "enable" : "disable") << endl;
        }
		else if (key == 27)	//ESC Pressed
		{
			break;
		}
	}

	status = VZ_StopStream(g_DeviceHandle);
    cout << "VZ_StopStream status: " << status << endl;

    status = VZ_CloseDevice(&g_DeviceHandle);
    cout << "CloseDevice status: " << status << endl;

    status = VZ_Shutdown();
    cout << "Shutdown status: " << status << endl;
	cv::destroyAllWindows();

	delete[] g_pDeviceListInfo;
	g_pDeviceListInfo = NULL;

    return 0;
}

bool InitDevice(const int deviceCount)
{
	VZ_SetHotPlugStatusCallback(HotPlugStateCallback, nullptr);

	g_pDeviceListInfo = new VzDeviceInfo[deviceCount];
    VzReturnStatus status = VZ_GetDeviceInfoList(deviceCount, g_pDeviceListInfo);
	g_DeviceHandle = 0;
	status = VZ_OpenDeviceByUri(g_pDeviceListInfo[0].uri, &g_DeviceHandle);
	if (status != VzReturnStatus::VzRetOK)
	{
		cout << "OpenDevice failed!" << endl;
		system("pause");
		return false;
	}
    
    cout << "sn  ==  " << g_pDeviceListInfo[0].serialNumber << endl;

	VzSensorIntrinsicParameters cameraParameters;
	status = VZ_GetSensorIntrinsicParameters(g_DeviceHandle, VzToFSensor, &cameraParameters);

	cout << "Get VZ_GetSensorIntrinsicParameters status: " << status << endl;
	cout << "ToF Sensor Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "ToF Sensor Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "K4: " << cameraParameters.k4 << endl;
	cout << "K5: " << cameraParameters.k5 << endl;
	cout << "K6: " << cameraParameters.k6 << endl;

    const int BufLen = 64;
	char fw[BufLen] = { 0 };
	VZ_GetFirmwareVersion(g_DeviceHandle, fw, BufLen);
	cout << "fw  ==  " << fw << endl;

    VZ_StartStream(g_DeviceHandle);
    //Wait for the device to upload image data
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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
    cout << "T/t: Enables or disables transforms the depth point into the geometry of the color camera" << endl;
	cout << "Esc: Program quit " << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------\n" << endl;
}

static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg,Point point)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	Point2d pointxy = point;	
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

void HotPlugStateCallback(const VzDeviceInfo* pInfo, int state, void* pUserData)
{
	cout << pInfo->uri<<" " << (state ==0? "add":"remove" )<<endl ;
}

void SavePointCloud(const char* pFileName, const VzFrame& depthFrame)
{
	if (depthFrame.pFrameData != NULL && pFileName != NULL)
	{
		ofstream PointCloudWriter;
		PointCloudWriter.open(pFileName);
		const int len = depthFrame.width * depthFrame.height;
		VzVector3f *worldV = new VzVector3f[len];

		VZ_ConvertDepthFrameToPointCloudVector(g_DeviceHandle, &depthFrame, worldV); // Convert Depth frame to World vectors.

		for (int i = 0; i < len; i++)
		{
			if (0 < worldV[i].z && worldV[i].z < g_Slope)
			{
				PointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << std::endl;
			}
		}
		delete[] worldV;
		worldV = NULL;
		std::cout << "Save point cloud successful in " <<pFileName<< std::endl;
		PointCloudWriter.close();
	}
	else
	{
		std::cout << "Current Depth Frame is NULL" << endl;
	}
}