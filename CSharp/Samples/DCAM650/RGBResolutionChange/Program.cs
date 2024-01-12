using System;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace RGBResolutionChange
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---RGBResolutionChange---");

            //about dev
            VzenseNebulaAPI VNAPI = new VzenseNebulaAPI();
            UInt32 deviceCount = 0;

            VzDeviceHandle deviceHandle = new IntPtr();
            VzReturnStatus status = VzReturnStatus.VzRetOthers;

            //about frame

            VzFrameReady FrameReady = new VzFrameReady();
            VzFrame RGBFrame = new VzFrame();

            //SDK Initialize
            status = VNAPI.VN_Initialize();
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VzInitialize failed status:" + status);
                Console.ReadKey(true);
                return;
            }

        //1.Search and notice the count of devices.
        //2.get infomation of the devices. 
        //3.open devices accroding to the info.
        GET:
            status = VNAPI.VN_GetDeviceCount(ref deviceCount);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VzGetDeviceCount failed! make sure the DCAM is connected");
                Thread.Sleep(1000);
                goto GET;
            }
            Console.WriteLine("Get device count: " + deviceCount);
            if (0 == deviceCount)
            {
                Thread.Sleep(1000);
                goto GET;
            }

            VzDeviceInfo[] pDeviceListInfo = new VzDeviceInfo[deviceCount];
            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo); ;
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("GetDeviceListInfo failed status:" + status);
                return;
            }
            else
            {
                if (VzConnectStatus.VzConnected != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect statu" + pDeviceListInfo[0].status);
                    Console.WriteLine("Call VN_OpenDevice with connect status :" + VzConnectStatus.VzConnected);
                    return;
                }
            }

            Console.WriteLine("uri:" + pDeviceListInfo[0].uri);
            Console.WriteLine("alias:" + pDeviceListInfo[0].alias);
            Console.WriteLine("ip:" + pDeviceListInfo[0].ip);
            Console.WriteLine("connectStatus:" + pDeviceListInfo[0].status);

            status = VNAPI.VN_OpenDeviceByUri(pDeviceListInfo[0].uri, ref deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("OpenDevice failed status:" + status);
                return;
            }

            Console.WriteLine("open device successful,status :" + status);

            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
            }

            //switch RGBResolution
            int resolution_w = new int();
            resolution_w =  640;
            int resolution_h = new int();
            resolution_h = 480;
            //1.640_480
            status = VNAPI.VN_SetColorResolution(deviceHandle, resolution_w, resolution_h);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetColorResolution failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("set to 640_480");
            }

            //Wait for the device to upload image data
            Thread.Sleep(2000);

            for (int i = 0; i < 10; i++)
            {
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    continue;
                }

                //cout resolution
                if (1 == FrameReady.color)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, VzFrameType.VzColorFrame, ref RGBFrame);
                    if (status == VzReturnStatus.VzRetOK && RGBFrame.pFrameData != IntPtr.Zero
                    && RGBFrame.width == 640)
                    {
                        Console.WriteLine("get Frame successful,status:" + status + "  "
                             + "resolution: " + RGBFrame.width + "x" + RGBFrame.height);
                    }
                }

            }

            resolution_w = 1600;
            resolution_h = 1200;
            //2.1600_1200
            status = VNAPI.VN_SetColorResolution(deviceHandle, resolution_w, resolution_h);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetColorResolution failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("set to 1600_1200");
            }

            //Wait for the device to upload image data
            Thread.Sleep(1500);

            for (int i = 0; i < 10; i++)
            {
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    continue;
                }

                //cout resolution
                if (1 == FrameReady.color)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, VzFrameType.VzColorFrame, ref RGBFrame);
                    if (status == VzReturnStatus.VzRetOK && RGBFrame.pFrameData != IntPtr.Zero
                                && RGBFrame.width == 1600)

                    {
                        Console.WriteLine("get Frame successful,status:" + status + "  "
                            + "resolution: " + RGBFrame.width + "x" + RGBFrame.height);
                    }
                }

            }
            //Stop capturing the image stream
            status = VNAPI.VN_StopStream(deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_StopStream failed status:" + status);
                return;
            }

            //1.close device
            //2.SDK shutdown
            status = VNAPI.VN_CloseDevice(ref deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_CloseDevice failed status:" + status);
                return;
            }
            status = VNAPI.VN_Shutdown();
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_Shutdown failed status:" + status);
                return;
            }
            Console.WriteLine("---end---");

            return;
        }
    }
}
