using System;
using System.Threading;
using System.Runtime.InteropServices;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace MultiConnection
{
    using VzDeviceHandle = System.IntPtr;

    class Program
    {
        //static VzDeviceInfo[] pDeviceListInfo;
        static void Main(string[] args)
        {
            Console.WriteLine("---MultiConnection---");

            //about dev
            VzenseNebulaAPI VNAPI = new VzenseNebulaAPI();
            UInt32 deviceCount = 0;
            VzReturnStatus status = VzReturnStatus.VzRetOthers;
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
            if (deviceCount < 2)
            {
                Thread.Sleep(1000);
                goto GET;
            }
            VzDeviceInfo[] pDeviceListInfo = new VzDeviceInfo[deviceCount];
            status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
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
            for (int i = 0; i < deviceCount; i++)
            {
                Console.WriteLine("CameraIndex: " + i);
                Console.WriteLine("uri:" + pDeviceListInfo[i].uri);
                Console.WriteLine("alias:" + pDeviceListInfo[i].alias);
                Console.WriteLine("connectStatus:" + pDeviceListInfo[i].status);
            }
            VzDeviceHandle[] deviceHandle = new IntPtr[deviceCount];
            for (int i = 0; i < deviceCount; i++)
            {
                status = VNAPI.VN_OpenDeviceByUri(pDeviceListInfo[i].uri, ref deviceHandle[i]);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("OpenDevice failed status:" + status);
                    return;
                }
            }

            for (int i = 0; i < deviceCount; i++)
            {
                //Starts capturing the image stream
                status = VNAPI.VN_StartStream(deviceHandle[i]);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_StartStream failed status:" + status);
                    return;
                }
            }

            //Wait for the device to upload image data
            Thread.Sleep(1000);

            //1.ReadNextFrame.
            //2.Get depth Frame acoording to Ready flag.
            for (int j = 0; j < 10; j++)
            {
                for (int i = 0; i < deviceCount; i++)
                {
                    if (IntPtr.Zero != deviceHandle[i])
                    {
                        VzFrame depthFrame = new VzFrame();
                        VzFrameReady frameReady = new VzFrameReady();
                        status = VNAPI.VN_GetFrameReady(deviceHandle[i], 1200, ref frameReady);

                        if (status != VzReturnStatus.VzRetOK)
                        {
                            Console.WriteLine(pDeviceListInfo[i].alias + "  VN_GetFrameReady failed status:" + status);
                            continue;
                        }

                        //Get depth frame, depth frame only output in following data mode
                        if (1 == frameReady.depth)
                        {
                            status = VNAPI.VN_GetFrame(deviceHandle[i], VzFrameType.VzDepthFrame, ref depthFrame);

                            if (status == VzReturnStatus.VzRetOK && depthFrame.pFrameData != IntPtr.Zero)
                            {
                                Console.WriteLine(pDeviceListInfo[i].alias + " frameIndex :" + depthFrame.frameIndex);
                            }
                            else
                            {
                                Console.WriteLine(pDeviceListInfo[i].alias + "VN_GetFrame VzFrameType.VzDepthFrame status:" + status);
                            }
                        }
                    }

                }
            }

            //1.close device
            //2.SDK shutdown
            for (int i = 0; i < deviceCount; i++)
            {
                status = VNAPI.VN_StopStream(deviceHandle[i]);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_StopStream failed status:" + status);
                }
                status = VNAPI.VN_CloseDevice(ref deviceHandle[i]);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_CloseDevice failed status:" + status);
                }
            }
            status = VNAPI.VN_Shutdown();
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_Shutdown failed status:" + status);
                return;
            }

            Console.WriteLine("--end--");

            return;
        }
    }
}
