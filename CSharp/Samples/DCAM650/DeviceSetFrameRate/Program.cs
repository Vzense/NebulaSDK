using System;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace DeviceSetFrameRate
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---DeviceSetFrameRate---");

            //about dev
            VzenseNebulaAPI VNAPI = new VzenseNebulaAPI();
            UInt32 deviceCount = 0;
            
            VzDeviceHandle deviceHandle = new IntPtr();
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
            if (0 == deviceCount)
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

            int frameRate = new int();
            frameRate = 5;
            status = VNAPI.VN_SetFrameRate(deviceHandle, frameRate);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetFrameRate failed status:" + status);
                return;
            }
            Console.WriteLine("set frame rate :" + frameRate + " is OK.");

            //Wait for the device to upload image data
            Thread.Sleep(1000);

            Console.WriteLine("Start testing the average frame rate for 30 seconds, Please wait patiently");
            //statistical frame rate

            const int TESTPERIOD = 30;//30 senconds
            int index = 0;
            UInt64 start = 0;
            start = GetTimeStampMS();

            for (; ; )
            {
                VzFrameReady FrameReady = new VzFrameReady();
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    continue;
                }

                if (1 == FrameReady.depth)
                {
                    VzFrame depthFrame = new VzFrame();
                    status = VNAPI.VN_GetFrame(deviceHandle, VzFrameType.VzDepthFrame, ref depthFrame);
                    if (depthFrame.pFrameData != IntPtr.Zero)
                    {
                        UInt64 diff = GetTimeStampMS() - start;
                        index++;
                        if (diff > (TESTPERIOD * 1000))
                        {
                            float fps = (index * TESTPERIOD * 1000.0f / diff) / TESTPERIOD;
                            index = 0;
                            Console.WriteLine(fps);
                            break;
                        }
                    }
                }

            }

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

            return ;
        }

        static UInt64 GetTimeStampMS()
        {
            TimeSpan ts = DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, 0);
            UInt64 ret = Convert.ToUInt64(ts.TotalMilliseconds);
            return ret;
        }
    }
}
