using System;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace MultiConnectionInMultiThread
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---MultiConnectionInMultiThread---");

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
                Console.WriteLine("VN_GetDeviceCount failed! make sure the DCAM is connected");
                Thread.Sleep(1000);
                goto GET;
            }
            Console.WriteLine("Get device count: " + deviceCount);
            if (deviceCount < 2)
            {
                Thread.Sleep(1000);
                goto GET;
            }

            VzDeviceHandle[] deviceHandle = new VzDeviceHandle[deviceCount];
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
                Device[] cDevice = new Device[deviceCount];
                Thread[] t = new Thread[deviceCount];
                for (UInt32 i = 0; i < deviceCount; i++)
                {
                    cDevice[i] = new Device(pDeviceListInfo[i], ref VNAPI);
                    t[i] = new Thread(cDevice[i].TestDevice);
                    t[i].Start();
                }

                for (UInt32 i = 0; i < deviceCount; i++)
                {
                    cDevice[i].WaitForTestDone();
                    t[i].Abort();
                }
            }

            status = VNAPI.VN_Shutdown();
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_Shutdown failed status:" + status);
                return;
            }

            Console.WriteLine("--end--");

            return ;
        }
    }

    class Device
    {
        public Device(VzDeviceInfo DeviceListInfo, ref VzenseNebulaAPI pVNAPI)
        {
            pDeviceListInfo.uri = DeviceListInfo.uri;
            pDeviceListInfo.alias = DeviceListInfo.alias;
            pDeviceListInfo.serialNumber = DeviceListInfo.serialNumber;
            pDeviceListInfo.status = DeviceListInfo.status;
            VNAPI = pVNAPI;
        }
        public void TestDevice()
        {
            Console.WriteLine("TestDevice");
            lock (this)
            {
                Console.WriteLine("uri:" + pDeviceListInfo.uri);
                Console.WriteLine("alias:" + pDeviceListInfo.alias);
                Console.WriteLine("connectStatus:" + pDeviceListInfo.status);

                VzReturnStatus status = VNAPI.VN_OpenDeviceByUri(pDeviceListInfo.uri, ref device_);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("OpenDevice " + pDeviceListInfo.serialNumber + " failed status:" + status);
                }

                // Starts capturing the image stream
                status = VNAPI.VN_StartStream(device_);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_StartStream " + pDeviceListInfo.serialNumber + " failed status:" + status);
                }

                //Wait for the device to upload image data
                Thread.Sleep(1000);

                // 1.ReadNextFrame.
                // 2.Get depth Frame acoording to Ready flag.
                for (int j = 0; j < 10; j++)
                {
                    VzFrame depthFrame = new VzFrame();
                    VzFrameReady frameReady = new VzFrameReady();
                    status = VNAPI.VN_GetFrameReady(device_, 1200, ref frameReady);

                    if (status != VzReturnStatus.VzRetOK)
                    {
                        Console.WriteLine(pDeviceListInfo.serialNumber + " VN_GetFrameReady failed status:" + status);
                        continue;
                    }

                    // Get depth frame, depth frame only output in following data mode
                    if (1 == frameReady.depth)
                    {
                        status = VNAPI.VN_GetFrame(device_, VzFrameType.VzDepthFrame, ref depthFrame);

                        if (status == VzReturnStatus.VzRetOK && depthFrame.pFrameData != IntPtr.Zero)
                        {
                            Console.WriteLine(pDeviceListInfo.serialNumber + " frameIndex :" + depthFrame.frameIndex);
                        }
                        else
                        {
                            Console.WriteLine(pDeviceListInfo.serialNumber + "VN_GetFrame VzFrameType.VzDepthFrame status:" + status);
                        }
                    }
                }

                // 1.close device
                // 2.SDK shutdown
                status = VNAPI.VN_StopStream(device_);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_StopStream failed status:" + status);
                }
                status = VNAPI.VN_CloseDevice(ref device_);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_CloseDevice failed status:" + status);
                }
                isTestDone_ = true;
            }
            return;
        }

        public bool WaitForTestDone()
        {
            while (!isTestDone_)
            {
                Thread.Sleep(1000);
            }
            return true;
        }
        
	    VzDeviceHandle device_;
        bool isTestDone_ = false;
        static VzenseNebulaAPI VNAPI;
        VzDeviceInfo pDeviceListInfo;
    }
}
