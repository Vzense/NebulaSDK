using System;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace DeviceSWTriggerMode
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---DeviceSWTriggerMode---");

            //about dev
            VzenseNebulaAPI VNAPI = new VzenseNebulaAPI();
            UInt32 deviceCount = 0;
            
            VzDeviceHandle deviceHandle = new IntPtr();
            VzReturnStatus status = VzReturnStatus.VzRetOthers;

            //about frame

            VzFrameReady FrameReady = new VzFrameReady();
            VzFrame depthFrame = new VzFrame();

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

            Console.WriteLine("uri:" + pDeviceListInfo[0].uri);
            Console.WriteLine("alias:" +pDeviceListInfo[0].alias);
           Console.WriteLine("ip:" + pDeviceListInfo[0].ip);
            Console.WriteLine("connectStatus:" + pDeviceListInfo[0].status);

            status = VNAPI.VN_OpenDeviceByUri(pDeviceListInfo[0].uri, ref deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("OpenDevice failed status:" + status);
                return;
            }

            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
            }

            //Wait for the device to upload image data
            Thread.Sleep(1000);

            Console.WriteLine("Software trigger test begins");

            //set slave true
            status = VNAPI.VN_SetWorkMode(deviceHandle, VzWorkMode.VzSoftwareTriggerMode);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetWorkMode failed status:" + status);
                return;
            }

            //Clearing cached images
            for (int i = 0; i < 5; i++)
            {
                status = VNAPI.VN_GetFrameReady(deviceHandle, 200, ref FrameReady);
            }

            //1.software trigger.
            //2.ReadNextFrame.
            //3.GetFrame acoording to Ready flag and Frametype.
            for (int i = 0; i < 10; i++)
            {

                //The minimum time interval to trigger a signal is 1000/FPS milliseconds
                Thread.Sleep(1000);

                //call the below api to trigger one frame, then the frame will be sent
                // if do not call this function, the frame will not be sent and the below call will return timeout fail
                status = VNAPI.VN_SetSoftwareSlaveTrigger(deviceHandle);

                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_SetSoftwareSlaveTrigger failed status:" + status);
                    continue;
                }

                //If no image is ready within 1000ms, the function will return VzRetGetFrameReadyTimeOut
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    //Thread.Sleep(1000);
                    continue;
                }

                //depthFrame for example.
                if (1 == FrameReady.depth)
                {
                    status = VNAPI.VN_GetFrame(deviceHandle, VzFrameType.VzDepthFrame, ref depthFrame);
                    if (depthFrame.pFrameData != IntPtr.Zero)
                    {
                        Console.WriteLine("get Frame successful,status:" + status + "  "
                            + "frameTpye:" + depthFrame.frameType + "  "
                            + "frameIndex:" + depthFrame.frameIndex);
                    }
                }

            }
            //set slave false
            status = VNAPI.VN_SetWorkMode(deviceHandle, VzWorkMode.VzActiveMode);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetWorkMode failed status:" + status);
                return;
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
    }
}
