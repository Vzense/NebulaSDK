using System;
using System.Text;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace RGBExposureTimeSetGet
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---RGBExposureTimeSetGet---");

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

            Console.WriteLine("open device successful,status :" + status);

            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
            }

            //switch exposure mode to manual
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, VzSensorType.VzColorSensor, VzExposureControlMode.VzExposureControlMode_Manual);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetExposureControlMode failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("switch to manual");
            }

            VzExposureTimeParams Params = new VzExposureTimeParams();
            Params.mode = VzExposureControlMode.VzExposureControlMode_Manual;
            Params.exposureTime = 3000;
            status = VNAPI.VN_SetExposureTime(deviceHandle, VzSensorType.VzColorSensor, Params);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("SetExposureTime:" + Params.exposureTime);
            }

            Params.exposureTime = 0;
            status = VNAPI.VN_GetExposureTime(deviceHandle, VzSensorType.VzColorSensor, ref Params);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("GetExposureTime:" + Params.exposureTime);
            }

            //switch exposure mode to auto
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, VzSensorType.VzColorSensor, VzExposureControlMode.VzExposureControlMode_Auto);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetExposureControlMode failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("switch to auto");
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

            return ;
        }
    }
}
