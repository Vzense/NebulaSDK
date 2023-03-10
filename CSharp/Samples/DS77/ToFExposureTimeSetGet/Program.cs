using System;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;
using System.Runtime.InteropServices;

namespace ToFExposureTimeSetGet
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---- ToFExposureTimeSetGet ----");

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
        //2.Get infomation of the devices. 
        //3.Open devices accroding to the info.
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

            Console.WriteLine("Open device successful,status :" + status);

            //Starts capturing the image stream
            status = VNAPI.VN_StartStream(deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_StartStream failed status:" + status);
                return;
            }

            //1.Default FrameRate
            //2.Set new ExposureTime
            //3.Change FrameRate to 15
            //4.Set new ExposureTime

            Console.WriteLine("---- Default FrameRate ----");
            //Set Control mode to manual
            status = VNAPI.VN_SetExposureControlMode(deviceHandle, VzSensorType.VzToFSensor, VzExposureControlMode.VzExposureControlMode_Manual);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetExposureControlMode failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("Set control mode to manual.");
            }

            //Get default frame rate
            int defaultframeRate = new int();
            defaultframeRate = 20;
            status = VNAPI.VN_GetFrameRate(deviceHandle, ref defaultframeRate);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetFrameRate failed status:" + status);
                return;
            }
            Console.WriteLine("Get default frame rate: " + defaultframeRate);

            //Get the range of the ToF exposure time 
            VzExposureTimeParams maxExposureTime = new VzExposureTimeParams();
            maxExposureTime.mode = VzExposureControlMode.VzExposureControlMode_Manual;
            maxExposureTime.exposureTime = 0;

            UInt32 uRGBExposureTimeMax = (UInt32)Marshal.SizeOf(maxExposureTime);
            Int32 iRGBExposureTimeMax = (Int32)uRGBExposureTimeMax;
            IntPtr pRGBExposureTimeMax = Marshal.AllocHGlobal(iRGBExposureTimeMax);
            status = VNAPI.VN_GetProperty(deviceHandle, "Py_ToFExposureTimeMax", pRGBExposureTimeMax, uRGBExposureTimeMax);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetProperty get Py_ToFExposureTimeMax failed status:" + status);
                return;
            }
            maxExposureTime = (VzExposureTimeParams)Marshal.PtrToStructure(pRGBExposureTimeMax, typeof(VzExposureTimeParams));
            Console.WriteLine("Recommended scope: 58 - " + maxExposureTime.exposureTime);
            Marshal.FreeHGlobal(pRGBExposureTimeMax);

            //Set new ExposureTime
            VzExposureTimeParams Params = new VzExposureTimeParams();
            Params.mode = VzExposureControlMode.VzExposureControlMode_Manual;
            Params.exposureTime = 400;
            status = VNAPI.VN_SetExposureTime(deviceHandle, VzSensorType.VzToFSensor, Params);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("Set exposure time " + Params.exposureTime + " is OK.");
            }

            Console.WriteLine("---- Set FrameRate to 15 ----");
            //Set new FrameRate
            int frameRate = new int();  //New frame rate, can change it
            frameRate = 15;
            status = VNAPI.VN_SetFrameRate(deviceHandle, frameRate);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetFrameRate set new FrameRate failed status:" + status);
                return;
            }
            Console.WriteLine("Set frame rate " + frameRate + " is OK.");

            //Need to get new ExposureTime Range due to FrameRate change.
            Params.exposureTime = 0;
            IntPtr pToFExposureTimeMax = Marshal.AllocHGlobal(iRGBExposureTimeMax);
            status = VNAPI.VN_GetProperty(deviceHandle, "Py_ToFExposureTimeMax", pToFExposureTimeMax, uRGBExposureTimeMax);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetProperty get Py_ToFExposureTimeMax failed status:" + status);
                return;
            }

            Params = (VzExposureTimeParams)Marshal.PtrToStructure(pToFExposureTimeMax, typeof(VzExposureTimeParams));
            Marshal.FreeHGlobal(pToFExposureTimeMax);
            Console.WriteLine("Recommended scope: 58 - " + Params.exposureTime);

            //Set new ExposureTime 500
            VzExposureTimeParams pParams = new VzExposureTimeParams();
            pParams.mode = VzExposureControlMode.VzExposureControlMode_Manual;
            pParams.exposureTime = 500;
            status = VNAPI.VN_SetExposureTime(deviceHandle, VzSensorType.VzToFSensor, pParams);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetExposureTime failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("Set exposure time " + pParams.exposureTime + " is OK.");
            }

            //Stop capturing the image stream
            status = VNAPI.VN_StopStream(deviceHandle);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_StopStream failed status:" + status);
                return;
            }

            //1.Close device
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

            Console.WriteLine("--- Test end, please reboot camera to restore the default settings ---");
            return ;
        }
    }
}
