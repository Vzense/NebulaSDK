using System;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace ToFFiltersSetGet
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("--------------ToFFiltersSetGet-------------");

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

            //The parameters of TimeFilter and ConfidenceFilter are stored in camera
            //The parameters of FlyingPixelFilter, FillHoleFilter and SpatialFilter are stored in SDK

            Console.WriteLine("-------------1------------ test TimeFilter --------------------------");

            VzTimeFilterParams TimeFilterParams = new VzTimeFilterParams();
            TimeFilterParams.threshold = 1;
            TimeFilterParams.enable = 0;
            status = VNAPI.VN_GetTimeFilterParams(deviceHandle, ref TimeFilterParams);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetTimeFilterParams failed status:" + status);
                return;
            }

            if(TimeFilterParams.enable == 0)
            {
                Console.WriteLine("The default TimeFilter switch is False");
                TimeFilterParams.enable = 1;
            }
            else
            {
                Console.WriteLine("The default TimeFilter switch is True" );
                TimeFilterParams.enable = 0;
            }
            
            status = VNAPI.VN_SetTimeFilterParams(deviceHandle, ref TimeFilterParams);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetTimeFilterParams failed status:" + status);
                return;
            }
            Console.WriteLine("Set TimeFilter switch to " + TimeFilterParams.enable + " is Ok.");

            Console.WriteLine("-------------2--------- test ConfidenceFilter -----------------------");

            VzConfidenceFilterParams confidenceFilterParams = new VzConfidenceFilterParams();
            confidenceFilterParams.enable = 1;
            confidenceFilterParams.threshold = 15;
            status = VNAPI.VN_GetConfidenceFilterParams(deviceHandle, ref confidenceFilterParams);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetConfidenceFilterParams failed status:" + status);
                return;
            }
            if (confidenceFilterParams.enable == 0)
            {
                Console.WriteLine("The default ConfidenceFilter switch is False");
                confidenceFilterParams.enable = 1;
            }
            else
            {
                Console.WriteLine("The default ConfidenceFilter switch is True");
                confidenceFilterParams.enable = 0;
            }
            
            status = VNAPI.VN_SetConfidenceFilterParams(deviceHandle, ref confidenceFilterParams);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetConfidenceFilterParams failed status:" + status);
                return;
            }
            Console.WriteLine("Set ConfidenceFilter switch to " + confidenceFilterParams.enable + " is Ok.");


            Console.WriteLine("-------------3--------- test FlyingPixelFilter ----------------------");

            VzFlyingPixelFilterParams flyingPixelFilterParams = new VzFlyingPixelFilterParams();
            flyingPixelFilterParams.enable = 1;
            flyingPixelFilterParams.threshold = 15;
            status = VNAPI.VN_GetFlyingPixelFilterParams(deviceHandle, ref flyingPixelFilterParams);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetFlyingPixelFilterParams failed status:" + status);
                return;
            }

            if (flyingPixelFilterParams.enable == 0)
            {
                Console.WriteLine("The default flyingPixelFilterParams switch is False");
                flyingPixelFilterParams.enable = 1;
            }
            else
            {
                Console.WriteLine("The default flyingPixelFilterParams switch is True");
                flyingPixelFilterParams.enable = 0;
            }

            status = VNAPI.VN_SetFlyingPixelFilterParams(deviceHandle, ref flyingPixelFilterParams);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetFlyingPixelFilterParams failed status:" + status);
                return;
            }
            Console.WriteLine("Set FlyingPixelFilter switch to " + flyingPixelFilterParams.enable + " is Ok.");

            Console.WriteLine("-------------4---------- test FillHoleFilter ------------------------");

            byte bFillHoleFilter = new byte();
            status = VNAPI.VN_GetFillHoleFilterEnabled(deviceHandle, ref bFillHoleFilter);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetFillHoleFilterEnabled failed status:" + status);
                return;
            }

            if (bFillHoleFilter == 0)
            {
                Console.WriteLine("The default FillHoleFilter switch is False");
                bFillHoleFilter = 1;
            }
            else
            {
                Console.WriteLine("The default FillHoleFilter switch is True");
                bFillHoleFilter = 0;
            }
            
            status = VNAPI.VN_SetFillHoleFilterEnabled(deviceHandle, bFillHoleFilter);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetFillHoleFilterEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("Set FillHoleFilter switch to " + bFillHoleFilter + " is Ok.");

            Console.WriteLine("-------------5---------- test SpatialFilter -------------------------");

            byte bSpatialFilter = new byte();
            status = VNAPI.VN_GetSpatialFilterEnabled(deviceHandle, ref bSpatialFilter);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetSpatialFilterEnabled failed status:" + status);
                return;
            }
            if (bSpatialFilter == 0)
            {
                Console.WriteLine("The default SpatialFilter switch is False");
                bSpatialFilter = 1;
            }
            else
            {
                Console.WriteLine("The default SpatialFilter switch is True");
                bSpatialFilter = 0;

            }
            status = VNAPI.VN_SetSpatialFilterEnabled(deviceHandle, bSpatialFilter);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetSpatialFilterEnabled failed status:" + status);
                return;
            }
            Console.WriteLine("Set SpatialFilter switch to " + bSpatialFilter + " is Ok.");

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

            Console.WriteLine("---Test end, please reboot camera to restore the default settings.----");
            return;
        }
    }
}
