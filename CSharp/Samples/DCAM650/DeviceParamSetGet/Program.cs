using System;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace DeviceParamSetGet
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---DeviceParamSetGet---");

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

            //cameraParameters
            VzSensorIntrinsicParameters cameraParameters = new VzSensorIntrinsicParameters();
            status = VNAPI.VN_GetSensorIntrinsicParameters(deviceHandle, VzSensorType.VzToFSensor, ref cameraParameters);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetCameraParameters failed status:" + status);
                return;
            }
            Console.WriteLine("Get VzGetCameraParameters status: " + status);
            Console.WriteLine("Depth Camera Intinsic:");
            Console.WriteLine("Fx: " + cameraParameters.fx);
            Console.WriteLine("Cx: " + cameraParameters.cx);
            Console.WriteLine("Fy: " + cameraParameters.fy);
            Console.WriteLine("Cy: " + cameraParameters.cy);

            Console.WriteLine("Depth Distortion Coefficient: ");
            Console.WriteLine("K1: " + cameraParameters.k1);
            Console.WriteLine("K2: " + cameraParameters.k2);
            Console.WriteLine("P1: " + cameraParameters.p1);
            Console.WriteLine("P2: " + cameraParameters.p2);
            Console.WriteLine("K3: " + cameraParameters.k3);
            Console.WriteLine("K4: " + cameraParameters.k4);
            Console.WriteLine("K5: " + cameraParameters.k5);
            Console.WriteLine("K6: " + cameraParameters.k6);

            //gmmgain
            byte gmmgain = 0;
            status = VNAPI.VN_GetIRGMMGain(deviceHandle, ref gmmgain);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetGMMGain failed status:" + status);
                return;
            }
            Console.WriteLine("default gmmgain: " + (int)gmmgain);

            gmmgain = 100;
            status = VNAPI.VN_SetIRGMMGain(deviceHandle, gmmgain);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetGMMGain failed status:" + status);
                return;
            }
            status = VNAPI.VN_GetIRGMMGain(deviceHandle, ref gmmgain);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_GetGMMGain failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("set gmmgain: " + (int)gmmgain + " succeeded");
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

            Console.WriteLine("--end--");
            return ;
        }
    }
}
