using System;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace DevHotPlugCallbackCpp
{
    using VzDeviceHandle = System.IntPtr;
    class Sensor
    {
        public static VzenseNebulaAPI VNAPI = new VzenseNebulaAPI();
        public static VzDeviceHandle deviceHandle = new IntPtr();

        public static void HotPlugStateCallback(ref VzDeviceInfo pInfo, int status, IntPtr contex)
        {
            HandleCallback(ref pInfo, status);
        }

        public VzReturnStatus registCallback(VzDeviceHandle deviceHand)
        {
            deviceHandle = deviceHand;
            IntPtr pt = IntPtr.Zero;

            VzenseNebulaAPI.PtrHotPlugStatusCallback hpcb = new VzenseNebulaAPI.PtrHotPlugStatusCallback(HotPlugStateCallback);
            return VNAPI.VN_SetHotPlugStatusCallback(hpcb, pt);
        }

        public static void HandleCallback(ref VzDeviceInfo pInfo, int status)
        {
            Console.WriteLine("uri " + status + "  " + pInfo.uri + "    " + (status == 0 ? "add" : "remove"));
            Console.WriteLine("alia " + status + "  " + pInfo.alias + "    " + (status == 0 ? "add" : "remove"));

            if (status == 0)
            {
                Console.WriteLine("VN_OpenDevice " + VNAPI.VN_OpenDeviceByUri(pInfo.uri, ref deviceHandle));
                Console.WriteLine("VN_StartStream " + VNAPI.VN_StartStream(deviceHandle));
            }
            else
            {
                Console.WriteLine("VN_StopStream " + VNAPI.VN_StopStream(deviceHandle));
                Console.WriteLine("VN_CloseDevice " + VNAPI.VN_CloseDevice(ref deviceHandle));
            }

        }
    }

    class Program
    {
        public static VzDeviceHandle deviceHandle = new IntPtr();
        public static VzenseNebulaAPI VNAPI = new VzenseNebulaAPI();

        static void Main(string[] args)
        {
            UInt32 deviceCount = 0;

            VzReturnStatus status = VNAPI.VN_Initialize();
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VzInitialize failed status:" + status);
                Console.Write("Press any key to continue . . . ");
                Console.ReadKey(true);
                return;
            }

        GET:
            status = VNAPI.VN_GetDeviceCount(ref deviceCount);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VzGetDeviceCount failed status:" + status);
                Console.Write("Press any key to continue . . . ");
                Console.ReadKey(true);
                return;
            }
            Console.WriteLine("Get device count: " + deviceCount);
            if (0 == deviceCount)
            {
                Thread.Sleep(1000);
                goto GET;
            }

            if (InitDevice(deviceCount))
            {
                Sensor s  = new Sensor();
                status = (VzReturnStatus)s.registCallback(deviceHandle);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("SetHotPlugStatusCallback failed status:" + status);
                }
                else
                {
                    Console.WriteLine(" wait for hotplug operation ");
                    // wait for hotplug
                    for (; ; )
                    {
                        Thread.Sleep(1000);
                    }
                }
                status = VNAPI.VN_CloseDevice(ref deviceHandle);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("CloseDevice failed status:" + status);
                }
            }
            status = VNAPI.VN_Shutdown();
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("Shutdown failed status:" + status);
            }
        }

        public static bool InitDevice(UInt32 deviceCount)
        {
            VzDeviceInfo[] pDeviceListInfo = new VzDeviceInfo[deviceCount];
            VzReturnStatus status = VNAPI.VN_GetDeviceInfoList(deviceCount, pDeviceListInfo);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("GetDeviceListInfo failed status:" + status);
                return false;
            }
            else
            {
                if (VzConnectStatus.VzConnected != pDeviceListInfo[0].status)
                {
                    Console.WriteLine("connect statu" + pDeviceListInfo[0].status);
                    Console.WriteLine("Call VN_OpenDevice with connect status :" + VzConnectStatus.VzConnected);
                    return false;
                }
            }

            Console.WriteLine("uri:" + pDeviceListInfo[0].uri);
            Console.WriteLine("alias:" + pDeviceListInfo[0].alias);
            Console.WriteLine("ip:" + pDeviceListInfo[0].ip);
            Console.WriteLine("connectStatus:" + pDeviceListInfo[0].status);

            deviceHandle = IntPtr.Zero;

            status = VNAPI.VN_OpenDeviceByUri(pDeviceListInfo[0].uri, ref deviceHandle);

            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("OpenDevice failed status:" + status);
                return false;
            }

            Console.WriteLine("open device successful,status :" + status);

            status = VNAPI.VN_StartStream(deviceHandle);

            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("StartStream failed status:" + status);
                return false;
            }

            return true;
        }
    }
}
