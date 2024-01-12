using System;
using System.IO;
using System.Threading;
using VzenseNebula_enums;
using VzenseNebula_types;
using VzenseNebula_api;

namespace PointCloudCaptureAndSaveDepthImgToColorSensor
{
    using VzDeviceHandle = System.IntPtr;
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("---PointCloudCaptureAndSaveDepthImgToColorSensor---");

            //about dev
            VzenseNebulaAPI VNAPI = new VzenseNebulaAPI();
            UInt32 deviceCount = 0;

            VzDeviceHandle deviceHandle = new IntPtr();
            VzReturnStatus status = VzReturnStatus.VzRetOthers;

            //about frame

            VzFrameReady FrameReady = new VzFrameReady();

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
            int resolution_w = 800;
            int resolution_h = 600;
            status = VNAPI.VN_SetColorResolution(deviceHandle, resolution_w, resolution_h);
            if (status != VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetColorResolution failed status:" + status);
                return;
            }
            else
            {
                Console.WriteLine("set to 800_600");
            }
            //Wait for the device to upload image data
            Thread.Sleep(1000);

            //set Mapper
            status = VNAPI.VN_SetTransformDepthImgToColorSensorEnabled(deviceHandle, 1);
            if (status == VzReturnStatus.VzRetOK)
            {
                Console.WriteLine("VN_SetTransformDepthImgToColorSensorEnabled Enabled");
            }

            //1.ReadNextFrame.
            //2.GetFrame acoording to Ready flag and Frametype.
            for (int i = 0; i < 20; i++)
            {
                status = VNAPI.VN_GetFrameReady(deviceHandle, 1200, ref FrameReady);
                if (status != VzReturnStatus.VzRetOK)
                {
                    Console.WriteLine("VN_GetFrameReady failed status:" + status);
                    continue;
                }

                if (1 == FrameReady.transformedDepth)
                {
                    VzFrame frame = new VzFrame();
                    status = VNAPI.VN_GetFrame(deviceHandle, VzFrameType.VzTransformDepthImgToColorSensorFrame, ref frame);
                    if (status == VzReturnStatus.VzRetOK && frame.pFrameData != IntPtr.Zero)
                    {
                        Console.WriteLine("get Frame successful,status:" + status + "  "
                            + "frameTpye:" + frame.frameType + "  "
                            + "frameIndex:" + frame.frameIndex);
                        // once save
                        int len = frame.width * frame.height;
                        VzVector3f[] worldV = new VzVector3f[len];
                        Console.WriteLine(frame.width + "," + frame.height);
                        frame.frameType = VzFrameType.VzTransformDepthImgToColorSensorFrame;

                        status = VNAPI.VN_ConvertDepthFrameToPointCloudVector(deviceHandle, ref frame, ref worldV[0]); //Convert Depth frame to World vectors.
                        //if (status == VzReturnStatus.VzRetOK)
                        {
                            try
                            {
                                StreamWriter PointCloudWriter = new StreamWriter("PointCloud.txt");
                                for (int index = 0; index < len; index++)
                                {
                                    if (0 < worldV[index].z && worldV[index].z < 0xFFFF)
                                    {
                                        string strBuf = worldV[index].x + "\t" + worldV[index].y + "\t" + worldV[index].z;
                                        PointCloudWriter.WriteLine(strBuf);
                                    }
                                }
                                PointCloudWriter.Flush();
                                PointCloudWriter.Close();
                            }
                            catch (Exception e)
                            {
                                Console.WriteLine("Exception: " + e.Message);
                            }
                            finally
                            {
                                Console.WriteLine("Save point cloud successful in PointCloud.txt");
                            }
                            break;
                        }
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
