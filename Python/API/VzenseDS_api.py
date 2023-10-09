from API.VzenseDS_define import * 
import platform
import os
import ctypes
from ctypes import *
gCallbackFuncList=[]

class VzenseTofCam():
    device_handle = c_void_p(0)
    def __init__(self):
        system_ = platform.system().lower()
        machine_ = platform.machine().lower()
        architecture_ = platform.architecture()[0]
        print('system:',system_)
        print('machine_:',machine_)
        print('architecture:',architecture_)
        if system_ == 'linux':
            if machine_ == 'x86_64':
                os_info = os.uname()
                print('os_info:',type(os_info))
                system_info = os_info.version
                print('version:',system_info)
                if system_info.find('18.04') != -1 or system_info.find('20.04') != -1 or system_info.find('22.04') != -1:
                    libpath = (os.path.abspath(os.path.dirname(os.path.realpath(__file__)) + os.path.sep))+"/Ubuntu18.04/Lib/libNebula_api.so"
                    print(libpath)
                    self.vz_cam_lib = cdll.LoadLibrary(libpath)
                else:
                    libpath = (os.path.abspath(os.path.dirname(os.path.realpath(__file__)) + os.path.sep))+"/Ubuntu16.04/Lib/libNebula_api.so"
                    print(libpath)
                    self.vz_cam_lib = cdll.LoadLibrary(libpath)
            elif machine_ == 'aarch64':
                libpath = (os.path.abspath(os.path.dirname(os.path.realpath(__file__)) + os.path.sep))+"/AArch64/Lib/libNebula_api.so"
                print(libpath)
                self.vz_cam_lib = cdll.LoadLibrary(libpath)
            else:
                print('do not supported OS', system_, machine_)
                exit()
        elif platform.system() == 'Windows':
            if machine_ == 'amd64':
                if architecture_ == '64bit':
                    libpath = (os.path.abspath(os.path.dirname(os.path.realpath(__file__)) + os.path.sep))+"/Windows/Bin/x64/Nebula_api.dll"
                    print(libpath)
                    self.vz_cam_lib = cdll.LoadLibrary(libpath)
                else:
                    libpath = (os.path.abspath(os.path.dirname(os.path.realpath(__file__)) + os.path.sep))+"/Windows/Bin/x86/Nebula_api.dll"
                    print(libpath)
                    self.vz_cam_lib = cdll.LoadLibrary(libpath)
            else:
                print('do not supported OS', system_, machine_)
                exit()
        else:
            print('do not supported OS', system_, machine_)
            exit()
            
        self.device_handle = c_void_p(0)
        self.vz_cam_lib.VZ_Initialize()
        
    def __del__(self):
        self.vz_cam_lib.VZ_Shutdown()
    
    def VZ_GetSDKVersion(self): 
        tmp = c_char * 64
        version = tmp()
        return self.vz_cam_lib.VZ_GetSDKVersion(version),version.value
 
    def VZ_GetDeviceCount(self):
        count = c_int()
        self.vz_cam_lib.VZ_GetDeviceCount(byref(count))
        return count.value
    
    def VZ_GetDeviceInfoList(self, cam_count = 1):
        tmp  = VzDeviceInfo* cam_count
        device_infolist = tmp() 
        return self.vz_cam_lib.VZ_GetDeviceInfoList(cam_count, device_infolist),device_infolist
    
    def VZ_GetDeviceInfo(self, cam_index = 0):
        device_info = VzDeviceInfo()
        return self.vz_cam_lib.VZ_GetDeviceInfo(cam_index,byref(device_info)), device_info
         
    def VZ_OpenDeviceByUri(self,  uri=c_char_p()):
        if uri:
            return self.vz_cam_lib.VZ_OpenDeviceByUri(uri, byref(self.device_handle))
        else:
            return VzReturnStatus.VzRetInputPointerIsNull
    
    def VZ_OpenDeviceByAlias(self,  alias=c_char_p()):
        if alias:
            return self.vz_cam_lib.VZ_OpenDeviceByAlias(alias, byref(self.device_handle))
        else:
            return VzReturnStatus.VzRetInputPointerIsNull
    
    def VZ_OpenDeviceByIP(self,  ip=c_char_p()):
        if ip:
            return self.vz_cam_lib.VZ_OpenDeviceByIP(ip, byref(self.device_handle))
        else:
            return VzReturnStatus.VzRetInputPointerIsNull, 0
        
    def VZ_CloseDevice(self):
        return self.vz_cam_lib.VZ_CloseDevice(byref(self.device_handle))

    def VZ_StartStream(self):
        return self.vz_cam_lib.VZ_StartStream(self.device_handle)
         
    def VZ_StopStream(self):
        return self.vz_cam_lib.VZ_StopStream(self.device_handle)
         
    def VZ_GetFrameReady(self,waitTime = c_uint16(33)):
        frameready = VzFrameReady()
        if not self.device_handle:
            return -3, frameready
        return self.vz_cam_lib.VZ_GetFrameReady(self.device_handle, waitTime, byref(frameready)), frameready

    def VZ_GetFrame(self,  frametype = VzFrameType.VzDepthFrame):   
        Vzframe = VzFrame() 
        return self.vz_cam_lib.VZ_GetFrame(self.device_handle, frametype.value, byref(Vzframe)), Vzframe
       
    def VZ_SetWorkMode(self,  mode = VzWorkMode.ActiveMode):
        return self.vz_cam_lib.VZ_SetWorkMode(self.device_handle, mode.value) 
               
    def VZ_GetWorkMode(self):
        mode = VzWorkMode(0)
        return self.vz_cam_lib.VZ_GetWorkMode(self.device_handle, byref(mode)), mode

    def VZ_SetSoftwareSlaveTrigger(self):
        return self.vz_cam_lib.VZ_SetSoftwareSlaveTrigger(self.device_handle)
    
    def VZ_RebootDevie(self):
        return self.vz_cam_lib.VZ_RebootDevie(self.device_handle)
    
    def VZ_GetSensorIntrinsicParameters(self, sensorType = VzSensorType.VzToFSensor):
        CameraParameters = VzSensorIntrinsicParameters()
        return self.vz_cam_lib.VZ_GetSensorIntrinsicParameters(self.device_handle, sensorType.value, byref(CameraParameters)), CameraParameters

    def VZ_GetSensorExtrinsicParameters(self):
        CameraExtrinsicParameters = VzSensorExtrinsicParameters()
        return self.vz_cam_lib.VZ_GetSensorExtrinsicParameters(self.device_handle, byref(CameraExtrinsicParameters)), CameraExtrinsicParameters
    
    def VZ_GetFirmwareVersion(self): 
        tmp = c_char * 64
        fw = tmp()
        return self.vz_cam_lib.VZ_GetFirmwareVersion(self.device_handle, fw, 63),fw.value

    def VZ_GetDeviceMACAddress(self):
        tmp = c_char * 18
        mac = tmp()
        return self.vz_cam_lib.VZ_GetDeviceMACAddress(self.device_handle, mac), mac.value

    def VZ_SetIRGMMGain(self, gmmgain = c_uint8(20)):
        return self.vz_cam_lib.VZ_SetIRGMMGain(self.device_handle, gmmgain) 
     
    def VZ_GetIRGMMGain(self):
        gmmgain = c_uint8(1)
        return self.vz_cam_lib.VZ_GetIRGMMGain(self.device_handle, byref(gmmgain)), gmmgain.value

    def VZ_SetColorPixelFormat(self,pixelFormat=VzPixelFormat.VzPixelFormatBGR888):
        return self.vz_cam_lib.VZ_SetColorPixelFormat(self.device_handle, pixelFormat)

    def VZ_SetColorResolution(self, w = c_int32(1600), h = c_int32(1200)):
        return self.vz_cam_lib.VZ_SetColorResolution(self.device_handle, w, h) 
     
    def VZ_GetColorResolution(self):
        w = c_int32(1600)
        h = c_int32(1200)
        return self.vz_cam_lib.VZ_GetColorResolution(self.device_handle, byref(w), byref(h)), w, h
        
    def VZ_SetFrameRate(self, value = c_uint8(30)):
        return self.vz_cam_lib.VZ_SetFrameRate(self.device_handle, value) 
     
    def VZ_GetFrameRate(self):
        value = c_uint8(1)
        return self.vz_cam_lib.VZ_GetFrameRate(self.device_handle, byref(value)), value.value

    def VZ_SetExposureControlMode(self, sensorType = VzSensorType.VzToFSensor, mode = VzExposureControlMode.VzExposureControlMode_Manual):
        return self.vz_cam_lib.VZ_SetExposureControlMode(self.device_handle, sensorType.value, mode.value) 
    
    def VZ_SetExposureTime(self, sensorType = VzSensorType.VzToFSensor, params = c_int32(0)):
        ExposureTime = VzExposureTimeParams(VzExposureControlMode.VzExposureControlMode_Manual.value,params) 
        return self.vz_cam_lib.VZ_SetExposureTime(self.device_handle, sensorType.value, ExposureTime) 
     
    def VZ_GetExposureTime(self, sensorType = VzSensorType.VzToFSensor):
        params = VzExposureTimeParams(VzExposureControlMode.VzExposureControlMode_Manual.value,0)
        return self.vz_cam_lib.VZ_GetExposureTime(self.device_handle, sensorType.value, byref(params)), params
 
    def VZ_SetTimeFilterParams(self, params = VzTimeFilterParams()): 
        return self.vz_cam_lib.VZ_SetTimeFilterParams(self.device_handle, params)
    
    def VZ_GetTimeFilterParams(self): 
        params = VzTimeFilterParams()
        return self.vz_cam_lib.VZ_GetTimeFilterParams(self.device_handle, byref(params)),params

    def VZ_SetConfidenceFilterParams(self, params = VzConfidenceFilterParams()): 
        return self.vz_cam_lib.VZ_SetConfidenceFilterParams(self.device_handle, params)
    
    def VZ_GetConfidenceFilterParams(self): 
        params = VzConfidenceFilterParams()
        return self.vz_cam_lib.VZ_GetConfidenceFilterParams(self.device_handle, byref(params)),params
    
    def VZ_SetFlyingPixelFilterParams(self, params = VzFlyingPixelFilterParams()): 
        return self.vz_cam_lib.VZ_SetFlyingPixelFilterParams(self.device_handle, params)
    
    def VZ_GetFlyingPixelFilterParams(self): 
        params = VzFlyingPixelFilterParams()
        return self.vz_cam_lib.VZ_GetFlyingPixelFilterParams(self.device_handle, byref(params)),params

    def VZ_SetFillHoleFilterEnabled(self, enable = c_bool(True)): 
        return self.vz_cam_lib.VZ_SetFillHoleFilterEnabled(self.device_handle, enable)
    
    def VZ_GetFillHoleFilterEnabled(self): 
        enable = c_bool(True)
        return self.vz_cam_lib.VZ_GetFillHoleFilterEnabled(self.device_handle, byref(enable)),enable.value

    def VZ_SetSpatialFilterEnabled(self, enable = c_bool(True)): 
        return self.vz_cam_lib.VZ_SetSpatialFilterEnabled(self.device_handle, enable)
    
    def VZ_GetSpatialFilterEnabled(self): 
        enable = c_bool(True)
        return self.vz_cam_lib.VZ_GetSpatialFilterEnabled(self.device_handle, byref(enable)),enable.value

    def VZ_SetTransformColorImgToDepthSensorEnabled(self, enabled = c_bool(True)): 
        return self.vz_cam_lib.VZ_SetTransformColorImgToDepthSensorEnabled(self.device_handle,  enabled)
    
    def VZ_GetTransformColorImgToDepthSensorEnabled(self): 
        enabled = c_bool(True)
        return self.vz_cam_lib.VZ_GetTransformColorImgToDepthSensorEnabled(self.device_handle,  byref(enabled)),enabled

    def VZ_SetTransformDepthImgToColorSensorEnabled(self, enabled = c_bool(True)): 
        return self.vz_cam_lib.VZ_SetTransformDepthImgToColorSensorEnabled(self.device_handle,  enabled)
    
    def VZ_GetTransformDepthImgToColorSensorEnabled(self): 
        enabled = c_bool(True)
        return self.vz_cam_lib.VZ_GetTransformDepthImgToColorSensorEnabled(self.device_handle,  byref(enabled)),enabled

    def VZ_ConvertDepthFrameToPointCloudVector(self, depthFrame = VzFrame()): 
        len = depthFrame.width*depthFrame.height
        tmp =VzVector3f*len
        pointlist = tmp()
        return self.vz_cam_lib.VZ_ConvertDepthFrameToPointCloudVector(self.device_handle, byref(depthFrame) ,pointlist),pointlist
 
    def VZ_SetHotPlugStatusCallback(self,callbackfunc= c_void_p): 
        callbackFunc_= ctypes.CFUNCTYPE(c_void_p,POINTER(VzDeviceInfo),c_int32)(callbackfunc)    
        gCallbackFuncList.append(callbackFunc_)
        return self.vz_cam_lib.VZ_SetHotPlugStatusCallback(callbackFunc_)

    def VZ_GetManaulMaxExposureTime(self):     
        tmp = r"Py_ToFExposureTimeMax"
        propertyKey = (c_char * 100)(*bytes(tmp, 'utf-8')) 
        maxExposureTime = VzExposureTimeParams(VzExposureControlMode.VzExposureControlMode_Manual.value) 
        return self.vz_cam_lib.VZ_GetProperty(self.device_handle,   byref(propertyKey),  byref(maxExposureTime),8),maxExposureTime.exposureTime

    def VZ_SetParamsByJson(self, imgpath):
        pimgpath = (c_char * 1000)(*bytes(imgpath, 'utf-8'))
        return self.vz_cam_lib.VZ_SetParamsByJson(self.device_handle,  byref(pimgpath))

    def VZ_SetColorGain(self, params = c_float(1.0)):
        return self.vz_cam_lib.VZ_SetColorGain(self.device_handle,  params)

    def VZ_GetColorGain(self):
        tmp = c_float*1
        params = tmp()
        return self.vz_cam_lib.VZ_GetColorGain(self.device_handle,  params), params

    def VZ_GetProperty(self, propertyKey=c_char_p(), pData= c_void_p, dataSize = c_int32(8)):
        return self.vz_cam_lib.VZ_GetProperty(self.device_handle,  propertyKey, pData, dataSize),pData

    def VZ_GetAutoMaxExposureTime(self):
        tmp = r"Py_RGBExposureTimeMax"
        propertyKey = (c_char * 100)(*bytes(tmp, 'utf-8'))
        maxExposureTime = VzExposureTimeParams(VzExposureControlMode.VzExposureControlMode_Auto.value)
        return self.vz_cam_lib.VZ_GetProperty(self.device_handle, byref(propertyKey), byref(maxExposureTime),
                                              8), maxExposureTime.exposureTime

    def VZ_GetManualMaxExposureTime(self):
        tmp = r"Py_RGBExposureTimeMax"
        propertyKey = (c_char * 100)(*bytes(tmp, 'utf-8'))
        maxExposureTime = VzExposureTimeParams(VzExposureControlMode.VzExposureControlMode_Manual.value)
        return self.vz_cam_lib.VZ_GetProperty(self.device_handle, byref(propertyKey), byref(maxExposureTime),
                                              8), maxExposureTime.exposureTime

    def VZ_SetAutoExposureTime(self, sensorType = VzSensorType.VzColorSensor, params = c_int32(0)):
        ExposureTime = VzExposureTimeParams(VzExposureControlMode.VzExposureControlMode_Auto.value,params)
        return self.vz_cam_lib.VZ_SetExposureTime(self.device_handle, sensorType.value, ExposureTime)

    def VZ_GetAutoExposureTime(self, sensorType = VzSensorType.VzColorSensor):
        params = VzExposureTimeParams(VzExposureControlMode.VzExposureControlMode_Auto.value,0)
        return self.vz_cam_lib.VZ_GetExposureTime(self.device_handle, sensorType.value, byref(params)), params