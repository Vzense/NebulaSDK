from pickle import FALSE, TRUE
import sys
sys.path.append('../../../')

from API.VzenseDS_api import *
import time

camera = VzenseTofCam()


camera_count = camera.VZ_GetDeviceCount()
retry_count = 100
while camera_count==0 and retry_count > 0:
    retry_count = retry_count-1
    camera_count = camera.VZ_GetDeviceCount()
    time.sleep(1)
    print("scaning......   ",retry_count)

device_info=VzDeviceInfo()

if camera_count > 1:
    ret,device_infolist=camera.VZ_GetDeviceInfoList(camera_count)
    if ret==0:
        device_info = device_infolist[0]
        for info in device_infolist: 
            print('cam uri:  ' + str(info.uri))
    else:
        print(' failed:' + ret)  
        exit()  
elif camera_count == 1:
    ret,device_info=camera.VZ_GetDeviceInfo()
    if ret==0:
        print('cam uri:' + str(device_info.uri))
    else:
        print(' failed:' + ret)   
        exit() 
else: 
    print("there are no camera found")
    exit()

if  VzConnectStatus.Connected.value != device_info.status:
	print("connect statu:",device_info.status)  
	print("Call VZ_OpenDeviceByUri with connect status :",VzConnectStatus.Connected.value)
	exit()
else:
    print("uri: "+str(device_info.uri))
    print("alias: "+str(device_info.alias))
    print("connectStatus: "+str(device_info.status))

ret = camera.VZ_OpenDeviceByUri(device_info.uri)
if  ret == 0:
    print("open device successful")
else:
    print('VZ_OpenDeviceByUri failed: ' + str(ret))   

ret = camera.VZ_StartStream()
if  ret == 0:
    print("start stream successful")
else:
    print("VZ_StartStream failed:"+ str(ret))   
 
ret,params = camera.VZ_GetTimeFilterParams()
if  ret == 0:
    print("The default TimeFilter switch is " + str(params.enable))
else:
    print("VZ_GetTimeFilterParams failed:"+ str(ret))   

params.enable = not params.enable
ret = camera.VZ_SetTimeFilterParams(params)
if  ret == 0:
    print("Set TimeFilter switch to "+ str(params.enable) + " is Ok")   
else:
    print("VZ_SetTimeFilterParams failed:"+ str(ret))   

ret,params = camera.VZ_GetConfidenceFilterParams()
if  ret == 0:
    print("The default ConfidenceFilter switch is " + str(params.enable))
else:
    print("VZ_GetConfidenceFilterParams failed:"+ str(ret))   

params.enable = not params.enable
ret = camera.VZ_SetConfidenceFilterParams(params)
if  ret == 0:
    print("Set ConfidenceFilter switch to "+ str(params.enable) + " is Ok")   
else:
    print("VZ_SetConfidenceFilterParams failed:"+ str(ret))   

ret,params = camera.VZ_GetFlyingPixelFilterParams()
if  ret == 0:
    print("The default FlyingPixelFilter switch is " + str(params.enable))
else:
    print("VZ_GetFlyingPixelFilterParams failed:"+ str(ret))   

params.enable = not params.enable
ret = camera.VZ_SetFlyingPixelFilterParams(params)
if  ret == 0:
    print("Set FlyingPixelFilter switch to "+ str(params.enable) + " is Ok")   
else:
    print("VZ_SetFlyingPixelFilterParams failed:"+ str(ret))   

ret,enable = camera.VZ_GetFillHoleFilterEnabled()
if  ret == 0:
    print("The default FillHoleFilter switch is " + str(enable))
else:
    print("VZ_GetFillHoleFilterEnabled failed:"+ str(ret))   

enable = not enable
ret = camera.VZ_SetFillHoleFilterEnabled(enable)
if  ret == 0:
    print("Set FillHoleFilter switch to "+ str(enable) + " is Ok")   
else:
    print("VZ_SetFillHoleFilterEnabled failed:"+ str(ret))   

ret,enable = camera.VZ_GetSpatialFilterEnabled()
if  ret == 0:
    print("The default SpatialFilter switch is " + str(enable))
else:
    print("VZ_GetSpatialFilterEnabled failed:"+ str(ret))   

enable = not enable
ret = camera.VZ_SetSpatialFilterEnabled(enable)
if  ret == 0:
    print("Set SpatialFilter switch to "+ str(enable) + " is Ok")   
else:
    print("VZ_SetSpatialFilterEnabled failed:"+ str(ret))   

ret = camera.VZ_StopStream()       
if  ret == 0:
    print("stop stream successful")
else:
    print('VZ_StopStream failed: ' + str(ret))  

ret = camera.VZ_CloseDevice()     
if  ret == 0:
    print("close device successful")
else:
    print('VZ_CloseDevice failed: ' + str(ret))   
    
print('Test end, please reboot camera to restore the default settings')  