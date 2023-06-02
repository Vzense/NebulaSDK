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

'''Get default frame rate'''
ret, rate = camera.VZ_GetFrameRate()
if ret != 0:
    print("VZ_GetFrameRate failed status:"+ str(ret))
    exit()

'''if need change the framerate, do first'''
print("---- To  VzExposureControlMode_Manual ----")
'''switch exposure mode to manual'''
ret = camera.VZ_SetExposureControlMode(VzSensorType.VzColorSensor,VzExposureControlMode.VzExposureControlMode_Manual)

if ret != 0:
    print("VZ_SetExposureControlMode failed status:" + str(ret))
    exit()
else:
    print("VZ_SetExposureControlMode  ok")

print("* step1. Get Color exposure time range with frameRate " + str(rate) + "*")

'''Get the range of the Color exposure time'''
ret, exposureTime = camera.VZ_GetManualMaxExposureTime()
if ret != 0:
    print("VZ_GetProperty get Py_RGBExposureTimeMax failed status:" + str(ret))
    exit()

print("Recommended scope: 100 - " + str(exposureTime))

print("* step2. Set and Get new ExposureTime *" )
'''Set new ExposureTime '''
ret = camera.VZ_SetExposureTime(VzSensorType.VzColorSensor, 3000)
if ret != 0:
    print("VZ_SetExposureTime failed status:" + str(ret))
    exit()
else:
    print("SetExposureTime:3000")


ret, params = camera.VZ_GetExposureTime(VzSensorType.VzColorSensor);
if ret != 0:
    print("VZ_GetExposureTime failed status:" + str(ret))
    exit()
else:
     print("GetExposureTime:" + str(params.exposureTime))

print("---- To VzExposureControlMode_Auto ----")
'''switch exposure mode to auto'''
ret = camera.VZ_SetExposureControlMode(VzSensorType.VzColorSensor, VzExposureControlMode.VzExposureControlMode_Auto)
if 0 != ret:
    print("VZ_SetExposureControlMode failed status:" + str(ret))
    exit()
else:
    print("VZ_SetExposureControlMode ok")

print("* step1. Get Color exposure time range *")
'''Get the range of the Auto Color exposure time'''
ret , exposureTime = camera.VZ_GetAutoMaxExposureTime()
if 0 != ret:
    print("VZ_GetProperty get Py_RGBExposureTimeMax failed status:" + str(ret))
    exit()
print("Recommended scope: 100 - " + str(exposureTime))

print("* step2. Set and Get new Auto Max Color exposure time range *" )
'''set new range of Auto Color exposure time.[100 maxExposureTime.exposureTime]'''
ret = camera.VZ_SetAutoExposureTime(VzSensorType.VzColorSensor, 10000)
if 0 != ret:
    print("VZ_SetExposureTimeAutoMax failed status:" + str(ret))
    exit()
else:
    print("SetExposureTimeAutoMax:10000")

'''Get the new range of the Auto Color exposure time. '''
ret, params = camera.VZ_GetAutoExposureTime(VzSensorType.VzColorSensor);
if 0 != ret:
    print("VZ_GetExposureTimeAutoMax failed status:" + str(ret))
    exit()
else:
    print("GetExposureTimeAutoMax:" + str(params.exposureTime))

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