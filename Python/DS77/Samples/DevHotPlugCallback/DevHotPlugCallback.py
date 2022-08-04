from pickle import FALSE, TRUE
import sys
sys.path.append('../../../')

from DS77.API.VzenseDS77_api import *
import time

camera = VzenseTofCam()


def HotPlugStateCallback(type_struct,  state = c_int32(0)):
    global camera
    if state ==0:
        print(str(type_struct.contents.alias) + "   add")
        ret = camera.VZ_OpenDeviceByUri(type_struct.contents.uri)
        if  ret == 0:
            print(str(type_struct.contents.alias) + " open success")
        else:
            print(str(type_struct.contents.alias) + " open failed",ret)
        ret = camera.VZ_StartStream()
        if  ret == 0:
            print(str(type_struct.contents.alias) + " startstream success")
        else:
            print(str(type_struct.contents.alias) + " startstream failed",ret)
    else:
        print(str(type_struct.contents.alias) + "   remove")
        ret = camera.VZ_StopStream()
        if  ret == 0:
            print(str(type_struct.contents.alias) + " stopstream success")
        else:
            print(str(type_struct.contents.alias) + " stopstream failed",ret)
        ret = camera.VZ_CloseDevice()
        if  ret == 0:
            print(str(type_struct.contents.alias) + " close success")
        else:
            print(str(type_struct.contents.alias) + " close failed",ret)

camera.VZ_SetHotPlugStatusCallback(HotPlugStateCallback)

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

print("uri: "+str(device_info.uri))
ret = camera.VZ_OpenDeviceByUri(device_info.uri)

if  ret == 0 or ret == -103:
    ret = camera.VZ_StartStream()
    if  ret == 0:
        print("startstream success")
    else:
        print("startstream failed",ret)

    while 1:
        ret, frameready = camera.VZ_GetFrameReady(c_uint16(80))
        if  ret !=0:
            print("VZ_GetFrameReady failed:",ret)
            time.sleep(1)
            continue
  
        if  frameready.depth:      
            ret,frame = camera.VZ_GetFrame(VzFrameType.VzDepthFrame)
            if  ret ==0:
                print("frameIndex: ",frame.frameIndex)
            else:
                print("get depth frame failed ",ret)
            time.sleep(1)
            continue
else:
    print('VZ_OpenDeviceByUri failed: ' + str(ret))  

ret = camera.VZ_StopStream()
if  ret == 0:
    print("stopstream success")
else:
    print("stopstream failed",ret)

ret = camera.VZ_CloseDevice()     
if  ret == 0:
    print("close device successful")
else:
    print('VZ_CloseDevice failed: ' + str(ret))   
                       
