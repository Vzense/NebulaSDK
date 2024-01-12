from pickle import FALSE, TRUE
import sys
sys.path.append('../../../')

from API.VzenseDS_api import *
import time

camera = VzenseTofCam()


camera_count = camera.VZ_GetDeviceCount()
retry_count = 100
while camera_count < 2 and retry_count > 0:
    retry_count = retry_count-1
    camera_count = camera.VZ_GetDeviceCount()
    time.sleep(1)
    print("scaning......   ",retry_count)


if camera_count < 2: 
    print("there are no camera or only one camera")
    exit()

print("cam count :",camera_count)
cameras = []

ret, device_infolist=camera.VZ_GetDeviceInfoList(camera_count)
if ret==0:
    for i in range(camera_count): 
        print('cam uri:  ' + str(device_infolist[i].uri))
        cam = VzenseTofCam()
        ret = cam.VZ_OpenDeviceByUri(device_infolist[i].uri)
        if  ret == 0:
            print(device_infolist[i].alias,"open successful")
            cameras.append(cam)
        else:
            print(device_infolist[i].alias,'VZ_OpenDeviceByUri failed: ' + str(ret))    
else:
    print(' failed:' + ret)  
    exit()  

for i in range(camera_count): 
    ret = cameras[i].VZ_StartStream()       
    if  ret == 0:
        print(device_infolist[i].alias,"start stream successful")
    else:
        print(device_infolist[i].alias,'VZ_StartStream failed: ' + str(ret))  

# show image 

while 1:
    for i in range(camera_count): 
        ret, frameready = cameras[i].VZ_GetFrameReady(c_uint16(1200))
        if  ret !=0:
            print("VZ_GetFrameReady failed:",ret)
            continue
                        
        if  frameready.depth:      
            ret,depthframe = cameras[i].VZ_GetFrame(VzFrameType.VzDepthFrame)
            if  ret == 0:
                print(device_infolist[i].alias,"  depth frameindex: ",depthframe.frameIndex)
            else:
                print("VZ_GetFrame error", ret)
        if  frameready.ir:
            ret,irframe = cameras[i].VZ_GetFrame(VzFrameType.VzIRFrame)
            if  ret == 0:
                print(device_infolist[i].alias,"  ir frameindex: ",irframe.frameIndex)
            else:
                print("VZ_GetFrame error", ret)

for i in range(camera_count): 
    
    ret = cameras[i].VZ_StopStream()       
    if  ret == 0:
        print("stop stream successful")
    else:
        print('VZ_StopStream failed: ' + str(ret))  

    ret = cameras[i].VZ_CloseDevice()       
    if  ret == 0:
        print("close device successful")
    else:
        print('VZ_CloseDevice failed: ' + str(ret))  
    
           