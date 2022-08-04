from pickle import FALSE, TRUE
import sys
sys.path.append('../../../')

from DS77.API.VzenseDS77_api import *
import cv2
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
        print(' failed:' , ret)  
        exit()  
elif camera_count == 1:
    ret,device_info=camera.VZ_GetDeviceInfo()
    if ret==0:
        print('cam uri:' + str(device_info.uri))
    else:
        print(' failed:', ret)   
        exit() 
else: 
    print("there are no camera found")
    exit()

print("uri: "+str(device_info.uri))
ret = camera.VZ_OpenDeviceByUri(device_info.uri)

if  ret == 0:

    ret = camera.VZ_StartStream()
    if  ret == 0:
        print("start stream successful")
    else:
        print("VZ_StartStream failed:",ret)

 
    colorSlope = c_uint16(7495)
    try:
        while 1:

            ret, frameready = camera.VZ_GetFrameReady(c_uint16(80))
            if  ret !=0:
                print("VZ_GetFrameReady failed:",ret)
                time.sleep(1)
                continue

            if  frameready.depth:
                ret,depthframe = camera.VZ_GetFrame(VzFrameType.VzDepthFrame)
                if  ret == 0:
                    frametmp = numpy.ctypeslib.as_array(depthframe.pFrameData, (1, depthframe.width * depthframe.height * 2))
                    frametmp.dtype = numpy.uint16
                    frametmp.shape = (depthframe.height, depthframe.width)

                    #convert ushort value to 0xff is just for display
                    img = numpy.int32(frametmp)
                    img = img*255/colorSlope
                    img = numpy.clip(img, 0, 255)
                    img = numpy.uint8(img)
                    frametmp = cv2.applyColorMap(img, cv2.COLORMAP_RAINBOW)
 
                    cv2.imshow("depthimage", frametmp)
                else:
                    print("get depth frame failed:",ret)
            if  frameready.ir:
                ret,irframe = camera.VZ_GetFrame(VzFrameType.VzIRFrame)
                if  ret == 0:
                    frametmp = numpy.ctypeslib.as_array(irframe.pFrameData, (1, irframe.dataLen))
                    frametmp.dtype = numpy.uint8
                    frametmp.shape = (irframe.height, irframe.width)
                    
                    cv2.imshow("irimage", frametmp)
                else:
                    print("get ir frame failed:",ret)

            key = cv2.waitKey(1)
            if  key == 27:
                cv2.destroyAllWindows()
                print("---end---")
                exit()
                   
    except Exception as e :
        print(e)
    finally :
        print('end')
else:
    print('VZ_OpenDeviceByUri failed: ' + str(ret))   
            

        
