import os, platform, numpy
from ctypes import *
from enum import Enum

class VzFrameType(Enum):
    VzDepthFrame       = 0     
    VzIRFrame          = 1
    VzConfidenceFrame  = 8 	
        
class VzSensorType(Enum):
    VzDepthSensor = 0x01

class VzPixelFormat(Enum):
    VzPixelFormatDepthMM16 = 0
    VzPixelFormatGray8     = 2   

class VzReturnStatus(Enum):
    VzRetOK                         =  0
    VzRetNoDeviceConnected          = -1
    VzRetInvalidDeviceIndex         = -2
    VzRetDevicePointerIsNull        = -3
    VzRetInvalidFrameType           = -4
    VzRetFramePointerIsNull         = -5
    VzRetNoPropertyValueGet         = -6
    VzRetNoPropertyValueSet         = -7
    VzRetPropertyPointerIsNull      = -8
    VzRetPropertySizeNotEnough      = -9
    VzRetInvalidDepthRange          = -10
    VzRetGetFrameReadyTimeOut       = -11
    VzRetInputPointerIsNull         = -12
    VzRetCameraNotOpened            = -13
    VzRetInvalidCameraType          = -14
    VzRetInvalidParams              = -15
    VzRetCurrentVersionNotSupport   = -16
    VzRetUpgradeImgError            = -17
    VzRetUpgradeImgPathTooLong      = -18
    VzRetUpgradeCallbackNotSet		= -19
    VzRetNoAdapterConnected			= -100
    VzRetReInitialized				= -101
    VzRetNoInitialized				= -102
    VzRetCameraOpened				= -103
    VzRetCmdError					= -104
    VzRetCmdSyncTimeOut				= -105
    VzRetIPNotMatch					= -106
    VzRetNotStopStream              = -107
    VzRetOthers                     = -255

class VzConnectStatus(Enum):
    ConnectUNKNOWN   = 0
    Unconnected      = 1
    Connected        = 2
    Opened           = 3
    UpgradeUnconnected = 4
    UpgradeConnected = 5

class VzDeviceType(Enum):
    NONE      = 0
    DS77Lite  = 770
    DS77CLite = 771
    DS77Pro   = 772
    DS77CPro  = 773

class VzWorkMode(Enum):
    ActiveMode          = 0x00
    HardwareTriggerMode = 0x01
    SoftwareTriggerMode = 0x02
 

