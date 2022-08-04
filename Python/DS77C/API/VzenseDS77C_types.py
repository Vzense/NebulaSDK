import os, platform, numpy
from ctypes import *
from enum import Enum
 

class VzRGB888Pixel(Structure):
    _pack_ = 1
    _fields_ = [("r", c_uint8),
                ("g", c_uint8),
                ("b", c_uint8)]

class VzBGR888Pixel(Structure):
    _pack_ = 1
    _fields_ = [("g", c_uint8),
                ("g", c_uint8),
                ("r", c_uint8)]

class VzVector3f(Structure):
    _pack_ = 1
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("z", c_float)]  

class VzVector2u16(Structure):
    _pack_ = 1
    _fields_ = [("x", c_uint16),
                ("y", c_uint16)]  

class VzDepthVector3(Structure):
    _pack_ = 1
    _fields_ = [("depthX", c_int),
                ("depthY", c_int),
                ("depthZ", c_uint16)] 

class VzSensorIntrinsicParameters(Structure):
    _pack_ = 1
    _fields_ = [("fx", c_double),
                ("fy", c_double),
                ("cx", c_double),
                ("cy", c_double),
                ("k1", c_double),
                ("k2", c_double),
                ("p1", c_double),
                ("p2", c_double),
                ("k3", c_double),
                ("k4", c_double),
                ("k5", c_double),
                ("k6", c_double)] 

class VzSensorExtrinsicParameters(Structure):
    _pack_ = 1
    _fields_ = [("rotation", c_double * 9),
                ("translation", c_double * 3)]  

class VzTimeStamp(Structure):
    _pack_ = 1       
    _fields_ = [("tm_sec", c_uint16),
                ("tm_min", c_uint16),
                ("tm_hour", c_uint16),
                ("tm_msec", c_uint16)]     

class VzFrame(Structure):
    _pack_ = 1
    _fields_ = [("frameIndex", c_uint32),
                ("frameType", c_int32),
                ("pixelFormat", c_int32),
                ("pFrameData", POINTER(c_uint8)),
                ("dataLen", c_uint32),
                ("exposureTime", c_float),
                ("depthRange", c_uint8),
                ("width", c_uint16),
                ("height", c_uint16),
                ("hardwaretimestamp", c_uint64)]

class VzFrameReady(Structure):
    _pack_ = 1
    _fields_ = [("depth", c_uint, 1),
                ("ir", c_uint, 1),
                ("color", c_uint, 1),
                ("transformedColor", c_uint, 1),
                ("transformedDepth", c_uint, 1),
                ("transformedIR", c_uint, 1),
                ("confidence", c_uint, 1),
                ("reserved", c_uint, 25)]

class VzDeviceInfo(Structure):
    _pack_ = 1
    _fields_ = [("SessionCount", c_int),
                ("devicetype", c_int32),
                ("uri", c_char * 256),
                ("alias", c_char * 64),                
                ("serialNumber", c_char * 64),
                ("ip", c_char * 17),
                ("status", c_int32)]

class VzConfidenceFilterParams(Structure):
    _pack_ = 1
    _fields_ = [("enable", c_bool),
                ("threshold", c_int32)]

class VzFlyingPixelFilterParams(Structure):
    _pack_ = 1
    _fields_ = [("enable", c_bool),
                ("threshold", c_int32)]

class VzSpatialFilterParams(Structure):
    _pack_ = 1
    _fields_ = [("enable", c_bool),
                ("validCount", c_int32),
                ("threshold", c_int32),
                ("doCount", c_int32)]

class VzFillHoleFilterParams(Structure):
    _pack_ = 1
    _fields_ = [("enable", c_bool),
                ("validCount", c_int32),
                ("threshold", c_int32),
                ("doCount", c_int32)]
 
class VzOverexposureFilterParams(Structure):
    _pack_ = 1
    _fields_ = [("enable", c_bool),
                ("threshold", c_int32)]
 