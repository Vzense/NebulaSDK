#ifndef VZENSEDS_ENUMS_H
#define VZENSEDS_ENUMS_H

/**
 * @brief Specifies the type of image frame.
 */
typedef enum{
	VzDepthFrame = 0,                           //!< Depth frame with 16 bits per pixel in millimeters.
	VzIRFrame = 1,                              //!< IR frame with 8 bits per pixel.
	VzColorFrame = 3,                           //!< Color frame with 24 bits per pixel in RGB/BGR format.
    VzTransformColorImgToDepthSensorFrame = 4,  //!< Color frame with 24 bits per pixel in RGB/BGR format, that is transformed to depth sensor space where the resolution is the same as the depth frame's resolution.\n 
	                                            //!< This frame type can be enabled using ::VZ_SetTransformColorImgToDepthSensorEnabled().
	VzTransformDepthImgToColorSensorFrame = 5,  //!< Depth frame with 16 bits per pixel, in millimeters, that is transformed to color sensor space where the resolution is same as the color frame's resolution.\n 
	                                            //!< This frame type can be enabled using ::VZ_SetTransformDepthImgToColorSensorEnabled().
	VzConfidenceFrame = 8,                      //!< Confidence frame with 16 bits per pixel.
}VzFrameType;

/**
 * @brief Specifies the image pixel format.
 */
typedef enum{
	VzPixelFormatDepthMM16 = 0,        //!< Depth image pixel format, 16 bits per pixel in mm.
	VzPixelFormatGray8 = 2,            //!< Gray image pixel format, 8 bits per pixel.

	//Color
	VzPixelFormatRGB888 = 3,           //!< Color image pixel format, 24 bits per pixel RGB format.
	VzPixelFormatBGR888 = 4           //!< Color image pixel format, 24 bits per pixel BGR format.
}VzPixelFormat;

/**
 * @brief Specifies the type of sensor.
 */
typedef enum {
    VzToFSensor = 0x01,          //!< ToF camera.
    VzColorSensor = 0x02         //!< Color camera.
}VzSensorType;

/**
 * @brief Return status codes for all APIs.\n 
 * 		  <code>VzRetOK = 0</code> means the API successfully completed its operation.\n 
 * 		  All other codes indicate a device, parameter, or API usage error.
 */
typedef enum
{
    VzRetOK                         =  0,   //!< The function completed successfully.
    VzRetNoDeviceConnected          = -1,   //!< There is no depth camera connected or the camera has not been connected correctly. Check the hardware connection or try unplugging and re-plugging the USB cable.
    VzRetInvalidDeviceIndex         = -2,   //!< The input device index is invalid.
    VzRetDevicePointerIsNull        = -3,   //!< The device structure pointer is null.
    VzRetInvalidFrameType           = -4,   //!< The input frame type is invalid.
    VzRetFramePointerIsNull         = -5,   //!< The output frame buffer is null.
    VzRetNoPropertyValueGet         = -6,   //!< Cannot get the value for the specified property.
    VzRetNoPropertyValueSet         = -7,   //!< Cannot set the value for the specified property.
    VzRetPropertyPointerIsNull      = -8,   //!< The input property value buffer pointer is null.
    VzRetPropertySizeNotEnough      = -9,   //!< The input property value buffer size is too small to store the specified property value.
    VzRetInvalidDepthRange          = -10,  //!< The input depth range mode is invalid.
    VzRetGetFrameReadyTimeOut       = -11,  //!< Capture the next image frame time out.
    VzRetInputPointerIsNull         = -12,  //!< An input pointer parameter is null.
    VzRetCameraNotOpened            = -13,  //!< The camera has not been opened.
    vzRetInvalidCameraType          = -14,  //!< The specified type of camera is invalid.
    VzRetInvalidParams              = -15,  //!< One or more of the parameter values provided are invalid.
    VzRetCurrentVersionNotSupport   = -16,  //!< This feature is not supported in the current version.
    VzRetUpgradeImgError            = -17,  //!< There is an error in the upgrade file.
    VzRetUpgradeImgPathTooLong      = -18,  //!< Upgrade file path length greater than 260.
	VzRetUpgradeCallbackNotSet		= -19,  //!< VZ_SetUpgradeStatusCallback is not called.
	VzRetProductNotSupport          = -20,  //!< The current product does not support this operation.
	VzRetNoConfigFolder				= -21,  //!< No product profile found.
	VzRetWebServerStartError        = -22,  //!< WebServer Start/Restart error(IP or PORT).
	VzRetGetOverStayFrame           = -23,  //!< The time from frame ready to get frame is out of 1s
	VzRetCreateLogDirError          = -24,  //!< Create log directory error
	VzRetCreateLogFileError			= -25,  //!< Create log file error
	VzRetNoAdapterConnected			= -100,	//!< There is no adapter connected
	VzRetReInitialized				= -101,	//!< The SDK has been Initialized
	VzRetNoInitialized				= -102,	//!< The SDK has not been Initialized
	VzRetCameraOpened				= -103,	//!< The camera has been opened.
	VzRetCmdError					= -104,	//!< Set/Get cmd control error
	VzRetCmdSyncTimeOut				= -105,	//!< Set cmd ok.but time out for the sync return 
    VzRetIPNotMatch                 = -106, //!< IP is not in the same network segment
    VzRetNotStopStream              = -107, //!< Please invoke VZ_StopStream first to close the data stream
    VzRetNotStartStream             = -108, //!< Please invoke VZ_StartStream first to get the data stream
	VzRetNoDriversFolder			= -109, //!< Please invoke VZ_StartStream first to get the data stream

	VzRetOthers = -255,	             //!< An unknown error occurred.
}VzReturnStatus;

typedef enum {
	VzConnectUNKNOWN = 0,
    VzUnconnected = 1,
    VzConnected = 2,
    VzOpened = 3,
    VzUpgradeUnconnected = 4,
    VzUpgradeConnected = 5,
}VzConnectStatus;

typedef enum
{
    VzActiveMode = 0x00,             //enter the active mode
    VzHardwareTriggerMode = 0x01,    //enter the hardware salve mode, at this time need to connect the hardware trigger wire, provide hardware signal, to trigger the image
    VzSoftwareTriggerMode = 0x02,    //enter the software salve mode, at this time need to invoke VZ_SetSoftwareSlaveTrigger, to trigger the image
}VzWorkMode;

typedef enum
{
    VzExposureControlMode_Auto = 0,
    VzExposureControlMode_Manual = 1,
}VzExposureControlMode;


#endif /* VZENSEDS_ENUMS_H */

