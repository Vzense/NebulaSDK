#ifndef VZENSE_ENUMS_H
#define VZENSE_ENUMS_H

/**
 * @brief Depth range setting.\n 
 *        These set estimated ranges. Detection distances may be greater than what is listed for the given setting. \n 
 *        Precision and minimum distance for depth detection varies with longer ranges.
 */
typedef enum {
	PsUnknown = -1,
	PsNearRange = 0,
	PsMidRange = 1,
	PsFarRange = 2,
	PsXNearRange = 3,
	PsXMidRange = 4,
	PsXFarRange = 5
}PsDepthRange;

/** 
 * @brief The data modes that determine the frame output from the device and the frame rate (fps). 
 */
typedef enum{
    PsModeSingleFreq = 3,
    PsModeDualFreq = 5,
}PsDataMode;

/**
 * @brief Camera device properties to get or set on a device.
 */
typedef enum{
	PsPropertySN_Str = 5,           //!< Device serial number (e.g.PD7110CGC9270020W). The maximum length is 63 bytes.
	PsPropertyFWVer_Str = 6,        //!< Device firmware version number (e.g. DCAM710_c086_pc_sv0.01_R6_20180917_b35). The maximum length is 63 bytes.
	PsPropertyHWVer_Str = 7,        //!< Device hardware version number (e.g. R6). The maximum length is 63 bytes.
	PsPropertyDataMode_UInt8 = 8,   //!< Sets the data mode when invoking ::PsSetDataMode(). See ::PsDataMode for more information.
	PsPropertyDataModeList = 9,		//!< Gets the data mode lists that the device support
	PsPropertyDepthRangeList = 10,	//!< Gets the depth range lists that the device support


	PsPropertyDeviceUpgradeFlag = 11,	//!< Sets Gets the network device UpgradeFlag
	PsPropertyDeviceMACAddr = 13,		//!< Sets Gets the network device MACAddr 
	PsPropertyDeviceIPAddr = 15,		//!< Sets Gets the network device IPAddress
	PsPropertyDeviceSubnetMask = 16,	//!< Sets Gets the network device SubnetMask

	PsPropertyDSP = 17,					//!< Sets Gets DSP status
	
	PsPropertyUpgradeIsSupported = 18,	//Get Upgrade is Supported or not 
	PsPropertyUpgradeState = 19,		//Set/Get Upgrade State 

}PsPropertyType;

/**
 * @brief Specifies the type of image frame.
 */
typedef enum{
	PsDepthFrame = 0,              //!< Depth frame with 16 bits per pixel in millimeters.
	PsIRFrame = 1,                 //!< IR frame with 16 bits per pixel.
	PsColorFrame = 3,              //!< Color frame with 24 bits per pixel in RGB/BGR format.
	PsTransformedColorFrame = 4,   //!< Color frame with 24 bits per pixel in RGB/BGR format, that is transformed to depth camera space where the resolution is the same as the depth frame's resolution.\n 
	                               //!< This frame type can be enabled using ::VZCT_SetTransformColorImgToDepthCameraEnabled().
	PsTransformedDepthFrame = 5,   //!< Depth frame with 16 bits per pixel, in millimeters, that is transformed to color camera space where the resolution is same as the color frame's resolution.\n 
	                               //!< This frame type can be enabled using ::VZCT_SetTransformDepthImgToColorCameraEnabled().
	PsConfidenceFrame = 8,         //!< Confidence frame with 16 bits per pixel.
}PsFrameType;

/**
 * @brief Specifies the type of camera sensor.
 */
typedef enum{
	PsToFSensor = 0x01,          //!< ToF camera.
	PsColorSensor = 0x02         //!< Color camera.
}PsSensorType;

/**
 * @brief Specifies the image pixel format.
 */
typedef enum{
	PsPixelFormatDepthMM16,        //!< Depth image pixel format, 16 bits per pixel in mm.
	PsPixelFormatGray16,           //!< IR image pixel format, 16 bits per pixel.
	PsPixelFormatGray8,            //!< Gray image pixel format, 8 bits per pixel.

	//Color
	PsPixelFormatRGB888,           //!< Color image pixel format, 24 bits per pixel RGB format.
	PsPixelFormatBGR888            //!< Color image pixel format, 24 bits per pixel BGR format.
}PsPixelFormat;

/**
 * @brief Return status codes for all APIs.\n 
 * 		  <code>PsRetOK = 0</code> means the API successfully completed its operation.\n 
 * 		  All other codes indicate a device, parameter, or API usage error.
 */
typedef enum
{
    PsRetOK                         =  0,   //!< The function completed successfully.
    PsRetNoDeviceConnected          = -1,   //!< There is no depth camera connected or the camera has not been connected correctly. Check the hardware connection or try unplugging and re-plugging the USB cable.
    PsRetInvalidDeviceIndex         = -2,   //!< The input device index is invalid.
    PsRetDevicePointerIsNull        = -3,   //!< The device structure pointer is null.
    PsRetInvalidFrameType           = -4,   //!< The input frame type is invalid.
    PsRetFramePointerIsNull         = -5,   //!< The output frame buffer is null.
    PsRetNoPropertyValueGet         = -6,   //!< Cannot get the value for the specified property.
    PsRetNoPropertyValueSet         = -7,   //!< Cannot set the value for the specified property.
    PsRetPropertyPointerIsNull      = -8,   //!< The input property value buffer pointer is null.
    PsRetPropertySizeNotEnough      = -9,   //!< The input property value buffer size is too small to store the specified property value.
    PsRetInvalidDepthRange          = -10,  //!< The input depth range mode is invalid.
    PsRetReadNextFrameTimeOut       = -11,  //!< Capture the next image frame time out.
    PsRetInputPointerIsNull         = -12,  //!< An input pointer parameter is null.
    PsRetCameraNotOpened            = -13,  //!< The camera has not been opened.
    PsRetInvalidCameraType          = -14,  //!< The specified type of camera is invalid.
    PsRetInvalidParams              = -15,  //!< One or more of the parameter values provided are invalid.
    PsRetCurrentVersionNotSupport   = -16,  //!< This feature is not supported in the current version.
    PsRetUpgradeImgError            = -17,  //!< There is an error in the upgrade file.
    PsRetUpgradeImgPathTooLong      = -18,  //!< Upgrade file path length greater than 260.
	PsRetUpgradeCallbackNotSet		= -19,  //!< Ps2_SetUpgradeStatusCallback is not called.
	PsRetNoAdapterConnected			= -100,	//!< There is no adapter connected
	PsRetReInitialized				= -101,	//!< The SDK has been Initialized
	PsRetNoInitialized				= -102,	//!< The SDK has bot been Initialized
	PsRetCameraOpened				= -103,	//!< The camera has been opened.
	PsRetCmdError					= -104,	//!< Set/Get cmd control error
	PsRetCmdSyncTimeOut				= -105,	//!< Set cmd ok.but time out for the sync return 
    PsRetIPNotMatch                 = -106, //!< IP is not in the same network segment
    PsRetNotStopStream              = -107, //!< Please invoke Ps2_StopStream first to close the data stream

	PsRetOthers = -255,	             //!< An unknown error occurred.
}PsReturnStatus;

/** 
 * @brief Specifies the filter type.
 */
typedef enum
{
	PsComputeRealDepthFilter = 0,   //!< Compute the real depth, in the depth camera coordinate system. Enabled by default.
	PsSmoothingFilter,              //!< Smoothing filter. Enabled by default.
}PsFilterType;

/** @brief Stream type 
 */
typedef enum {
	PsStreamDepth = 0,             //!< Depth Stream
	PsStreamIR = 1,                //!< IR Stream
	PsStreamColor = 2,             //!< Color Stream
	PsStreamAudio = 3,             //!< Audio Stream
	PsStreamIMU = 4                //!< IMU Data Stream
}PsStreamType;

/**	@brief	rgb supported resolution 
 */
typedef enum {
    PsColor_Resolution_640_480 = 2,
    PsColor_Resolution_1600_1200 = 4,
    PsColor_Resolution_800_600 = 5
}PsResolution;

typedef enum
{
	LinkeUNKNOWN = 0,
	LinkUSB = 1,
	LinkSocket = 2,
	LinkMIPI = 3,
}PsLinkType;

typedef enum {
	ConnectUNKNOWN,
	Unconnected,
	Connected,
	Opened,
	UpgradeUnconnected,
	UpgradeConnected,
}PsConnectStatus;

typedef enum
{
	NONE,
    DS77Lite = 770,
    DS77CLite = 771,
    DS77Pro = 772,
    DS77CPro = 773,
	MAX,
}PsDeviceType;

typedef enum
{
	PsDsp_Depth_Enable = 0,
	PsDsp_Ir_Enable,
	PsDsp_Color_Enable,
 	PsDsp_Depth_ComputeRealDepthByIntrinsic,
	PsDsp_Depth_SpatialFilter,
	PsDsp_Depth_TimeFilter,
	PsDsp_Depth_Distortion,
    PsDsp_Depth_ComputeRealDepthByPlaneness,
	PsDsp_Color_Distortion,
	PsDsp_IR_Distortion,
    PsDsp_Depth_ConfidenceFilter,
    PsDsp_Depth_OverexposureFilter,
	PsDsp_DepthFusion_Filter,
}PsDSPType;

typedef enum
{
        DEVICE_NORMAL = 0x00,
        DEVICE_UPGRADE_BEGIN = 0x01,
        DEVICE_PRE_UPGRADE_IMG_COPY = 0x02,
        DEVICE_UPGRADE_IMG_CHECK_DOING = 0x03,
        DEVICE_UPGRADE_IMG_CHECK_DONE = 0x04,
        DEVICE_UPGRADE_UPGRAD_DOING = 0x05,
        DEVICE_UPGRADE_RECHECK_DOING = 0x06,
        DEVICE_UPGRADE_RECHECK_DONE = 0x07,
        DEVICE_UPGRADE_UPGRAD_DONE = 0x08,
}PsDeviceStatus;

typedef enum
{
	DEVICE_JSON_NORMAL = 0x00,
	DEVICE_UPGRADE_JSON_BEGIN = 0x01,
	DEVICE_PRE_UPGRADE_JSON_COPY = 0x02,
	DEVICE_UPGRADE_JSON_CHECK_DOING = 0x03,
	DEVICE_UPGRADE_JSON_CHECK_DONE = 0x04,
	DEVICE_UPGRADE_JSON_UPGRAD_DOING = 0x05,
	DEVICE_UPGRADE_JSON_UPGRAD_DONE = 0x06,
 }PsUpgradeJsonStatus;

typedef enum
{
    SlaveOFF = 0x00,                    //exit the salve mode
    SlaveForHardwareTrigger = 0x01,     //enter the hardware salve mode, at this time need to connect the hardware trigger wire, provide hardware signal, to trigger the image
    SlaveForSoftwareTrigger = 0x02,     //enter the hardware salve mode, at this time need to invoke VZCT_SetSoftwareSlaveTrigger, to trigger the image
}PsSlaveModeStatus;

#endif /* VZENSE_ENUMS_H */

