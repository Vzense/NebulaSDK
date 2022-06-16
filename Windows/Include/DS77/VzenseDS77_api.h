#ifndef VZENSEDS_API_H
#define VZENSEDS_API_H

/**
* @file VzenseDS_api.h
* @brief Vzense API header file.
* Copyright (c) 2019-2022 Vzense Interactive, Inc.
*/

/*! \mainpage VzenseDS API Documentation
*
* \section intro_sec Introduction
*
* Welcome to the VzenseDS API documentation. This documentation enables you to quickly get started in your development efforts to programmatically interact with the Vzense CW TOF Camera (eg:DS77).
*/

#include "VzenseDS77_define.h"

/**
* @brief         Initializes the API on the device. This function must be invoked before any other Vzense APIs.
* @return        ::VzRetOK if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_Initialize();

/**
* @brief         Shuts down the API on the device and clears all resources allocated by the API. After invoking this function, no other Vzense APIs can be invoked.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_Shutdown();

/**
* @brief         Gets the version of SDK.
* @param[in]     version         Pointer to a variable in which to store the returned version value.
* @param[in]     length          The maximum length is 63 bytes.
* @return        ::VzRetOK       if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetSDKVersion(char* version, int length);

/**
* @brief         Returns the number of camera devices currently connected.
* @param[out]    pDeviceCount    Pointer to a 32-bit integer variable in which to return the device count.
* @return        ::VzRetOK       if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetDeviceCount(uint32_t* pDeviceCount);

/**
* @brief         Returns the info of the deviceIndex camera device.
* @param[in]     deviceIndex    The index of the device to open. Device indices range from 0 to device count - 1.
* @param[out]    pDevices       Pointer to a buffer in which to store the device info.
* @return        ::VzRetOK      if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetDeviceInfo(VzDeviceInfo* pDevices, uint32_t deviceIndex);

/**
* @brief         Returns the info lists of the deviceCount camera devices.
* @param[in]     deviceCount     the number of camera devices.
* @param[out]    pDevicesInfoList    Pointer to a buffer in which to store the devices list infos.
* @return        ::VzRetOK       if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetDeviceInfoList(VzDeviceInfo* pDevicesInfoList, uint32_t deviceCount);

/**
* @brief         Opens the device specified by <code>uri</code>. The device must be subsequently closed using VZCT_CloseDevice().
* @param[in]     uri          the uri of the device. See ::VzDeviceInfo for more information.
* @param[out]    pDevices     the handle of the device on which to open.
* @return:       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_OpenDevice(const char* uri, VzDeviceHandle* pDevice);

/**
* @brief         Opens the device specified by <code>alias</code>. The device must be subsequently closed using VZCT_CloseDevice().
* @param[in]     alias        the alias of the device. See ::VzDeviceInfo for more information.
* @param[out]    pDevices     the handle of the device on which to open.
* @return:       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_OpenDeviceByAlias(const char* alias, VzDeviceHandle* pDevice);

/**
* @brief         Opens the device specified by <code>uri</code>. The device must be subsequently closed using VZCT_CloseDevice().
* @param[in]     uri          the uri of the device. See ::VzDeviceInfo for more information.
* @param[out]    pDevices     the handle of the device on which to open.
* @return:       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_OpenDeviceByIP(const char* ip, VzDeviceHandle* pDevice);

/**
* @brief         Closes the device specified by <code>device</code> that was opened using VZCT_OpenDevice.
* @param[in]     device       The handle of the device to close.
* @return:       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_CloseDevice(VzDeviceHandle* device);

/**
* @brief         Starts capturing the image stream indicated by <code>device</code>. Invoke VZCT_StopStream() to stop capturing the image stream.
* @param[in]     device          The handle of the device on which to start capturing the image stream.                        
* @return        ::VzRetOK if    the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_StartStream(VzDeviceHandle device);

/**
* @brief         Stops capturing the image stream on the device specified by <code>device</code>. that was started using VZCT_StartStream.
* @param[in]     device       The handle of the device on which to stop capturing the image stream.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_StopStream(VzDeviceHandle device);

/**
* @brief         Captures the next image frame from the device specified by <code>device</code>. This API must be invoked before capturing frame data using VZCT_GetFrame().
* @param[in]     device         The handle of the device on which to read the next frame.
* @param[out]    pFrameReady    Pointer to a buffer in which to store the signal on which image is ready to be get.
* @return        ::VzRetOK      if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_ReadNextFrame(VzDeviceHandle device, VzFrameReady* pFrameReady);

/**
* @brief         Returns the image data for the current frame from the device specified by <code>device</code>. Before invoking this API, invoke VZCT_ReadNextFrame() to capture one image frame from the device.
* @param[in]     device       The handle of the device to capture an image frame from.
* @param[in]     frameType    The image frame type.
* @param[out]    pVzFrame     Pointer to a buffer in which to store the returned image data.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetFrame(VzDeviceHandle device, VzFrameType frameType, VzFrame* pVzFrame);

/**
* @brief         Set the waittime of read next frame.
* @param[in]     device       The handle of the device on which to enable or disable the feature.
* @param[in]     time         The unit is millisecond, the value is in the range (0,65535) and the default value is 350 millisecond.
* You can change the value according to the frame rate. For example,the frame rate is 30, so the theoretical waittime interval is 33ms, but if set the time value is 20ms,
* it means the max wait time is 20 ms when capturing next frame, so when call the VZCT_ReadNextFrame, it may return VzRetReadNextFrameTimeOut(-11).
* so the value range that recommended is [50.350].
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetWaitTimeOfReadNextFrame(VzDeviceHandle device, uint16_t time);

/**
* @brief        Set the working mode of the camera.
* @param[in]    device      The handle of the device
* @param[in]    status      The work mode of camera.
* @return       ::VzRetOK   if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetWorkMode(VzDeviceHandle device, VzWorkMode mode);

/**
* @brief        Get the working mode of the camera.
* @param[in]    device      The handle of the device
* @param[in]    pMode       The work mode of camera.
* @return       ::VzRetOK   if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetWorkMode(VzDeviceHandle device, VzWorkMode* pMode);

/**
* @brief        Trigger frame data once in software slave mode using invoke this API.
* @param[in]    device      The handle of the device.
* @return       ::VzRetOK   if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetSoftwareSlaveTrigger(VzDeviceHandle device);

/**
* @brief         Returns the internal intrinsic and distortion coefficient parameters from the device specified by <code>device</code>.
* @param[in]     device                        The handle of the device from which to get the internal parameters. 
* @param[in]     sensorType                    The type of sensor (depth or color) from which to get parameter information. Pass in the applicable value defined by ::VzSensorType.
* @param[out]    pSensorIntrinsicParameters    Pointer to a VzSensorIntrinsicParameters variable in which to store the parameter values.
* @return        ::PsRetOK                     if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetSensorIntrinsicParameters(VzDeviceHandle device, VzSensorType sensorType, VzSensorIntrinsicParameters* pSensorIntrinsicParameters);

/**
* @brief         Returns the camera rotation and translation coefficient parameters from the device specified by <code>device</code>.
* @param[in]     device                        The handle of the device from which to get the extrinsic parameters. 
* @param[out]    pSensorExtrinsicParameters    Pointer to a ::VzSensorExtrinsicParameters variable in which to store the parameters.
* @return        ::PsRetOK                     if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetSensorExtrinsicParameters(VzDeviceHandle device, VzSensorExtrinsicParameters* pSensorExtrinsicParameters);

/**
* @brief         Gets the firmware version number.
* @param[in]     device              The handle of the device on which to set the pulse count.
* @param[in]     pFirmwareVersion    Pointer to a variable in which to store the returned fw value.
* @param[in]     length              The maximum length is 63 bytes.
* @return        ::PsRetOK           if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetFirmwareVersionNumber(VzDeviceHandle device, char* pFirmwareVersion, int length);

/**
* @brief         Gets the MAC from the device specified by <code>device</code>.
* @param[in]     device            The handle of the device.
* @param[out]    pMAC              Pointer to a buffer in which to store the device MAC. the buffer default size is 18, and the last buffer set '\0'.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetDeviceMACAddress(VzDeviceHandle device, char* pMAC);

/**
* @brief        Sets the device GMM gain on a device.
* @param[in]    device       The handle of the device on which to set the GMM gain.
* @param[in]    gmmgain      The GMM gain value to set. See ::VzGMMGain for more information.The GMM gain value is in the range [0,4095].
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetIRGMMGain(VzDeviceHandle device, uint8_t gmmgain);

/**
* @brief        Returns the the device's GMM gain.
* @param[in]    device       The handle of the device from which to get the GMM gain.
* @param[out]   gmmgain      Pointer to a variable in which to store the returned GMM gain.
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetIRGMMGain(VzDeviceHandle device, uint8_t* pGmmgain);

/**
* @brief         Sets the color image pixel format on the device specified by <code>device</code>. Currently only RGB and BGR formats are supported.
* @param[in]     device         The handle of the device to set the pixel format. 
* @param[in]     pixelFormat    The color pixel format to use. Pass in one of the values defined by ::VzPixelFormat. Currently only <code>VzPixelFormatRGB888</code> and <code>VzPixelFormatBGR888</code> are supported.
* @return        ::VzRetOK      if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetColorPixelFormat(VzDeviceHandle device, VzPixelFormat pixelFormat);

/**
* @brief        Sets the color frame Resolution.
* @param[in]    device       The handle of the device.
* @param[in]    resolution   The resolution value to set. See ::VzResolution for more information.
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetColorResolution(VzDeviceHandle device, VzResolution resolution);

/**
* @brief        Returns the the color frame Resolution.
* @param[in]    device        The handle of the device.
* @param[out]   resolution    Pointer to a variable in which to store the returned resolution.
* @return       ::VzRetOK     if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetColorResolution(VzDeviceHandle device, uint16_t* resolution);

/**
* @brief         Sets the tof frame rate.The interface takes a long time, about 500 ms.
* @param[in]     device       The handle of the device on which to set the framerate.
* @param[in]     value        The rate value, in range [1,25].
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetFrameRate(VzDeviceHandle device, uint8_t value);
/**
* @brief         Gets the tof frame rate.
* @param[in]     device       The handle of the device on which to get the framerate.
* @param[in]     pValue       The rate value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetFrameRate(VzDeviceHandle device, uint8_t* pValue);

/**
* @brief        Set the exposure time of Tofsensor.
* @param[in]    device          The handle of the device on which to set the exposure time(ns).
* @param[in]    exposureTime    the exposure time.
* @return       ::VzRetOK       if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetToFExposureTime(VzDeviceHandle device, uint32_t exposureTime);

/**
* @brief         Get the exposure time of Tofsensor.
* @param[in]     device           The handle of the device on which to get the exposure time(ns).
* @param[out]    pExposureTime    the exposure time.
* @return        ::VzRetOK        if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetToFExposureTime(VzDeviceHandle device, uint32_t* pExposureTime);

/**
* @brief        Enables or disables the Time filter.
* @param[in]    device       The handle of the device
* @param[in]    bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/ 
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetTimeFilterEnabled(VzDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the TimeFilter feature is enabled or disabled.
* @param[in]     device       The handle of the device
* @param[out]    bEnabled     Pointer to a variable in which to store the returned Boolean value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetTimeFilterEnabled(VzDeviceHandle device, bool *pEnabled);

/**
* @brief         Set the parameters of the Confidence filter.
* @param[in]     device       The handle of the device
* @param[out]    params       Pointer to a variable in which to store the parameters.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetConfidenceFilterParams(VzDeviceHandle device, VzConfidenceFilterParams params);

/**
* @brief         Get the parameters of the ConfidenceFilter feature.
* @param[in]     device       The handle of the device
* @param[out]    pParams      Pointer to a variable in which to store the returned value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetConfidenceFilterParams(VzDeviceHandle device, VzConfidenceFilterParams *pParams);

/**
* @brief        Set the parameters of the FlyingPixel filter.
* @param[in]    device       The handle of the device.
* @param[out]   params       Pointer to a variable in which to store the parameters.
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetFlyingPixelFilterParams(VzDeviceHandle device, const VzFlyingPixelFilterParams params);

/**
* @brief         Get the parameters of the Confidence filter.
* @param[in]     device       The handle of the device
* @param[out]    pParams      Pointer to a variable in which to store the returned value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetFlyingPixelFilterParams(VzDeviceHandle device, VzFlyingPixelFilterParams* params);

/**
* @brief        Set the parameters of the FillHole filter.
* @param[in]    device       The handle of the device.
* @param[out]   params       Pointer to a variable in which to store the parameters.
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetFillHoleFilterParams(VzDeviceHandle device, const VzFillHoleFilterParams params);

/**
* @brief         Get the parameters of the FillHole filter.
* @param[in]     device       The handle of the device
* @param[out]    pParams      Pointer to a variable in which to store the returned value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetFillHoleFilterParams(VzDeviceHandle device, VzFillHoleFilterParams* params);

/**
* @brief        Set the parameters of the Spatial filter.
* @param[in]    device       The handle of the device.
* @param[out]   params       Pointer to a variable in which to store the parameters.
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetSpatialFilterParams(VzDeviceHandle device, const VzSpatialFilterParams params);

/**
* @brief         Get the parameters of the Spatial filter.
* @param[in]     device       The handle of the device
* @param[out]    pParams      Pointer to a variable in which to store the returned value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetSpatialFilterParams(VzDeviceHandle device, VzSpatialFilterParams* params);

/**
* @brief        Set the parameters of the Overexposure filter.
* @param[in]    device       The handle of the device.
* @param[out]   params       Pointer to a variable in which to store the parameters.
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetOverexposureFilterParams(VzDeviceHandle device, const VzOverexposureFilterParams params);

/**
* @brief         Get the parameters of the Overexposure filter.
* @param[in]     device       The handle of the device
* @param[out]    pParams      Pointer to a variable in which to store the returned value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetOverexposureFilterParams(VzDeviceHandle device, VzOverexposureFilterParams* params);

/**
* @brief         Enables or disables transforms a color image into the geometry of the depth sensor. When enabled, VZCT_GetFrame() can\n
*                be invoked passing ::VzTransformedColorFrame as the frame type for get a color image which each pixel matches the \n
*                corresponding pixel coordinates of the depth sensor. The resolution of the transformed color frame is the same as that\n
*                of the depth image.
* @param[in]     device       The handle of the device on which to enable or disable mapping.
* @param[in]     bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetTransformColorImgToDepthSensorEnabled(VzDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the transformed of the color image to depth sensor space feature is enabled or disabled.
* @param[in]     device       The handle of the device on which to enable or disable the feature.
* @param[out]    bEnabled     Pointer to a variable in which to store the returned Boolean value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetTransformColorImgToDepthSensorEnabled(VzDeviceHandle device, bool *bEnabled);

/**
* @brief         Enables or disables transforms the depth map into the geometry of the color sensor. When enabled, VZCT_GetFrame() can\n
*                be invoked passing ::VzTransformedDepthFrame as the frame type for get a depth image which each pixel matches the \n
*                corresponding pixel coordinates of the color sensor. The resolution of the transformed depth frame is the same as that\n
*                of the color image.
* @param[in]     device       The handle of the device on which to enable or disable mapping.
* @param[in]     bEnabled     Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetTransformDepthImgToColorSensorEnabled(VzDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the transformed of the depth image to color space feature is enabled or disabled.
* @param[in]     device       The handle of the device on which to enable or disable the feature.
* @param[out]    bEnabled     Pointer to a variable in which to store the returned Boolean value.
* @return        ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_GetTransformDepthImgToColorSensorEnabled(VzDeviceHandle device, bool *bEnabled);

/**
* @brief         Returns the point value of the frame that the mapping of the depth image to Color space.
* @param[in]     device           The handle of the device on which to enable or disable the feature.
* @param[in]     pointInDepth     The point in depth frame.
* @param[in]     colorSize        The size(x = w,y = h) of color frame.
* @param[out]    pPointInColor    The point in the color frame.
* @return        ::VzRetOK        if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_TransformedDepthPointToColorPoint(const VzDeviceHandle device, const VzDepthVector3 depthPoint, const VzVector2u16 colorSize, VzVector2u16* pPointInColor);

/**
* @brief         Converts the input points from depth coordinate space to world coordinate space.
* @param[in]     device          The handle of the device on which to perform the operation.
* @param[in]     pDepthVector    Pointer to a buffer containing the x, y, and z values of the depth coordinates to be converted. \n
*                                x and y are measured in pixels, where 0, 0 is located at the top left corner of the image. \n
*                                z is measured in millimeters, based on the ::VzPixelFormat depth frame.
* @param[out]    pWorldVector    Pointer to a buffer in which to output the converted x, y, and z values of the world coordinates, measured in millimeters.
* @param[in]     pointCount      The number of points to convert.
* @param[in]     pSensorParam    The intrinsic parameters for the depth sensor. See ::VzSensorIntrinsicParameters.
* @return        ::VzRetOK       if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_ConvertDepthToPointCloud(VzDeviceHandle device, VzDepthVector3* pDepthVector, VzVector3f* pWorldVector, int32_t pointCount, VzSensorIntrinsicParameters* pSensorParam);

/**
* @brief         Converts the input Depth frame from depth coordinate space to world coordinate space on the device.
* @param[in]     device          The handle of the device on which to perform the operation.
* @param[in]     depthFrame      The depth frame.
* @param[out]    pWorldVector    Pointer to a buffer in which to output the converted x, y, and z values of the world coordinates, measured in millimeters.
* @return        ::VzRetOK       if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_ConvertDepthFrameToPointCloudVector(VzDeviceHandle device, const VzFrame& depthFrame, VzVector3f* pWorldVector);

/**
* @brief        Sets hotplug status callback function
* @param[in]    pCallback    Pointer to the callback function. See ::PtrHotPlugStatusCallback 
* @param[in]    pUserData    Pointer to the user data. See ::PtrHotPlugStatusCallback
* @return       ::VzRetOK    if the function succeeded, or one of the error values defined by ::VzReturnStatus.
*/
VZENSE_C_API_EXPORT VzReturnStatus VZCT_SetHotPlugStatusCallback(PtrHotPlugStatusCallback pCallback, const void* pUserData);

#endif /* VZENSEDS_API_H */
