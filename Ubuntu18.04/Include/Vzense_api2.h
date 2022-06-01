#ifndef VZENSE_API2_H
#define VZENSE_API2_H

/**
* @file Vzense_api2.h
* @brief Vzense API header file.
* Copyright (c) 2018-2019 Pico Interactive, Inc.
*/

/*! \mainpage Vzense API Documentation
*
* \section intro_sec Introduction
*
* Welcome to the Vzense API documentation. This documentation enables you to quickly get started in your development efforts to programmatically interact with the Vzense CW TOF Camera (eg:DS77).
*/

#include "Vzense_define.h"

/**
* @brief         Initializes the API on the device. This function must be invoked before any other Vzense APIs.
* @return        ::PsRetOK if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_Initialize();

/**
* @brief         Shuts down the API on the device and clears all resources allocated by the API. After invoking this function, no other Vzense APIs can be invoked.
* @return        ::PsRetOK    if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_Shutdown();

/**
* @brief         Returns the number of camera devices currently connected.
* @param[out]    pDeviceCount    Pointer to a 32-bit integer variable in which to return the device count.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetDeviceCount(uint32_t* pDeviceCount);

/**
* @brief         Returns the info lists of the deviceCount camera devices.
* @param[in]     deviceCount        the number of camera devices.
* @param[out]    pDevicesList    Pointer to a buffer in which to store the deviceCount devices infos.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetDeviceListInfo(PsDeviceInfo* pDevicesList, uint32_t deviceCount);

/**
* @brief         Returns the info of the deviceIndex camera device.
* @param[in]     deviceIndex    The index of the device to open. Device indices range from 0 to device count - 1.
* @param[out]    pDevices    Pointer to a buffer in which to store the device info.
* @return         ::PsRetOK    if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetDeviceInfo(PsDeviceInfo* pDevices, uint32_t deviceIndex);

/**
* @brief         Opens the device specified by <code>uri</code>. The device must be subsequently closed using PsCloseDevice().
* @param[in]     uri            the uri of the device. See ::PsDeviceInfo for more information.
* @param[out]    pDevices    the handle of the device on which to open.
* @return:         ::PsRetOK    if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_OpenDevice(const char* uri, PsDeviceHandle* pDevice);

/**
* @brief         Opens the device specified by <code>alias</code>. The device must be subsequently closed using PsCloseDevice().
* @param[in]     alias        the alias of the device. See ::PsDeviceInfo for more information.
* @param[out]    pDevices    the handle of the device on which to open.
* @return:         ::PsRetOK    if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_OpenDeviceByAlias(const char* alias, PsDeviceHandle* pDevice);

/**
* @brief         Opens the device specified by <code>uri</code>. The device must be subsequently closed using PsCloseDevice().
* @param[in]     uri            the uri of the device. See ::PsDeviceInfo for more information.
* @param[out]    pDevices    the handle of the device on which to open.
* @return:         ::PsRetOK    if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_OpenDeviceByIP(const char* ip, PsDeviceHandle* pDevice);

/**
* @brief         Closes the device specified by <code>device</code> that was opened using PsOpenDevice.
* @param[in]     device        The handle of the device to close.
* @return:         ::PsRetOK    if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_CloseDevice(PsDeviceHandle* device);

/**
* @brief         Starts capturing the image stream indicated by <code>device</code>. \n
Invoke VZCT_StopStream() to stop capturing the image stream.
* @param[in]     device            The handle of the device on which to start capturing the image stream.                        
* @return         ::PsRetOK if    the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_StartStream(PsDeviceHandle device);

/**
* @brief         Stops capturing the image stream on the device specified by <code>device</code>. that was started using VZCT_StartStream.
* @param[in]     device            The handle of the device on which to stop capturing the image stream.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_StopStream(PsDeviceHandle device);

/**
* @brief         Captures the next image frame from the device specified by <code>device</code>. This API must be invoked before capturing frame data using PsGetFrame().
* @param[in]     device            The handle of the device on which to read the next frame.
* @param[out]    pFrameReady        Pointer to a buffer in which to store the signal on which image is ready to be get.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_ReadNextFrame(PsDeviceHandle device, PsFrameReady* pFrameReady);

/**
* @brief         Returns the image data for the current frame from the device specified by <code>device</code>.\n
Before invoking this API, invoke PsReadNextFrame() to capture one image frame from the device.
* @param[in]     device            The handle of the device to capture an image frame from.
* @param[in]     frameType        The image frame type.
* @param[out]    pPsFrame        Pointer to a buffer in which to store the returned image data.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetFrame(PsDeviceHandle device, PsFrameType frameType, PsFrame* pPsFrame);

/**
* @brief          Sets the output data mode for the device specified by <code>device</code>.The interface takes a long time, about 500 ms.
* @param[in]     device            The handle of the device for which to set the data mode.
* @param[in]    dataMode        The output data mode. See ::PsDataMode for more information.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetDataMode(PsDeviceHandle device, PsDataMode dataMode);

/**
* @brief          Returns the output data mode from the device specified by <code>device</code>.
* @param[in]    device            The handle of the device for which to set the data mode.
* @param[Out]    pDataMode        The output data mode. See ::PsDataMode for more information.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetDataMode(PsDeviceHandle device, PsDataMode* pDataMode);

/**
* @brief         Returns the depth range mode from the device specified by <code>device</code>.
* @param[in]     device            The handle of the device from which to get the depth range.
* @param[out]    pDepthRange        Pointer to a ::PsDepthRange variable in which to store the returned depth range mode.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetDepthRange(PsDeviceHandle device, PsDepthRange* pDepthRange);

/**
* @brief         Sets the depth range mode for the device specified by <code>device</code> only when the datamode of Camera is SingleFreqMode. The interface takes a long time, about 500 ms.
* @param[in]     device            The handle of the device on which to set the depth range.
* @param[in]     depthRange         Specifies the depth range mode.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetDepthRange(PsDeviceHandle device, PsDepthRange depthRange);

/**
* @brief         Returns the the device's GMM gain.
* @param[in]    device            The handle of the device from which to get the GMM gain.
* @param[out]     gmmgain         Pointer to a variable in which to store the returned GMM gain.
* @return        ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetIRGMMGain(PsDeviceHandle device, uint8_t* pGmmgain);

/**
* @brief         Sets the device GMM gain on a device.
* @param[in]    device            The handle of the device on which to set the GMM gain.
* @param[in]     gmmgain            The GMM gain value to set. See ::PsGMMGain for more information.The GMM gain value is in the range [0,4095].
* @return        ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetIRGMMGain(PsDeviceHandle device, uint8_t gmmgain);
/**
* @brief         Returns the internal intrinsic and distortion coefficient parameters from the device specified by <code>device</code>.
* @param[in]     device                The handle of the device from which to get the internal parameters. 
* @param[in]     sensorType            The type of sensor (depth or color) from which to get parameter information. Pass in the applicable value defined by ::PsSensorType.
* @param[out]     pCameraParameters    Pointer to a PsCameraParameters variable in which to store the parameter values.
* @return         ::PsRetOK            if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetCameraParameters(PsDeviceHandle device, PsSensorType sensorType, PsCameraParameters* pCameraParameters);

/**
* @brief         Returns the camera rotation and translation coefficient parameters from the device specified by <code>device</code>.
* @param[in]     device                        The handle of the device from which to get the extrinsic parameters. 
* @param[out]     pCameraExtrinsicParameters     Pointer to a ::PsGetCameraExtrinsicParameters variable in which to store the parameters.
* @return         ::PsRetOK                    if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetCameraExtrinsicParameters(PsDeviceHandle device, PsCameraExtrinsicParameters* pCameraExtrinsicParameters);

/**
* @brief         Sets the color image pixel format on the device specified by <code>device</code>. Currently only RGB and BGR formats are supported.
* @param[in]     device            The handle of the device to set the pixel format. 
* @param[in]     pixelFormat        The color pixel format to use. Pass in one of the values defined by ::PsPixelFormat. Currently only <code>PsPixelFormatRGB888</code> and <code>PsPixelFormatBGR888</code> are supported.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetColorPixelFormat(PsDeviceHandle device, PsPixelFormat pixelFormat);

/**
* @brief         Sets the color frame Resolution.
* @param[in]    device            The handle of the device on which to set the GMM gain.
* @param[in]     resolution        The resolution value to set. See ::PsResolution for more information.
* @return        ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetColorResolution(PsDeviceHandle device, PsResolution resolution);

/**
* @brief         Returns the the color frame Resolution.
* @param[in]    device            The handle of the device from which to get the GMM gain.
* @param[out]     resolution         Pointer to a variable in which to store the returned resolution.
* @return        ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetColorResolution(PsDeviceHandle device, uint16_t* resolution);

VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetFrameMode(PsDeviceHandle device, PsFrameType frameType, PsFrameMode* pFrameMode);

/**
* @brief         Gets the MeasuringRange in depthRange.
* @param[in]     device            The handle of the device on which to set the WDR style.
* @param[in]     depthRange         Specifies the depth range mode.
* @param[out]    pMeasuringRange A pointer to a ::PsMeasuringRange variable in which to store the MeasuringRange in depthRange.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetDepthMeasuringRange(PsDeviceHandle device, PsMeasuringRange* pMeasuringRange);

/**
* @brief         Converts the input points from world coordinate space to depth coordinate space.
* @param[in]    device            The handle of the device on which to perform the operation.
* @param[in]    pWorldVector     Pointer to a buffer containing the x, y, and z values of the input world coordinates to be converted, measured in millimeters.
* @param[out]    pDepthVector     Pointer to a buffer in which to output the converted x, y, and z values of the depth coordinates. \n
*                                x and y are measured in pixels, where 0, 0 is located at the top left corner of the image. \n
*                                z is measured in millimeters, based on the ::PsPixelFormat depth frame.
* @param[in]    pointCount         The number of coordinates to convert.
* @param[in]    pCameraParam    The intrinsic camera parameters for the depth camera. See ::PsGetCameraParameters.
* @return        ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_ConvertPointCloudToDepth(PsDeviceHandle device, PsVector3f* pWorldVector, PsDepthVector3* pDepthVector, int32_t pointCount, PsCameraParameters* pCameraParam);

/**
* @brief         Converts the input points from depth coordinate space to world coordinate space.
* @param[in]     device            The handle of the device on which to perform the operation.
* @param[in]     pDepthVector     Pointer to a buffer containing the x, y, and z values of the depth coordinates to be converted. \n
*                                   x and y are measured in pixels, where 0, 0 is located at the top left corner of the image. \n
*                                z is measured in millimeters, based on the ::PsPixelFormat depth frame.
* @param[out]     pWorldVector     Pointer to a buffer in which to output the converted x, y, and z values of the world coordinates, measured in millimeters.
* @param[in]     pointCount         The number of points to convert.
* @param[in]    pCameraParam    The intrinsic camera parameters for the depth camera. See ::PsGetCameraParameters.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_ConvertDepthToPointCloud(PsDeviceHandle device, PsDepthVector3* pDepthVector, PsVector3f* pWorldVector, int32_t pointCount, PsCameraParameters* pCameraParam);

/**
* @brief         Converts the input Depth frame from depth coordinate space to world coordinate space on the device.
* @param[in]     device            The handle of the device on which to perform the operation.
* @param[in]     depthFrame        The depth frame.
* @param[out]     pWorldVector     Pointer to a buffer in which to output the converted x, y, and z values of the world coordinates, measured in millimeters.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_ConvertDepthFrameToPointCloudVector(PsDeviceHandle device, const PsFrame& depthFrame, PsVector3f* pWorldVector);

/**
* @brief        Enables or disables the syncronize feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[in]    bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetSynchronizeEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the syncronize feature is enabled or disabled.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetSynchronizeEnabled(PsDeviceHandle device, bool *pEnabled);

/**
* @brief         Enables or disables the ToF(Depth and IR) img distortion correction feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature. 
* @param[in]     bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetToFDistortionCorrectionEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the ToF(Depth and IR) img  distortion correction feature is enabled or disabled.
* @param[in]    device            The handle of the device on which to enable or disable the feature. 
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetToFDistortionCorrectionEnabled(PsDeviceHandle device, bool *pEnabled);

/**
* @brief         Enables or disables the color frame distortion correction feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature. 
* @param[in]     bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetColorDistortionCorrectionEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the color frame distortion correction feature is enabled or disabled.
* @param[in]    device            The handle of the device on which to enable or disable the feature. 
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetColorDistortionCorrectionEnabled(PsDeviceHandle device, bool *pEnabled);

/**
* @brief        Enables or disables the ComputeRealDepth feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature. 
* @param[in]    bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetComputeRealDepthCorrectionEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the ComputeRealDepth feature is enabled or disabled.
* @param[in]    device            The handle of the device on which to enable or disable the feature. 
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetComputeRealDepthCorrectionEnabled(PsDeviceHandle device, bool *pEnabled);

/**
* @brief        Enables or disables the TimeFilter feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature. 
* @param[in]    bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/ 
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetTimeFilterEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the TimeFilter feature is enabled or disabled.
* @param[in]    device            The handle of the device on which to enable or disable the feature. 
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetTimeFilterEnabled(PsDeviceHandle device, bool *pEnabled);

/**
* @brief         Returns the Boolean value of whether the ConfidenceFilter feature is enabled or disabled.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetConfidenceFilterEnabled(PsDeviceHandle device, bool enable);

/**
* @brief         Returns the Boolean value of whether the ConfidenceFilter feature is enabled or disabled.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetConfidenceFilterEnabled(PsDeviceHandle device, bool *pEnabled);

/**
* @brief        Enables or disables the Depth Stream feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[in]    bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetDepthFrameEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief        Enables or disables the IR Stream feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[in]    bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetIrFrameEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief        Enables or disables the Confidence Stream feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[in]    bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetConfidenceFrameEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief        Enables or disables the color Stream feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[in]    bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetColorFrameEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief         Enables or disables transforms a color image into the geometry of the depth camera. When enabled, PsGetFrame() can\n
*                be invoked passing ::PsTransformedColorFrame as the frame type for get a color image which each pixel matches the \n
*                corresponding pixel coordinates of the depth camera. The resolution of the transformed color frame is the same as that\n
*                of the depth image.
* @param[in]     device           The handle of the device on which to enable or disable mapping.
* @param[in]     bEnabled         Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return        ::PsRetOK       if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetTransformColorImgToDepthCameraEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the transformed of the color image to depth camera space feature is enabled or disabled.
* @param[in]     device          The handle of the device on which to enable or disable the feature.
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return        ::PsRetOK       if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_IsTransformColorImgToDepthCameraEnabled(PsDeviceHandle device, bool *bEnabled);

/**
* @brief         Enables or disables transforms the depth map into the geometry of the color camera. When enabled, PsGetFrame() can\n
*                be invoked passing ::PsTransformedDepthFrame as the frame type for get a depth image which each pixel matches the \n
*                corresponding pixel coordinates of the color camera. The resolution of the transformed depth frame is the same as that\n
*                of the color image.
* @param[in]     device            The handle of the device on which to enable or disable mapping.
* @param[in]     bEnabled         Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetTransformDepthImgToColorCameraEnabled(PsDeviceHandle device, bool bEnabled);

/**
* @brief         Returns the Boolean value of whether the transformed of the depth image to color space feature is enabled or disabled.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[out]    bEnabled        Pointer to a variable in which to store the returned Boolean value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_IsTransformDepthImgToColorCameraEnabled(PsDeviceHandle device, bool *bEnabled);
/**
* @brief         Sets upgrade status callback function
* @param[in]     device            The handle of the device on which to set the pulse count.
* @param[in]    pCallback        Pointer to the callback function. See ::PtrUpgradeStatusCallback
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetUpgradeStatusCallback(PsDeviceHandle device, PtrUpgradeStatusCallback pCallback);

/**
* @brief         Sets upgrade status callback function
* @param[in]     device            The handle of the device on which to set the pulse count.
* @param[in]    pImgPath        Pointer to the path of firmware file.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_StartUpgradeFirmWare(PsDeviceHandle device, char* pImgPath);
/**
* @brief         Sets hotplug status callback function
* @param[in]    pCallback        Pointer to the callback function. See ::PtrHotPlugStatusCallback 
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetHotPlugStatusCallback(PtrHotPlugStatusCallback pCallback);
/**
* @brief         Sets hotplug status callback function for c plus plus
* @param[in]    pCallback        Pointer to the callback function. See ::PtrHotPlugStatusCallback
* @param[in]    pContex            Pointer to the object of C++ class
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetHotPlugStatusCallback_(PtrHotPlugStatusCallback_ pCallback, void* pContex);

/**
* @brief         Gets the serial number.
* @param[in]     device            The handle of the device on which to set the pulse count.
* @param[in]     sn                 Pointer to a variable in which to store the returned sn value.
* @param[in]     length             The maximum length is 63 bytes.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetSerialNumber(PsDeviceHandle device, char* sn, int length);

/**
* @brief         Gets the firmware version number.
* @param[in]     device            The handle of the device on which to set the pulse count.
* @param[in]     fw                 Pointer to a variable in which to store the returned fw value.
* @param[in]     length             The maximum length is 63 bytes.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetFirmwareVersionNumber(PsDeviceHandle device, char* fw, int length);

/**
* @brief         Sets the tof frame rate.The interface takes a long time, about 500 ms.
* @param[in]     device            The handle of the device on which to set the pulse count.
* @param[in]     value             The rate value, in range [1,25].
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetFrameRate(PsDeviceHandle device, uint8_t value);
/**
* @brief         Gets the tof frame rate.
* @param[in]     device            The handle of the device on which to set the pulse count.
* @param[in]     pValue             The rate value.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetFrameRate(PsDeviceHandle device, uint8_t* pValue);

/**
* @brief        Enables or disables the StandBy feature.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[in]    bEnabled        Set to <code>true</code> to enable the feature or <code>false</code> to disable the feature.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetStandByEnabled(PsDeviceHandle device, bool bEnabled);
/**
* @brief         Set the waittime of read next frame.
* @param[in]     device            The handle of the device on which to enable or disable the feature.
* @param[in]     time             The unit is millisecond, the value is in the range (0,65535) and the default value is 350 millisecond.
* You can change the value according to the frame rate. For example,the frame rate is 30, so the theoretical waittime interval is 33ms, but if set the time value is 20ms,
* it means the max wait time is 20 ms when capturing next frame, so when call the VZCT_ReadNextFrame, it may return PsRetReadNextFrameTimeOut(-11).
* so the value range that recommended is [50.350].
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetWaitTimeOfReadNextFrame(PsDeviceHandle device, uint16_t time);

/**
* @brief         Gets the version of SDK.
* @param[in]     version         Pointer to a variable in which to store the returned version value.
* @param[in]     length             The maximum length is 63 bytes.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetSDKVersion(char* version, int length);

/**
* @brief         Returns the point value of the frame that the mapping of the depth image to Color space.
* @param[in]    device            The handle of the device on which to enable or disable the feature.
* @param[in]    pointInDepth    The point in depth frame.
* @param[in]    colorSize            The size(x = w,y = h) of color frame.

* @param[out]    pPointInColor        The point in the color frame.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetTransformedPointDepthToColor(const PsDeviceHandle device, const PsDepthVector3 depthPoint, const PsVector2u16 colorSize, PsVector2u16* pPointInColor);


/**
* @brief        Trigger frame data once in slave mode.
* @param[in]    device      The handle of the device on which to enable or disable the feature.
* @param[in]    status      The status of slave mode.
* @return       ::PsRetOK   if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetSlaveModeStatus(PsDeviceHandle device, PsSlaveModeStatus status);

/**
* @brief        Trigger frame data once in software slave mode using invoke this API.
* @param[in]    device      The handle of the device on which to enable or disable the feature.
* @return       ::PsRetOK   if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetSoftwareSlaveTrigger(PsDeviceHandle device);

/**
* @brief        Set the exposure time of Tofsensor.
* @param[in]    device            The handle of the device on which to set the exposure time.
* @param[in]    exposureTime    the exposure time.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_SetToFExposureTime(PsDeviceHandle device, uint32_t exposureTime);

/**
* @brief        Get the exposure time of Tofsensor.
* @param[in]    device            The handle of the device on which to get the exposure time.
* @param[out]    pExposureTime            the exposure time.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetToFExposureTime(PsDeviceHandle device, uint32_t* pExposureTime);

/**
* @brief         Gets IP from the device specified by <code>uri</code>.
* @param[in]     pURI            the uri of the device. See ::PsDeviceInfo for more information.
* @param[out]    pIP            Pointer to a buffer in which to store the device IP. the buffer default size is 17, and the last buffer set '\0'.
* @return:         ::PsRetOK    if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetDeviceIP(const char* pURI, char* pIP);

/**
* @brief         Gets the MAC from the device specified by <code>device</code>.
* @param[in]     device            The handle of the device.
* @param[out]    pMAC                Pointer to a buffer in which to store the device MAC. the buffer default size is 18, and the last buffer set '\0'.
* @return         ::PsRetOK        if the function succeeded, or one of the error values defined by ::PsReturnStatus.
*/
VZENSE_C_API_EXPORT PsReturnStatus VZCT_GetDeviceMAC(PsDeviceHandle device, char* pMAC);
#endif /* VZENSE_API2_H */
