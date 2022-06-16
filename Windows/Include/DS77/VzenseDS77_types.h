#ifndef VZENSEDS_TYPES_H
#define VZENSEDS_TYPES_H

#include <stdint.h>
#include "VzenseDS77_enums.h"

typedef uint16_t PsDepthPixel;  //!< Depth image pixel type in 16-bit
typedef uint16_t PsGray16Pixel; //!< Gray image pixel type in 16-bit
typedef uint8_t PsGray8Pixel;   //!< Gray image pixel type in 8-bit

#pragma pack (push, 1)
/**
 * @brief Color image pixel type in 24-bit RGB format.
 */
typedef struct
{
	uint8_t r;	//!< Red
	uint8_t g;	//!< Green
	uint8_t b;	//!< Blue
} PsRGB888Pixel;

/**
 * @brief Color image pixel type in 24-bit BGR format.
 */
typedef struct
{
	uint8_t b;	//!< Blue
	uint8_t g;	//!< Green
	uint8_t r;	//!< Red
} PsBGR888Pixel;

/**
 * @brief Stores the x, y, and z components of a 3D vector.
 */
typedef struct  
{
	float x, y, z;	//!< The x, y, and z components of the vector.
}PsVector3f;

/**
 * @brief Stores the x, y, and z components of a 2D vector.
 */
typedef struct
{
	uint16_t x;
	uint16_t y;
}PsVector2u16;

/**
 * @brief Contains depth information for a given pixel.
 */
typedef struct
{
	int          depthX;    //!< The x coordinate of the pixel.
	int          depthY;    //!< The y coordinate of the pixel.
	PsDepthPixel depthZ;    //!< The depth of the pixel, in millimeters.
}PsDepthVector3;

/**
 * @brief Camera intrinsic parameters and distortion coefficients.
 */
typedef struct
{
	double	fx;  //!< Focal length x (pixel)
	double	fy;  //!< Focal length y (pixel)
	double	cx;  //!< Principal point x (pixel)
	double	cy;  //!< Principal point y (pixel)
	double	k1;  //!< Radial distortion coefficient, 1st-order
	double	k2;  //!< Radial distortion coefficient, 2nd-order
	double	p1;  //!< Tangential distortion coefficient
	double	p2;  //!< Tangential distortion coefficient
	double	k3;  //!< Radial distortion coefficient, 3rd-order
	double	k4;  //!< Radial distortion coefficient, 4st-order
	double	k5;  //!< Radial distortion coefficient, 5nd-order
	double	k6;  //!< Radial distortion coefficient, 6rd-order
}PsCameraParameters;

/** 
 * @brief Specifies the camera’s location and orientation extrinsic parameters.
 */
typedef struct
{
	double rotation[9];     //!< Orientation stored as an array of 9 double representing a 3x3 rotation matrix.
	double translation[3];  //!< Location stored as an array of 3 double representing a 3-D translation vector.
}PsCameraExtrinsicParameters;
/**
* @brief
*/
typedef struct
{
	uint16_t tm_sec;   // seconds after the minute - [0, 60] including leap second
	uint16_t tm_min;   // minutes after the hour - [0, 59]
	uint16_t tm_hour;  // hours since midnight - [0, 23]
	uint16_t tm_msec;  // millisecond after the second - [0, 999]
}PsTimeStamp;
/**
 * @brief Depth/IR/Color image frame data.
 */
typedef struct
{
	uint32_t       frameIndex;    //!< The index of the frame.
	PsFrameType    frameType;     //!< The type of frame. See ::PsFrameType for more information.
	PsPixelFormat  pixelFormat;   //!< The pixel format used by a frame. See ::PsPixelFormat for more information.
	uint8_t        imuFrameNo;    //!< Used to synchronize with IMU, in the range of 0 to 255.
	uint8_t*       pFrameData;    //!< A buffer containing the frame’s image data.
	uint32_t       dataLen;       //!< The length of pFrame, in bytes.
	float          exposureTime;  //!< The exposure time, in milliseconds.
	PsDepthRange   depthRange;    //!< The depth range mode of the current frame. Used only for depth frames.
	uint16_t       width;		  //!< The width of the frame, in pixels.
	uint16_t       height;        //!< The height of the frame, in pixels.
    uint64_t       timestamp;	  //!< The timestamp of the frame.
}PsFrame;

typedef struct
{
	uint32_t depth : 1;
	uint32_t ir : 1;
	uint32_t color : 1;
	uint32_t transformedColor : 1;
	uint32_t transformedDepth : 1;
	uint32_t transformedIR : 1;
	uint32_t confidence : 1;
	uint32_t reserved : 25;
}PsFrameReady;

struct Device;
typedef Device* PsDeviceHandle;

typedef struct
{
	int SessionCount;
	PsDeviceType devicetype;
	char uri[256];
	char fw[50];
	char alias[64];
	PsConnectStatus status;
}PsDeviceInfo;

typedef struct
{
    bool enable;
    int threshold;
} PsConfidenceFilterParams;

typedef struct
{
    bool enable;
    int	threshold;
} PsFlyingPixelFilterParams;

typedef struct
{
    bool enable;
    int	validCount;
    int threshold;
    int doCount;
} PsSpatialFilterParams;

typedef struct
{
    bool enable;
    int	validCount;
    int	threshold;
    int doCount;
} PsFillHoleFilterParams;

typedef struct
{
    bool enable;
    int threshold;
} PsOverexposureFilterParams;

union IPAddr_
{
	uint32_t ip_int32;
	struct
	{
		uint8_t s3;
		uint8_t s2;
		uint8_t s1;
		uint8_t s0;
	}ip_int8;
};
#pragma pack (pop)

/**
* @brief hotplug status callback function
* pInfo     return the info of the Device, See ::PsDeviceInfo
* state     0:device added , 1:device removed
* pUserData Pointer to user data, which can be null
*/
typedef void(*PtrHotPlugStatusCallback)(const PsDeviceInfo* pInfo, int state, void* pUserData);

typedef void(*PtrUpgradeStatusCallback)(int status, int params, void* pUserData);

#endif /* VZENSEDS_TYPES_H */
