#include "vzense_manager.hpp"

//define call back function
void VzenseManager::paramCallback(vzense_param::Vzensetof_roscppConfig& config,uint32_t level)
{
    ROS_INFO("Request: %d %d %d %d %d %d %d %d %d",
                config.FrameRate,
                config.IRGMMGain,
                config.RGBResloution,
                config.ToFManual,
                config.ToFExposureTime,
                config.RGBManual,
                config.RGBExposureTime,
                config.WorkMode,
                config.SoftwareTrigger);
    if(config_.IRGMMGain != config.IRGMMGain)
    {
        config_.IRGMMGain = config.IRGMMGain;
        VzReturnStatus status= VZ_SetIRGMMGain(deviceHandle_,config_.IRGMMGain);
        ROS_INFO_STREAM( "SetIRGMMGain status: " << status);
    }
    if(config_.FrameRate != config.FrameRate)
    {
        config_.FrameRate = config.FrameRate;
        VzReturnStatus status= VZ_SetFrameRate(deviceHandle_,config_.FrameRate);
        ROS_INFO_STREAM( "SetFrameRate status: " << status);
    }
    if(config_.RGBResloution != config.RGBResloution)
    {
        config_.RGBResloution = config.RGBResloution;
        int w = 640;
        int h = 480;
        switch (config_.RGBResloution)
        {
        case 0:
            w = 1600;
            h = 1200;
            break;
        case 1:
            w = 800;
            h = 600;
            break;
        case 2:
            w = 640;
            h = 480;
            break;
        }
        VzReturnStatus status= VZ_SetColorResolution(deviceHandle_,w,h);
        ROS_INFO_STREAM( "SetColorResolution status: " << status);
    }
    if(config_.ToFManual != config.ToFManual)
    {
        config_.ToFManual = config.ToFManual;
        VzReturnStatus status= VZ_SetExposureControlMode(deviceHandle_,VzToFSensor,(VzExposureControlMode)config_.ToFManual);
        ROS_INFO_STREAM( "SetExposureControlMode tof status: " << status);
        if(config_.ToFManual != 1)
        {
          config_.ToFExposureTime = 0;  
        }
    }
    if(config_.ToFExposureTime != config.ToFExposureTime&&config_.ToFManual==1)
    {
        config_.ToFExposureTime = config.ToFExposureTime;
        VzExposureTimeParams exposureTime;
        exposureTime.mode = VzExposureControlMode_Manual;
        exposureTime.exposureTime = config_.ToFExposureTime;
        VzReturnStatus status= VZ_SetExposureTime(deviceHandle_,VzToFSensor,exposureTime);
        ROS_INFO_STREAM( "SetExposureTime tof status: " << status);
    }
    if(config_.RGBManual != config.RGBManual)
    {
        config_.RGBManual = config.RGBManual;
        VzReturnStatus status= VZ_SetExposureControlMode(deviceHandle_,VzColorSensor,(VzExposureControlMode)config_.RGBManual);
        ROS_INFO_STREAM( "SetExposureControlMode color status: " << status);
        if(config_.RGBManual != 1)
        {
          config_.RGBExposureTime = 0;  
        }
    }
    if(config_.RGBExposureTime != config.RGBExposureTime&&config_.RGBManual==1)
    {
        config_.RGBExposureTime = config.RGBExposureTime;
        VzExposureTimeParams exposureTime;
        exposureTime.mode = VzExposureControlMode_Manual;
        exposureTime.exposureTime = config_.RGBExposureTime;
        VzReturnStatus status= VZ_SetExposureTime(deviceHandle_,VzColorSensor,exposureTime);
        ROS_INFO_STREAM( "SetExposureTime color status: " << status);
    }
    if(config_.WorkMode != config.WorkMode)
    {
        config_.WorkMode = config.WorkMode;
        VzReturnStatus status= VZ_SetWorkMode(deviceHandle_,(VzWorkMode)config_.WorkMode);
        ROS_INFO_STREAM( "SetWorkMode status: " << status);
    }
    config_.SoftwareTrigger = config.SoftwareTrigger;
}

VzenseManager::VzenseManager(int32_t device_index, const string &camera_name) :
        color_nh_(camera_name + "/color"),
        depth_nh_(camera_name + "/depth"),
        ir_nh_(camera_name + "/ir"),
        alignedDepth_nh_(camera_name + "/transformedDepth"),        
        alignedColor_nh_(camera_name + "/transformedColor"),
        camera_name_(camera_name),
        color_info_(new camera_info_manager::CameraInfoManager(color_nh_)),
        depth_info_(new camera_info_manager::CameraInfoManager(depth_nh_)),
        ir_info_(new camera_info_manager::CameraInfoManager(ir_nh_)),
        alignedDepth_info_(new camera_info_manager::CameraInfoManager(alignedDepth_nh_)),
        alignedColor_info_(new camera_info_manager::CameraInfoManager(alignedColor_nh_)),
        color_it_(new image_transport::ImageTransport(color_nh_)),
        depth_it_(new image_transport::ImageTransport(depth_nh_)),
        ir_it_(new image_transport::ImageTransport(ir_nh_)),
        alignedDepth_it_(new image_transport::ImageTransport(alignedDepth_nh_)),
        alignedColor_it_(new image_transport::ImageTransport(alignedColor_nh_)),
        rgb_width_(-1),
        rgb_height_(-1),
        slope_(1450),
        deviceHandle_(0),
        sessionIndex_(0)
{
    signal(SIGSEGV, VzenseManager::sigsegv_handler);

    // Initialise the API
    checkVzReturnStatus(VZ_Initialize(), "Initialize failed!");

    // Get number of available devices
    uint32_t device_count = 0;
GET:
    int checkDeviceSec = 0;
	VzReturnStatus status = VZ_GetDeviceCount(&device_count);
	if (status != VzReturnStatus::VzRetOK || device_count < 1)
	{
        ROS_INFO("check device cost:%d second.", checkDeviceSec++);
        ros::Duration(1).sleep();
		goto GET;	
	}
    ROS_INFO("Get device count: %d", device_count);

    // Verify device index selection
    this->device_index_ = device_index;
    if (this->device_index_ < 0
        || this->device_index_ >= device_count)
        {
        throw std::runtime_error(
                "Device index outside of available devices range 0-" + std::to_string(device_count));
        }
        
    
    VzDeviceInfo* pPsDeviceInfo = new VzDeviceInfo;
    status =  VZ_GetDeviceInfo(device_index_,pPsDeviceInfo);
    ROS_INFO("pPsDeviceInfo.uri:%s", pPsDeviceInfo->uri);

    // Attempt to open the device
    checkVzReturnStatus(VZ_OpenDeviceByUri(pPsDeviceInfo->uri, &deviceHandle_), "OpenDevice failed!");

    ROS_INFO("Successfully connected to device %d", this->device_index_);

    status= VZ_StartStream(deviceHandle_);
    ROS_INFO_STREAM( "Start Depth Frame status: " << status);
    /* add user define api call start*/
    // such as call the VZ_SetSpatialFilterEnabled
   
    /*
    status= VZ_SetSpatialFilterEnabled(deviceHandle_,true);
    ROS_INFO_STREAM( "SetSpatialFilterEnabled status: " << status);
    */
  
    /* add user define api call end*/

    int rate=0;
    status= VZ_GetFrameRate(deviceHandle_,&rate);
    config_.FrameRate=rate;
    ROS_INFO_STREAM( "GetFrameRate status: " << status);
    uint8_t gmmgain=0;
    status= VZ_GetIRGMMGain(deviceHandle_,&gmmgain);
    config_.IRGMMGain = gmmgain;
    ROS_INFO_STREAM( "GetIRGMMGain status: " << status);
    
    VzWorkMode mode;
    status= VZ_GetWorkMode(deviceHandle_,&mode);
    config_.WorkMode= mode;
    ROS_INFO_STREAM( "GetWorkMode status: " << status);
    
    {
        VzExposureControlMode pControlMode;
        status= VZ_GetExposureControlMode(deviceHandle_,VzToFSensor,&pControlMode);
        config_.ToFManual = pControlMode;
        ROS_INFO_STREAM( "GetExposureControlMode tof status: " << status);

        if(config_.ToFManual == VzExposureControlMode_Manual)
        {
            VzExposureTimeParams pExposureTime;
            pExposureTime.mode = VzExposureControlMode_Manual;
            pExposureTime.exposureTime=0;
            status= VZ_GetExposureTime(deviceHandle_,VzToFSensor,&pExposureTime);
            config_.ToFExposureTime=pExposureTime.exposureTime;
            ROS_INFO_STREAM( "GetExposureTime ToF status: " << status);
        }
   }
    {
        VzExposureControlMode pControlMode;
        status= VZ_GetExposureControlMode(deviceHandle_,VzColorSensor,&pControlMode);
        config_.RGBManual = pControlMode;
        ROS_INFO_STREAM( "GetExposureControlMode color status: " << status);
        if(pControlMode == VzExposureControlMode_Manual)
        {
            VzExposureTimeParams pExposureTime;
            pExposureTime.mode = VzExposureControlMode_Manual;
            pExposureTime.exposureTime=0;
            status= VZ_GetExposureTime(deviceHandle_,VzColorSensor,&pExposureTime);
            config_.RGBExposureTime=pExposureTime.exposureTime;
            ROS_INFO_STREAM( "GetExposureTime color status: " << status);
        }
    }
    ROS_INFO("ctl: %d %d %d %d %d %d %d %d",
                config_.FrameRate,
                config_.IRGMMGain,
                config_.RGBResloution,
                config_.ToFManual,
                config_.ToFExposureTime,
                config_.RGBManual,
                config_.RGBExposureTime,
                config_.WorkMode);
}
 
void VzenseManager::run() 
{
    // Initialise ROS nodes
    set_sensor_intrinsics();
    sensor_msgs::CameraInfoPtr color_ci(new sensor_msgs::CameraInfo(color_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr depth_ci(new sensor_msgs::CameraInfo(depth_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr ir_ci(new sensor_msgs::CameraInfo(ir_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr alignedDepth_ci(new sensor_msgs::CameraInfo(alignedDepth_info_->getCameraInfo()));
    sensor_msgs::CameraInfoPtr alignedColor_ci(new sensor_msgs::CameraInfo(alignedColor_info_->getCameraInfo()));
      
    // CameraPublisher 	advertiseCamera (const std::string &base_topic, uint32_t queue_size, bool latch=false)
    this->color_pub_ = this->color_it_->advertiseCamera("image_raw", 30);
    this->depth_pub_ = this->depth_it_->advertiseCamera("image_raw", 30);
    this->ir_pub_ = this->ir_it_->advertiseCamera("image_raw", 30);
    this->alignedDepth_pub_ = this->alignedDepth_it_->advertiseCamera("image_raw", 30);
    this->alignedColor_pub_ = this->alignedColor_it_->advertiseCamera("image_raw", 30);

    // Containers for frames
    VzReturnStatus status;
    ros::Time now = ros::Time::now();
    int missed_frames = 0;
    sensor_msgs::ImagePtr depth_msg;
    sensor_msgs::ImagePtr ir_msg;
    sensor_msgs::ImagePtr color_msg;
    sensor_msgs::ImagePtr alignedDetph_msg;
    sensor_msgs::ImagePtr alignedColor_msg;

    while (ros::ok()) {
        ros::spinOnce();
        // Get next frame set
        if(config_.WorkMode == VzSoftwareTriggerMode && config_.SoftwareTrigger ==1)
        {
            VZ_SetSoftwareSlaveTrigger(deviceHandle_);
        }
        VzFrameReady psReadFrame = {0};
        VzReturnStatus status =  VZ_GetFrameReady(deviceHandle_, 1200, &psReadFrame);
        if (status != VzRetOK)
        {
            continue;
        }

        now = ros::Time::now();

        if (1 == psReadFrame.depth)
        {
            psReadFrame.depth = (true == fillImagePtr(now, VzDepthFrame, depth_ci, depth_msg)) ? 1 : 0;
        }

        if (1 == psReadFrame.ir)
        {
            psReadFrame.ir = (true == fillImagePtr(now, VzIRFrame, ir_ci, ir_msg)) ? 1 : 0;           
        }

        if (1 == psReadFrame.color)
        {
            psReadFrame.color = (true == fillImagePtr(now, VzColorFrame, color_ci, color_msg)) ? 1 : 0;           
        }

        if (1 == psReadFrame.transformedDepth)
        {
            psReadFrame.transformedDepth = (true == fillImagePtr(now, VzTransformDepthImgToColorSensorFrame, alignedDepth_ci, alignedDetph_msg)) ? 1 : 0;            
        }
        if (1 == psReadFrame.transformedColor)
        {
            psReadFrame.transformedColor = (true == fillImagePtr(now, VzTransformColorImgToDepthSensorFrame, alignedColor_ci, alignedColor_msg)) ? 1 : 0;            
        }  
        if (1 == psReadFrame.depth)
        {
            this->depth_pub_.publish(depth_msg, depth_ci);
        }

        if (1 == psReadFrame.ir)
        {
            this->ir_pub_.publish(ir_msg, ir_ci);
        }

        if (1 == psReadFrame.color)
        {
            this->color_pub_.publish(color_msg, color_ci);
        }

        if (1 == psReadFrame.transformedDepth)
        {
            this->alignedDepth_pub_.publish(alignedDetph_msg, alignedDepth_ci);
        }
        
        if (1 == psReadFrame.transformedColor)
        {
            this->alignedColor_pub_.publish(alignedColor_msg, alignedColor_ci);
        }


    }

    status = VZ_StopStream(deviceHandle_);
    ROS_INFO_STREAM( "Stop Depth Frame status: " << status);
    status = VZ_CloseDevice(&deviceHandle_);
    ROS_INFO_STREAM( "CloseDevice status: " << status);
    status = VZ_Shutdown();
    ROS_INFO_STREAM( "Shutdown status: " << status );
}

bool VzenseManager::fillImagePtr(const ros::Time& time, const VzFrameType type, sensor_msgs::CameraInfoPtr& cameraInfoPtr, sensor_msgs::ImagePtr& imagePtr)
{
    bool ret = false;

    VzFrame frame = {0};
    VZ_GetFrame(deviceHandle_, type, &frame);
    
    if (frame.pFrameData != NULL)
    {
        int cvMatType = CV_16UC1;
        std::string imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
        switch (type)
        {
        case VzIRFrame:
            cvMatType = CV_8UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_8UC1;
            break;
        case VzDepthFrame:
        case VzTransformDepthImgToColorSensorFrame:
            cvMatType = CV_16UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
            break;
        case VzColorFrame:
 	case VzTransformColorImgToDepthSensorFrame:
            cvMatType = CV_8UC3;
            imageEncodeType = sensor_msgs::image_encodings::BGR8;
            break;
        default:
            return ret;
        }

        cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
        cameraInfoPtr->height = frame.height;
        cameraInfoPtr->width = frame.width;
        cameraInfoPtr->header.stamp = time;
        imagePtr = cv_bridge::CvImage(cameraInfoPtr->header, imageEncodeType, mat).toImageMsg();
        ret = true;
    }

    return ret;
}

void VzenseManager::sigsegv_handler(int sig) {
    signal(SIGSEGV, SIG_DFL);
    ROS_ERROR("Segmentation fault, stopping camera driver (%d).", sig);    
    ros::shutdown();
}

void VzenseManager::checkVzReturnStatus(VzReturnStatus status, const std::string &message_on_fail) {
    if (status == VzReturnStatus::VzRetOK)
        return;
    ROS_ERROR(message_on_fail.c_str());
    if (0 != deviceHandle_)
    {
        VZ_StopStream(deviceHandle_);
        ROS_INFO_STREAM("Stop Depth Frame status: " << status);
        VZ_CloseDevice(&deviceHandle_);
        ROS_INFO_STREAM("CloseDevice status: " << status);
    }
    VZ_Shutdown();
    ROS_INFO_STREAM("Shutdown status: " << status);

    throw std::runtime_error(message_on_fail);
}

void VzenseManager::set_sensor_intrinsics() {
    std::string camera_frame(this->camera_name_ + "_frame"), 
                color_frame(this->camera_name_ + "_color_frame"),
                aligneddepth_frame(this->camera_name_ + "_transformedDepth_frame"),
                alignedcolor_frame(this->camera_name_ + "_transformedColor_frame"),
                depth_frame(this->camera_name_ + "_depth_frame"),
                ir_frame(this->camera_name_ + "_ir_frame");

    // Get camera parameters (extrinsic)
    checkVzReturnStatus(VZ_GetSensorExtrinsicParameters(deviceHandle_, &this->extrinsics_),
                      "Could not get extrinsics!");

    // Setup tf broadcaster
    static tf2_ros::StaticTransformBroadcaster tf_broadcaster;
    ros::Time now = ros::Time::now();

    // PsCameraExtrinsicParameters to ROS transform
    tf::Transform transform;
    tf::Matrix3x3 rotation_matrix(extrinsics_.rotation[0], extrinsics_.rotation[1], extrinsics_.rotation[2],
                                  extrinsics_.rotation[3], extrinsics_.rotation[4], extrinsics_.rotation[5],
                                  extrinsics_.rotation[6], extrinsics_.rotation[7], extrinsics_.rotation[8]);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // Publish static TFs
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = now;
    msg.transform.rotation.w = 1.0;

    // Camera base to Color Frame
    msg.header.frame_id = camera_frame;
    msg.child_frame_id = color_frame;
    tf_broadcaster.sendTransform(msg);

    // Color Frame to Aligned Frame
    msg.header.frame_id = color_frame;
    msg.child_frame_id = depth_frame;
    tf_broadcaster.sendTransform(msg);
    
    msg.header.frame_id = depth_frame;
    msg.child_frame_id = alignedcolor_frame;
    tf_broadcaster.sendTransform(msg);

    // Color Frame to Depth Frame
    msg.transform.translation.x = extrinsics_.translation[0] / 1000;
    msg.transform.translation.y = extrinsics_.translation[1] / 1000;
    msg.transform.translation.z = extrinsics_.translation[2] / 1000;
    msg.transform.rotation = orientation;
    msg.header.frame_id = color_frame;
    msg.child_frame_id = aligneddepth_frame;
    tf_broadcaster.sendTransform(msg);

    // Get camera parameters (intrinsic)
    checkVzReturnStatus(VZ_GetSensorIntrinsicParameters(deviceHandle_, VzToFSensor, &this->depth_intrinsics_),
                      "Could not get depth intrinsics!");
    checkVzReturnStatus(VZ_GetSensorIntrinsicParameters(deviceHandle_, VzColorSensor, &this->color_intrinsics_),
                      "Could not get color intrinsics!");

    // Initialise camera info messages
    sensor_msgs::CameraInfo info_msg;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = color_frame;
    info_msg.D = {color_intrinsics_.k1, color_intrinsics_.k2, color_intrinsics_.p1, color_intrinsics_.p2,
                  color_intrinsics_.k3};
    info_msg.K = {color_intrinsics_.fx, 0, color_intrinsics_.cx, 
                    0, color_intrinsics_.fy, color_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {color_intrinsics_.fx, 0, color_intrinsics_.cx,
                    0, 0, color_intrinsics_.fy,
                    color_intrinsics_.cy, 0, 0, 
                    0, 1, 0};
    info_msg.R.fill(0);
    info_msg.R[0] = 1;
    info_msg.R[4] = 1;
    info_msg.R[8] = 1;
    color_info_->setCameraInfo(info_msg);
    alignedDepth_info_->setCameraInfo(info_msg);

    info_msg.header.frame_id = depth_frame;
    info_msg.D = {depth_intrinsics_.k1, depth_intrinsics_.k2, depth_intrinsics_.p1, depth_intrinsics_.p2,
                  depth_intrinsics_.k3};
    info_msg.K = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, depth_intrinsics_.fy, depth_intrinsics_.cy,
                    0, 0, 1};
    info_msg.P = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, 0, depth_intrinsics_.fy,
                    depth_intrinsics_.cy, 0, 0,
                    0, 1, 0};
    
    depth_info_->setCameraInfo(info_msg);
    info_msg.header.frame_id = ir_frame;
    ir_info_->setCameraInfo(info_msg);
    alignedColor_info_->setCameraInfo(info_msg);

    ROS_INFO("Successfully received intrinsic and extrinsic parameters for device %d", this->device_index_);

    checkVzReturnStatus(VZ_SetTransformColorImgToDepthSensorEnabled(deviceHandle_, true),
                      "Could not SetTransformColorImgToDepthSensorEnabled!");
    checkVzReturnStatus(VZ_SetTransformDepthImgToColorSensorEnabled(deviceHandle_, true),
                      "Could not SetTransformDepthImgToColorSensorEnabled!");
}
