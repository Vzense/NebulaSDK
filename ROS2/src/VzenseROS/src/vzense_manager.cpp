#include "vzense_manager.hpp"
#include <thread>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std::chrono_literals;

VzenseManager::VzenseManager(int32_t device_index,std::string camera_name) :
        Node(camera_name),
        camera_name_(camera_name),      
        rgb_width_(-1),
        rgb_height_(-1),
        slope_(1450),
        deviceHandle_(0),
        sessionIndex_(0)
{
    signal(SIGSEGV, VzenseManager::sigsegv_handler);
    
    color_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/color/image_raw", 30);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/depth/image_raw", 30);
    ir_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/ir/image_raw", 30);
    alignedDepth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/transformedDepth/image_raw", 30);
    alignedColor_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_+"/transformedColor/image_raw", 30);
    
    colorinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/color/camera_info", 30);
    depthinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/depth/camera_info", 30);
    irinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/ir/camera_info", 30);
    alignedDepthinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/transformedDepth/camera_info", 30);
    alignedColorinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/transformedColor/camera_info", 30);
    pointclound2info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_name_+"/depth/points/camera_info", 30);
   
    pointclound2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(camera_name_+"/depth/points", 30);

    if(initDCAM(device_index))
    {
        timer_ = this->create_wall_timer(
          100ms, std::bind(&VzenseManager::timeout, this));
    }
}
void VzenseManager::sigsegv_handler(int sig) 
{
    signal(SIGSEGV, SIG_DFL);
    cout<<"Segmentation fault, stopping camera driver : %" << sig <<endl;
    rclcpp::shutdown();
}
bool VzenseManager::initDCAM(int32_t device_index)
{ 
    VzReturnStatus status = VzReturnStatus::VzRetOK;
    // Initialise the API
    status =VZ_Initialize();
    if (status != VzReturnStatus::VzRetOK)
    {
        RCLCPP_INFO(this->get_logger(),"VzInitialize failed! %d" ,status);
        return false;
    }
   // Get number of available devices
    uint32_t device_count = 0;
GET:
    int checkDeviceSec = 0;
    status = VZ_GetDeviceCount(&device_count);
    if (status != VzReturnStatus::VzRetOK)
    {
        RCLCPP_INFO(this->get_logger(), "VzGetDeviceCount failed! %d" ,status);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Get device count: %d" ,device_count); 
    if (0 == device_count)
    {
        this_thread::sleep_for(chrono::seconds(1));
        goto GET;
    }
    VzDeviceInfo* pPsDeviceInfo = new VzDeviceInfo;
    status =  VZ_GetDeviceInfo(device_index,pPsDeviceInfo);
    if (status != VzReturnStatus::VzRetOK)
    {
        RCLCPP_INFO(this->get_logger(), "VZ_GetDeviceInfo failed! %d" ,status);
        return false;
    }
    status = VZ_OpenDeviceByUri(pPsDeviceInfo->uri, &deviceHandle_);
    if (status != VzReturnStatus::VzRetOK)
    {
        RCLCPP_INFO(this->get_logger(), "VZ_OpenDeviceByUri failed! %d" ,status);
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "sn: %s" , pPsDeviceInfo->serialNumber);

    VZ_StartStream(deviceHandle_);
    
    /* add user define api call start*/
    // such as call the VZ_SetSpatialFilterEnabled
   
    /*
    status= VZ_SetSpatialFilterEnabled(deviceHandle_,true);
    RCLCPP_INFO( "SetSpatialFilterEnabled status: " << status);
    */
  
    /* add user define api call end*/
 
    set_sensor_intrinsics();

    const int BufLen = 64;
    char fw[BufLen] = { 0 };
    VZ_GetFirmwareVersion(deviceHandle_, fw, BufLen);
    RCLCPP_INFO(this->get_logger(), "fw: %s " , fw);

    VZ_SetTransformColorImgToDepthSensorEnabled(deviceHandle_, true);
    VZ_SetTransformDepthImgToColorSensorEnabled(deviceHandle_, true);
    RCLCPP_INFO(this->get_logger(), "------ the camera is runing ok ------" );
    return true;
}
void VzenseManager::timeout() 
{    
    // Get next frame set
    VzFrameReady psReadFrame = {0};
    VzReturnStatus status =  VZ_GetFrameReady(deviceHandle_, 1200, &psReadFrame);
    if (status != VzRetOK)
    {
        RCLCPP_INFO(this->get_logger(), "VZ_GetFrameReady failed! %d" ,status);
        return;
    }
    if (1 == psReadFrame.depth)
    {
        publicImage(VzDepthFrame, depth_pub_); 
    }
    if (1 == psReadFrame.ir)
    {
        publicImage(VzIRFrame, ir_pub_);           
    }
    if (1 == psReadFrame.color)
    {
        publicImage(VzColorFrame, color_pub_);           
    }
    if (1 == psReadFrame.transformedDepth)
    {
        publicImage(VzTransformDepthImgToColorSensorFrame, alignedDepth_pub_);            
    }
    if (1 == psReadFrame.transformedColor)
    {
        publicImage(VzTransformColorImgToDepthSensorFrame, alignedColor_pub_);            
    }  
}

bool VzenseManager::shutDownDCAM()
{ 
    bool bret = true;
    VzReturnStatus status = VzReturnStatus::VzRetOK;
     
    status = VZ_StopStream(deviceHandle_);
    if (status != VzReturnStatus::VzRetOK)
    {
        RCLCPP_INFO(this->get_logger(), "VZ_StopStream failed! %d" ,status);
        bret = false;
    }
    status = VZ_CloseDevice(&deviceHandle_);
    if (status != VzReturnStatus::VzRetOK)
    {
        RCLCPP_INFO(this->get_logger(), "VZ_CloseDevice failed! %d" ,status);
        bret = false;
    }
    status = VZ_Shutdown();
    if (status != VzReturnStatus::VzRetOK)
    {
        RCLCPP_INFO(this->get_logger(), "VZ_Shutdown failed! %d" ,status);
        bret = false;
    }
    return bret;
}

bool VzenseManager::publicImage(const VzFrameType type, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub)
{    
    std::string camera_frame(this->camera_name_ + "_frame"), 
                color_frame(this->camera_name_ + "_color_frame"),
                aligneddepth_frame(this->camera_name_ + "_transformedDepth_frame"),
                alignedcolor_frame(this->camera_name_ + "_transformedColor_frame"),
                depth_frame(this->camera_name_ + "_depth_frame"),
                ir_frame(this->camera_name_ + "_ir_frame"),
                points_frame(this->camera_name_ + "_points_frame");
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
        {
            cvMatType = CV_8UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_8UC1;
            cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
            cv_bridge::CvImage cvi_;
            cvi_.header.stamp = rclcpp::Clock().now();
            cvi_.header.frame_id = ir_frame;
            cvi_.encoding = "8UC1";
            cvi_.image = mat;
            sensor_msgs::msg::Image im_msg;
            cvi_.toImageMsg(im_msg);
            pub->publish(im_msg);
            ret = true;
        }
            break;
        case VzDepthFrame:
        {
            VzFrame &srcFrame = frame;
            const int len = srcFrame.width * srcFrame.height;
            VzVector3f* worldV = new VzVector3f[len];
            VZ_ConvertDepthFrameToPointCloudVector(deviceHandle_, &srcFrame, worldV); //Convert Depth frame to World vectors.

            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            cloud.points.resize(len);
            for (int i = 0; i < len; i++)
            { 
                if (0 != worldV[i].z && worldV[i].z !=65535)
                {
                    cloud.points[i].x = worldV[i].x/1000;
                    cloud.points[i].y = worldV[i].y/1000;
                    cloud.points[i].z = worldV[i].z/1000;
                    cloud.points[i].r = 255;
                    cloud.points[i].g = 255;
                    cloud.points[i].b = 255;
                  
                }
            }
            delete [] worldV;
            pcl::toROSMsg(cloud, output_msg);
            output_msg.header.frame_id=points_frame;
            output_msg.header.stamp = rclcpp::Clock().now();
            pointclound2_pub_->publish(output_msg);
        }
        case VzTransformDepthImgToColorSensorFrame:
        {
            cvMatType = CV_16UC1;
            imageEncodeType = sensor_msgs::image_encodings::TYPE_16UC1;
            cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
            cv_bridge::CvImage cvi_;
            cvi_.header.stamp = rclcpp::Clock().now();
            cvi_.header.frame_id = depth_frame;
            cvi_.encoding = "16UC1";
            cvi_.image = mat;
            sensor_msgs::msg::Image im_msg;     
            cvi_.toImageMsg(im_msg);
            pub->publish(im_msg);
            ret = true;
        }
            break;
        case VzColorFrame:
        case VzTransformColorImgToDepthSensorFrame:
        {
            cvMatType = CV_8UC3;
            imageEncodeType = sensor_msgs::image_encodings::BGR8;
            cv::Mat mat = cv::Mat(frame.height, frame.width, cvMatType, frame.pFrameData);
            cv_bridge::CvImage cvi_;
            cvi_.header.stamp = rclcpp::Clock().now();
            cvi_.header.frame_id = color_frame;
            cvi_.encoding = "bgr8";
            cvi_.image = mat;
            sensor_msgs::msg::Image im_msg;
            cvi_.toImageMsg(im_msg);
            pub->publish(im_msg);
            ret = true;
        }    
            break;
        default:
            ret = false;
            break;
        }

    }

    return ret;
}

void VzenseManager::set_sensor_intrinsics() 
{
    std::string camera_frame(this->camera_name_ + "_frame"), 
                color_frame(this->camera_name_ + "_color_frame"),
                aligneddepth_frame(this->camera_name_ + "_transformedDepth_frame"),
                alignedcolor_frame(this->camera_name_ + "_transformedColor_frame"),
                depth_frame(this->camera_name_ + "_depth_frame"),
                ir_frame(this->camera_name_ + "_ir_frame"),
                points_frame(this->camera_name_ + "_points_frame");

    // Get camera parameters (extrinsic)
    VZ_GetSensorExtrinsicParameters(deviceHandle_, &this->extrinsics_);

    // Setup tf broadcaster
    static tf2_ros::StaticTransformBroadcaster tf_broadcaster(this);
 

    // PsCameraExtrinsicParameters to ROS transform
 
    tf2::Matrix3x3 rotation_matrix(extrinsics_.rotation[0], extrinsics_.rotation[1], extrinsics_.rotation[2],
                                  extrinsics_.rotation[3], extrinsics_.rotation[4], extrinsics_.rotation[5],
                                  extrinsics_.rotation[6], extrinsics_.rotation[7], extrinsics_.rotation[8]);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    // Publish static TFs
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.transform.rotation.w = 1.0;

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = points_frame;
    tf_broadcaster.sendTransform(msg);

    msg.header.frame_id = camera_frame;
    msg.child_frame_id = ir_frame;
    tf_broadcaster.sendTransform(msg);

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
    msg.transform.rotation = tf2::toMsg(quaternion);
    msg.header.frame_id = color_frame;
    msg.child_frame_id = aligneddepth_frame;
    tf_broadcaster.sendTransform(msg);

    // Get camera parameters (intrinsic)
    VZ_GetSensorIntrinsicParameters(deviceHandle_, VzToFSensor, &this->depth_intrinsics_);
    VZ_GetSensorIntrinsicParameters(deviceHandle_, VzColorSensor, &this->color_intrinsics_);

    // Initialise camera info messages
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.distortion_model = "plumb_bob";
    info_msg.header.frame_id = color_frame;
    info_msg.d = {color_intrinsics_.k1, color_intrinsics_.k2, color_intrinsics_.p1, color_intrinsics_.p2,
                  color_intrinsics_.k3};
    info_msg.k = {color_intrinsics_.fx, 0, color_intrinsics_.cx, 
                    0, color_intrinsics_.fy, color_intrinsics_.cy,
                    0, 0, 1};
    info_msg.p = {color_intrinsics_.fx, 0, color_intrinsics_.cx,
                    0, 0, color_intrinsics_.fy,
                    color_intrinsics_.cy, 0, 0, 
                    0, 1, 0};
    info_msg.r.fill(0);
    info_msg.r[0] = 1;
    info_msg.r[4] = 1;
    info_msg.r[8] = 1;
    color_info_=info_msg;
    alignedDepth_info_=info_msg;

    info_msg.header.frame_id = depth_frame;
    info_msg.d = {depth_intrinsics_.k1, depth_intrinsics_.k2, depth_intrinsics_.p1, depth_intrinsics_.p2,
                  depth_intrinsics_.k3};
    info_msg.k = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, depth_intrinsics_.fy, depth_intrinsics_.cy,
                    0, 0, 1};
    info_msg.p = {depth_intrinsics_.fx, 0, depth_intrinsics_.cx,
                    0, 0, depth_intrinsics_.fy,
                    depth_intrinsics_.cy, 0, 0,
                    0, 1, 0};
    
    depth_info_=info_msg;
    alignedColor_info_=info_msg;

    info_msg.header.frame_id = ir_frame;
    ir_info_=info_msg;
    colorinfo_pub_->publish(color_info_);
    depthinfo_pub_->publish(depth_info_);
    irinfo_pub_->publish(ir_info_);
    alignedColorinfo_pub_->publish(alignedColor_info_);
    alignedDepthinfo_pub_->publish(alignedDepth_info_);
    
    info_msg.header.frame_id = points_frame;
    pointclound2_info_ = info_msg;
    pointclound2info_pub_->publish(pointclound2_info_);
}
