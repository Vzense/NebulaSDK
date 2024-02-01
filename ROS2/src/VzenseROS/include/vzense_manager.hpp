#ifndef VZENSE_MANAGER_H
#define VZENSE_MANAGER_H

#include "rclcpp/rclcpp.hpp"
// 使用std_msgs的内置消息或者自定义数据来传递图像数据
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

// opencv读取的图像数据传给sensor_msgs的图像消息
#include <cv_bridge/cv_bridge.hpp>

// 用来发布和订阅图像信息
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include "VzenseNebula_api.h"
using namespace std;
using namespace cv;


class VzenseManager : public rclcpp::Node{
public:
    explicit VzenseManager(int32_t device_index,std::string topic_name);
    bool initDCAM(int32_t device_index = 0);
    bool shutDownDCAM();
private:
    static void sigsegv_handler(int sig);
    bool publicImage(const VzFrameType type, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub);
    void set_sensor_intrinsics();
    void timeout();
private:
    sensor_msgs::msg::CameraInfo color_info_, depth_info_, ir_info_, alignedDepth_info_, alignedColor_info_, pointclound2_info_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_, depth_pub_, ir_pub_, alignedDepth_pub_, alignedColor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr colorinfo_pub_, depthinfo_pub_, irinfo_pub_, alignedDepthinfo_pub_, alignedColorinfo_pub_, pointclound2info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointclound2_pub_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

private:
    std::string camera_name_;
    // Nebula SDK var
    int32_t device_index_;
    uint16_t slope_;
    int rgb_width_, rgb_height_;
    VzDeviceHandle deviceHandle_;
    unsigned int sessionIndex_;  
    VzSensorIntrinsicParameters depth_intrinsics_{}, color_intrinsics_{};
    VzSensorExtrinsicParameters extrinsics_{};
};


#endif //VZENSE_MANAGER_H
