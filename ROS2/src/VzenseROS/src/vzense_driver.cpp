#include "rclcpp/rclcpp.hpp"
#include <vzense_manager.hpp>

int main(int argc, char *argv[]) 
{
    rclcpp::init(argc, argv);

    int32_t device_index = 0;
    auto node = std::make_shared<VzenseManager>(device_index,"Vzense");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
