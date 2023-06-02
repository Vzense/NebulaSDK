#include "rclcpp/rclcpp.hpp"
#include <vzense_manager.hpp>

int main(int argc, char *argv[]) 
{
    if(argc==3)
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<VzenseManager>(string(argv[2]),string(argv[1]));
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    else
    {
        cout<<"argc is: "<<argc<<", needed >=2" <<endl;
    }
    return 0;
}
