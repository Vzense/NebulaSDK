#include <iostream>
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include <vzense_manager.hpp>

int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "vzense_manager");

    int32_t device_index = ros::param::param<int32_t>("~device_index", 0);

    VzenseManager manager = VzenseManager(device_index);
    dynamic_reconfigure::Server<vzense_param::Vzensetof_roscppConfig> server;
    dynamic_reconfigure::Server<vzense_param::Vzensetof_roscppConfig>::CallbackType f;
 
    f = boost::bind(&VzenseManager::paramCallback,&manager,_1,_2);
    server.setCallback(f);
    manager.run();

    return 0;
}
