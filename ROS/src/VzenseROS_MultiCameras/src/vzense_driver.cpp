#include <iostream>
#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include <vzense_manager.hpp>

int main(int argc, char *argv[]) 
{
    if(argc>=3)
    {
        ros::init(argc, argv, argv[1]);

        VzenseManager manager = VzenseManager(argv[2],argv[1]);
        dynamic_reconfigure::Server<vzense_param::Vzensetof_roscppConfig> server;
        dynamic_reconfigure::Server<vzense_param::Vzensetof_roscppConfig>::CallbackType f;
    
        f = boost::bind(&VzenseManager::paramCallback,&manager,_1,_2);
        server.setCallback(f);
        manager.run();
    }
    else
    {
        ROS_INFO("argc is: %d , needed >=3",argc);
    }
    return 0;
}
