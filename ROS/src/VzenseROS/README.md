
# ROS Wrapper for the Vzense ToF library

## Overview
This ROS package facilitates depth IR and RGB data acquisition and processing for the DS77C depth cameras.

## Installation

- **Install the recommended [ROS distribution](http://wiki.ros.org/Distributions) for your operating system**
  - [ROS Install page](http://wiki.ros.org/ROS/Installation)

  **Version verified**
  |system|details|
  |---|---|
  |Ubuntu20.04|Noetic Ninjemys|
  |Ubuntu18.04|Melodic Morenia|
  |Ubuntu16.04|Kinetic Kame|
  |AArch64|Melodic|
- **Install the Vzense ROS package**

  - [Install NebulaSDK](https://github.com/Vzense/NebulaSDK)
    
    ```console
    git clone https://github.com/Vzense/NebulaSDK
    ```
    <p align="center"><img src="./doc/img/step0.png" /></p>
  - **Update SDK to ROS package**
    ```console
    cd ROS/src
    catkin_init_workspace
    ```
    After run <b>catkin_init_workspace</b>, it will generate the <b>CmakeLists.txt</b> in the <b>ROS/src</b> folder
    <p align="center"><img src="./doc/img/step1.png" /></p>

    ```console
    cd VzenseROS
    ```
    <p align="center"><img src="./doc/img/step2.png" /></p>

    <b>install.py</b>: copy <b>NebulaSDK</b> (match with your operating system) to <b>dependencies</b>, with the cmd "<b>python install.py (your operating system)</b>", take <b>Ubuntu18.04</b> as an example：
    ```console
    python install.py Ubuntu18.04
    ```
    <p align="center"><img src="./doc/img/step3.png" /></p>

 - **Build the VzenseROS package**
  ```console
  cd ../../
  catkin_make
  ```
  <p align="center"><img src="./doc/img/step4.png" /></p>
  <p align="center"><img src="./doc/img/step5.png" /></p>

 - **Environment setup**
  ```console
  source devel/setup.bash 
  ```
## Usage
- **Starting camera node**
    ```console
    roslaunch vzense_camera vzense_camera.launch
    ```
    <p align="center"><img src="./doc/img/step6.png" /></p>
    - <b>With Rviz show frame</b>
    
    ```console
    rviz
    ```
    <p align="center"><img src="./doc/img/step7.png" /></p>
    <p align="center"><img src="./doc/img/step8.png" /></p>
    - <b>With RQT dynamic reconfigure</b>

    ```console
    rosrun rqt_reconfigure rqt_reconfigure
    ```
    <p align="center"><img src="./doc/img/step9.png" /></p>
- **Show PointCloud**
    ```console
    roslaunch vzense_camera vzense_pointCloudxyz.launch
    ```
    <p align="center"><img src="./doc/img/step10.png" /></p>
    <p align="center"><img src="./doc/img/step11.png" /></p>
- **Show PointCloud with RGB**

    ```console
    roslaunch vzense_camera vzense_pointCloudxyzrgb.launch
    ```
    <p align="center"><img src="./doc/img/step12.png" /></p>
 
## Published Topics
The vzense_manager publishes messages defined by the [sensor_msgs](http://wiki.ros.org/sensor_msgs) package on the following topics
- /Vzense/depth/camera_info
- /Vzense/color/image_raw
- /Vzense/depth/image_raw
- /Vzense/ir/image_raw
- /Vzense/transformedDepth/image_raw
- /Vzense/transformedColor/image_raw

## Programming guide
If developers need to set camera parameters or algorithm switches, please refer to the following process.
Take calling <b>VZ_SetSpatialFilterParams</b> as an example
- Find the api From **dependencies/Include/VzenseNebula_api.h**
<p align="center"><img src="./doc/img/step13.png" /></p>

- Add the code into **/src/vzense_manager.cpp**
<p align="center"><img src="./doc/img/step14.png" /></p>
