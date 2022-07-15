project(DeviceConnectByIP)
add_executable( ${PROJECT_NAME} DeviceConnectByIP.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)