project(DeviceHotPlugCallbackcplusplus)
add_executable( ${PROJECT_NAME} DeviceHotPlugCallbackcplusplus.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)