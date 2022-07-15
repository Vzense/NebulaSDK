project(DeviceHWTriggerMode)
add_executable( ${PROJECT_NAME} DeviceHWTriggerMode.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)