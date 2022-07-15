project(DeviceSWTriggerMode)
add_executable( ${PROJECT_NAME} DeviceSWTriggerMode.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)