project(DeviceStartStopSteaming)
add_executable( ${PROJECT_NAME} DeviceStartStopSteaming.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)