project(DevHotPlugCallbackCpp)
add_executable( ${PROJECT_NAME} DeviceHotPlugCallbackcplusplus.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api)