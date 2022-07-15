project(DeviceConnectByAlias)
add_executable( ${PROJECT_NAME} DeviceConnectByAlias.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)