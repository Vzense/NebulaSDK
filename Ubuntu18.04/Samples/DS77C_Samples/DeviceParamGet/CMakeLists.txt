project(DeviceParamGet)
add_executable( ${PROJECT_NAME} DeviceParamGet.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)