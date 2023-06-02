project(DeviceParamSetGet)
add_executable( ${PROJECT_NAME} DeviceParamSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)