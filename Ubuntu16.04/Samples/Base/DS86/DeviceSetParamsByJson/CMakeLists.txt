project(DeviceSetParamsByJson)
add_executable( ${PROJECT_NAME} DeviceSetParamsByJson.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)