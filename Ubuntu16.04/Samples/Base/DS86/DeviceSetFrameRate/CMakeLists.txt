project(DeviceSetFrameRate)
add_executable( ${PROJECT_NAME} DeviceSetFrameRate.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)