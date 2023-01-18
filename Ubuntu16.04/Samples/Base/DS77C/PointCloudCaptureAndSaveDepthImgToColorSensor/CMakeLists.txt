project(PointCloudCaptureAndSaveDepthImgToColorSensor)
add_executable( ${PROJECT_NAME} PointCloudCaptureAndSaveDepthImgToColorSensor.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)