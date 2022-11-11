project(PointCloudCaptureAndSave)
add_executable( ${PROJECT_NAME} PointCloudCaptureAndSave.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)