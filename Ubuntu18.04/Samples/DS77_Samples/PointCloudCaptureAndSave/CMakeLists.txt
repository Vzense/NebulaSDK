project(PointCloudCaptureAndSave)
add_executable( ${PROJECT_NAME} PointCloudCaptureAndSave.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)