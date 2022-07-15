project(TransformColorImgToDepthSensorFrame)
add_executable( ${PROJECT_NAME} TransformColorImgToDepthSensorFrame.cpp)
target_link_libraries(${PROJECT_NAME} vzenseDS_api)