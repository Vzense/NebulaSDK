project(PointCloudVectorAndSave)
add_executable( ${PROJECT_NAME} PointCloudVectorAndSave.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)